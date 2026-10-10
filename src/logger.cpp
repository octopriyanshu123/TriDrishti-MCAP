// Logs five channels into one MCAP file, one thread per channel:
//   /camera/video/h264  foxglove.CompressedVideo  H.264, 30 fps  -> Foxglove Image panel
//   /sensor/pointcloud  foxglove.PointCloud       3D, 10 Hz      -> Foxglove 3D panel
//   /sensor/scan2d      foxglove.PointCloud       2D, 20 Hz      -> Foxglove 3D panel
//   /plot/signals       i2w_example.Signals       JSON, 50 Hz    -> Foxglove Plot panel
//   /gauge/speed        i2w_example.Gauge         JSON, 10 Hz    -> Foxglove Gauge panel
// The 2D scan and 3D cloud span x (and y) from -512 to 512, with the height
// forming several peaks that rise and fall over time.
// The video comes from a V4L2 camera, or a generated test pattern when the
// device is "test" or cannot be opened.
#define MCAP_IMPLEMENTATION
#include <mcap/writer.hpp>

#include <google/protobuf/descriptor.pb.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "camera.hpp"
#include "foxglove/CompressedVideo.pb.h"
#include "foxglove/PointCloud.pb.h"
#include "h264_encoder.hpp"

namespace
{

  std::atomic<bool> g_stop{false};

  void on_signal(int) { g_stop = true; }

  mcap::Timestamp utc_now_ns()
  {
    return static_cast<mcap::Timestamp>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
  }

  void set_stamp(google::protobuf::Timestamp *stamp, mcap::Timestamp ns)
  {
    stamp->set_seconds(static_cast<int64_t>(ns / 1000000000ULL));
    stamp->set_nanos(static_cast<int32_t>(ns % 1000000000ULL));
  }

  // Adds file after everything it imports, so strict readers can build them in order.
  void add_file_with_deps(const google::protobuf::FileDescriptor *file, google::protobuf::FileDescriptorSet &fd_set,
                          std::unordered_set<std::string> &seen)
  {
    if (!seen.insert(file->name()).second)
    {
      return;
    }
    for (int i = 0; i < file->dependency_count(); ++i)
    {
      add_file_with_deps(file->dependency(i), fd_set, seen);
    }
    file->CopyTo(fd_set.add_file());
  }

  std::string build_file_descriptor_set(const google::protobuf::Descriptor *descriptor)
  {
    google::protobuf::FileDescriptorSet fd_set;
    std::unordered_set<std::string> seen;
    add_file_with_deps(descriptor->file(), fd_set, seen);
    return fd_set.SerializeAsString();
  }

  // Registers the message's schema and a protobuf channel for it; returns the channel id.
  mcap::ChannelId add_proto_channel(mcap::McapWriter &writer, const std::string &topic,
                                    const google::protobuf::Descriptor *descriptor)
  {
    mcap::Schema schema(descriptor->full_name(), "protobuf", build_file_descriptor_set(descriptor));
    writer.addSchema(schema);
    mcap::Channel channel(topic, "protobuf", schema.id);
    writer.addChannel(channel);
    return channel.id;
  }

  mcap::ChannelId add_json_channel(mcap::McapWriter &writer, const std::string &topic, const std::string &schema_name,
                                   const std::string &schema_text)
  {
    mcap::Schema schema(schema_name, "jsonschema", schema_text);
    writer.addSchema(schema);
    mcap::Channel channel(topic, "json", schema.id);
    writer.addChannel(channel);
    return channel.id;
  }

  constexpr const char *kSignalsSchema = R"({
"title": "Signals",
"type": "object",
"properties": {
  "sine": { "type": "number" },
  "cosine": { "type": "number" },
  "sawtooth": { "type": "number" },
  "noisy": { "type": "number" }
}
})";

  constexpr const char *kGaugeSchema = R"({
"title": "Gauge",
"type": "object",
"properties": {
  "value": { "type": "number" },
  "min": { "type": "number" },
  "max": { "type": "number" }
}
})";

  // State shared by all logging threads. McapWriter is not thread-safe, so every
  // write goes through writer_mutex.
  struct SharedLog
  {
    mcap::McapWriter writer;
    std::mutex writer_mutex;
    std::atomic<bool> failed{false};
    std::chrono::steady_clock::time_point deadline;
    bool run_forever = false;

    bool running() const
    {
      return !g_stop && !failed && (run_forever || std::chrono::steady_clock::now() < deadline);
    }

    // Writes one message under the lock; on failure, stops every thread.
    bool write(mcap::ChannelId channel_id, uint32_t seq, mcap::Timestamp stamp, const std::string &data,
               const char *topic)
    {
      mcap::Message msg;
      msg.channelId = channel_id;
      msg.sequence = seq;
      msg.logTime = stamp;
      msg.publishTime = stamp;
      msg.data = reinterpret_cast<const std::byte *>(data.data());
      msg.dataSize = data.size();

      mcap::Status written;
      {
        std::lock_guard<std::mutex> lock(writer_mutex);
        written = writer.write(msg);
      }
      if (!written.ok())
      {
        std::fprintf(stderr, "%s: write failed at message %u: %s\n", topic, seq, written.message.c_str());
        failed = true;
        return false;
      }
      return true;
    }
  };

  // Fills `out` with one serialized message for time `now`, `t` seconds after start.
  using MakeMessage = std::function<void(mcap::Timestamp now, double t, std::string &out)>;

  // One channel = one thread, at a fixed rate. Messages are built and serialized
  // outside the lock; only the write itself is serialized.
  void log_channel(SharedLog &log, mcap::ChannelId channel_id, const char *topic, int rate_hz, MakeMessage make)
  {
    const auto period = std::chrono::nanoseconds(1000000000 / rate_hz);
    const auto start = std::chrono::steady_clock::now();
    auto next = start;
    std::string serialized;

    for (uint32_t seq = 0; log.running(); ++seq)
    {
      const auto now = utc_now_ns();
      make(now, std::chrono::duration<double>(next - start).count(), serialized);
      if (!log.write(channel_id, seq, now, serialized, topic))
      {
        return;
      }
      next += period;
      std::this_thread::sleep_until(next);
    }
  }

  // ---------------------------------------------------------------- video

  constexpr int kVideoWidth = 640;
  constexpr int kVideoHeight = 480;
  constexpr int kVideoFps = 30;
  constexpr int kVideoBitrateKbps = 2000;

  // Moving colour gradient with a bouncing white square, as YUYV (2 pixels per 4 bytes).
  void fill_test_pattern(std::vector<unsigned char> &yuyv, int width, int height, int frame)
  {
    const int box = height / 6;
    const int box_x = static_cast<int>((width - box) * (0.5 + 0.5 * std::sin(frame * 0.05)));
    const int box_y = static_cast<int>((height - box) * (0.5 + 0.5 * std::cos(frame * 0.07)));
    for (int y = 0; y < height; ++y)
    {
      unsigned char *row = yuyv.data() + static_cast<size_t>(y) * width * 2;
      const bool box_row = y >= box_y && y < box_y + box;
      for (int x = 0; x < width; x += 2)
      {
        const bool in_box = box_row && x >= box_x && x < box_x + box;
        const unsigned char luma = in_box ? 235 : static_cast<unsigned char>(16 + ((x + y + frame * 4) % 220));
        unsigned char *p = row + x * 2;
        p[0] = luma;
        p[1] = in_box ? 128 : static_cast<unsigned char>(x * 255 / width);
        p[2] = luma;
        p[3] = in_box ? 128 : static_cast<unsigned char>(y * 255 / height);
      }
    }
  }

  // The video thread: camera (or test pattern) -> H.264 -> foxglove.CompressedVideo.
  void log_video(SharedLog &log, mcap::ChannelId channel_id, const char *topic, std::string device)
  {
    // Raw YUYV avoids decoding JPEG before re-encoding to H.264.
    Camera camera(g_stop);
    const bool live = device != "test" && camera.open(device, kVideoWidth, kVideoHeight, kVideoFps, V4L2_PIX_FMT_YUYV);
    if (!live)
    {
      camera.close();
      if (device != "test")
      {
        std::fprintf(stderr, "%s: no camera at %s, logging a test pattern instead\n", topic, device.c_str());
      }
    }
    const int width = live ? static_cast<int>(camera.width()) : kVideoWidth;
    const int height = live ? static_cast<int>(camera.height()) : kVideoHeight;

    H264Encoder encoder;
    if (!encoder.open(width, height, kVideoFps, kVideoBitrateKbps))
    {
      log.failed = true;
      return;
    }

    foxglove::CompressedVideo video;
    video.set_frame_id("camera");
    video.set_format("h264");
    std::string serialized;
    // Capture time of each frame, keyed by pts, so a packet keeps its frame's timestamp.
    std::unordered_map<int64_t, mcap::Timestamp> capture_times;
    int64_t next_pts = 0;
    uint32_t frames = 0;

    const auto write_packet = [&](const uint8_t *data, int size, int64_t pts, bool)
    {
      const auto it = capture_times.find(pts);
      const mcap::Timestamp stamp = it != capture_times.end() ? it->second : utc_now_ns();
      if (it != capture_times.end())
      {
        capture_times.erase(it);
      }
      set_stamp(video.mutable_timestamp(), stamp);
      video.set_data(data, static_cast<size_t>(size));
      video.SerializeToString(&serialized);
      return log.write(channel_id, frames++, stamp, serialized, topic);
    };

    const auto encode = [&](const unsigned char *yuyv, int stride)
    {
      const int64_t pts = next_pts++;
      capture_times[pts] = utc_now_ns();
      return encoder.encode(yuyv, stride, pts, write_packet);
    };

    if (live)
    {
      // The camera paces the loop.
      while (log.running())
      {
        if (!camera.grab([&](const unsigned char *yuyv, size_t) { return encode(yuyv, static_cast<int>(camera.stride())); }))
        {
          if (!g_stop && !log.failed)
          {
            log.failed = true;
          }
          break;
        }
      }
      camera.close();
    }
    else
    {
      std::vector<unsigned char> yuyv(static_cast<size_t>(width) * height * 2);
      const auto period = std::chrono::nanoseconds(1000000000 / kVideoFps);
      auto next = std::chrono::steady_clock::now();
      for (int frame = 0; log.running(); ++frame)
      {
        fill_test_pattern(yuyv, width, height, frame);
        if (!encode(yuyv.data(), width * 2))
        {
          log.failed = true;
          break;
        }
        next += period;
        std::this_thread::sleep_until(next);
      }
    }

    if (!log.failed && !encoder.flush(write_packet))
    {
      log.failed = true;
    }
  }

  // ---------------------------------------------------------------- peaks

  constexpr double kExtent = 512.0; // points span -512..512

  // A Gaussian bump whose height breathes between 40% and 100% over time.
  struct Peak
  {
    double x, y;   // centre (y unused in 2D)
    double height; // maximum height
    double width;  // standard deviation
    double speed;  // rad/s of the breathing
    double phase;

    double scale(double t) const { return height * (0.7 + 0.3 * std::sin(speed * t + phase)); }
  };

  // x, y, z, intensity float32 = 16 bytes per point; intensity = height, so the
  // 3D panel can colour the points by it.
  void init_xyzi_cloud(foxglove::PointCloud &msg, const char *frame_id)
  {
    msg.set_frame_id(frame_id);
    msg.mutable_pose()->mutable_position();
    msg.mutable_pose()->mutable_orientation()->set_w(1.0); // a zero quaternion would be invalid
    msg.set_point_stride(4 * sizeof(float));
    const char *names[] = {"x", "y", "z", "intensity"};
    for (uint32_t i = 0; i < 4; ++i)
    {
      auto *field = msg.add_fields();
      field->set_name(names[i]);
      field->set_offset(i * sizeof(float));
      field->set_type(foxglove::PackedElementField::FLOAT32);
    }
  }

  // 2D: one point per unit of x from -512 to 512 (1025 points) in the z=0 plane,
  // with y a sum of peaks plus sensor noise.
  MakeMessage make_scan2d()
  {
    static const std::vector<Peak> peaks = {
        {-400, 0, 80, 25, 1.3, 0.0}, {-200, 0, 160, 40, 0.7, 1.0}, {0, 0, 60, 20, 2.1, 2.0},
        {150, 0, 220, 35, 0.9, 3.0}, {380, 0, 120, 50, 1.6, 4.0},
    };
    constexpr int kPoints = 2 * static_cast<int>(kExtent) + 1;
    auto msg = std::make_shared<foxglove::PointCloud>();
    auto rng = std::make_shared<std::mt19937>(1);
    auto points = std::make_shared<std::vector<float>>(static_cast<size_t>(kPoints) * 4);
    init_xyzi_cloud(*msg, "base_link");
    return [msg, rng, points](mcap::Timestamp now, double t, std::string &out)
    {
      std::normal_distribution<double> noise(0.0, 1.5);
      set_stamp(msg->mutable_timestamp(), now);
      float *p = points->data();
      for (int i = 0; i < kPoints; ++i)
      {
        const double x = -kExtent + i;
        double y = 0.0;
        for (const auto &peak : peaks)
        {
          const double dx = (x - peak.x) / peak.width;
          y += peak.scale(t) * std::exp(-0.5 * dx * dx);
        }
        y += noise(*rng);
        *p++ = static_cast<float>(x);
        *p++ = static_cast<float>(y);
        *p++ = 0.0f;
        *p++ = static_cast<float>(y);
      }
      msg->set_data(points->data(), points->size() * sizeof(float));
      msg->SerializeToString(&out);
    };
  }

  // 3D: a grid over x, y in -512..512 (every 8 units, 129 x 129 points) with
  // z a sum of 2D peaks plus sensor noise.
  MakeMessage make_pointcloud()
  {
    static const std::vector<Peak> peaks = {
        {-300, -250, 150, 60, 0.8, 0.0}, {250, -200, 220, 80, 0.5, 1.5}, {0, 0, 100, 50, 1.4, 3.0},
        {-200, 300, 180, 70, 1.1, 4.5},  {320, 280, 130, 55, 0.6, 2.2},
    };
    constexpr int kStep = 8;
    constexpr int kSide = 2 * static_cast<int>(kExtent) / kStep + 1;
    auto msg = std::make_shared<foxglove::PointCloud>();
    auto rng = std::make_shared<std::mt19937>(2);
    auto points = std::make_shared<std::vector<float>>(static_cast<size_t>(kSide) * kSide * 4);
    init_xyzi_cloud(*msg, "base_link");
    return [msg, rng, points](mcap::Timestamp now, double t, std::string &out)
    {
      std::normal_distribution<double> noise(0.0, 1.0);
      double heights[8];
      for (size_t k = 0; k < peaks.size(); ++k)
      {
        heights[k] = peaks[k].scale(t);
      }
      set_stamp(msg->mutable_timestamp(), now);
      float *p = points->data();
      for (int iy = 0; iy < kSide; ++iy)
      {
        const double y = -kExtent + iy * kStep;
        for (int ix = 0; ix < kSide; ++ix)
        {
          const double x = -kExtent + ix * kStep;
          double z = 0.0;
          for (size_t k = 0; k < peaks.size(); ++k)
          {
            const double dx = (x - peaks[k].x) / peaks[k].width;
            const double dy = (y - peaks[k].y) / peaks[k].width;
            z += heights[k] * std::exp(-0.5 * (dx * dx + dy * dy));
          }
          z += noise(*rng);
          *p++ = static_cast<float>(x);
          *p++ = static_cast<float>(y);
          *p++ = static_cast<float>(z);
          *p++ = static_cast<float>(z);
        }
      }
      msg->set_data(points->data(), points->size() * sizeof(float));
      msg->SerializeToString(&out);
    };
  }

  // ---------------------------------------------------------------- plot and gauge

  // Plot: four signals to compare on one chart.
  MakeMessage make_signals()
  {
    auto rng = std::make_shared<std::mt19937>(3);
    return [rng](mcap::Timestamp, double t, std::string &out)
    {
      std::normal_distribution<double> noise(0.0, 0.1);
      const double sine = std::sin(2.0 * M_PI * 0.5 * t);
      const double cosine = std::cos(2.0 * M_PI * 0.5 * t);
      const double sawtooth = 2.0 * (t * 0.25 - std::floor(t * 0.25 + 0.5));
      char json[160];
      const int size = std::snprintf(json, sizeof(json), R"({"sine":%.6f,"cosine":%.6f,"sawtooth":%.6f,"noisy":%.6f})",
                                     sine, cosine, sawtooth, sine + noise(*rng));
      out.assign(json, static_cast<size_t>(size));
    };
  }

  // Gauge: a speed in 0..100 that wanders slowly. min/max are logged too so the
  // panel's range is visible in the data.
  MakeMessage make_gauge()
  {
    auto rng = std::make_shared<std::mt19937>(4);
    return [rng](mcap::Timestamp, double t, std::string &out)
    {
      std::normal_distribution<double> noise(0.0, 1.0);
      const double value = std::clamp(50.0 + 35.0 * std::sin(0.3 * t) + 10.0 * std::sin(1.7 * t) + noise(*rng), 0.0, 100.0);
      char json[96];
      const int size = std::snprintf(json, sizeof(json), R"({"value":%.3f,"min":0,"max":100})", value);
      out.assign(json, static_cast<size_t>(size));
    };
  }

} // namespace

int main(int argc, char **argv)
{
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  const std::string path = argc > 1 ? argv[1] : "logger.mcap";
  const int seconds = argc > 2 ? std::atoi(argv[2]) : 10; // 0 = until Ctrl+C
  const std::string device = argc > 3 ? argv[3] : "/dev/video0";
  const std::string compression = argc > 4 ? argv[4] : "zstd";
  if (seconds < 0 || (compression != "zstd" && compression != "none"))
  {
    std::fprintf(stderr, "usage: %s [output.mcap] [seconds>=0, 0=until Ctrl+C] [device|test] [zstd|none]\n", argv[0]);
    return 1;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  SharedLog log;

  // zstd shrinks the point clouds; the H.264 chunks barely change, which is cheap.
  mcap::McapWriterOptions options("");
  options.compression = compression == "zstd" ? mcap::Compression::Zstd : mcap::Compression::None;
  const auto opened = log.writer.open(path, options);
  if (!opened.ok())
  {
    std::fprintf(stderr, "failed to open %s: %s\n", path.c_str(), opened.message.c_str());
    return 1;
  }

  // add channels before starting threads
  const auto video_id = add_proto_channel(log.writer, "/camera/video/h264", foxglove::CompressedVideo::descriptor());
  const auto cloud_id = add_proto_channel(log.writer, "/sensor/pointcloud", foxglove::PointCloud::descriptor());
  const auto scan_id = add_proto_channel(log.writer, "/sensor/scan2d", foxglove::PointCloud::descriptor());
  const auto plot_id = add_json_channel(log.writer, "/plot/signals", "i2w_example.Signals", kSignalsSchema);
  const auto gauge_id = add_json_channel(log.writer, "/gauge/speed", "i2w_example.Gauge", kGaugeSchema);

  log.run_forever = seconds == 0;
  log.deadline = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
  std::printf("logging video, 3D cloud, 2D scan, plot and gauge to %s (%s, %s)\n", path.c_str(), compression.c_str(),
              seconds > 0 ? (std::to_string(seconds) + " s").c_str() : "Ctrl+C to stop");

  // one thread per channel
  std::vector<std::thread> threads;
  threads.emplace_back(log_video, std::ref(log), video_id, "/camera/video/h264", device);
  threads.emplace_back(log_channel, std::ref(log), cloud_id, "/sensor/pointcloud", 10, make_pointcloud());
  threads.emplace_back(log_channel, std::ref(log), scan_id, "/sensor/scan2d", 20, make_scan2d());
  threads.emplace_back(log_channel, std::ref(log), plot_id, "/plot/signals", 50, make_signals());
  threads.emplace_back(log_channel, std::ref(log), gauge_id, "/gauge/speed", 10, make_gauge());
  for (auto &thread : threads)
  {
    thread.join();
  }

  log.writer.close();
  google::protobuf::ShutdownProtobufLibrary();
  if (log.failed)
  {
    return 1;
  }
  std::printf("wrote %s\n", path.c_str());
  return 0;
}

// How to run: ./mcap_logger logger.mcap 10 /dev/video0 zstd
//             ./mcap_logger logger.mcap 10 test          (no camera)
