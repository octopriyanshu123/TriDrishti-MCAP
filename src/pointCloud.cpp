// Logs simulated 1D, 2D and 3D range data to MCAP (protobuf), one thread per channel:
//   /sensor/range       i2w_example.Range      1D rangefinder, 50 Hz   -> Foxglove Plot panel
//   /sensor/scan        foxglove.LaserScan     2D LiDAR, 10 Hz         -> Foxglove 3D panel
//   /sensor/pointcloud  foxglove.PointCloud    3D LiDAR, 10 Hz         -> Foxglove 3D panel
// The 2D and 3D sensors "see" the same box-shaped room, with random noise on every reading.
#define MCAP_IMPLEMENTATION
#include <mcap/writer.hpp>

#include <google/protobuf/descriptor.pb.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "foxglove/LaserScan.pb.h"
#include "foxglove/PointCloud.pb.h"
#include "i2w_example/Range.pb.h"

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

  // Sensor at the origin, not rotated (a zero quaternion would be invalid).
  void set_identity(foxglove::Pose *pose)
  {
    pose->mutable_position();
    pose->mutable_orientation()->set_w(1.0);
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

  // Room the simulated sensors sit in, in metres relative to the sensor.
  constexpr double kRoomMin[3] = {-4.0, -3.0, -1.0}; // floor 1 m below the sensor
  constexpr double kRoomMax[3] = {4.0, 3.0, 2.0};

  // Distance from the origin along unit direction d to the first wall of the room.
  double distance_to_room(const double d[3])
  {
    double best = 1e9;
    for (int i = 0; i < 3; ++i)
    {
      if (d[i] > 1e-9)
      {
        best = std::min(best, kRoomMax[i] / d[i]);
      }
      else if (d[i] < -1e-9)
      {
        best = std::min(best, kRoomMin[i] / d[i]);
      }
    }
    return best;
  }

  // State shared by all logging threads. McapWriter is not thread-safe, so every
  // write goes through writer_mutex.
  struct SharedLog
  {
    mcap::McapWriter writer;
    std::mutex writer_mutex;
    std::atomic<bool> failed{false};
    std::chrono::steady_clock::time_point deadline;
    bool run_forever = false;
  };

  // Fills `out` with one serialized message for time `now`, `t` seconds after start.
  using MakeMessage = std::function<void(mcap::Timestamp now, double t, std::string &out)>;

  // One sensor = one channel = one thread. Messages are built and serialized outside
  // the lock; only the write itself is serialized.
  void log_sensor(SharedLog &log, mcap::ChannelId channel_id, const char *topic, int rate_hz, MakeMessage make)
  {
    const auto period = std::chrono::nanoseconds(1000000000 / rate_hz);
    const auto start = std::chrono::steady_clock::now();
    auto next = start;
    std::string serialized;

    for (uint32_t seq = 0; !g_stop && !log.failed && (log.run_forever || next < log.deadline); ++seq)
    {
      const auto now = utc_now_ns();
      make(now, std::chrono::duration<double>(next - start).count(), serialized);

      mcap::Message msg;
      msg.channelId = channel_id;
      msg.sequence = seq;
      msg.logTime = now;
      msg.publishTime = now;
      msg.data = reinterpret_cast<const std::byte *>(serialized.data());
      msg.dataSize = serialized.size();

      mcap::Status written;
      {
        std::lock_guard<std::mutex> lock(log.writer_mutex);
        written = log.writer.write(msg);
      }
      if (!written.ok())
      {
        std::fprintf(stderr, "%s: write failed at message %u: %s\n", topic, seq, written.message.c_str());
        log.failed = true;
        return;
      }

      next += period;
      std::this_thread::sleep_until(next);
    }
  }

  // 1D: a distance that drifts slowly, plus sensor noise.
  MakeMessage make_range()
  {
    auto msg = std::make_shared<i2w_example::Range>();
    auto rng = std::make_shared<std::mt19937>(1);
    msg->set_frame_id("base_link");
    msg->set_min_range(0.02);
    msg->set_max_range(4.0);
    return [msg, rng](mcap::Timestamp now, double t, std::string &out)
    {
      std::normal_distribution<double> noise(0.0, 0.02);
      set_stamp(msg->mutable_timestamp(), now);
      msg->set_range(2.0 + 0.8 * std::sin(0.5 * t) + noise(*rng));
      msg->SerializeToString(&out);
    };
  }

  // 2D: 360 readings (1 degree apart) around the sensor, in the room's horizontal plane.
  MakeMessage make_scan()
  {
    constexpr int kReadings = 360;
    auto msg = std::make_shared<foxglove::LaserScan>();
    auto rng = std::make_shared<std::mt19937>(2);
    msg->set_frame_id("base_link");
    set_identity(msg->mutable_pose());
    msg->set_start_angle(-M_PI);
    msg->set_end_angle(M_PI - 2.0 * M_PI / kReadings);
    msg->mutable_ranges()->Resize(kReadings, 0.0);
    msg->mutable_intensities()->Resize(kReadings, 0.0);
    return [msg, rng](mcap::Timestamp now, double, std::string &out)
    {
      std::normal_distribution<double> noise(0.0, 0.03);
      std::uniform_real_distribution<double> intensity(50.0, 255.0);
      set_stamp(msg->mutable_timestamp(), now);
      for (int i = 0; i < kReadings; ++i)
      {
        const double angle = -M_PI + 2.0 * M_PI * i / kReadings;
        const double dir[3] = {std::cos(angle), std::sin(angle), 0.0};
        msg->set_ranges(i, distance_to_room(dir) + noise(*rng));
        msg->set_intensities(i, intensity(*rng));
      }
      msg->SerializeToString(&out);
    };
  }

  // 3D: a 16-ring LiDAR (-15..+15 degrees, 1024 points per ring) packed as
  // x, y, z, intensity float32 = 16 bytes per point.
  MakeMessage make_pointcloud()
  {
    constexpr int kRings = 16;
    constexpr int kColumns = 1024;
    constexpr uint32_t kStride = 4 * sizeof(float);
    auto msg = std::make_shared<foxglove::PointCloud>();
    auto rng = std::make_shared<std::mt19937>(3);
    auto points = std::make_shared<std::vector<float>>(static_cast<size_t>(kRings) * kColumns * 4);
    msg->set_frame_id("base_link");
    set_identity(msg->mutable_pose());
    msg->set_point_stride(kStride);
    const char *names[] = {"x", "y", "z", "intensity"};
    for (uint32_t i = 0; i < 4; ++i)
    {
      auto *field = msg->add_fields();
      field->set_name(names[i]);
      field->set_offset(i * sizeof(float));
      field->set_type(foxglove::PackedElementField::FLOAT32);
    }
    return [msg, rng, points](mcap::Timestamp now, double, std::string &out)
    {
      std::normal_distribution<double> noise(0.0, 0.02);
      std::uniform_real_distribution<float> intensity(0.0f, 255.0f);
      set_stamp(msg->mutable_timestamp(), now);
      float *p = points->data();
      for (int ring = 0; ring < kRings; ++ring)
      {
        const double elevation = (-15.0 + 30.0 * ring / (kRings - 1)) * M_PI / 180.0;
        for (int col = 0; col < kColumns; ++col)
        {
          const double azimuth = 2.0 * M_PI * col / kColumns;
          const double dir[3] = {std::cos(elevation) * std::cos(azimuth), std::cos(elevation) * std::sin(azimuth),
                                 std::sin(elevation)};
          const double r = distance_to_room(dir) + noise(*rng);
          *p++ = static_cast<float>(r * dir[0]);
          *p++ = static_cast<float>(r * dir[1]);
          *p++ = static_cast<float>(r * dir[2]);
          *p++ = intensity(*rng);
        }
      }
      msg->set_data(points->data(), points->size() * sizeof(float));
      msg->SerializeToString(&out);
    };
  }

} // namespace

int main(int argc, char **argv)
{
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  const std::string path = argc > 1 ? argv[1] : "pointcloud.mcap";
  const int seconds = argc > 2 ? std::atoi(argv[2]) : 10; // 0 = until Ctrl+C
  const std::string compression = argc > 3 ? argv[3] : "zstd";
  if (seconds < 0 || (compression != "zstd" && compression != "none"))
  {
    std::fprintf(stderr, "usage: %s [output.mcap] [seconds>=0, 0=until Ctrl+C] [zstd|none]\n", argv[0]);
    return 1;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  SharedLog log;

  // Point clouds are raw floats and compress well, unlike JPEG/H.264.
  mcap::McapWriterOptions options("");
  options.compression = compression == "zstd" ? mcap::Compression::Zstd : mcap::Compression::None;
  const auto opened = log.writer.open(path, options);
  if (!opened.ok())
  {
    std::fprintf(stderr, "failed to open %s: %s\n", path.c_str(), opened.message.c_str());
    return 1;
  }

  // add channels before starting threads
  const auto range_id = add_proto_channel(log.writer, "/sensor/range", i2w_example::Range::descriptor());
  const auto scan_id = add_proto_channel(log.writer, "/sensor/scan", foxglove::LaserScan::descriptor());
  const auto cloud_id = add_proto_channel(log.writer, "/sensor/pointcloud", foxglove::PointCloud::descriptor());

  log.run_forever = seconds == 0;
  log.deadline = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
  std::printf("logging 1D/2D/3D to %s (%s, %s)\n", path.c_str(), compression.c_str(),
              seconds > 0 ? (std::to_string(seconds) + " s").c_str() : "Ctrl+C to stop");

  // one thread per channel
  std::vector<std::thread> threads;
  threads.emplace_back(log_sensor, std::ref(log), range_id, "/sensor/range", 50, make_range());
  threads.emplace_back(log_sensor, std::ref(log), scan_id, "/sensor/scan", 10, make_scan());
  threads.emplace_back(log_sensor, std::ref(log), cloud_id, "/sensor/pointcloud", 10, make_pointcloud());
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

// How to run: ./mcap_pointcloud_writer pointcloud.mcap 10 zstd
