// Records a V4L2 camera into an MCAP file as H.264 video, one
// foxglove.CompressedVideo (protobuf) message per frame, viewable in Foxglove's Image panel.
// Raw YUYV frames are converted to YUV420P and encoded with libx264 via FFmpeg.
#define MCAP_IMPLEMENTATION
#include <mcap/writer.hpp>

#include <google/protobuf/descriptor.pb.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "camera.hpp"
#include "foxglove/CompressedVideo.pb.h"
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

  // A protobuf MCAP schema is a serialized FileDescriptorSet holding the message's
  // .proto file and every file it imports (here google/protobuf/timestamp.proto).
  std::string build_file_descriptor_set(const google::protobuf::Descriptor *descriptor)
  {
    google::protobuf::FileDescriptorSet fd_set;
    std::unordered_set<std::string> seen;
    add_file_with_deps(descriptor->file(), fd_set, seen);
    return fd_set.SerializeAsString();
  }

} // namespace

int main(int argc, char **argv)
{
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  const std::string path = argc > 1 ? argv[1] : "camera_h264.mcap";
  const std::string device = argc > 2 ? argv[2] : "/dev/video0";
  const int seconds = argc > 3 ? std::atoi(argv[3]) : 10; // 0 = until Ctrl+C
  const int width = argc > 4 ? std::atoi(argv[4]) : 640;
  const int height = argc > 5 ? std::atoi(argv[5]) : 480;
  const int fps = argc > 6 ? std::atoi(argv[6]) : 30;
  const int bitrate_kbps = argc > 7 ? std::atoi(argv[7]) : 2000;
  if (seconds < 0 || width <= 0 || height <= 0 || fps <= 0 || bitrate_kbps <= 0)
  {
    std::fprintf(stderr,
                 "usage: %s [output.mcap] [device] [seconds>=0, 0=until Ctrl+C] [width] [height] [fps] [bitrate_kbps]\n",
                 argv[0]);
    return 1;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  // Raw YUYV avoids decoding JPEG before re-encoding to H.264.
  Camera camera(g_stop);
  if (!camera.open(device, width, height, fps, V4L2_PIX_FMT_YUYV))
  {
    return 1;
  }

  H264Encoder encoder;
  if (!encoder.open(static_cast<int>(camera.width()), static_cast<int>(camera.height()), fps, bitrate_kbps))
  {
    return 1;
  }

  mcap::McapWriter writer;
  mcap::McapWriterOptions options("");
  options.compression = mcap::Compression::None; // H.264 is already compressed
  const auto opened = writer.open(path, options);
  if (!opened.ok())
  {
    std::fprintf(stderr, "failed to open %s: %s\n", path.c_str(), opened.message.c_str());
    return 1;
  }

  // Schema name must be the full protobuf message name; Foxglove matches on it.
  const auto *descriptor = foxglove::CompressedVideo::descriptor();
  mcap::Schema schema(descriptor->full_name(), "protobuf", build_file_descriptor_set(descriptor));
  writer.addSchema(schema);

  mcap::Channel channel("/camera/video/h264", "protobuf", schema.id);
  writer.addChannel(channel);

  std::printf("recording %ux%u H.264 @ %d kbps from %s to %s (%s)\n", camera.width(), camera.height(), bitrate_kbps,
              device.c_str(), path.c_str(), seconds > 0 ? (std::to_string(seconds) + " s").c_str() : "Ctrl+C to stop");

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
  foxglove::CompressedVideo video;
  video.set_frame_id("camera");
  video.set_format("h264");
  std::string serialized;
  // Capture time of each frame, keyed by pts, so a packet keeps its frame's timestamp.
  std::unordered_map<int64_t, mcap::Timestamp> capture_times;
  int64_t next_pts = 0;
  uint32_t frames = 0;
  uint32_t keyframes = 0;
  bool failed = false;

  const auto write_packet = [&](const uint8_t *data, int size, int64_t pts, bool keyframe)
  {
    const auto it = capture_times.find(pts);
    const mcap::Timestamp stamp = it != capture_times.end() ? it->second : utc_now_ns();
    if (it != capture_times.end())
    {
      capture_times.erase(it);
    }
    video.mutable_timestamp()->set_seconds(static_cast<int64_t>(stamp / 1000000000ULL));
    video.mutable_timestamp()->set_nanos(static_cast<int32_t>(stamp % 1000000000ULL));
    video.set_data(data, static_cast<size_t>(size));
    video.SerializeToString(&serialized);

    mcap::Message msg;
    msg.channelId = channel.id;
    msg.sequence = frames;
    msg.logTime = stamp;
    msg.publishTime = stamp;
    msg.data = reinterpret_cast<const std::byte *>(serialized.data());
    msg.dataSize = serialized.size();
    const auto written = writer.write(msg);
    if (!written.ok())
    {
      std::fprintf(stderr, "write failed at frame %u: %s\n", frames, written.message.c_str());
      return false;
    }
    ++frames;
    keyframes += keyframe ? 1 : 0;
    return true;
  };

  while (!g_stop && (seconds == 0 || std::chrono::steady_clock::now() < deadline))
  {
    const bool ok = camera.grab([&](const unsigned char *yuyv, size_t)
                                {
      const int64_t pts = next_pts++;
      capture_times[pts] = utc_now_ns();
      return encoder.encode(yuyv, static_cast<int>(camera.stride()), pts, write_packet); });
    if (!ok)
    {
      failed = !g_stop;
      break;
    }
  }

  camera.close();
  if (!encoder.flush(write_packet))
  {
    failed = true;
  }
  writer.close();
  std::printf("wrote %u frames (%u keyframes) to %s\n", frames, keyframes, path.c_str());
  google::protobuf::ShutdownProtobufLibrary();
  return failed ? 1 : 0;
}

// How to run: ./mcap_compressed_proto_image_writer camera_h264.mcap /dev/video0 10 640 480 30 2000
