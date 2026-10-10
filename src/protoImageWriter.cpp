// Records MJPEG frames from a V4L2 camera into an MCAP file as
// foxglove.CompressedImage (protobuf encoding), viewable in Foxglove's Image panel.
// Unlike the JSON writer, the JPEG bytes are stored as-is (no base64).
#define MCAP_IMPLEMENTATION
#include <mcap/writer.hpp>

#include <google/protobuf/descriptor.pb.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_set>

#include "camera.hpp"
#include "foxglove/CompressedImage.pb.h"

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
  // .proto file and every file it imports (here google/protobuf/timestamp.proto),
  // so readers can decode it without the .proto files.
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

  const std::string path = argc > 1 ? argv[1] : "camera_proto.mcap";
  const std::string device = argc > 2 ? argv[2] : "/dev/video0";
  const int seconds = argc > 3 ? std::atoi(argv[3]) : 10; // 0 = until Ctrl+C
  const int width = argc > 4 ? std::atoi(argv[4]) : 640;
  const int height = argc > 5 ? std::atoi(argv[5]) : 480;
  const int fps = argc > 6 ? std::atoi(argv[6]) : 30;
  if (seconds < 0 || width <= 0 || height <= 0 || fps <= 0)
  {
    std::fprintf(stderr, "usage: %s [output.mcap] [device] [seconds>=0, 0=until Ctrl+C] [width] [height] [fps]\n",
                 argv[0]);
    return 1;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  Camera camera(g_stop);
  if (!camera.open(device, width, height, fps))
  {
    return 1;
  }

  mcap::McapWriter writer;
  mcap::McapWriterOptions options("");
  options.compression = mcap::Compression::None; // JPEG is already compressed
  const auto opened = writer.open(path, options);
  if (!opened.ok())
  {
    std::fprintf(stderr, "failed to open %s: %s\n", path.c_str(), opened.message.c_str());
    return 1;
  }

  // Schema name must be the full protobuf message name; Foxglove matches on it.
  const auto *descriptor = foxglove::CompressedImage::descriptor();
  mcap::Schema schema(descriptor->full_name(), "protobuf", build_file_descriptor_set(descriptor));
  writer.addSchema(schema);

  mcap::Channel channel("/camera/image/compressed", "protobuf", schema.id);
  writer.addChannel(channel);

  std::printf("recording %ux%u MJPEG from %s to %s (%s)\n", camera.width(), camera.height(), device.c_str(),
              path.c_str(), seconds > 0 ? (std::to_string(seconds) + " s").c_str() : "Ctrl+C to stop");

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
  foxglove::CompressedImage image;
  image.set_frame_id("camera");
  image.set_format("jpeg");
  std::string serialized;
  uint32_t frames = 0;
  bool failed = false;

  while (!g_stop && (seconds == 0 || std::chrono::steady_clock::now() < deadline))
  {
    const bool ok = camera.grab([&](const unsigned char *jpeg, size_t size)
                                {
      const auto now = utc_now_ns();
      image.mutable_timestamp()->set_seconds(static_cast<int64_t>(now / 1000000000ULL));
      image.mutable_timestamp()->set_nanos(static_cast<int32_t>(now % 1000000000ULL));
      image.set_data(jpeg, size);
      image.SerializeToString(&serialized);

      mcap::Message msg;
      msg.channelId = channel.id;
      msg.sequence = frames;
      msg.logTime = now;
      msg.publishTime = now;
      msg.data = reinterpret_cast<const std::byte *>(serialized.data());
      msg.dataSize = serialized.size();
      const auto written = writer.write(msg);
      if (!written.ok())
      {
        std::fprintf(stderr, "write failed at frame %u: %s\n", frames, written.message.c_str());
        return false;
      }
      ++frames;
      return true; });
    if (!ok)
    {
      failed = !g_stop;
      break;
    }
  }

  camera.close();
  writer.close();
  std::printf("wrote %u frames to %s\n", frames, path.c_str());
  google::protobuf::ShutdownProtobufLibrary();
  return failed ? 1 : 0;
}

// How to run: ./mcap_proto_image_writer camera_proto.mcap /dev/video0 10 640 480 30
