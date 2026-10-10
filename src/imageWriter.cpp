// Records MJPEG frames from a V4L2 camera into an MCAP file as
// foxglove.CompressedImage (JSON encoding), viewable in Foxglove's Image panel.
#define MCAP_IMPLEMENTATION
#include <mcap/writer.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "camera.hpp"

namespace
{

  // Foxglove recognises this schema by name; in JSON, bytes fields are base64 strings.
  constexpr const char *kImageSchemaName = "foxglove.CompressedImage";
  constexpr const char *kImageSchemaText = R"({
"title": "foxglove.CompressedImage",
"type": "object",
"properties": {
  "timestamp": {
    "type": "object",
    "properties": {
      "sec": { "type": "integer", "minimum": 0 },
      "nsec": { "type": "integer", "minimum": 0, "maximum": 999999999 }
    }
  },
  "frame_id": { "type": "string" },
  "data": { "type": "string", "contentEncoding": "base64" },
  "format": { "type": "string" }
}
})";

  std::atomic<bool> g_stop{false};

  void on_signal(int) { g_stop = true; }

  mcap::Timestamp utc_now_ns()
  {
    return static_cast<mcap::Timestamp>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
  }

  void append_base64(std::string &out, const unsigned char *data, size_t size)
  {
    static constexpr char kTable[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t i = 0;
    for (; i + 2 < size; i += 3)
    {
      const uint32_t v = (data[i] << 16) | (data[i + 1] << 8) | data[i + 2];
      out += kTable[(v >> 18) & 63];
      out += kTable[(v >> 12) & 63];
      out += kTable[(v >> 6) & 63];
      out += kTable[v & 63];
    }
    if (i + 1 == size)
    {
      const uint32_t v = data[i] << 16;
      out += kTable[(v >> 18) & 63];
      out += kTable[(v >> 12) & 63];
      out += "==";
    }
    else if (i + 2 == size)
    {
      const uint32_t v = (data[i] << 16) | (data[i + 1] << 8);
      out += kTable[(v >> 18) & 63];
      out += kTable[(v >> 12) & 63];
      out += kTable[(v >> 6) & 63];
      out += '=';
    }
  }

} // namespace

int main(int argc, char **argv)
{
  const std::string path = argc > 1 ? argv[1] : "camera.mcap";
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

  mcap::Schema schema(kImageSchemaName, "jsonschema", kImageSchemaText);
  
  writer.addSchema(schema);

  mcap::Channel channel("/camera/image/compressed", "json", schema.id);
  writer.addChannel(channel);

  std::printf("recording %ux%u MJPEG from %s to %s (%s)\n", camera.width(), camera.height(), device.c_str(),
              path.c_str(), seconds > 0 ? (std::to_string(seconds) + " s").c_str() : "Ctrl+C to stop");

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
  std::string json;
  uint32_t frames = 0;
  bool failed = false;

  while (!g_stop && (seconds == 0 || std::chrono::steady_clock::now() < deadline))
  {
    const bool ok = camera.grab([&](const unsigned char *jpeg, size_t size)
                                {
      const auto now = utc_now_ns();
      json.clear();
      json.reserve(size * 4 / 3 + 160);
      json += "{\"timestamp\":{\"sec\":";
      json += std::to_string(now / 1000000000ULL);
      json += ",\"nsec\":";
      json += std::to_string(now % 1000000000ULL);
      json += "},\"frame_id\":\"camera\",\"format\":\"jpeg\",\"data\":\"";
      append_base64(json, jpeg, size);
      json += "\"}";

      mcap::Message msg;
      msg.channelId = channel.id;
      msg.sequence = frames;
      msg.logTime = now;
      msg.publishTime = now;
      msg.data = reinterpret_cast<const std::byte *>(json.data());
      msg.dataSize = json.size();
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
  return failed ? 1 : 0;
}

// How to run: ./mcap_image_writer camera.mcap /dev/video0 10 640 480 30
