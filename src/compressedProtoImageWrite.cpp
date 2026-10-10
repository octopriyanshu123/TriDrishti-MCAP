// Records a V4L2 camera into an MCAP file as H.264 video, one
// foxglove.CompressedVideo (protobuf) message per frame, viewable in Foxglove's Image panel.
// Raw YUYV frames are converted to YUV420P and encoded with libx264 via FFmpeg.
#define MCAP_IMPLEMENTATION
#include <mcap/writer.hpp>

#include <google/protobuf/descriptor.pb.h>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

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

  std::string av_error(int err)
  {
    char buf[AV_ERROR_MAX_STRING_SIZE] = {};
    av_strerror(err, buf, sizeof(buf));
    return buf;
  }

  // YUYV -> YUV420P -> H.264 (Annex B). Settings follow Foxglove's requirements:
  // no B-frames, and SPS/PPS repeated in-band before every keyframe.
  class H264Encoder
  {
  public:
    ~H264Encoder()
    {
      sws_freeContext(sws_);
      av_packet_free(&packet_);
      av_frame_free(&frame_);
      avcodec_free_context(&ctx_);
    }

    bool open(int width, int height, int fps, int bitrate_kbps)
    {
      const AVCodec *codec = avcodec_find_encoder_by_name("libx264");
      if (!codec)
      {
        std::fprintf(stderr, "FFmpeg was built without libx264\n");
        return false;
      }
      ctx_ = avcodec_alloc_context3(codec);
      ctx_->width = width;
      ctx_->height = height;
      ctx_->pix_fmt = AV_PIX_FMT_YUV420P;
      ctx_->time_base = AVRational{1, fps};
      ctx_->framerate = AVRational{fps, 1};
      ctx_->gop_size = fps; // a keyframe every second, so playback can seek
      ctx_->max_b_frames = 0;
      ctx_->bit_rate = static_cast<int64_t>(bitrate_kbps) * 1000;
      // Without AV_CODEC_FLAG_GLOBAL_HEADER, x264 writes SPS/PPS before each keyframe.
      av_opt_set(ctx_->priv_data, "preset", "veryfast", 0);
      av_opt_set(ctx_->priv_data, "tune", "zerolatency", 0); // one frame in, one frame out
      av_opt_set(ctx_->priv_data, "profile", "baseline", 0);

      const int err = avcodec_open2(ctx_, codec, nullptr);
      if (err < 0)
      {
        std::fprintf(stderr, "avcodec_open2: %s\n", av_error(err).c_str());
        return false;
      }

      frame_ = av_frame_alloc();
      frame_->format = ctx_->pix_fmt;
      frame_->width = width;
      frame_->height = height;
      if (av_frame_get_buffer(frame_, 0) < 0)
      {
        std::fprintf(stderr, "av_frame_get_buffer failed\n");
        return false;
      }
      packet_ = av_packet_alloc();
      sws_ = sws_getContext(width, height, AV_PIX_FMT_YUYV422, width, height, AV_PIX_FMT_YUV420P, SWS_BILINEAR,
                            nullptr, nullptr, nullptr);
      return sws_ != nullptr;
    }

    // Encodes one YUYV frame; on_packet(data, size, pts, keyframe) runs for each finished frame.
    template <typename F>
    bool encode(const unsigned char *yuyv, int stride, int64_t pts, F &&on_packet)
    {
      if (av_frame_make_writable(frame_) < 0)
      {
        return false;
      }
      const uint8_t *src[] = {yuyv};
      const int src_stride[] = {stride};
      sws_scale(sws_, src, src_stride, 0, ctx_->height, frame_->data, frame_->linesize);
      frame_->pts = pts;
      return send(frame_, on_packet);
    }

    // Drains frames still inside the encoder.
    template <typename F>
    bool flush(F &&on_packet) { return send(nullptr, on_packet); }

  private:
    template <typename F>
    bool send(AVFrame *frame, F &&on_packet)
    {
      int err = avcodec_send_frame(ctx_, frame);
      if (err < 0)
      {
        std::fprintf(stderr, "avcodec_send_frame: %s\n", av_error(err).c_str());
        return false;
      }
      while ((err = avcodec_receive_packet(ctx_, packet_)) == 0)
      {
        const bool ok = on_packet(packet_->data, packet_->size, packet_->pts, (packet_->flags & AV_PKT_FLAG_KEY) != 0);
        av_packet_unref(packet_);
        if (!ok)
        {
          return false;
        }
      }
      if (err != AVERROR(EAGAIN) && err != AVERROR_EOF)
      {
        std::fprintf(stderr, "avcodec_receive_packet: %s\n", av_error(err).c_str());
        return false;
      }
      return true;
    }

    AVCodecContext *ctx_ = nullptr;
    AVFrame *frame_ = nullptr;
    AVPacket *packet_ = nullptr;
    SwsContext *sws_ = nullptr;
  };

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
