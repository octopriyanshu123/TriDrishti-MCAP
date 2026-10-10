// YUYV -> YUV420P -> H.264 (Annex B) with libx264 through FFmpeg, shared by the H.264 writers.
#pragma once

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <cstdint>
#include <cstdio>
#include <string>

inline std::string av_error(int err)
{
  char buf[AV_ERROR_MAX_STRING_SIZE] = {};
  av_strerror(err, buf, sizeof(buf));
  return buf;
}

// Settings follow Foxglove's requirements: no B-frames, and SPS/PPS repeated
// in-band before every keyframe.
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
