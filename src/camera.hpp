// Minimal V4L2 capture (MJPEG or raw YUYV) using mmap'd buffers, shared by the image writers.
#pragma once

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

class Camera
{
public:
  // stop is checked when a blocking call is interrupted by a signal (e.g. Ctrl+C).
  explicit Camera(const std::atomic<bool> &stop) : stop_(stop) {}
  ~Camera() { close(); }

  // pixel_format is a V4L2 fourcc, e.g. V4L2_PIX_FMT_MJPEG (JPEG frames) or V4L2_PIX_FMT_YUYV (raw).
  bool open(const std::string &device, uint32_t width, uint32_t height, uint32_t fps,
            uint32_t pixel_format = V4L2_PIX_FMT_MJPEG)
  {
    fd_ = ::open(device.c_str(), O_RDWR);
    if (fd_ < 0)
    {
      std::fprintf(stderr, "open %s: %s\n", device.c_str(), std::strerror(errno));
      return false;
    }

    v4l2_capability cap{};
    if (xioctl(VIDIOC_QUERYCAP, &cap) < 0 || !(cap.device_caps & V4L2_CAP_VIDEO_CAPTURE) ||
        !(cap.device_caps & V4L2_CAP_STREAMING))
    {
      std::fprintf(stderr, "%s is not a streaming video capture device (try another /dev/videoN)\n", device.c_str());
      return false;
    }

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = pixel_format;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if (xioctl(VIDIOC_S_FMT, &fmt) < 0 || fmt.fmt.pix.pixelformat != pixel_format)
    {
      std::fprintf(stderr, "%s does not support %.4s capture\n", device.c_str(),
                   reinterpret_cast<const char *>(&pixel_format));
      return false;
    }
    width_ = fmt.fmt.pix.width;
    height_ = fmt.fmt.pix.height;
    stride_ = fmt.fmt.pix.bytesperline;

    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = fps;
    xioctl(VIDIOC_S_PARM, &parm); // best effort; the driver picks the nearest rate

    v4l2_requestbuffers req{};
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(VIDIOC_REQBUFS, &req) < 0 || req.count < 2)
    {
      std::fprintf(stderr, "VIDIOC_REQBUFS: %s\n", std::strerror(errno));
      return false;
    }

    for (uint32_t i = 0; i < req.count; ++i)
    {
      v4l2_buffer buf{};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if (xioctl(VIDIOC_QUERYBUF, &buf) < 0)
      {
        std::fprintf(stderr, "VIDIOC_QUERYBUF: %s\n", std::strerror(errno));
        return false;
      }
      void *start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
      if (start == MAP_FAILED)
      {
        std::fprintf(stderr, "mmap: %s\n", std::strerror(errno));
        return false;
      }
      buffers_.push_back({start, buf.length});
      if (xioctl(VIDIOC_QBUF, &buf) < 0)
      {
        std::fprintf(stderr, "VIDIOC_QBUF: %s\n", std::strerror(errno));
        return false;
      }
    }

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(VIDIOC_STREAMON, &type) < 0)
    {
      std::fprintf(stderr, "VIDIOC_STREAMON: %s\n", std::strerror(errno));
      return false;
    }
    streaming_ = true;
    return true;
  }

  // Blocks until a frame is ready, hands its bytes to on_frame, then requeues it.
  template <typename F>
  bool grab(F &&on_frame)
  {
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (xioctl(VIDIOC_DQBUF, &buf) < 0)
    {
      if (!stop_)
      {
        std::fprintf(stderr, "VIDIOC_DQBUF: %s\n", std::strerror(errno));
      }
      return false;
    }
    const bool ok = on_frame(static_cast<const unsigned char *>(buffers_[buf.index].start), buf.bytesused);
    xioctl(VIDIOC_QBUF, &buf);
    return ok;
  }

  void close()
  {
    if (streaming_)
    {
      v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(VIDIOC_STREAMOFF, &type);
      streaming_ = false;
    }
    for (auto &b : buffers_)
    {
      munmap(b.start, b.length);
    }
    buffers_.clear();
    if (fd_ >= 0)
    {
      ::close(fd_);
      fd_ = -1;
    }
  }

  uint32_t width() const { return width_; }
  uint32_t height() const { return height_; }
  uint32_t stride() const { return stride_; } // bytes per row, for raw formats

private:
  // Retries ioctl when interrupted by a signal, unless we were asked to stop.
  int xioctl(unsigned long request, void *arg)
  {
    int r;
    do
    {
      r = ioctl(fd_, request, arg);
    } while (r == -1 && errno == EINTR && !stop_);
    return r;
  }

  struct Buffer
  {
    void *start;
    size_t length;
  };
  const std::atomic<bool> &stop_;
  int fd_ = -1;
  bool streaming_ = false;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  uint32_t stride_ = 0;
  std::vector<Buffer> buffers_;
};
