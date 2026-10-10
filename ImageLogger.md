# Camera Image Logger (MCAP)

Three programs record a V4L2 camera into MCAP so it can be played back in
Foxglove's **Image** panel:

| Program | Source | Message | Encoding | Topic |
|---|---|---|---|---|
| `mcap_image_writer` | [src/imageWriter.cpp](src/imageWriter.cpp) | `foxglove.CompressedImage` (JPEG) | JSON | `/camera/image/compressed` |
| `mcap_proto_image_writer` | [src/protoImageWriter.cpp](src/protoImageWriter.cpp) | `foxglove.CompressedImage` (JPEG) | Protobuf | `/camera/image/compressed` |
| `mcap_compressed_proto_image_writer` | [src/compressedProtoImageWrite.cpp](src/compressedProtoImageWrite.cpp) | `foxglove.CompressedVideo` (H.264) | Protobuf | `/camera/video/h264` |

All three use [src/camera.hpp](src/camera.hpp) for capture.

## How capture works (shared)

`camera.hpp` is a small V4L2 wrapper that has no OpenCV dependency:

1. Opens the device and checks that it is a streaming **video capture** node.
2. Requests a pixel format at the chosen size and fps. The JPEG writers ask
   for **MJPEG**, so every frame arrives from the camera as a finished JPEG
   and nothing gets re-encoded. The H.264 writer asks for raw **YUYV**
   instead, so it never has to decode a JPEG before encoding video.
3. Maps 4 kernel buffers into memory (`mmap`), then dequeues each frame,
   passes its bytes to the writer, and requeues the buffer.

Ctrl+C (SIGINT) and SIGTERM stop the loop cleanly, so the MCAP file is always
closed with a valid footer.

> **Picking the device:** a webcam often shows up as two nodes. On the dev
> laptop, `/dev/video0` is the camera and `/dev/video1` is a metadata node
> that produces no frames. List the formats a node supports with:
> ```bash
> v4l2-ctl -d /dev/video0 --list-formats-ext
> ```
> A node that lists no formats is not a camera. The writer rejects it with
> "not a streaming video capture device".

## JSON logger setup

**Dependencies:** none beyond the vendored MCAP headers.

**Schema:** a JSON Schema string written in the source file. The schema name
must be `foxglove.CompressedImage`; Foxglove recognises the message by that
name.

```cpp
mcap::Schema schema("foxglove.CompressedImage", "jsonschema", kImageSchemaText);
mcap::Channel channel("/camera/image/compressed", "json", schema.id);
```

**Per frame:** JSON can't hold raw bytes, so the JPEG is **base64-encoded**
into a string. The schema marks that string with
`"contentEncoding": "base64"`, which is Foxglove's rule for bytes in JSON:

```json
{"timestamp":{"sec":1760130000,"nsec":123},"frame_id":"camera","format":"jpeg","data":"/9j/4AAQ..."}
```

**Run:**
```bash
./scripts/build.sh
#                             output      device      seconds  w   h   fps
./scripts/run_image_writer.sh camera.mcap /dev/video0 10       640 480 30
```

## Protobuf logger setup

**Dependencies:** the protobuf library and `protoc`
(`sudo apt install libprotobuf-dev protobuf-compiler`; tested with 3.12.4).

**Schema file:** [proto/foxglove/CompressedImage.proto](proto/foxglove/CompressedImage.proto),
copied from [foxglove/schemas](https://github.com/foxglove/schemas). Keep the
field numbers exactly as upstream, or Foxglove will misread the data.

**Code generation:** [CMakeLists.txt](CMakeLists.txt) runs `protoc` to generate
`foxglove/CompressedImage.pb.{h,cc}` in `build/proto_gen/`, builds them into a
`foxglove_proto` library, and links the writer against it. (It calls `protoc`
through a plain `add_custom_command` because `protobuf_generate` in CMake 3.22
puts the output files of `.proto` files in subfolders in the wrong place.)

**Schema in the MCAP file:** for protobuf, the MCAP schema data is a
serialized `FileDescriptorSet` that contains the message's `.proto` and
everything it imports (`google/protobuf/timestamp.proto`). The writer builds
it from the generated descriptor and adds each import **before** the file that
imports it, so strict readers can load the files in order:

```cpp
const auto *descriptor = foxglove::CompressedImage::descriptor();
mcap::Schema schema(descriptor->full_name(), "protobuf", build_file_descriptor_set(descriptor));
mcap::Channel channel("/camera/image/compressed", "protobuf", schema.id);
```

Because the descriptors travel inside the file, readers don't need the
`.proto` files to decode it.

**Per frame:** the raw JPEG bytes go straight into the `bytes data` field and
the message is serialized. There is no text conversion:

```cpp
image.mutable_timestamp()->set_seconds(now / 1000000000ULL);
image.mutable_timestamp()->set_nanos(now % 1000000000ULL);
image.set_data(jpeg, size);
image.SerializeToString(&serialized);
```

**Run:**
```bash
./scripts/build.sh
#                                   output            device      seconds  w   h   fps
./scripts/run_proto_image_writer.sh camera_proto.mcap /dev/video0 10       640 480 30
```

## H.264 video logger setup

**Dependencies:** everything the protobuf logger needs, plus the FFmpeg
development libraries with libx264
(`sudo apt install libavcodec-dev libavutil-dev libswscale-dev`; tested with
FFmpeg 4.4). CMake finds them through `pkg-config`.

**Schema file:** [proto/foxglove/CompressedVideo.proto](proto/foxglove/CompressedVideo.proto),
also copied from foxglove/schemas. Its field numbers are **different** from
CompressedImage's (`frame_id = 2`, `data = 3`, `format = 4`), so don't copy one
file to make the other. CMake generates both in the same `protoc` loop.

**Pipeline per frame:**

```
camera (YUYV) -> sws_scale -> YUV420P -> libx264 -> H.264 frame -> CompressedVideo -> MCAP
```

**Encoder settings:** these follow Foxglove's rules for H.264:

| Setting | Value | Why |
|---|---|---|
| Bitstream | Annex B (`00 00 00 01` start codes) | the format Foxglove expects |
| Messages | one encoded frame per message | Foxglove decodes frame by frame |
| B-frames | `max_b_frames = 0`, `tune=zerolatency` | Foxglove doesn't support B-frames; each input frame comes straight back out |
| SPS/PPS | sent before **every** keyframe (no `GLOBAL_HEADER` flag) | playback can start or seek at any keyframe |
| Keyframe interval | `gop_size = fps` (one per second) | limits how far back a seek has to go |
| Profile / preset | `baseline`, `veryfast` | low CPU, decodes everywhere |
| Bitrate | 2000 kbps by default (7th argument) | trade quality for size |

Each message keeps the capture time of its frame: the writer maps the
encoder's `pts` back to the timestamp taken when the frame was dequeued. On
exit the encoder is flushed, so the last frames aren't lost.

**Run:**
```bash
./scripts/build.sh
#                                              output           device      seconds  w   h   fps  kbps
./scripts/run_compressed_proto_image_writer.sh camera_h264.mcap /dev/video0 10       640 480 30   2000
```

Check the YUYV sizes your camera supports with `v4l2-ctl --list-formats-ext`.
They're often fewer and slower than the MJPEG ones (on the dev laptop, YUYV at
1280×720 runs at only 10 fps).

## Comparison

Measured on the dev laptop's integrated webcam, 640×480 at 30 fps, 3-second
recording, 85 frames each:

| | JSON (JPEG) | Protobuf (JPEG) | Protobuf (H.264) |
|---|---|---|---|
| File size | 6.96 MB | 5.20 MB | **0.73 MB** |
| Approx. size per second | ~2.3 MB/s | ~1.7 MB/s | **~0.25 MB/s** (2 Mbps) |
| Relative to JSON | 1× | ~0.75× | **~0.1×** |
| How the image is stored | base64 text (+33% bytes) | raw JPEG bytes | H.264 (keyframe + differences) |
| Each frame decodes on its own | yes | yes | no, only from the last keyframe |
| CPU per frame | base64 encode | one copy + serialize | YUV convert + x264 encode (highest) |
| Image quality | camera's JPEG | camera's JPEG | set by bitrate |
| Build dependencies | none | libprotobuf + protoc | + FFmpeg/libx264 |
| Readable in a text editor / `mcap cat` | yes | no (binary) | no (binary) |
| Foxglove Image panel | ✅ | ✅ | ✅ |

**Use JSON** for quick experiments, or when you want zero dependencies and
messages you can read by eye.

**Use protobuf JPEG** when every frame must stand on its own: pulling single
frames out for datasets or labelling, or when the CPU can't spare time for
video encoding.

**Use H.264** for long recordings or on a robot with limited disk space or
upload bandwidth. Files are about 7× smaller than protobuf JPEG at similar
visual quality. The costs are encoding CPU, and that a frame can only be
decoded starting from the keyframe before it. If the robot's SoC has a
hardware encoder (e.g. `h264_v4l2m2m`, `h264_nvenc`, `h264_vaapi`), swapping
the encoder name in `H264Encoder::open` can remove most of that CPU cost.

The MCAP files themselves are uncompressed. Compression is turned off in this
sandbox, and JPEG and H.264 data barely compress anyway.

## Viewing in Foxglove

1. Open the `.mcap` file in Foxglove.
2. Add an **Image** panel.
3. Select the topic: `/camera/image/compressed` for the JPEG writers, or
   `/camera/video/h264` for the H.264 writer.
