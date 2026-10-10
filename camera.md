# Combined Logger: Camera Output Explained

[src/logger.cpp](src/logger.cpp) (`mcap_logger`) writes five channels into one
MCAP file. This page explains how to run it and what each line of its console
output means. Most of the output comes from the H.264 camera encoder.

## Running it

```bash
./mcap_logger logger.mcap 10 /dev/video0 zstd
#             │           │  │           └─ chunk compression: zstd | none
#             │           │  └─ camera device, or "test" for a generated pattern
#             │           └─ seconds to record (0 = until Ctrl+C)
#             └─ output file
```

Or use `./scripts/run_logger.sh` with the same arguments.

The program starts one thread per channel. All threads share a single
`McapWriter`, and a mutex lets only one thread write at a time.

| Thread | Topic | Rate | Messages in 10 s |
|---|---|---|---|
| Camera → H.264 | `/camera/video/h264` | camera's frame rate (~30 fps) | ~290–300 |
| 3D point cloud | `/sensor/pointcloud` | 10 Hz | 100 |
| 2D scan | `/sensor/scan2d` | 20 Hz | 200 |
| Plot signals | `/plot/signals` | 50 Hz | 500 |
| Gauge | `/gauge/speed` | 10 Hz | 100 |

If the camera cannot be opened, the program prints a warning and the video
thread encodes a moving test pattern instead.

## Example output

```
logging video, 3D cloud, 2D scan, plot and gauge to logger.mcap (zstd, 10 s)
[libx264 @ 0x7b5880000c40] using cpu capabilities: MMX2 SSE2Fast SSSE3 SSE4.2 AVX FMA3 BMI2 AVX2
[libx264 @ 0x7b5880000c40] profile Constrained Baseline, level 3.0, 4:2:0, 8-bit
[libx264 @ 0x7b5880000c40] frame I:38    Avg QP:18.29  size: 19885
[libx264 @ 0x7b5880000c40] frame P:251   Avg QP:20.00  size:  7474
[libx264 @ 0x7b5880000c40] mb I  I16..4: 45.7%  0.0% 54.3%
[libx264 @ 0x7b5880000c40] mb P  I16..4: 23.7%  0.0%  6.2%  P16..4: 43.6% 13.2%  2.8%  0.0%  0.0%    skip:10.4%
[libx264 @ 0x7b5880000c40] final ratefactor: 20.58
[libx264 @ 0x7b5880000c40] coded y,uvDC,uvAC intra: 51.7% 61.0% 25.5% inter: 24.4% 20.9% 0.7%
[libx264 @ 0x7b5880000c40] i16 v,h,dc,p: 26% 30% 29% 15%
[libx264 @ 0x7b5880000c40] i4 v,h,dc,ddl,ddr,vr,hd,vl,hu: 23% 19% 29%  5%  5%  5%  5%  4%  4%
[libx264 @ 0x7b5880000c40] i8c dc,h,v,p: 65% 16% 17%  2%
[libx264 @ 0x7b5880000c40] kb/s:2185.37
wrote logger.mcap
```

Only the first and last lines come from `mcap_logger`. Everything tagged
`[libx264 @ ...]` comes from the x264 encoder inside FFmpeg. The hex number is
the address of the encoder context, which only matters if several encoders are
running at once.

x264 prints its summary when the encoder is closed, which happens when the video
thread finishes. That is why it appears just before `wrote logger.mcap`. If
stdout is redirected, the lines can appear in a different order, because
`printf` output is buffered and x264 writes to stderr.

### Start and finish

| Line | Meaning |
|---|---|
| `logging ... to logger.mcap (zstd, 10 s)` | The file is open, all five channels are registered, and the threads have started. |
| `wrote logger.mcap` | All threads finished, the writer closed, and the file has a valid footer. If a write fails, this line is replaced by an error and the exit code is 1. |

### Encoder setup

| Line | Meaning |
|---|---|
| `using cpu capabilities: ... AVX2` | The SIMD instruction sets x264 found on this CPU and will use for speed. Not an error. |
| `profile Constrained Baseline` | The H.264 profile requested in [src/h264_encoder.hpp](src/h264_encoder.hpp) (`profile=baseline`). It has no B-frames, which Foxglove needs, and every decoder supports it. |
| `level 3.0` | The H.264 level. It is the decoder capability needed for this resolution and frame rate. 640×480 at 30 fps fits in level 3.0. |
| `4:2:0, 8-bit` | Colour layout. The camera's YUYV (4:2:2) frames are converted to YUV420P before encoding, so colour is stored at half resolution in both directions. |

### Frame counts and sizes

```
frame I:38    Avg QP:18.29  size: 19885
frame P:251   Avg QP:20.00  size:  7474
```

- **I-frames (keyframes), 38:** complete pictures that decode without any other
  frame. Playback can only start or seek at an I-frame.
- **P-frames, 251:** frames stored as differences from the previous frame, so
  they are much smaller.
- **Total, 38 + 251 = 289 frames.** At exactly 30 fps, 10 s would be 300 frames.
  The camera delivered about 29 fps, and the first frame arrives a little after
  the clock starts.
- **Why 38 keyframes, not 10:** `gop_size = fps` asks for a keyframe at least
  once a second, which gives about 10. x264 also adds a keyframe whenever the
  picture changes a lot (scene-cut detection), for example when the camera moves
  or auto-exposure adjusts. The other ~28 come from that.
- **Avg QP:** the average quantiser. Lower means finer detail and more bytes.
  Values around 18–20 are good quality. P-frames use a slightly higher QP
  because they are predicted from the frame before.
- **size:** average bytes per frame. A keyframe is about 19.9 KB and a P-frame
  about 7.5 KB, so a keyframe costs about 2.7 P-frames.

### Macroblock types

H.264 splits each frame into 16×16-pixel **macroblocks** and chooses how to
code each one.

```
mb I  I16..4: 45.7%  0.0% 54.3%
mb P  I16..4: 23.7%  0.0%  6.2%  P16..4: 43.6% 13.2%  2.8%  0.0%  0.0%    skip:10.4%
```

- **`I16..4`:** intra blocks, predicted only from pixels in the same frame.
  - The three columns are 16×16, 8×8 and 4×4 prediction.
  - 8×8 is always 0% because Baseline profile does not allow it.
  - 4×4 is used for detailed areas, 16×16 for smooth areas.
- **`P16..4`:** inter blocks, predicted from the previous frame using motion
  vectors.
  - The five columns are partition sizes: 16×16, 16×8/8×16, 8×8, 8×4/4×8 and
    4×4.
  - Most blocks here move as one 16×16 piece.
- **`skip`:** blocks copied unchanged from the previous frame at almost no cost.
  10.4% is low for a still scene, which means the camera image is noisy or
  moving.
- **Intra blocks in P-frames:** 23.7% + 6.2% ≈ 30% of P-frame blocks were coded
  as intra. The encoder found no good match in the previous frame for them,
  which is typical for sensor noise or lighting changes.

### Rate control

| Line | Meaning |
|---|---|
| `final ratefactor: 20.58` | x264 adjusts an internal quality setting to meet the requested bitrate. This is where it ended up. |
| `kb/s:2185.37` | The actual average bitrate. The requested bitrate was 2000 kb/s (`kVideoBitrateKbps` in [src/logger.cpp](src/logger.cpp)). |

Check: (38 × 19885 + 251 × 7474) bytes over 289/30 s ≈ 2186 kb/s.

The bitrate came out about 9% over the target. The extra keyframes are the main
cause, and over a 10 s clip the rate control has little time to correct.

### Coding statistics (mostly for tuning)

| Line | Meaning |
|---|---|
| `coded y,uvDC,uvAC intra: 51.7% 61.0% 25.5% inter: 24.4% 20.9% 0.7%` | Percentage of blocks that needed residual data after prediction. The three columns are brightness (Y), the average of each colour block (uvDC) and colour detail (uvAC). The figures are given separately for intra and inter blocks. Low inter values mean motion prediction worked well. |
| `i16 v,h,dc,p: 26% 30% 29% 15%` | How often each 16×16 intra direction was used: vertical, horizontal, DC (flat average) and plane (gradient). |
| `i4 v,h,dc,ddl,ddr,vr,hd,vl,hu: ...` | The same for the nine 4×4 intra directions, which include the diagonals. |
| `i8c dc,h,v,p: 65% 16% 17% 2%` | The same for the 8×8 chroma (colour) blocks. DC dominates because colour usually changes slowly. |

None of these need action. They show what kind of picture the camera produced.

## What else is in the file

Only the video thread prints anything; the other four threads log silently.
In this run `logger.mcap` was about **19.7 MB**:

- **Video:** about 2.6 MB (≈ 2185 kb/s × 9.6 s). H.264 is already
  compressed, so zstd barely shrinks it.
- **3D cloud:** about 26.6 MB uncompressed. It is 129 × 129 points × 16 bytes
  × 100 messages.
- **2D scan:** about 3.3 MB uncompressed. It is 1025 points × 16 bytes × 200
  messages.
- **Plot and gauge:** small JSON messages.

zstd compresses the point clouds to roughly half their size, which accounts for
most of the 19.7 MB.

To count the messages per topic:

```bash
./build/mcap_example_reader logger.mcap
```

## Tuning the video

These constants are at the top of the video section in
[src/logger.cpp](src/logger.cpp):

| Constant | Default | Effect |
|---|---|---|
| `kVideoWidth`, `kVideoHeight` | 640, 480 | Requested camera size. The driver may pick the nearest supported size. |
| `kVideoFps` | 30 | Requested frame rate. Also sets the keyframe interval to 1 s. |
| `kVideoBitrateKbps` | 2000 | Target bitrate. Raise it for detail, lower it for smaller files. |

The encoder settings (`preset veryfast`, `tune zerolatency`, `profile
baseline`, no B-frames) are in [src/h264_encoder.hpp](src/h264_encoder.hpp).
They follow Foxglove's requirements for `foxglove.CompressedVideo`. See
[ImageLogger.md](ImageLogger.md) for why.
