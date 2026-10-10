# Plain MCAP Example

A standalone sandbox for logging with the foxglove MCAP C++ library directly.
It does not use i2w, eCAL, or DataTamer. The i2w runtime logger
(`runtime.logs()`) is a separate path; see [runtime logging](../../docs/runtime-logging.md).

The MCAP C++ headers (`releases/cpp/v2.1.3`) are vendored in `include/mcap/`
and used header-only with compression disabled, so there is no download and no
lz4/zstd dependency. To upgrade, replace that folder with
`cpp/mcap/include/mcap/` from a newer foxglove/mcap release.

- `mcap_example_writer` logs a simulated motor signal (`target_rpm`,
  `actual_rpm`, `current_a`) as JSON messages on `/demo/motor`.
- `mcap_example_reader` prints the channels, the first messages, and
  per-topic message counts.
- `mcap_image_writer` / `mcap_proto_image_writer` record a V4L2 camera as
  `foxglove.CompressedImage` (JSON / protobuf), and
  `mcap_compressed_proto_image_writer` records it as H.264
  `foxglove.CompressedVideo`. See
  [ImageLogger.md](ImageLogger.md) for setup and a comparison.

## Build

```bash
./scripts/build.sh
```

## Run

```bash
# [output.mcap] [samples] [rate_hz]; defaults: motor.mcap 500 100
./scripts/run_writer.sh motor.mcap 500 100
./scripts/run_reader.sh motor.mcap
```

Open `motor.mcap` in Foxglove or PlotJuggler to plot the fields.

```bash
# [output.mcap] [device] [seconds, 0=until Ctrl+C] [width] [height] [fps]
./scripts/run_image_writer.sh camera.mcap /dev/video0 10 640 480 30
./scripts/run_proto_image_writer.sh camera_proto.mcap /dev/video0 10 640 480 30
# H.264 adds [bitrate_kbps]
./scripts/run_compressed_proto_image_writer.sh camera_h264.mcap /dev/video0 10 640 480 30 2000
```

The protobuf writers need `libprotobuf-dev` and `protobuf-compiler`; the H.264
writer also needs `libavcodec-dev`, `libavutil-dev` and `libswscale-dev`.
