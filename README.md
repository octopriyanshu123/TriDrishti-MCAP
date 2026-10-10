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
