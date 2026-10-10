# Point Cloud Logging (MCAP): Format Decisions

This page covers the formats for logging 1D, 2D and 3D range data to MCAP,
and the simulated logger that uses them
([src/pointCloud.cpp](src/pointCloud.cpp), see [Implementation](#implementation)).
Open questions about the real sensors are at the end.

## Recommendation

Use **protobuf with the official Foxglove schemas**, the same approach as the
camera loggers (see [ImageLogger.md](ImageLogger.md)). Foxglove's panels
understand these schemas, and the existing `proto/` + `protoc` setup in
[CMakeLists.txt](CMakeLists.txt) can be reused.

| Data | Typical sensor | Format | Foxglove panel |
|---|---|---|---|
| **1D**: one distance | ultrasonic, ToF or laser rangefinder | small custom message | **Plot** |
| **2D**: one ring of distances | 2D LiDAR (RPLidar, Hokuyo) | `foxglove.LaserScan` | **3D** |
| **3D**: x, y, z points | 3D LiDAR, depth camera, stereo | `foxglove.PointCloud` | **3D** |
| 2D map built from scans | occupancy or elevation map | `foxglove.Grid` | 3D |

## 1D: single range

One number per reading isn't really a point cloud. A small message, like the
motor logger's, is enough:

```
range       float64   metres
min_range   float64   sensor limits, to tell "nothing in range" apart from a real reading
max_range   float64
frame_id    string    which sensor / where it is mounted
```

Plot `range` over time in the Plot panel. If you want the reading to appear
in 3D space, use a `LaserScan` with a single range instead.

## 2D: `foxglove.LaserScan`

A 2D LiDAR measures distances at evenly spaced angles in one plane, so the
scan is stored in polar form:

```
timestamp, frame_id, pose
start_angle, end_angle   radians; angle of reading i is interpolated between them
ranges[]                 distance for each angle
intensities[]            optional, same length as ranges
```

**Why LaserScan instead of a point cloud:** it stores exactly what the sensor
reported (angle + distance, including "no return" readings) and Foxglove turns
it into points itself. It also keeps the scan structure that 2D SLAM and
filtering code expect.

> **Size note:** in Foxglove's protobuf schema, `ranges` and `intensities` are
> `double`, so a reading costs 8 bytes. That's about the same as an x, y
> `float32` point cloud, so LaserScan doesn't save much space here. The ROS
> `sensor_msgs/LaserScan` uses `float32` and is about half the size. Choose
> LaserScan because it matches the sensor, not to save space.

## 3D: `foxglove.PointCloud`

This is the same idea as ROS `PointCloud2`: one packed binary buffer plus a
description of what each point contains.

```
timestamp, frame_id, pose
point_stride   bytes per point
fields[]       name, offset, type  (e.g. FLOAT32, UINT8, UINT16 ...)
data           packed bytes: point 0, point 1, ...
```

Example layout for a LiDAR with intensity (16 bytes per point):

```
fields: x(FLOAT32, offset 0)  y(FLOAT32, 4)  z(FLOAT32, 8)  intensity(FLOAT32, 12)
data:   [x y z i][x y z i][x y z i] ...
```

- **Choose fields per sensor:** add `ring`, `time` or `rgb` without changing
  the schema. Foxglove can colour points by any field.
- **Raw bytes:** like the image `data` field, there's no conversion per point.
  The buffer can usually be copied straight from the driver.
- **Use `float32`:** millimetre precision is far beyond what the sensors
  resolve, and `float64` doubles the size.

## Turn on MCAP compression for point clouds

The camera loggers write uncompressed MCAP files, because JPEG and H.264 are
already compressed. **Point clouds are raw floats and compress well:**

| Compression | Typical size reduction for point clouds | Notes |
|---|---|---|
| none | 1× | current sandbox setting |
| lz4 | ~1.5–2× | very fast, low CPU |
| **zstd** | **~2–4×** | best default for recording |

The sandbox currently builds with `MCAP_COMPRESSION_NO_LZ4` and
`MCAP_COMPRESSION_NO_ZSTD` defined. The point cloud logger should instead link
`libzstd` (`sudo apt install libzstd-dev`) and set
`options.compression = mcap::Compression::Zstd`. Compression applies to each
chunk of the file, so readers decompress transparently. Foxglove opens
compressed files directly.

These ratios are typical figures for real sensor data, not measurements. They
depend on the scene and the fields.

**Measured on the simulated logger:** zstd saved only **~8%** (5 s: 13.4 MB →
12.3 MB). That's expected: the simulator adds random noise to every reading and
draws intensity uniformly at random, and random bits don't compress. Real
sensors report rounded values (e.g. ranges in mm, integer intensities), which
compress much better. Re-measure with real recordings before relying on any
ratio.

## Implementation

[src/pointCloud.cpp](src/pointCloud.cpp) (`mcap_pointcloud_writer`) logs
**simulated** data for all three formats into one MCAP file, **one thread per
channel**. As in [src/writer.cpp](src/writer.cpp), the threads share one
`McapWriter` behind a mutex. Each thread builds and serializes its message
outside the lock, so only the write itself is serialized.

| Thread | Topic | Schema | Rate | Payload | Message size |
|---|---|---|---|---|---|
| 1D | `/sensor/range` | `i2w_example.Range` ([proto](proto/i2w_example/Range.proto)) | 50 Hz | 2 m ± 0.8 m slow drift + noise | ~51 B |
| 2D | `/sensor/scan` | `foxglove.LaserScan` | 10 Hz | 360 readings, 1° apart + random intensity | ~5.8 KB |
| 3D | `/sensor/pointcloud` | `foxglove.PointCloud` | 10 Hz | 16 rings × 1024 points, x/y/z/intensity `float32` | ~262 KB |

The 2D and 3D sensors sit at the origin of the same simulated box-shaped room
(x ±4 m, y ±3 m, floor 1 m below, ceiling 2 m above). Each reading is the
distance to the nearest wall plus Gaussian noise, so in Foxglove the scan
lines up with the middle of the point cloud. Everything uses frame
`base_link`, with an identity `pose`.

**Schemas:** the Foxglove `.proto` files in [proto/foxglove/](proto/foxglove/)
(`PointCloud`, `LaserScan`, `PackedElementField`, `Pose`, `Vector3`,
`Quaternion`) are copied unchanged from
[foxglove-sdk](https://github.com/foxglove/foxglove-sdk/tree/main/schemas/proto/foxglove).
`CMakeLists.txt` generates all `.proto` files in a single `protoc` call,
because they import each other.

**Compression:** this writer is the only one built with zstd (`libzstd-dev`).
The reader is also built with zstd, so it can open these files.

**Run:**
```bash
./scripts/build.sh
#                                output          seconds(0=Ctrl+C)  compression
./scripts/run_pointcloud_writer.sh pointcloud.mcap 10                 zstd   # or: none
./scripts/run_reader.sh pointcloud.mcap
```

To view the recording, follow [Viewing in Foxglove](#viewing-in-foxglove).

## Viewing in Foxglove

These steps work in both the Foxglove desktop app and the web app
(app.foxglove.dev). Foxglove decompresses zstd MCAP files automatically.
Button names can differ slightly between Foxglove versions.

### 1. Open the file

1. Record a file first, e.g.
   `./scripts/run_pointcloud_writer.sh pointcloud.mcap 10 zstd`.
2. In Foxglove, choose **Open local file** on the start screen (or
   **File → Open** in the desktop app), then pick `pointcloud.mcap`. You can
   also drag the file onto the Foxglove window.
3. The playback bar appears at the bottom. Press **play** (or space), or drag
   the slider to scrub through the recording.

To confirm the file loaded, add a **Data Source Info** panel. It should list
the three topics:

| Topic | Schema | Expected rate |
|---|---|---|
| `/sensor/range` | `i2w_example.Range` | 50 Hz |
| `/sensor/scan` | `foxglove.LaserScan` | 10 Hz |
| `/sensor/pointcloud` | `foxglove.PointCloud` | 10 Hz |

### 2. Set up the layout

Add the panels from the **Add panel** list (3D, Plot, Raw Messages, Data
Source Info, ...). To place several panels side by side, use **Split right** /
**Split down** in a panel's toolbar menu.

A useful layout for this file:

```
+-----------------------------+--------------------+
|                             |  Plot              |
|  3D                         |  /sensor/range     |
|  /sensor/scan               +--------------------+
|  /sensor/pointcloud         |  Raw Messages      |
|                             |  (any topic)       |
+-----------------------------+--------------------+
|  playback bar                                    |
+--------------------------------------------------+
```

### 3. 3D panel: 2D scan and 3D point cloud

1. Add a **3D** panel and open its settings (gear icon or the panel's
   settings sidebar).
2. Under **Frame**, set **Display frame** to `base_link`. All three topics use
   this frame, so it's usually picked automatically.
3. Under **Topics**, turn on the visibility (eye icon) of `/sensor/pointcloud`
   and `/sensor/scan`.
4. Click `/sensor/pointcloud` to change how it looks:
   - **Point size:** 2–3 px is easy to see. Raise it if the cloud looks
     empty.
   - **Color mode:** pick **Color map** (or **Gradient**), then set
     **Color by** to `z` to show height (floor vs walls) or to `intensity`.
     The fields come from the `fields` list in the message (x, y, z,
     intensity).
   - **Decay time:** keep at `0` to show only the latest cloud. A few seconds
     keeps older clouds on screen, which helps once the sensor moves.
5. Click `/sensor/scan` and set **Color mode** to **Flat** with a contrasting
   colour (e.g. red), and raise the point size. The scan should outline the
   room at the sensor's height, in the middle of the point cloud.
6. Navigate: left-drag rotates, right-drag (or Shift + drag) pans, scroll
   zooms. Switch between **3D** and **2D** (top-down) view with the toggle in
   the panel corner. Top-down shows the 8 × 6 m room most clearly.

What you should see: a box-shaped room 8 m × 6 m, with the floor 1 m below the
sensor. The cloud has 16 horizontal rings that hit the floor and walls, and
the red scan ring traces the walls through the middle.

### 4. Plot panel: 1D range

1. Add a **Plot** panel and click **Add series** (or **+**) in its settings.
2. Set the message path to `/sensor/range.range`. Typing `/sensor/range.`
   autocompletes the field names.
3. Optional: add `/sensor/range.min_range` and `/sensor/range.max_range` as
   two more series. They draw the sensor limits as flat lines, so readings
   outside them stand out.
4. The x-axis defaults to time. The `range` line should drift slowly between
   about 1.2 m and 2.8 m, with small noise on top.

### 5. Raw Messages panel: inspect a message

Add a **Raw Messages** panel and pick a topic to see the decoded fields of
the current message. That's handy for checking timestamps, `frame_id`,
`point_stride` and the `fields` list of the point cloud. (The point data
itself is binary and shows as a byte array.)

### 6. Save the layout

Once the panels are set up, save the layout (layout menu → **Save** /
**Export**) so the next recording opens with the same view.

### Troubleshooting

| Problem | Likely cause / fix |
|---|---|
| 3D panel is empty | Display frame isn't `base_link`, or the topic's eye icon is off. Also try a larger point size. |
| "Missing transform" warning | Display frame is set to a frame that isn't in the file. Set it to `base_link`. |
| Cloud is one colour | Color mode is Flat. Switch to Color map and choose a **Color by** field. |
| Plot is empty | Message path is wrong. It must be `/sensor/range.range` (topic, then `.field`). |
| File won't open / topics missing | The recording was cut off without closing. Stop the writer with Ctrl+C or `kill` (both close the file cleanly), never `kill -9`. |

## Alternatives considered

| Option | Why not (for now) |
|---|---|
| ROS 2 `sensor_msgs/PointCloud2` / `LaserScan` (CDR) | Only worth it if recordings will be replayed with `ros2 bag` or ROS tools. Same layout ideas, so switching later is easy. |
| PCD / PLY files | Good for single snapshots. They don't hold a timestamped series or sit alongside other topics the way MCAP does. |
| JSON | Base64 would inflate the data by 33% and converting it costs CPU. That was already a problem for images, and point clouds are bigger. |
| Draco (lossy point-cloud compression) | 10×+ smaller, but lossy and an extra dependency. Revisit only if disk or upload space becomes a real limit. |

## Open questions

1. **Which sensors?** Model numbers decide the fields (intensity, ring, time,
   rgb) and whether a vendor SDK or ROS driver supplies the data.
2. **Swap simulation for real data:** replace the `make_*()` functions in
   `pointCloud.cpp` with reads from the sensor driver. The threads, channels
   and schemas stay the same.
3. **ROS 2 replay?** If recordings must work with `ros2 bag`, use
   `PointCloud2` / `LaserScan` in CDR instead of the Foxglove schemas.
