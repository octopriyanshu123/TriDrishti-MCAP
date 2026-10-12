You can sync them by timestamp. You don't need to record them into the same file.

**Why it works already:** both writers stamp every message with `utc_now_ns()`, which is the system wall clock (`std::chrono::system_clock`). That stamp goes into the MCAP `logTime` and into the message's own `timestamp` field. Two files recorded on the same machine therefore share one timeline, and you match messages by time.

## Option 1: Open both files in Foxglove

Open or drag in both `.mcap` files at once. Foxglove merges them into one timeline ordered by log time. The Image panel and the 3D panel then play back together, with nothing to convert.

## Option 2: Merge them into one file

The `mcap` CLI has a merge command that interleaves the messages by log time:

```bash
mcap merge camera_h264.mcap pointcloud.mcap -o merged.mcap
```

The CLI isn't installed on this machine. You can download a release binary from github.com/foxglove/mcap, or install it with `go install github.com/foxglove/mcap/go/cli/mcap@latest`.

## Option 3: Pair the frames in code

Use this if you need matched pairs, for example to project points onto the image. Read both files with `mcap::McapReader`. For each image, take the point cloud whose timestamp is nearest, and reject the pair if the gap is too big:

```cpp
// clouds: timestamps (ns) of every point cloud, sorted ascending
auto it = std::lower_bound(clouds.begin(), clouds.end(), image_ns);
// compare *it and *(it - 1), keep the closer one
// reject if |cloud_ns - image_ns| > 50 ms (half the 10 Hz cloud period)
```

Even perfectly synced data can be up to half a period apart. The cloud runs at 10 Hz and the camera at 30 fps, so the gap can be up to 50 ms.

## What makes the sync accurate

1. **Use the same clock everywhere.** Keep `system_clock` in both programs. Never stamp messages with `steady_clock`, because its values mean nothing across processes.
2. **Stamp when the data is captured, not when it is written.**
   - Point cloud: the simulated cloud is stamped when it is generated, which is fine. A real LiDAR should use the sensor's own timestamp from its packets.
   - Video: frames are stamped when they leave the camera driver, a few milliseconds after exposure. For tighter sync, use the kernel timestamp V4L2 gives each frame (`buf.timestamp`), converted from the monotonic clock to wall time. Ask if you want this added to [camera.hpp](src/camera.hpp).
3. **Use a common clock across machines.** If the camera and LiDAR are on different computers, synchronise the clocks with chrony/NTP (about 1 ms) or PTP (`linuxptp`, microseconds). Otherwise the timestamps drift apart.
4. **Use hardware trigger or sync for exact alignment.** Many LiDARs accept a PPS/PTP input, and industrial cameras accept a trigger input. Then the capture itself is aligned, not just the labels.

For this project, recording on one PC, option 1 or 2 is enough. Option 3 is only needed when your own code consumes matched image and cloud pairs.