#define MCAP_IMPLEMENTATION
#include <mcap/writer.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "common.hpp"

namespace
{

  mcap::Timestamp utc_now_ns()
  {
    return static_cast<mcap::Timestamp>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
  }

  // State shared by all logging threads. McapWriter is not thread-safe, so every
  // write goes through writer_mutex.
  struct SharedLog
  {
    mcap::McapWriter writer;
    std::mutex writer_mutex;
    std::atomic<bool> failed{false};
    int samples = 0;
    int rate_hz = 0;
  };

  // Which schema a channel uses, and therefore which JSON fields it writes.
  enum class Payload
  {
    Motor,      // kSchemaText:  target_rpm, actual_rpm, current_a
    MotorSpeed, // kSchemaText2: target_speed, actual_speed
  };

  // One motor = one channel = one thread.
  struct MotorTask
  {
    mcap::ChannelId channel_id = 0;
    std::string topic;
    Payload payload = Payload::Motor;
    double target = 0.0; // rpm for Motor, speed for MotorSpeed
  };

  void log_motor(SharedLog &log, const MotorTask &task)
  {
    const auto period = std::chrono::nanoseconds(1000000000 / log.rate_hz);
    auto next = std::chrono::steady_clock::now();
    char json[160];

    for (int i = 0; i < log.samples && !log.failed; ++i)
    {
      const double t = static_cast<double>(i) / log.rate_hz;
      int size = 0;
      if (task.payload == Payload::Motor)
      {
        Motor m;
        m.target_rpm = task.target;
        m.actual_rpm = task.target - 200.0 * std::exp(-t) + 5.0 * std::sin(2.0 * M_PI * 2.0 * t);
        m.current_a = 3.0 + 0.5 * std::sin(2.0 * M_PI * 0.5 * t);
        size = to_json(m, json, sizeof(json));
      }
      else
      {
        MotorSpeed m;
        m.target_speed = task.target;
        m.actual_speed = task.target * (1.0 - std::exp(-t)) + 0.02 * std::sin(2.0 * M_PI * 2.0 * t);
        size = to_json(m, json, sizeof(json));
      }

      const auto now = utc_now_ns();
      mcap::Message msg;
      msg.channelId = task.channel_id;
      msg.sequence = static_cast<uint32_t>(i);
      msg.logTime = now;
      msg.publishTime = now;
      msg.data = reinterpret_cast<const std::byte *>(json);
      msg.dataSize = static_cast<uint64_t>(size);

      mcap::Status written;
      {
        std::lock_guard<std::mutex> lock(log.writer_mutex);
        written = log.writer.write(msg);
      }
      if (!written.ok())
      {
        std::fprintf(stderr, "%s: write failed at sample %d: %s\n", task.topic.c_str(), i, written.message.c_str());
        log.failed = true;
        return;
      }

      next += period;
      std::this_thread::sleep_until(next);
    }
  }

} // namespace

int main(int argc, char **argv)
{
  const std::string path = argc > 1 ? argv[1] : "motor.mcap";
  const int samples = argc > 2 ? std::atoi(argv[2]) : 500;
  const int rate_hz = argc > 3 ? std::atoi(argv[3]) : 100;
  if (samples <= 0 || rate_hz <= 0)
  {
    std::fprintf(stderr, "usage: %s [output.mcap] [samples>0] [rate_hz>0]\n", argv[0]);
    return 1;
  }

  SharedLog log;
  log.samples = samples;
  log.rate_hz = rate_hz;

  // Set the option of logging
  mcap::McapWriterOptions options("");
  options.compression = mcap::Compression::None;

  // make the write with path and options
  const auto opened = log.writer.open(path, options);
  if (!opened.ok())
  {
    std::fprintf(stderr, "failed to open %s: %s\n", path.c_str(), opened.message.c_str());
    return 1;
  }

  // set the schemas; addSchema() assigns schema.id, so add before using the id
  mcap::Schema schema(kSchemaName, "jsonschema", kSchemaText);
  mcap::Schema schema2(kSchemaName2, "jsonschema", kSchemaText2);
  log.writer.addSchema(schema);
  log.writer.addSchema(schema2);

  // add channels before starting threads; channel 1 uses schema, 2 and 3 use schema2
  mcap::Channel channel1("/demo/motor1", "json", schema.id);
  log.writer.addChannel(channel1);
  mcap::Channel channel2("/demo/motor2", "json", schema2.id);
  log.writer.addChannel(channel2);
  mcap::Channel channel3("/demo/motor3", "json", schema2.id);
  log.writer.addChannel(channel3);

  // one thread per channel
  std::vector<MotorTask> tasks = {
      {channel1.id, channel1.topic, Payload::Motor, 1500.0},
      {channel2.id, channel2.topic, Payload::MotorSpeed, 1.0},
      {channel3.id, channel3.topic, Payload::MotorSpeed, 2.0},
  };

  std::vector<std::thread> threads;
  for (const auto &task : tasks)
  {
    threads.emplace_back(log_motor, std::ref(log), std::cref(task));
  }
  for (auto &thread : threads)
  {
    thread.join();
  }

  log.writer.close();
  if (log.failed)
  {
    return 1;
  }
  std::printf("wrote %d samples on each of %zu channels to %s\n", samples, tasks.size(), path.c_str());
  return 0;
}

//                                  File name  Sample  hz
// How to run ./mcap_example_writer mu.mcap    1000    500
