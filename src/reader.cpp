// Reads an MCAP file back: summary, per-channel counts, and the first few messages.
#define MCAP_IMPLEMENTATION
#include <mcap/reader.hpp>

#include <cstdlib>
#include <iostream>
#include <map>
#include <string>

#include "common.hpp"

// Set when the reader reports a problem while iterating messages.
bool g_problem = false;

void on_read_problem(const mcap::Status &status)
{
  std::cerr << "read problem: " << status.message << '\n';
  g_problem = true;
}

// Decodes one message with the struct matching its channel's schema and prints the fields.
void print_parsed(const mcap::MessageView &view)
{
  const std::string json(reinterpret_cast<const char *>(view.message.data), view.message.dataSize);
  const std::string schema_name = view.schema ? view.schema->name : "";

  std::cout << "  " << view.channel->topic << " [" << view.message.sequence << "] t=" << view.message.logTime << ' ';

  if (schema_name == kSchemaName)
  {
    Motor m;
    if (parse(json, m))
    {
      std::cout << "target_rpm=" << m.target_rpm << " actual_rpm=" << m.actual_rpm << " current_a=" << m.current_a
                << '\n';
      return;
    }
  }
  else if (schema_name == kSchemaName2)
  {
    MotorSpeed m;
    if (parse(json, m))
    {
      std::cout << "target_speed=" << m.target_speed << " actual_speed=" << m.actual_speed << '\n';
      return;
    }
  }

  // Unknown schema or a payload that does not match it: show the raw bytes.
  std::cout << "unparsed (" << schema_name << "): " << json << '\n';
  g_problem = true;
}

int main(int argc, char **argv)
{
  const std::string path = argc > 1 ? argv[1] : "motor.mcap";
  const int show = argc > 2 ? std::atoi(argv[2]) : 5;

  mcap::McapReader reader;
  const auto opened = reader.open(path);
  if (!opened.ok())
  {
    std::cerr << "failed to open " << path << ": " << opened.message << '\n';
    return 1;
  }

  const auto summary = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
  if (!summary.ok())
  {
    std::cerr << "failed to read summary: " << summary.message << '\n';
    return 1;
  }

  std::cout << "file: " << path << '\n';
  for (const auto &[id, channel] : reader.channels())
  {
    std::cout << "channel " << id << ": topic=" << channel->topic << " encoding=" << channel->messageEncoding << '\n';
  }

  std::map<std::string, std::uint64_t> counts;

  mcap::LinearMessageView x  =  reader.readMessages(on_read_problem);

  for (const mcap::MessageView &view : x)
  {

    ++counts[view.channel->topic];

    print_parsed(view);
  }

  for (const auto &[topic, count] : counts)
  {
    std::cout << topic << ": " << count << " messages\n";
  }
  reader.close();
  return g_problem ? 1 : 0;
}

// How to run
// ./mcap_example_reader  file name mu.mcap
