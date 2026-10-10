// Message definitions shared by the MCAP writer and reader: schema names,
// JSON schemas, typed structs, and the JSON encode/parse for each struct.
#pragma once

#include <cstddef>
#include <cstdio>
#include <string>

inline constexpr const char *kSchemaName = "i2w_example.Motor";
inline constexpr const char *kSchemaName2 = "i2w_example.MotorSpeed";

inline constexpr const char *kSchemaText = R"({
"title": "Motor",
"type": "object",
"properties": {
  "target_rpm": { "type": "number" },
  "actual_rpm": { "type": "number" },
  "current_a": { "type": "number" }
}
})";

inline constexpr const char *kSchemaText2 = R"({
"title": "MotorSpeed",
"type": "object",
"properties": {
  "target_speed": { "type": "number" },
  "actual_speed": { "type": "number" }
}
})";

// Payload of channels using kSchemaText.
struct Motor
{
  double target_rpm = 0.0;
  double actual_rpm = 0.0;
  double current_a = 0.0;
};

// Payload of channels using kSchemaText2.
struct MotorSpeed
{
  double target_speed = 0.0;
  double actual_speed = 0.0;
};

// The format strings are the single source of truth for the wire layout:
// to_json() writes exactly this text and parse() reads exactly this text.
// This is not a general JSON parser; it only accepts what to_json() produces.
inline constexpr const char *kMotorFormat = R"({"target_rpm":%lf,"actual_rpm":%lf,"current_a":%lf})";
inline constexpr const char *kMotorSpeedFormat = R"({"target_speed":%lf,"actual_speed":%lf})";

// Writes the JSON into out and returns its length (no allocation).
inline int to_json(const Motor &m, char *out, std::size_t size)
{
  return std::snprintf(out, size, kMotorFormat, m.target_rpm, m.actual_rpm, m.current_a);
}

inline int to_json(const MotorSpeed &m, char *out, std::size_t size)
{
  return std::snprintf(out, size, kMotorSpeedFormat, m.target_speed, m.actual_speed);
}

// MCAP payloads are not null-terminated, so callers pass a std::string copy.
inline bool parse(const std::string &json, Motor &out)
{
  return std::sscanf(json.c_str(), kMotorFormat, &out.target_rpm, &out.actual_rpm, &out.current_a) == 3;
}

inline bool parse(const std::string &json, MotorSpeed &out)
{
  return std::sscanf(json.c_str(), kMotorSpeedFormat, &out.target_speed, &out.actual_speed) == 2;
}

