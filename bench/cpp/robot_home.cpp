// Return-to-ready move, as its own step so the hardware harness can put one between runs and
// a human can run one by hand.
//
// This is libfranka's own `MotionGenerator` from `examples/examples_common.cpp` (linked in as
// `libexamples_common.a`, exactly as the benchmark clients link it) driving the arm to the
// ready pose the examples use. Nothing else happens: no torques, no model, no measurement.
//
// The Rust equivalent is `bench/rust/src/bin/robot_probe.rs --home`, which uses the ported
// `MotionGenerator` from `crates/franka-rs/examples/common/`.
//
// Usage: robot_home <robot-hostname> [--speed 0.2] [--out FILE]

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

namespace {

std::string json_escape(const std::string& in) {
  std::string out;
  for (char c : in) {
    if (c == '"' || c == '\\') {
      out.push_back('\\');
    }
    out.push_back(c);
  }
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  std::string host;
  std::string out_path;
  double speed = 0.2;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](const char* what) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << what << std::endl;
        std::exit(2);
      }
      return std::string(argv[++i]);
    };
    if (a == "--speed") {
      speed = std::stod(next("--speed"));
    } else if (a == "--out") {
      out_path = next("--out");
    } else if (!a.empty() && a[0] == '-') {
      std::cerr << "unknown flag " << a << std::endl;
      return 2;
    } else {
      host = a;
    }
  }
  if (host.empty()) {
    std::cerr << "usage: " << argv[0] << " <robot-hostname> [--speed 0.2] [--out FILE]"
              << std::endl;
    return 2;
  }

  std::string error;
  bool homed = false;
  try {
    franka::Robot robot(host, franka::RealtimeConfig::kIgnore);
    setDefaultBehavior(robot);
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(speed, q_goal);
    robot.control(motion_generator);
    homed = true;
  } catch (const franka::Exception& e) {
    error = e.what();
    std::cerr << "franka exception: " << error << std::endl;
  } catch (const std::exception& e) {
    error = e.what();
    std::cerr << "exception: " << error << std::endl;
  }

  std::ostringstream json;
  json << "{\n";
  json << "  \"host\": \"" << json_escape(host) << "\",\n";
  json << "  \"speed\": " << speed << ",\n";
  json << "  \"homed\": " << (homed ? "true" : "false") << ",\n";
  json << "  \"error\": "
       << (error.empty() ? std::string("null") : "\"" + json_escape(error) + "\"") << "\n";
  json << "}\n";

  std::cout << json.str();
  if (!out_path.empty()) {
    std::ofstream f(out_path);
    f << json.str();
  }
  return homed ? 0 : 1;
}
