// Instrumented joint-velocity benchmark for C++ libfranka.
//
// Runs the `generate_joint_velocity_motion` profile (same amplitude and period as the
// libfranka example) for a configurable duration and records, per control cycle, a
// CLOCK_MONOTONIC timestamp, `RobotState::time`, and `control_command_success_rate`.
//
// Two variants:
//   --variant control  robot.control(motion_generator_callback, ControllerMode::kJointImpedance,
//                                    limit_rate=true, kDefaultCutoffFrequency)
//   --variant active   robot.startJointVelocityControl(...) then readOnce/writeOnce.
//
// Nothing is allocated or printed inside the loop: the sample array is sized and touched up
// front, and all statistics are computed after the motion has finished.
//
// The Rust counterpart is ../rust/src/main.rs; both write the same JSON schema.

#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <franka/active_control.h>
#include <franka/active_motion_generator.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"

namespace {

constexpr double kTimeMax = 1.0;   // s, the libfranka example's `time_max`
constexpr double kOmegaMax = 1.0;  // rad/s, the libfranka example's `omega_max`

struct Sample {
  int64_t t_enter_ns;   // CLOCK_MONOTONIC at callback entry / readOnce() return
  int64_t t_sent_ns;    // CLOCK_MONOTONIC after writeOnce() returned (active variant only)
  uint64_t state_time_ms;
  double success_rate;
};

inline int64_t monotonic_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

// The `generate_joint_velocity_motion` profile, verbatim from the libfranka example.
inline double omega_at(double time) {
  double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, kTimeMax)) / kTimeMax));
  return cycle * kOmegaMax / 2.0 * (1.0 - std::cos(2.0 * M_PI / kTimeMax * time));
}

struct Stats {
  double p50 = 0, p99 = 0, p999 = 0, min = 0, max = 0, mean = 0;
  size_t n = 0;
  size_t max_at = 0;  // index of the worst sample, before sorting
};

// Nearest-rank percentile over an already-sorted vector.
double percentile(const std::vector<double>& sorted, double q) {
  if (sorted.empty()) {
    return 0.0;
  }
  size_t rank = static_cast<size_t>(std::ceil(q * static_cast<double>(sorted.size())));
  if (rank == 0) {
    rank = 1;
  }
  if (rank > sorted.size()) {
    rank = sorted.size();
  }
  return sorted[rank - 1];
}

Stats summarize(std::vector<double> values) {
  Stats s;
  s.n = values.size();
  if (values.empty()) {
    return s;
  }
  double sum = 0.0;
  for (size_t i = 0; i < values.size(); ++i) {
    sum += values[i];
    if (values[i] > values[s.max_at]) {
      s.max_at = i;
    }
  }
  s.mean = sum / static_cast<double>(values.size());
  std::sort(values.begin(), values.end());
  s.min = values.front();
  s.max = values.back();
  s.p50 = percentile(values, 0.50);
  s.p99 = percentile(values, 0.99);
  s.p999 = percentile(values, 0.999);
  return s;
}

std::string stats_json(const Stats& s) {
  char buf[512];
  std::snprintf(buf, sizeof(buf),
                "{\"n\": %zu, \"min\": %.3f, \"p50\": %.3f, \"p99\": %.3f, \"p999\": %.3f, "
                "\"max\": %.3f, \"mean\": %.3f, \"max_at_cycle\": %zu}",
                s.n, s.min, s.p50, s.p99, s.p999, s.max, s.mean, s.max_at);
  return std::string(buf);
}

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

struct MlockResult {
  bool requested = false;
  bool ok = false;
  std::string error;
  std::string rlimit;
};

MlockResult apply_mlockall(bool requested) {
  MlockResult r;
  r.requested = requested;
  struct rlimit rl;
  if (getrlimit(RLIMIT_MEMLOCK, &rl) == 0) {
    r.rlimit = (rl.rlim_cur == RLIM_INFINITY) ? "unlimited" : std::to_string(rl.rlim_cur);
  } else {
    r.rlimit = "unknown";
  }
  if (!requested) {
    return r;
  }
  if (mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
    r.ok = true;
  } else {
    r.error = std::strerror(errno);
    std::cerr << "warning: mlockall(MCL_CURRENT|MCL_FUTURE) failed: " << r.error
              << " (RLIMIT_MEMLOCK=" << r.rlimit << ")" << std::endl;
  }
  return r;
}

std::string sched_json() {
  int policy = sched_getscheduler(0);
  struct sched_param param;
  int prio = 0;
  if (sched_getparam(0, &param) == 0) {
    prio = param.sched_priority;
  }
  const char* name = "OTHER";
  switch (policy) {
    case SCHED_FIFO:
      name = "FIFO";
      break;
    case SCHED_RR:
      name = "RR";
      break;
    case SCHED_OTHER:
      name = "OTHER";
      break;
    case SCHED_BATCH:
      name = "BATCH";
      break;
    case SCHED_IDLE:
      name = "IDLE";
      break;
    default:
      name = "UNKNOWN";
      break;
  }
  char buf[128];
  std::snprintf(buf, sizeof(buf), "{\"policy\": \"%s\", \"priority\": %d}", name, prio);
  return std::string(buf);
}

}  // namespace

int main(int argc, char** argv) {
  std::string host;
  std::string variant = "control";
  std::string condition = "unspecified";
  std::string out_path;
  std::string cell_first = "unspecified";
  double duration_s = 30.0;
  int rep = 1;
  int order_in_cell = 0;
  bool want_mlock = false;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](const char* what) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << what << std::endl;
        std::exit(2);
      }
      return std::string(argv[++i]);
    };
    if (a == "--variant") {
      variant = next("--variant");
    } else if (a == "--duration") {
      duration_s = std::stod(next("--duration"));
    } else if (a == "--out") {
      out_path = next("--out");
    } else if (a == "--condition") {
      condition = next("--condition");
    } else if (a == "--rep") {
      rep = std::stoi(next("--rep"));
    } else if (a == "--order") {
      order_in_cell = std::stoi(next("--order"));
    } else if (a == "--cell-first") {
      cell_first = next("--cell-first");
    } else if (a == "--mlock") {
      want_mlock = true;
    } else if (!a.empty() && a[0] == '-') {
      std::cerr << "unknown flag " << a << std::endl;
      return 2;
    } else {
      host = a;
    }
  }
  if (host.empty() || (variant != "control" && variant != "active")) {
    std::cerr << "usage: " << argv[0]
              << " <robot-hostname> [--variant control|active] [--duration 30]"
                 " [--mlock] [--condition NAME] [--rep N] [--order N]"
                 " [--cell-first cpp|rust] [--out FILE]"
              << std::endl;
    return 2;
  }

  MlockResult mlock = apply_mlockall(want_mlock);
  // Both libraries raise the calling thread to the highest SCHED_FIFO priority in the Robot
  // constructor even under RealtimeConfig::kIgnore, so record the policy the process was
  // *launched* with as well as the one it ends up running the loop with.
  const std::string sched_before = sched_json();

  // Preallocate and touch the sample array: 1 kHz plus generous headroom, never resized in
  // the loop.
  const size_t capacity = static_cast<size_t>(duration_s * 1000.0 * 1.5) + 4096;
  std::vector<Sample> samples(capacity);
  std::memset(samples.data(), 0, samples.size() * sizeof(Sample));
  size_t count = 0;

  try {
    // RealtimeConfig::kIgnore: this box is not PREEMPT_RT (no /sys/kernel/realtime), so
    // realtime priority is applied externally with `chrt -f 80`.
    franka::Robot robot(host, franka::RealtimeConfig::kIgnore);
    setDefaultBehavior(robot);

    // Home the arm before measuring, exactly like the libfranka examples.
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    robot.control(motion_generator);

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    struct rusage ru_before;
    getrusage(RUSAGE_SELF, &ru_before);
    const int64_t wall_start = monotonic_ns();

    if (variant == "control") {
      double time = 0.0;
      robot.control(
          [&](const franka::RobotState& state, franka::Duration period) -> franka::JointVelocities {
            const int64_t now = monotonic_ns();
            if (count < capacity) {
              samples[count].t_enter_ns = now;
              samples[count].t_sent_ns = 0;
              samples[count].state_time_ms = state.time.toMSec();
              samples[count].success_rate = state.control_command_success_rate;
              ++count;
            }
            time += period.toSec();
            const double omega = omega_at(time);
            franka::JointVelocities velocities = {{0.0, 0.0, 0.0, omega, omega, omega, omega}};
            if (time >= duration_s) {
              return franka::MotionFinished(velocities);
            }
            return velocities;
          },
          franka::ControllerMode::kJointImpedance, true, franka::kDefaultCutoffFrequency);
    } else {
      double time = 0.0;
      bool finished = false;
      auto active_control = robot.startJointVelocityControl(
          research_interface::robot::Move::ControllerMode::kJointImpedance);
      while (!finished) {
        auto read_once_return = active_control->readOnce();
        const int64_t t_read = monotonic_ns();
        const franka::RobotState& state = read_once_return.first;
        const franka::Duration period = read_once_return.second;
        time += period.toSec();
        const double omega = omega_at(time);
        franka::JointVelocities velocities = {{0.0, 0.0, 0.0, omega, omega, omega, omega}};
        if (time >= duration_s) {
          velocities = franka::MotionFinished(velocities);
          finished = true;
        }
        const uint64_t state_time_ms = state.time.toMSec();
        const double success_rate = state.control_command_success_rate;
        active_control->writeOnce(velocities);
        const int64_t t_sent = monotonic_ns();
        if (count < capacity) {
          samples[count].t_enter_ns = t_read;
          samples[count].t_sent_ns = t_sent;
          samples[count].state_time_ms = state_time_ms;
          samples[count].success_rate = success_rate;
          ++count;
        }
      }
    }

    const int64_t wall_end = monotonic_ns();
    struct rusage ru_after;
    getrusage(RUSAGE_SELF, &ru_after);

    // --- statistics, all computed after the loop ---
    const double wall_s = static_cast<double>(wall_end - wall_start) * 1e-9;

    std::vector<double> intervals;
    intervals.reserve(count);
    for (size_t i = 1; i < count; ++i) {
      intervals.push_back(static_cast<double>(samples[i].t_enter_ns - samples[i - 1].t_enter_ns) *
                          1e-3);
    }
    std::vector<double> latencies;
    if (variant == "active") {
      latencies.reserve(count);
      for (size_t i = 0; i < count; ++i) {
        latencies.push_back(static_cast<double>(samples[i].t_sent_ns - samples[i].t_enter_ns) *
                            1e-3);
      }
    }

    // Saturating: a backwards `state.time` step counts as dt = 0 (no loss) and is reported
    // separately, so C++ and Rust score such an event identically.
    uint64_t lost_cycles = 0, lost_states = 0, max_consecutive = 0, consecutive = 0;
    uint64_t backwards_steps = 0;
    for (size_t i = 1; i < count; ++i) {
      const uint64_t prev = samples[i - 1].state_time_ms;
      const uint64_t cur = samples[i].state_time_ms;
      if (cur < prev) {
        ++backwards_steps;
        consecutive = 0;
        continue;
      }
      const uint64_t dt = cur - prev;
      if (dt > 1) {
        ++lost_cycles;
        lost_states += dt - 1;
        ++consecutive;
        max_consecutive = std::max(max_consecutive, consecutive);
      } else {
        consecutive = 0;
      }
    }

    // Skip cycle 0: no command has been acknowledged yet, so its success rate is always 0.
    double sr_min = 1.0, sr_max = 0.0, sr_sum = 0.0;
    size_t sr_n = 0;
    for (size_t i = 1; i < count; ++i) {
      sr_min = std::min(sr_min, samples[i].success_rate);
      sr_max = std::max(sr_max, samples[i].success_rate);
      sr_sum += samples[i].success_rate;
      ++sr_n;
    }
    const double sr_avg = sr_n ? sr_sum / static_cast<double>(sr_n) : 0.0;
    const double sr_final = count ? samples[count - 1].success_rate : 0.0;

    auto tv_delta = [](const struct timeval& a, const struct timeval& b) {
      return (static_cast<double>(a.tv_sec - b.tv_sec) +
              static_cast<double>(a.tv_usec - b.tv_usec) * 1e-6);
    };
    const double user_s = tv_delta(ru_after.ru_utime, ru_before.ru_utime);
    const double sys_s = tv_delta(ru_after.ru_stime, ru_before.ru_stime);

    const Stats interval_stats = summarize(intervals);
    const Stats latency_stats = summarize(latencies);

    std::ostringstream json;
    json.setf(std::ios::fixed);
    json << "{\n";
    json << "  \"lang\": \"cpp\",\n";
    json << "  \"library\": \"libfranka 0.20.4\",\n";
    json << "  \"variant\": \"" << json_escape(variant) << "\",\n";
    json << "  \"condition\": \"" << json_escape(condition) << "\",\n";
    json << "  \"rep\": " << rep << ",\n";
    json << "  \"order_in_cell\": " << order_in_cell << ",\n";
    json << "  \"cell_first_client\": \"" << json_escape(cell_first) << "\",\n";
    json << "  \"host\": \"" << json_escape(host) << "\",\n";
    json << "  \"duration_s\": " << duration_s << ",\n";
    json << "  \"limit_rate\": " << (variant == "control" ? "true" : "null") << ",\n";
    json << "  \"cycles\": " << count << ",\n";
    json << "  \"wall_s\": " << wall_s << ",\n";
    json << "  \"sched\": " << sched_json() << ",\n";
    json << "  \"sched_at_start\": " << sched_before << ",\n";
    json << "  \"mlockall\": {\"requested\": " << (mlock.requested ? "true" : "false")
         << ", \"ok\": " << (mlock.ok ? "true" : "false") << ", \"error\": "
         << (mlock.error.empty() ? std::string("null")
                                 : "\"" + json_escape(mlock.error) + "\"")
         << ", \"rlimit_memlock\": \"" << json_escape(mlock.rlimit) << "\"},\n";
    json << "  \"interval_us\": " << stats_json(interval_stats) << ",\n";
    json << "  \"latency_us\": "
         << (variant == "active" ? stats_json(latency_stats) : std::string("null")) << ",\n";
    json << "  \"lost\": {\"cycles\": " << lost_cycles << ", \"states\": " << lost_states
         << ", \"max_consecutive\": " << max_consecutive
         << ", \"backwards_time_steps\": " << backwards_steps << "},\n";
    json << "  \"success_rate\": {\"min\": " << sr_min << ", \"avg\": " << sr_avg
         << ", \"max\": " << sr_max << ", \"final\": " << sr_final << ", \"n\": " << sr_n
         << "},\n";
    json << "  \"cpu\": {\"user_s\": " << user_s << ", \"sys_s\": " << sys_s
         << ", \"total_s\": " << (user_s + sys_s) << ", \"percent\": "
         << (wall_s > 0 ? (user_s + sys_s) / wall_s * 100.0 : 0.0)
         << ", \"minor_faults\": " << (ru_after.ru_minflt - ru_before.ru_minflt)
         << ", \"major_faults\": " << (ru_after.ru_majflt - ru_before.ru_majflt)
         << ", \"vol_ctx_switches\": " << (ru_after.ru_nvcsw - ru_before.ru_nvcsw)
         << ", \"invol_ctx_switches\": " << (ru_after.ru_nivcsw - ru_before.ru_nivcsw) << "}\n";
    json << "}\n";

    std::cout << json.str();
    if (!out_path.empty()) {
      std::ofstream f(out_path);
      f << json.str();
    }
  } catch (const franka::Exception& e) {
    std::cerr << "franka exception: " << e.what() << std::endl;
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "exception: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
