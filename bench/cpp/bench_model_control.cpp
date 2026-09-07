// Instrumented model-in-the-loop torque benchmark for C++ libfranka.
//
// Runs an operational-space impedance controller with inertia shaping at 1 kHz through
// `Robot::control(std::function<Torques(...)>)` and records, per control cycle, how long the
// five `franka::Model` calls took, how long the whole controller took, the inter-callback
// interval, `RobotState::time` and `control_command_success_rate`.
//
// Per cycle the controller evaluates, in this order:
//
//   1. M = mass(state), c = coriolis(state), g = gravity(state),
//      J = zeroJacobian(kEndEffector, state), T = pose(kEndEffector, state);
//   2. Lambda = (J M^-1 J^T + 1e-6 I6)^-1   (M^-1 through a Cholesky factorisation),
//      e      = [p_d - p ; orientation error from the quaternion between T and the initial
//                pose],
//      xdot   = J dq,
//      F      = Lambda (Kp e - Kd xdot),
//      Jbar   = M^-1 J^T Lambda,
//      tau    = J^T F + (I7 - J^T Jbar^T)(-Kn dq) + c,
//      tau    = limitRate(kMaxTorqueRate, tau, state.tau_J_d).
//
// with Kp = diag(200 x3, 20 x3), Kd = 2 sqrt(Kp), Kn = 0.5 and p_d the initial end-effector
// position plus a 0.05 m, 0.5 Hz sinusoid along z.
//
// `g` is not part of tau (the robot compensates gravity itself); it is evaluated because the
// point of the variant is to price the five model calls a model-based controller makes, and
// its first element is accumulated into a checksum so neither compiler can elide the call.
//
// The library is asked to do nothing on top: `limit_rate = false` and
// `cutoff_frequency = kMaxCutoffFrequency`, so the client-side `limitRate` above is the only
// limiter and no low-pass filter runs. franka-rs is driven with exactly the same two
// arguments.
//
// Nothing is allocated or printed by *this* code inside the loop: every Eigen type is
// fixed-size (stack), and the sample array is sized and zeroed before the motion starts.
// libfranka's Pinocchio backend does allocate internally -- `RobotModel::computeJacobian` and
// `RobotModel::computeForwardKinematics` construct a fresh `pinocchio::Data` on every call --
// which is one of the things this benchmark measures.
//
// `--hardware` turns on two guards for runs against a real arm. After the timed region of
// every cycle (so the compute statistics stay comparable with a sim run), the commanded
// torque and the end-effector excursion from the starting pose are checked against
// kGuardTauLimit and kGuardEeDeviationLimit. On a violation the controller does *not* kill
// the loop: it replaces the command with a rate-limited step towards zero torque and returns
// it with the motion-finished flag set, so libfranka ends the motion the normal way. The
// reason, the cycle and the offending values go into the JSON.
//
// A ControlException from the control loop is caught and recorded in the JSON as
// `control_exception` rather than swallowing the samples collected so far; the program then
// exits 3 (results written, loop ended badly) instead of 1.
//
// The Rust counterpart is ../rust/src/bin/bench_model_control.rs; both write the same JSON
// schema as the joint-velocity benchmark plus `compute_us` and `model_us`.

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

#include <Eigen/Dense>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

// The version of the libfranka this binary was linked against. bench/cpp/CMakeLists.txt
// derives it from the versioned soname in LIBFRANKA_BUILD_DIR and passes it in; the fallback
// keeps the value this benchmark reported when it was developed, against libfranka 0.20.4.
#ifndef BENCH_LIBFRANKA_VERSION
#define BENCH_LIBFRANKA_VERSION "0.20.4"
#endif

#include "examples_common.h"

namespace {

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Matrix67d = Eigen::Matrix<double, 6, 7>;
using Matrix76d = Eigen::Matrix<double, 7, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

// Impedance gains. Kd is the critically damped companion of Kp.
constexpr double kKpTranslation = 200.0;
constexpr double kKpRotation = 20.0;
constexpr double kNullspaceDamping = 0.5;
// The desired end-effector position tracks a slow sinusoid along z.
constexpr double kSetpointAmplitude = 0.05;  // m
constexpr double kSetpointFrequency = 0.5;   // Hz
// Regularisation added to the operational-space inertia before inverting it.
constexpr double kLambdaRegularisation = 1e-6;
// `--hardware` guards: the largest commanded joint torque and the largest end-effector
// excursion from the starting pose that the controller is allowed to reach before it finishes
// the motion cleanly.
constexpr double kGuardTauLimit = 20.0;          // Nm
constexpr double kGuardEeDeviationLimit = 0.10;  // m

struct Sample {
  int64_t t_enter_ns;   // CLOCK_MONOTONIC at callback entry, before the first model call
  int64_t model_ns;     // duration of the five franka::Model calls
  int64_t compute_ns;   // duration of the whole controller, model calls included
  uint64_t state_time_ms;
  double success_rate;
};

inline int64_t monotonic_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
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
  std::string variant = "model";
  std::string condition = "unspecified";
  std::string out_path;
  std::string cell_first = "unspecified";
  std::string provenance = "harness";
  double duration_s = 30.0;
  int rep = 1;
  int order_in_cell = 0;
  int reflex_events_before = 0;
  bool want_mlock = false;
  bool hardware = false;
  // Overridable only so the guard itself can be exercised against the simulator, where the
  // controller never comes near the real limits; the hardware defaults are the constants.
  double guard_tau_limit = kGuardTauLimit;
  double guard_ee_limit = kGuardEeDeviationLimit;

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
    } else if (a == "--provenance") {
      provenance = next("--provenance");
    } else if (a == "--reflex-events") {
      reflex_events_before = std::stoi(next("--reflex-events"));
    } else if (a == "--mlock") {
      want_mlock = true;
    } else if (a == "--hardware") {
      hardware = true;
    } else if (a == "--guard-tau") {
      guard_tau_limit = std::stod(next("--guard-tau"));
    } else if (a == "--guard-ee") {
      guard_ee_limit = std::stod(next("--guard-ee"));
    } else if (!a.empty() && a[0] == '-') {
      std::cerr << "unknown flag " << a << std::endl;
      return 2;
    } else {
      host = a;
    }
  }
  if (host.empty() || variant != "model") {
    std::cerr << "usage: " << argv[0]
              << " <robot-hostname> [--variant model] [--duration 30] [--mlock]"
                 " [--condition NAME] [--rep N] [--order N] [--cell-first cpp|rust]"
                 " [--hardware] [--guard-tau 20] [--guard-ee 0.10] [--reflex-events N]"
                 " [--provenance NAME] [--out FILE]"
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

    // The torque example's thresholds, not the joint-velocity example's: an impedance
    // controller pushes against the arm's own inertia and would trip the tighter ones.
    robot.setCollisionBehavior(
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
        {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    franka::Model model = robot.loadModel();

    // The equilibrium is the pose the arm is in when the loop starts, taken from the model's
    // own forward kinematics (not O_T_EE) so both clients start from the same number.
    const franka::RobotState initial_state = robot.readOnce();
    const std::array<double, 16> initial_pose_array =
        model.pose(franka::Frame::kEndEffector, initial_state);
    const Eigen::Matrix4d initial_pose = Eigen::Matrix4d::Map(initial_pose_array.data());
    const Eigen::Vector3d position_0(initial_pose.block<3, 1>(0, 3));
    const Eigen::Quaterniond orientation_d(Eigen::Matrix3d(initial_pose.block<3, 3>(0, 0)));

    Vector6d gain_p;
    gain_p << kKpTranslation, kKpTranslation, kKpTranslation, kKpRotation, kKpRotation,
        kKpRotation;
    const Vector6d gain_d = 2.0 * gain_p.cwiseSqrt();

    struct rusage ru_before;
    getrusage(RUSAGE_SELF, &ru_before);
    const int64_t wall_start = monotonic_ns();

    double time = 0.0;
    double gravity_checksum = 0.0;
    double tau_max_abs = 0.0;
    double ee_deviation_max = 0.0;
    // --hardware guard state.
    bool guard_tripped = false;
    std::string guard_reason;
    size_t guard_cycle = 0;
    double guard_tau = 0.0;
    double guard_ee = 0.0;
    std::string control_exception;
    auto control_callback =
        [&](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
          const int64_t t_enter = monotonic_ns();

          // --- step 1: the five model calls -------------------------------------------
          const std::array<double, 49> mass_array = model.mass(state);
          const std::array<double, 7> coriolis_array = model.coriolis(state);
          const std::array<double, 7> gravity_array = model.gravity(state);
          const std::array<double, 42> jacobian_array =
              model.zeroJacobian(franka::Frame::kEndEffector, state);
          const std::array<double, 16> pose_array = model.pose(franka::Frame::kEndEffector, state);
          const int64_t t_model = monotonic_ns();

          // --- step 2: operational-space impedance with inertia shaping ---------------
          const Eigen::Map<const Matrix7d> mass(mass_array.data());
          const Eigen::Map<const Vector7d> coriolis(coriolis_array.data());
          const Eigen::Map<const Matrix67d> jacobian(jacobian_array.data());
          const Eigen::Map<const Vector7d> dq(state.dq.data());
          const Eigen::Map<const Eigen::Matrix4d> pose(pose_array.data());
          const Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
          const Eigen::Vector3d position = pose.block<3, 1>(0, 3);

          const Matrix7d mass_inverse = mass.llt().solve(Matrix7d::Identity());
          const Matrix6d lambda =
              (jacobian * mass_inverse * jacobian.transpose() +
               kLambdaRegularisation * Matrix6d::Identity())
                  .inverse();

          time += period.toSec();
          Eigen::Vector3d position_d = position_0;
          position_d.z() +=
              kSetpointAmplitude * std::sin(2.0 * M_PI * kSetpointFrequency * time);

          Vector6d error;
          error.head(3) = position_d - position;
          Eigen::Quaterniond orientation(rotation);
          if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() = -orientation.coeffs();
          }
          const Eigen::Quaterniond error_quaternion = orientation.inverse() * orientation_d;
          error.tail(3) =
              rotation * Eigen::Vector3d(error_quaternion.x(), error_quaternion.y(),
                                         error_quaternion.z());

          const Vector6d velocity = jacobian * dq;
          const Vector6d wrench =
              lambda * (gain_p.cwiseProduct(error) - gain_d.cwiseProduct(velocity));
          const Matrix76d jacobian_bar = mass_inverse * jacobian.transpose() * lambda;
          const Vector7d tau =
              jacobian.transpose() * wrench +
              (Matrix7d::Identity() - jacobian.transpose() * jacobian_bar.transpose()) *
                  (-kNullspaceDamping * dq) +
              coriolis;

          std::array<double, 7> tau_array{};
          Vector7d::Map(tau_array.data(), 7) = tau;
          tau_array = franka::limitRate(franka::kMaxTorqueRate, tau_array, state.tau_J_d);
          const int64_t t_done = monotonic_ns();

          if (count < capacity) {
            samples[count].t_enter_ns = t_enter;
            samples[count].model_ns = t_model - t_enter;
            samples[count].compute_ns = t_done - t_enter;
            samples[count].state_time_ms = state.time.toMSec();
            samples[count].success_rate = state.control_command_success_rate;
            ++count;
          }
          // Diagnostics, outside the timed region: `gravity` is otherwise unused, and the
          // two maxima are the sanity check that the controller stayed where it started.
          gravity_checksum += gravity_array[0];
          for (double value : tau_array) {
            tau_max_abs = std::max(tau_max_abs, std::abs(value));
          }
          ee_deviation_max = std::max(ee_deviation_max, (position - position_0).norm());

          // --hardware guards, deliberately outside the timed region so the compute
          // statistics stay comparable with a simulator run. On a violation the command is
          // replaced by a rate-limited step towards zero torque and the motion is finished
          // through the normal path -- the loop is never killed.
          if (hardware && !guard_tripped) {
            const double ee_deviation = (position - position_0).norm();
            double tau_peak = 0.0;
            for (double value : tau_array) {
              tau_peak = std::max(tau_peak, std::abs(value));
            }
            if (tau_peak > guard_tau_limit || ee_deviation > guard_ee_limit) {
              guard_tripped = true;
              guard_reason = tau_peak > guard_tau_limit ? "tau" : "ee_deviation";
              guard_cycle = count;
              guard_tau = tau_peak;
              guard_ee = ee_deviation;
              const std::array<double, 7> zero{};
              tau_array = franka::limitRate(franka::kMaxTorqueRate, zero, state.tau_J_d);
              return franka::MotionFinished(franka::Torques(tau_array));
            }
          }

          franka::Torques torques(tau_array);
          if (time >= duration_s) {
            return franka::MotionFinished(torques);
          }
          return torques;
        };

    // A ControlException is recorded rather than thrown away: the samples collected up to
    // that point are still worth having, especially on hardware.
    try {
      robot.control(control_callback, false, franka::kMaxCutoffFrequency);
    } catch (const franka::Exception& e) {
      control_exception = e.what();
      std::cerr << "control loop ended with an exception: " << control_exception << std::endl;
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
    std::vector<double> computes;
    std::vector<double> models;
    computes.reserve(count);
    models.reserve(count);
    for (size_t i = 0; i < count; ++i) {
      computes.push_back(static_cast<double>(samples[i].compute_ns) * 1e-3);
      models.push_back(static_cast<double>(samples[i].model_ns) * 1e-3);
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
    const Stats compute_stats = summarize(computes);
    const Stats model_stats = summarize(models);

    std::ostringstream json;
    json.setf(std::ios::fixed);
    json << "{\n";
    json << "  \"lang\": \"cpp\",\n";
    json << "  \"library\": \"libfranka " BENCH_LIBFRANKA_VERSION "\",\n";
    json << "  \"variant\": \"" << json_escape(variant) << "\",\n";
    json << "  \"condition\": \"" << json_escape(condition) << "\",\n";
    json << "  \"rep\": " << rep << ",\n";
    json << "  \"order_in_cell\": " << order_in_cell << ",\n";
    json << "  \"cell_first_client\": \"" << json_escape(cell_first) << "\",\n";
    json << "  \"host\": \"" << json_escape(host) << "\",\n";
    json << "  \"provenance\": \"" << json_escape(provenance) << "\",\n";
    json << "  \"hardware\": " << (hardware ? "true" : "false") << ",\n";
    json << "  \"reflex_events_before_run\": " << reflex_events_before << ",\n";
    json << "  \"control_exception\": "
         << (control_exception.empty() ? std::string("null")
                                       : "\"" + json_escape(control_exception) + "\"")
         << ",\n";
    json << "  \"duration_s\": " << duration_s << ",\n";
    json << "  \"limit_rate\": false,\n";
    json << "  \"cycles\": " << count << ",\n";
    json << "  \"wall_s\": " << wall_s << ",\n";
    json << "  \"sched\": " << sched_json() << ",\n";
    json << "  \"sched_at_start\": " << sched_before << ",\n";
    json << "  \"mlockall\": {\"requested\": " << (mlock.requested ? "true" : "false")
         << ", \"ok\": " << (mlock.ok ? "true" : "false") << ", \"error\": "
         << (mlock.error.empty() ? std::string("null") : "\"" + json_escape(mlock.error) + "\"")
         << ", \"rlimit_memlock\": \"" << json_escape(mlock.rlimit) << "\"},\n";
    json << "  \"interval_us\": " << stats_json(interval_stats) << ",\n";
    json << "  \"latency_us\": null,\n";
    json << "  \"compute_us\": " << stats_json(compute_stats) << ",\n";
    json << "  \"model_us\": " << stats_json(model_stats) << ",\n";
    json << "  \"guard\": {\"enabled\": " << (hardware ? "true" : "false")
         << ", \"tau_limit_nm\": " << guard_tau_limit
         << ", \"ee_deviation_limit_m\": " << guard_ee_limit
         << ", \"tripped\": " << (guard_tripped ? "true" : "false") << ", \"reason\": "
         << (guard_reason.empty() ? std::string("null") : "\"" + guard_reason + "\"")
         << ", \"cycle\": " << guard_cycle << ", \"tau_at_trip\": " << guard_tau
         << ", \"ee_deviation_at_trip\": " << guard_ee << "},\n";
    json << "  \"controller\": {\"tau_max_abs\": " << tau_max_abs
         << ", \"ee_deviation_max_m\": " << ee_deviation_max
         << ", \"gravity_checksum\": " << gravity_checksum << "},\n";
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
    if (!control_exception.empty()) {
      // Results were written; the loop still ended badly. 3, not 1, so the harness can tell
      // this apart from a failure that produced no JSON at all.
      return 3;
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
