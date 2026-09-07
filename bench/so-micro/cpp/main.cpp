// Per-call microbenchmark of the FCI v5 (Franka Emika Robot, FER) libfcimodels model path, C++ side.
//
// Prices the five franka::Model calls a model-based controller makes -- gravity,
// coriolis, mass, pose(kEndEffector), zeroJacobian(kEndEffector) -- individually and as
// the five-call sequence, over a captured libfcimodels_x64.so. The Rust counterpart is
// ../rust/src/main.rs, which drives the same shared object through the crate's
// SoModelBackend.
//
// libfranka 0.9.2's franka::Model only has a constructor taking a franka::Network, i.e.
// it can only be built by downloading the library from a robot. Everything downstream of
// that download is reproduced here verbatim from the 0.9.2 sources so the comparison is
// against the real call path and not a straw man:
//
//   * LibraryLoader is src/library_loader.cpp, Poco::SharedLibrary and all;
//   * ModelLibrary is src/model_library.h / .cpp -- the entry points are held in
//     std::function members and resolved once in the constructor's initialiser list;
//   * the Model:: member bodies are copied from src/model.cpp, including the
//     uninitialised `std::array<double, N> output;` each one declares.
//
// No robot, no simulator, no network: this loads a file and calls functions out of it.
//
// Usage: so_micro_cpp <path-to-libfcimodels_x64.so> [iterations]

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <sched.h>
#include <time.h>

#include <Poco/SharedLibrary.h>

// src/libfcimodels.h (libfranka 0.9.2), the entry points this benchmark uses.
extern "C" {
void M_NE(const double q[7],
          const double I_load[9],
          double m_load,
          const double F_x_Cload[3],
          double M_NE[49]);
void O_J_J9(const double q[7], const double F_T_EE[16], double b_O_J_J9[42]);
void O_T_J9(const double q[7], const double F_T_EE[16], double b_O_T_J9[16]);
void c_NE(const double q[7],
          const double dq[7],
          const double I_load[9],
          double m_load,
          const double F_x_Cload[3],
          double c_NE[7]);
void g_NE(const double q[7],
          const double g_earth[3],
          double m_load,
          const double F_x_Cload[3],
          double g_NE[7]);
}

// --- franka::LibraryLoader (src/library_loader.cpp, 0.9.2) ----------------------------
class LibraryLoader {
 public:
  explicit LibraryLoader(const std::string& filepath) { library_.load(filepath); }
  ~LibraryLoader() {
    try {
      library_.unload();
    } catch (...) {
    }
  }
  void* getSymbol(const std::string& symbol_name) { return library_.getSymbol(symbol_name); }

 private:
  Poco::SharedLibrary library_;
};

// --- franka::ModelLibrary (src/model_library.h / .cpp, 0.9.2) -------------------------
// Only the five entry points this benchmark calls; the real class binds all thirty, which
// costs load time, not call time.
class ModelLibrary {
 public:
  explicit ModelLibrary(const std::string& path)
      : loader_(path),
        mass{reinterpret_cast<decltype(&M_NE)>(loader_.getSymbol("M_NE"))},
        zero_jacobian_ee{reinterpret_cast<decltype(&O_J_J9)>(loader_.getSymbol("O_J_J9"))},
        ee{reinterpret_cast<decltype(&O_T_J9)>(loader_.getSymbol("O_T_J9"))},
        coriolis{reinterpret_cast<decltype(&c_NE)>(loader_.getSymbol("c_NE"))},
        gravity{reinterpret_cast<decltype(&g_NE)>(loader_.getSymbol("g_NE"))} {}

 private:
  LibraryLoader loader_;

 public:
  const std::function<decltype(M_NE)> mass;
  const std::function<decltype(O_J_J9)> zero_jacobian_ee;
  const std::function<decltype(O_T_J9)> ee;
  const std::function<decltype(c_NE)> coriolis;
  const std::function<decltype(g_NE)> gravity;
};

// The subset of franka::RobotState the five calls read.
struct State {
  std::array<double, 7> q;
  std::array<double, 7> dq;
  std::array<double, 16> F_T_EE;
  std::array<double, 9> I_total;
  double m_total;
  std::array<double, 3> F_x_Ctotal;
  std::array<double, 3> O_ddP_O;
};

// --- franka::Model (src/model.cpp, 0.9.2), the kEndEffector switch arms ---------------
class Model {
 public:
  explicit Model(const std::string& path) : library_(new ModelLibrary(path)) {}

  std::array<double, 16> pose_ee(const State& s) const {
    std::array<double, 16> output;
    library_->ee(s.q.data(), s.F_T_EE.data(), output.data());
    return output;
  }
  std::array<double, 42> zeroJacobian_ee(const State& s) const {
    std::array<double, 42> output;
    library_->zero_jacobian_ee(s.q.data(), s.F_T_EE.data(), output.data());
    return output;
  }
  std::array<double, 49> mass(const State& s) const noexcept {
    std::array<double, 49> output;
    library_->mass(s.q.data(), s.I_total.data(), s.m_total, s.F_x_Ctotal.data(), output.data());
    return output;
  }
  std::array<double, 7> coriolis(const State& s) const noexcept {
    std::array<double, 7> output;
    library_->coriolis(s.q.data(), s.dq.data(), s.I_total.data(), s.m_total, s.F_x_Ctotal.data(),
                       output.data());
    return output;
  }
  std::array<double, 7> gravity(const State& s) const noexcept {
    std::array<double, 7> output;
    library_->gravity(s.q.data(), s.O_ddP_O.data(), s.m_total, s.F_x_Ctotal.data(), output.data());
    return output;
  }

 private:
  std::unique_ptr<ModelLibrary> library_;
};

static int64_t monotonic_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000 + ts.tv_nsec;
}

static void sleep_ns(long nanos) {
  struct timespec ts;
  ts.tv_sec = 0;
  ts.tv_nsec = nanos;
  nanosleep(&ts, nullptr);
}

// Nearest-rank percentile over an already-sorted vector.
static double percentile(const std::vector<double>& sorted, double q) {
  size_t rank = static_cast<size_t>(std::ceil(q * static_cast<double>(sorted.size())));
  if (rank < 1) {
    rank = 1;
  }
  if (rank > sorted.size()) {
    rank = sorted.size();
  }
  return sorted[rank - 1];
}

static void report(const std::string& name, std::vector<double>& values) {
  std::sort(values.begin(), values.end());
  const double mean =
      std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
  std::printf("%-16s p50 %8.3f  p90 %8.3f  p99 %8.3f  min %8.3f  mean %8.3f\n", name.c_str(),
              percentile(values, 0.50), percentile(values, 0.90), percentile(values, 0.99),
              values.front(), mean);
}

// Untimed filler work, used only to move the core up its frequency ramp.
static double filler(double a, uint64_t iterations) {
  for (uint64_t i = 0; i < iterations; ++i) {
    a = a * 1.0000001 + 1e-7;
    if (a > 1e30) {
      a *= 1e-30;
    }
  }
  return a;
}

int main(int argc, char** argv) {
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(2, &set);
  sched_setaffinity(0, sizeof(set), &set);

  if (argc < 2) {
    std::fprintf(stderr, "usage: so_micro_cpp <libfcimodels_x64.so> [iterations]\n");
    return 2;
  }
  const size_t n = argc > 2 ? std::stoul(argv[2]) : 100000;

  Model model{std::string(argv[1])};

  // Robot L's read-only probe of 2026-09-05
  // (bench/results/20260905-fer-hw/L/model_probe_cpp_0.9.2.txt), with a non-zero dq so
  // coriolis is not evaluated at rest. The Rust side uses exactly these values.
  State s;
  s.q = {-0.000230663600055855, -0.785250788805778, 0.000051590539422385,
         -2.35692138653159,     0.000811206067415174, 1.57033887690968,
         0.785072525387837};
  s.dq = {0.11, -0.22, 0.33, -0.44, 0.55, -0.66, 0.77};
  s.F_T_EE = {0.707099974155426, -0.707099974155426, 0.0, 0.0,
              0.707099974155426, 0.707099974155426,  0.0, 0.0,
              0.0,               0.0,                1.0, 0.0,
              0.0,               0.0,                0.103399999439716, 1.0};
  s.I_total = {0.00100000004749745, 0.0, 0.0, 0.0, 0.00249999994412065, 0.0, 0.0, 0.0,
               0.00170000002253801};
  s.m_total = 0.730000019073486;
  s.F_x_Ctotal = {-0.00999999977648258, 0.0, 0.0299999993294477};
  s.O_ddP_O = {0.0, 0.0, -9.81};

  double checksum = 0.0;

  std::printf("# steady state (back to back, warm core)\n");
#define BENCH_ONE(NAME, EXPR)                                       \
  {                                                                 \
    for (size_t i = 0; i < n / 10; ++i) {                           \
      checksum += (EXPR)[0];                                        \
    }                                                               \
    std::vector<double> samples;                                    \
    samples.reserve(n);                                             \
    for (size_t i = 0; i < n; ++i) {                                \
      const int64_t t0 = monotonic_ns();                            \
      const auto out = (EXPR);                                      \
      const int64_t t1 = monotonic_ns();                            \
      checksum += out[0];                                           \
      samples.push_back(static_cast<double>(t1 - t0) * 1e-3);       \
    }                                                               \
    report(NAME, samples);                                          \
  }

  BENCH_ONE("gravity", model.gravity(s))
  BENCH_ONE("coriolis", model.coriolis(s))
  BENCH_ONE("mass", model.mass(s))
  BENCH_ONE("pose_ee", model.pose_ee(s))
  BENCH_ONE("zero_jacobian_ee", model.zeroJacobian_ee(s))

  // The five calls in the order bench/cpp/bench_model_control.cpp makes them.
#define FIVE_CALL()                                                 \
  ([&] {                                                            \
    const auto mass = model.mass(s);                                \
    const auto coriolis = model.coriolis(s);                        \
    const auto gravity = model.gravity(s);                          \
    const auto jacobian = model.zeroJacobian_ee(s);                 \
    const auto pose = model.pose_ee(s);                             \
    return mass[0] + coriolis[0] + gravity[0] + jacobian[0] + pose[0]; \
  }())

  {
    for (size_t i = 0; i < n / 10; ++i) {
      checksum += FIVE_CALL();
    }
    std::vector<double> samples;
    samples.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      const int64_t t0 = monotonic_ns();
      const double sum = FIVE_CALL();
      const int64_t t1 = monotonic_ns();
      checksum += sum;
      samples.push_back(static_cast<double>(t1 - t0) * 1e-3);
    }
    report("five-call", samples);
  }

  const size_t duty_cycles = std::min<size_t>(std::max<size_t>(n / 5, 2000), 20000);

  std::printf("\n# duty cycled (~1 ms idle before each cycle, as in a 1 kHz control loop)\n");
  {
    std::vector<double> samples;
    samples.reserve(duty_cycles);
    for (size_t i = 0; i < duty_cycles; ++i) {
      sleep_ns(900000);
      const int64_t t0 = monotonic_ns();
      const double sum = FIVE_CALL();
      const int64_t t1 = monotonic_ns();
      checksum += sum;
      samples.push_back(static_cast<double>(t1 - t0) * 1e-3);
    }
    report("five-call", samples);
  }

  std::printf("\n# duty cycled, after N iterations of untimed filler work\n");
  {
    const uint64_t warms[] = {0, 500, 1500, 4000, 12000, 40000};
    for (uint64_t warm : warms) {
      std::vector<double> samples;
      samples.reserve(duty_cycles);
      for (size_t i = 0; i < duty_cycles; ++i) {
        sleep_ns(900000);
        checksum += filler(1.0, warm);
        const int64_t t0 = monotonic_ns();
        const double sum = FIVE_CALL();
        const int64_t t1 = monotonic_ns();
        checksum += sum;
        samples.push_back(static_cast<double>(t1 - t0) * 1e-3);
      }
      report("filler " + std::to_string(warm), samples);
    }
  }

  std::printf("\nchecksum %.6f\n", checksum);
  return 0;
}
