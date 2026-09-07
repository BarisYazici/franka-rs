// Offline microbenchmark of libfranka's `franka::Model` (Pinocchio backend), and the
// generator of the shared input set the Rust counterpart replays.
//
// No robot and no simulator are involved: the program draws `--count` random (q, dq) pairs
// with a fixed seed, writes them to `states.json`, evaluates the five model calls the
// `model` control variant makes per cycle on every one of them, times each call
// individually with CLOCK_MONOTONIC, and writes
//
//   states.json   the inputs, so ../rust replays *exactly* the same q/dq,
//   reference.bin  every output as little-endian f64, for the cross-check,
//   cpp.json       the per-call timing statistics.
//
// The Rust counterpart is ../rust/src/main.rs. Both use the same nearest-rank percentile
// code as bench/cpp/bench_joint_velocity.cpp.
//
// A note on `coriolis`. `franka::Model::coriolis(const RobotState&)` forwards to the
// *deprecated* five-argument `RobotModel::coriolis`, which builds the full Coriolis matrix
// with `pinocchio::computeCoriolisMatrix` and multiplies by dq. The non-deprecated
// six-argument overload instead evaluates `rnea(q, dq, 0) - generalizedGravity(q)`, which is
// what franka-rs's native backend does for *both* entry points. Both are timed here
// (`coriolis` and `coriolis_rnea`) so the Rust column can be compared against the algorithm
// it actually implements as well as against the API call a user writes.

#include <time.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <franka/model.h>
#include <franka/robot_model.h>
#include <franka/robot_model_base.h>

namespace {

// FR3 joint limits, taken from crates/franka-rs/tests/data/fr3.urdf <limit lower/upper>,
// identical to the ones tools/model-reference draws from.
constexpr std::array<double, 7> kQMin = {-2.7501, -1.7918, -2.9065, -3.0481,
                                         -2.8101, 0.54092, -3.0196};
constexpr std::array<double, 7> kQMax = {2.7501, 1.7918, 2.9065, -0.1458, 2.8101, 4.5205, 3.0196};
constexpr double kDqLimit = 2.0;  // dq is drawn from [-2, 2] rad/s

constexpr std::array<double, 16> kIdentity = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

// The Franka Hand flange->EE transform plus a 0.5 kg load: a representative, *non-zero*
// total load, so the last-link inertia update inside the Pinocchio backend is exercised the
// way it is on a real arm. Derived exactly like Robot::Impl::convertRobotState derives it,
// with the numbers precomputed from tools/model-reference's `hand_and_load` configuration.
constexpr std::array<double, 16> kFTEE = {0.7071, -0.7071, 0, 0, 0.7071, 0.7071, 0, 0,
                                          0,      0,       1, 0, 0,      0,      0.1034, 1};
constexpr double kMEe = 0.73;
constexpr std::array<double, 3> kFxCee = {-0.01, 0.0, 0.03};
constexpr std::array<double, 9> kIEe = {0.001, 0, 0, 0, 0.0025, 0, 0, 0, 0.0017};
constexpr double kMLoad = 0.5;
constexpr std::array<double, 3> kFxCload = {0.01, 0.02, 0.03};
constexpr std::array<double, 9> kILoad = {0.001, 0, 0, 0, 0.002, 0, 0, 0, 0.003};

constexpr std::array<double, 3> kGravityEarth = {0.0, 0.0, -9.81};

// The number of doubles written per sample to reference.bin, in this order:
// mass(49), coriolis(7), coriolis_rnea(7), gravity(7), zero_jacobian(42), pose(16).
constexpr size_t kRefDoublesPerSample = 49 + 7 + 7 + 7 + 42 + 16;

// --- ports of libfranka src/load_calculations.cpp -------------------------------------

std::array<double, 3> combineCenterOfMass(double m_ee,
                                          const std::array<double, 3>& F_x_Cee,
                                          double m_load,
                                          const std::array<double, 3>& F_x_Cload) {
  std::array<double, 3> F_x_Ctotal{};
  if ((m_ee + m_load) > 0) {
    for (size_t i = 0; i < F_x_Ctotal.size(); i++) {
      F_x_Ctotal[i] = (m_ee * F_x_Cee[i] + m_load * F_x_Cload[i]) / (m_ee + m_load);
    }
  }
  return F_x_Ctotal;
}

Eigen::Matrix3d skewSymmetricMatrixFromVector(const Eigen::Vector3d& input) {
  Eigen::Matrix3d input_hat;
  input_hat << 0, -input(2), input(1), input(2), 0, -input(0), -input(1), input(0), 0;
  return input_hat;
}

std::array<double, 9> combineInertiaTensor(double m_ee,
                                           const std::array<double, 3>& F_x_Cee,
                                           const std::array<double, 9>& I_ee,
                                           double m_load,
                                           const std::array<double, 3>& F_x_Cload,
                                           const std::array<double, 9>& I_load,
                                           double m_total,
                                           const std::array<double, 3>& F_x_Ctotal) {
  if (m_total == 0) {
    return std::array<double, 9>{};
  }
  Eigen::Vector3d center_of_mass_ee(F_x_Cee.data());
  Eigen::Vector3d center_of_mass_load(F_x_Cload.data());
  Eigen::Vector3d center_of_mass_total(F_x_Ctotal.data());
  Eigen::Matrix3d inertia_ee(I_ee.data());
  Eigen::Matrix3d inertia_load(I_load.data());
  if (m_ee == 0) {
    inertia_ee = Eigen::Matrix3d::Zero();
  }
  if (m_load == 0) {
    inertia_load = Eigen::Matrix3d::Zero();
  }
  Eigen::Matrix3d inertia_ee_flange =
      inertia_ee - m_ee * (skewSymmetricMatrixFromVector(center_of_mass_ee) *
                           skewSymmetricMatrixFromVector(center_of_mass_ee));
  Eigen::Matrix3d inertia_load_flange =
      inertia_load - m_load * (skewSymmetricMatrixFromVector(center_of_mass_load) *
                               skewSymmetricMatrixFromVector(center_of_mass_load));
  Eigen::Matrix3d inertia_total_flange = inertia_ee_flange + inertia_load_flange;
  std::array<double, 9> I_total;
  Eigen::Map<Eigen::Matrix3d> inertia_total(I_total.data(), 3, 3);
  inertia_total =
      inertia_total_flange + m_total * (skewSymmetricMatrixFromVector(center_of_mass_total) *
                                        skewSymmetricMatrixFromVector(center_of_mass_total));
  return I_total;
}

// --- timing and statistics, identical to bench/cpp/bench_joint_velocity.cpp ------------

inline int64_t monotonic_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

struct Stats {
  double p50 = 0, p99 = 0, p999 = 0, min = 0, max = 0, mean = 0;
  size_t n = 0;
};

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
  for (double v : values) {
    sum += v;
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
                "{\"n\": %zu, \"min\": %.4f, \"p50\": %.4f, \"p99\": %.4f, \"p999\": %.4f, "
                "\"max\": %.4f, \"mean\": %.4f}",
                s.n, s.min, s.p50, s.p99, s.p999, s.max, s.mean);
  return std::string(buf);
}

std::string readFile(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("cannot open " + path);
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

void writeDouble(FILE* out, double value) {
  char buffer[40];
  std::snprintf(buffer, sizeof(buffer), "%.17g", value);
  std::fputs(buffer, out);
}

template <size_t N>
void writeArray(FILE* out, const std::array<double, N>& values) {
  std::fputc('[', out);
  for (size_t i = 0; i < N; ++i) {
    if (i != 0) {
      std::fputc(',', out);
    }
    writeDouble(out, values[i]);
  }
  std::fputc(']', out);
}

}  // namespace

int main(int argc, char** argv) {
  std::string urdf_path;
  std::string out_dir = ".";
  size_t count = 10000;
  size_t warmup = 1000;
  uint64_t seed = 20260904;

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&](const char* what) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << what << std::endl;
        std::exit(2);
      }
      return std::string(argv[++i]);
    };
    if (a == "--out-dir") {
      out_dir = next("--out-dir");
    } else if (a == "--count") {
      count = static_cast<size_t>(std::stoul(next("--count")));
    } else if (a == "--warmup") {
      warmup = static_cast<size_t>(std::stoul(next("--warmup")));
    } else if (a == "--seed") {
      seed = std::stoull(next("--seed"));
    } else if (!a.empty() && a[0] == '-') {
      std::cerr << "unknown flag " << a << std::endl;
      return 2;
    } else {
      urdf_path = a;
    }
  }
  if (urdf_path.empty()) {
    std::cerr << "usage: " << argv[0]
              << " <fr3.urdf> [--out-dir DIR] [--count 10000] [--warmup 1000] [--seed N]"
              << std::endl;
    return 2;
  }

  std::string urdf;
  try {
    urdf = readFile(urdf_path);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << std::endl;
    return 1;
  }

  const double m_total = kMEe + kMLoad;
  const std::array<double, 3> F_x_Ctotal =
      combineCenterOfMass(kMEe, kFxCee, kMLoad, kFxCload);
  const std::array<double, 9> I_total = combineInertiaTensor(kMEe, kFxCee, kIEe, kMLoad,
                                                             kFxCload, kILoad, m_total,
                                                             F_x_Ctotal);

  // Draw the inputs first and write them out, so the Rust side replays exactly these.
  std::vector<std::array<double, 7>> qs(count + warmup);
  std::vector<std::array<double, 7>> dqs(count + warmup);
  {
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_real_distribution<double> dq_dist(-kDqLimit, kDqLimit);
    for (size_t i = 0; i < qs.size(); ++i) {
      for (size_t j = 0; j < 7; ++j) {
        qs[i][j] = kQMin[j] + unit(rng) * (kQMax[j] - kQMin[j]);
        dqs[i][j] = dq_dist(rng);
      }
    }
  }

  // Exactly the construction path libfranka uses internally: the Pinocchio backend wrapped
  // in the public Model facade (see tools/model-reference/main.cpp).
  franka::Model model(std::unique_ptr<RobotModelBase>(new franka::RobotModel(urdf)));

  // Preallocated: nothing is allocated or printed between the first and the last timestamp.
  std::vector<double> t_mass(count), t_coriolis(count), t_coriolis_rnea(count);
  std::vector<double> t_gravity(count), t_jacobian(count), t_pose(count), t_total(count);
  std::vector<double> reference(count * kRefDoublesPerSample);
  double checksum = 0.0;

  for (size_t i = 0; i < count + warmup; ++i) {
    const std::array<double, 7>& q = qs[i];
    const std::array<double, 7>& dq = dqs[i];

    // Same order as the `model` control variant: mass, coriolis, gravity, Jacobian, pose.
    // mass() first also pins down the Pinocchio backend's last-link inertia cache.
    const int64_t t0 = monotonic_ns();
    std::array<double, 49> mass = model.mass(q, I_total, m_total, F_x_Ctotal);
    const int64_t t1 = monotonic_ns();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    // The overload franka::Model::coriolis(const RobotState&) forwards to.
    std::array<double, 7> coriolis = model.coriolis(q, dq, I_total, m_total, F_x_Ctotal);
#pragma GCC diagnostic pop
    const int64_t t2 = monotonic_ns();
    std::array<double, 7> gravity = model.gravity(q, m_total, F_x_Ctotal, kGravityEarth);
    const int64_t t3 = monotonic_ns();
    std::array<double, 42> jacobian = model.zeroJacobian(franka::Frame::kEndEffector, q, kFTEE,
                                                         kIdentity);
    const int64_t t4 = monotonic_ns();
    std::array<double, 16> pose = model.pose(franka::Frame::kEndEffector, q, kFTEE, kIdentity);
    const int64_t t5 = monotonic_ns();
    // Extra, not part of the per-cycle total: the non-deprecated RNEA-based overload, which
    // is the algorithm franka-rs's native backend implements for both entry points.
    std::array<double, 7> coriolis_rnea =
        model.coriolis(q, dq, I_total, m_total, F_x_Ctotal, kGravityEarth);
    const int64_t t6 = monotonic_ns();

    if (i < warmup) {
      checksum += mass[0] + coriolis[0] + gravity[0] + jacobian[0] + pose[0] + coriolis_rnea[0];
      continue;
    }
    const size_t k = i - warmup;
    t_mass[k] = static_cast<double>(t1 - t0) * 1e-3;
    t_coriolis[k] = static_cast<double>(t2 - t1) * 1e-3;
    t_gravity[k] = static_cast<double>(t3 - t2) * 1e-3;
    t_jacobian[k] = static_cast<double>(t4 - t3) * 1e-3;
    t_pose[k] = static_cast<double>(t5 - t4) * 1e-3;
    t_total[k] = static_cast<double>(t5 - t0) * 1e-3;
    t_coriolis_rnea[k] = static_cast<double>(t6 - t5) * 1e-3;

    double* ref = reference.data() + k * kRefDoublesPerSample;
    std::memcpy(ref, mass.data(), 49 * sizeof(double));
    std::memcpy(ref + 49, coriolis.data(), 7 * sizeof(double));
    std::memcpy(ref + 56, coriolis_rnea.data(), 7 * sizeof(double));
    std::memcpy(ref + 63, gravity.data(), 7 * sizeof(double));
    std::memcpy(ref + 70, jacobian.data(), 42 * sizeof(double));
    std::memcpy(ref + 112, pose.data(), 16 * sizeof(double));
  }

  // --- outputs --------------------------------------------------------------------------
  const std::string states_path = out_dir + "/states.json";
  FILE* states = std::fopen(states_path.c_str(), "w");
  if (states == nullptr) {
    std::cerr << "error: cannot write " << states_path << std::endl;
    return 1;
  }
  std::fputs("{\n\"meta\":{", states);
  std::fprintf(states, "\"generator\":\"bench/model-micro/cpp\",\"rng\":\"std::mt19937_64\",");
  std::fprintf(states, "\"seed\":%llu,\"count\":%zu,\"warmup\":%zu,",
               static_cast<unsigned long long>(seed), count, warmup);
  std::fprintf(states, "\"urdf\":\"%s\",", urdf_path.c_str());
  std::fputs("\"ref_doubles_per_sample\":", states);
  std::fprintf(states, "%zu,", kRefDoublesPerSample);
  std::fputs("\"ref_layout\":\"mass(49),coriolis(7),coriolis_rnea(7),gravity(7),"
             "zero_jacobian(42),pose(16)\"},\n",
             states);
  std::fputs("\"config\":{\"F_T_EE\":", states);
  writeArray(states, kFTEE);
  std::fputs(",\"EE_T_K\":", states);
  writeArray(states, kIdentity);
  std::fputs(",\"m_total\":", states);
  writeDouble(states, m_total);
  std::fputs(",\"F_x_Ctotal\":", states);
  writeArray(states, F_x_Ctotal);
  std::fputs(",\"I_total\":", states);
  writeArray(states, I_total);
  std::fputs(",\"gravity_earth\":", states);
  writeArray(states, kGravityEarth);
  std::fputs("},\n\"samples\":[\n", states);
  // The warmup samples are written too, so the Rust side warms up on identical inputs; the
  // first `warmup` entries are the warmup set and the remaining `count` are the measured one.
  for (size_t i = 0; i < qs.size(); ++i) {
    std::fputs(i == 0 ? "{\"q\":" : ",\n{\"q\":", states);
    writeArray(states, qs[i]);
    std::fputs(",\"dq\":", states);
    writeArray(states, dqs[i]);
    std::fputc('}', states);
  }
  std::fputs("\n]}\n", states);
  std::fclose(states);

  const std::string reference_path = out_dir + "/reference.bin";
  std::ofstream ref_file(reference_path, std::ios::binary);
  if (!ref_file) {
    std::cerr << "error: cannot write " << reference_path << std::endl;
    return 1;
  }
  ref_file.write(reinterpret_cast<const char*>(reference.data()),
                 static_cast<std::streamsize>(reference.size() * sizeof(double)));
  ref_file.close();

  const Stats s_mass = summarize(t_mass);
  const Stats s_coriolis = summarize(t_coriolis);
  const Stats s_coriolis_rnea = summarize(t_coriolis_rnea);
  const Stats s_gravity = summarize(t_gravity);
  const Stats s_jacobian = summarize(t_jacobian);
  const Stats s_pose = summarize(t_pose);
  const Stats s_total = summarize(t_total);

  std::ostringstream json;
  json << "{\n";
  json << "  \"lang\": \"cpp\",\n";
  json << "  \"library\": \"libfranka 0.20.4 (franka::RobotModel, Pinocchio)\",\n";
  json << "  \"count\": " << count << ",\n";
  json << "  \"warmup\": " << warmup << ",\n";
  json << "  \"seed\": " << seed << ",\n";
  json << "  \"urdf\": \"" << urdf_path << "\",\n";
  json << "  \"checksum\": " << checksum << ",\n";
  json << "  \"calls\": {\n";
  json << "    \"mass\": " << stats_json(s_mass) << ",\n";
  json << "    \"coriolis\": " << stats_json(s_coriolis) << ",\n";
  json << "    \"gravity\": " << stats_json(s_gravity) << ",\n";
  json << "    \"zero_jacobian\": " << stats_json(s_jacobian) << ",\n";
  json << "    \"pose\": " << stats_json(s_pose) << ",\n";
  json << "    \"coriolis_rnea\": " << stats_json(s_coriolis_rnea) << "\n";
  json << "  },\n";
  json << "  \"total\": " << stats_json(s_total) << "\n";
  json << "}\n";

  const std::string out_path = out_dir + "/cpp.json";
  std::ofstream out(out_path);
  out << json.str();
  out.close();
  std::cout << json.str();
  return 0;
}
