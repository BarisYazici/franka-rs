// Copyright (c) 2026 franka-rs contributors
// Use of this source code is governed by the Apache-2.0 license.
//
// Reference dumper for the franka-rs `Model` conformance suite.
//
// Builds a `franka::Model` around libfranka's own Pinocchio-based
// `franka::RobotModel` (src/robot_model.cpp) from the FR3 URDF and writes every
// quantity the `franka::Model` API exposes for a deterministic sample set to a
// JSON fixture consumed by `crates/franka-rs/tests/model_conformance.rs`.
//
// `combineCenterOfMass` / `combineInertiaTensor` are ported verbatim from
// libfranka's src/load_calculations.cpp so that `m_total` / `F_x_Ctotal` /
// `I_total` are derived exactly the way `convertRobotState` derives them.

#include <franka/model.h>
#include <franka/robot_model.h>
#include <franka/robot_model_base.h>

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "sha256.h"

namespace {

constexpr int kNumFrames = 10;
constexpr const char* kFrameNames[kNumFrames] = {"Joint1", "Joint2", "Joint3",     "Joint4",
                                                 "Joint5", "Joint6", "Joint7",     "Flange",
                                                 "EndEffector", "Stiffness"};

// FR3 joint limits, taken from reference/libfranka/test/fr3.urdf <limit lower/upper>.
constexpr std::array<double, 7> kQMin = {-2.7501, -1.7918, -2.9065, -3.0481,
                                         -2.8101, 0.54092, -3.0196};
constexpr std::array<double, 7> kQMax = {2.7501, 1.7918, 2.9065, -0.1458, 2.8101, 4.5205, 3.0196};

constexpr double kDqLimit = 2.0;  // random dq is drawn from [-2, 2] rad/s
constexpr uint64_t kRngSeed = 20260904;

constexpr std::array<double, 16> kIdentity = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

// ---------------------------------------------------------------------------
// Port of libfranka src/load_calculations.cpp (0.20.4 == 0.21.2, byte identical)
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// JSON helpers (17 significant digits, exactly what round-trips a double)
// ---------------------------------------------------------------------------

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

void writeVector(FILE* out, const std::vector<double>& values) {
  std::fputc('[', out);
  for (size_t i = 0; i < values.size(); ++i) {
    if (i != 0) {
      std::fputc(',', out);
    }
    writeDouble(out, values[i]);
  }
  std::fputc(']', out);
}

struct LoadConfig {
  std::string name;
  std::array<double, 16> F_T_EE;
  std::array<double, 16> EE_T_K;
  double m_ee;
  std::array<double, 3> F_x_Cee;
  std::array<double, 9> I_ee;
  double m_load;
  std::array<double, 3> F_x_Cload;
  std::array<double, 9> I_load;
  // Derived exactly like Robot::Impl::convertRobotState does.
  double m_total;
  std::array<double, 3> F_x_Ctotal;
  std::array<double, 9> I_total;

  void deriveTotals() {
    m_total = m_ee + m_load;
    F_x_Ctotal = combineCenterOfMass(m_ee, F_x_Cee, m_load, F_x_Cload);
    I_total = combineInertiaTensor(m_ee, F_x_Cee, I_ee, m_load, F_x_Cload, I_load, m_total,
                                   F_x_Ctotal);
  }
};

std::vector<LoadConfig> makeLoadConfigs() {
  std::vector<LoadConfig> configs;

  LoadConfig none;
  none.name = "no_load";
  none.F_T_EE = kIdentity;
  none.EE_T_K = kIdentity;
  none.m_ee = 0.0;
  none.F_x_Cee = {0, 0, 0};
  none.I_ee = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  none.m_load = 0.0;
  none.F_x_Cload = {0, 0, 0};
  none.I_load = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  none.deriveTotals();
  configs.push_back(none);

  // Franka Hand flange->EE transform, column-major.
  const std::array<double, 16> hand = {0.7071, -0.7071, 0, 0, 0.7071, 0.7071, 0, 0,
                                       0,      0,       1, 0, 0,      0,      0.1034, 1};

  LoadConfig hand_load;
  hand_load.name = "hand_and_load";
  hand_load.F_T_EE = hand;
  hand_load.EE_T_K = kIdentity;
  hand_load.m_ee = 0.73;
  hand_load.F_x_Cee = {-0.01, 0.0, 0.03};
  hand_load.I_ee = {0.001, 0, 0, 0, 0.0025, 0, 0, 0, 0.0017};
  hand_load.m_load = 0.5;
  hand_load.F_x_Cload = {0.01, 0.02, 0.03};
  hand_load.I_load = {0.001, 0, 0, 0, 0.002, 0, 0, 0, 0.003};
  hand_load.deriveTotals();
  configs.push_back(hand_load);

  // Same as above, plus a stiffness frame offset: translation {0,0,0.05} and a
  // 30 degree rotation about z.
  LoadConfig hand_load_k = hand_load;
  hand_load_k.name = "hand_load_and_stiffness";
  const double c30 = std::cos(30.0 * M_PI / 180.0);
  const double s30 = std::sin(30.0 * M_PI / 180.0);
  hand_load_k.EE_T_K = {c30, s30, 0, 0, -s30, c30, 0, 0, 0, 0, 1, 0, 0, 0, 0.05, 1};
  configs.push_back(hand_load_k);

  return configs;
}

struct Sample {
  std::string kind;
  std::array<double, 7> q;
  std::array<double, 7> dq;
};

std::vector<Sample> makeSamples(size_t random_count) {
  std::mt19937_64 rng(kRngSeed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> dq_dist(-kDqLimit, kDqLimit);
  std::bernoulli_distribution coin(0.5);

  std::vector<Sample> samples;

  // 1) zero pose, at rest.
  Sample zero;
  zero.kind = "zero";
  zero.q = {0, 0, 0, 0, 0, 0, 0};
  zero.dq = {0, 0, 0, 0, 0, 0, 0};
  samples.push_back(zero);

  // 2) the libfranka "ready" pose, with unit joint velocity.
  Sample ready;
  ready.kind = "ready";
  ready.q = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
  ready.dq = {1, 1, 1, 1, 1, 1, 1};
  samples.push_back(ready);

  // 3) six poses at random joint-limit corners.
  for (int corner = 0; corner < 6; ++corner) {
    Sample sample;
    sample.kind = "limit_corner";
    for (size_t j = 0; j < 7; ++j) {
      sample.q[j] = coin(rng) ? kQMax[j] : kQMin[j];
      sample.dq[j] = dq_dist(rng);
    }
    samples.push_back(sample);
  }

  // 4) uniformly random configurations inside the joint limits.
  for (size_t i = 0; i < random_count; ++i) {
    Sample sample;
    sample.kind = "random";
    for (size_t j = 0; j < 7; ++j) {
      sample.q[j] = kQMin[j] + unit(rng) * (kQMax[j] - kQMin[j]);
      sample.dq[j] = dq_dist(rng);
    }
    samples.push_back(sample);
  }

  return samples;
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

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: model-reference <fr3.urdf> <output.json> [random_sample_count]\n";
    return 2;
  }

  const std::string urdf_path = argv[1];
  const std::string output_path = argv[2];
  const size_t random_count = argc > 3 ? static_cast<size_t>(std::atoi(argv[3])) : 192;

  std::string urdf;
  try {
    urdf = readFile(urdf_path);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }

  const std::string urdf_sha256 = franka_rs_reference::sha256_hex(urdf);

  // Exactly the construction path libfranka uses internally: the Pinocchio
  // backend wrapped in the public Model facade.
  franka::Model model(std::unique_ptr<RobotModelBase>(new franka::RobotModel(urdf)));

  const std::vector<LoadConfig> configs = makeLoadConfigs();
  const std::vector<Sample> samples = makeSamples(random_count);

  const std::array<double, 3> gravity_earth = {0.0, 0.0, -9.81};
  const std::array<double, 3> gravity_earth_alt = {0.1, -0.2, -9.7};

  FILE* out = std::fopen(output_path.c_str(), "w");
  if (out == nullptr) {
    std::cerr << "error: cannot write " << output_path << "\n";
    return 1;
  }

  std::fputs("{\n\"meta\":{", out);
  std::fputs("\"generator\":\"tools/model-reference\",", out);
  std::fputs("\"libfranka_version\":\"0.20.4\",", out);
  std::fprintf(out, "\"urdf_source\":\"%s\",", "reference/libfranka/test/fr3.urdf");
  std::fprintf(out, "\"urdf_sha256\":\"%s\",", urdf_sha256.c_str());
  std::fprintf(out, "\"rng\":\"std::mt19937_64\",\"rng_seed\":%llu,",
               static_cast<unsigned long long>(kRngSeed));
  std::fprintf(out, "\"random_sample_count\":%zu,", random_count);
  std::fputs("\"jacobian_layout\":\"6x7 column-major, rows [vx,vy,vz,wx,wy,wz]\",", out);
  std::fputs("\"pose_layout\":\"4x4 column-major\",", out);
  std::fputs("\"gravity_earth\":", out);
  writeArray(out, gravity_earth);
  std::fputs(",\"gravity_earth_alt\":", out);
  writeArray(out, gravity_earth_alt);
  std::fputs(",\"q_min\":", out);
  writeArray(out, kQMin);
  std::fputs(",\"q_max\":", out);
  writeArray(out, kQMax);
  std::fputs(",\"frames\":[", out);
  for (int f = 0; f < kNumFrames; ++f) {
    std::fprintf(out, "%s\"%s\"", f == 0 ? "" : ",", kFrameNames[f]);
  }
  std::fputs("]},\n", out);

  std::fputs("\"load_configs\":[\n", out);
  for (size_t c = 0; c < configs.size(); ++c) {
    const LoadConfig& cfg = configs[c];
    std::fprintf(out, "%s{\"name\":\"%s\",", c == 0 ? "" : ",\n", cfg.name.c_str());
    std::fputs("\"F_T_EE\":", out);
    writeArray(out, cfg.F_T_EE);
    std::fputs(",\"EE_T_K\":", out);
    writeArray(out, cfg.EE_T_K);
    std::fputs(",\"m_ee\":", out);
    writeDouble(out, cfg.m_ee);
    std::fputs(",\"F_x_Cee\":", out);
    writeArray(out, cfg.F_x_Cee);
    std::fputs(",\"I_ee\":", out);
    writeArray(out, cfg.I_ee);
    std::fputs(",\"m_load\":", out);
    writeDouble(out, cfg.m_load);
    std::fputs(",\"F_x_Cload\":", out);
    writeArray(out, cfg.F_x_Cload);
    std::fputs(",\"I_load\":", out);
    writeArray(out, cfg.I_load);
    std::fputs(",\"m_total\":", out);
    writeDouble(out, cfg.m_total);
    std::fputs(",\"F_x_Ctotal\":", out);
    writeArray(out, cfg.F_x_Ctotal);
    std::fputs(",\"I_total\":", out);
    writeArray(out, cfg.I_total);
    std::fputc('}', out);
  }
  std::fputs("\n],\n", out);

  std::fputs("\"samples\":[\n", out);
  for (size_t s = 0; s < samples.size(); ++s) {
    const Sample& sample = samples[s];
    std::fprintf(out, "%s{\"index\":%zu,\"kind\":\"%s\",\"q\":", s == 0 ? "" : ",\n", s,
                 sample.kind.c_str());
    writeArray(out, sample.q);
    std::fputs(",\"dq\":", out);
    writeArray(out, sample.dq);
    std::fputs(",\"cases\":[", out);

    for (size_t c = 0; c < configs.size(); ++c) {
      const LoadConfig& cfg = configs[c];
      std::fprintf(out, "%s{\"config\":%zu", c == 0 ? "" : ",", c);

      // Dynamics first, and mass() before gravity(): franka::RobotModel caches the
      // last-link inertia and its gravity() only refreshes the cache when
      // m_total > 0, so the mass() call in front of it pins the cache down.
      std::array<double, 49> mass = model.mass(sample.q, cfg.I_total, cfg.m_total, cfg.F_x_Ctotal);
      std::array<double, 7> coriolis = model.coriolis(sample.q, sample.dq, cfg.I_total,
                                                      cfg.m_total, cfg.F_x_Ctotal, gravity_earth);
      std::array<double, 7> gravity =
          model.gravity(sample.q, cfg.m_total, cfg.F_x_Ctotal, gravity_earth);
      std::array<double, 7> gravity_alt =
          model.gravity(sample.q, cfg.m_total, cfg.F_x_Ctotal, gravity_earth_alt);

      std::vector<double> poses;
      std::vector<double> body_jacobians;
      std::vector<double> zero_jacobians;
      poses.reserve(16 * kNumFrames);
      body_jacobians.reserve(42 * kNumFrames);
      zero_jacobians.reserve(42 * kNumFrames);

      for (int f = 0; f < kNumFrames; ++f) {
        franka::Frame frame = static_cast<franka::Frame>(f);
        std::array<double, 16> pose = model.pose(frame, sample.q, cfg.F_T_EE, cfg.EE_T_K);
        std::array<double, 42> body =
            model.bodyJacobian(frame, sample.q, cfg.F_T_EE, cfg.EE_T_K);
        std::array<double, 42> zero =
            model.zeroJacobian(frame, sample.q, cfg.F_T_EE, cfg.EE_T_K);
        poses.insert(poses.end(), pose.begin(), pose.end());
        body_jacobians.insert(body_jacobians.end(), body.begin(), body.end());
        zero_jacobians.insert(zero_jacobians.end(), zero.begin(), zero.end());
      }

      std::fputs(",\"pose\":", out);
      writeVector(out, poses);
      std::fputs(",\"body_jacobian\":", out);
      writeVector(out, body_jacobians);
      std::fputs(",\"zero_jacobian\":", out);
      writeVector(out, zero_jacobians);
      std::fputs(",\"mass\":", out);
      writeArray(out, mass);
      std::fputs(",\"coriolis\":", out);
      writeArray(out, coriolis);
      std::fputs(",\"gravity\":", out);
      writeArray(out, gravity);
      std::fputs(",\"gravity_alt\":", out);
      writeArray(out, gravity_alt);
      std::fputc('}', out);
    }
    std::fputs("]}", out);
  }
  std::fputs("\n]\n}\n", out);
  std::fclose(out);

  std::cerr << "wrote " << output_path << " (" << samples.size() << " samples x "
            << configs.size() << " load configurations)\n";
  std::cerr << "urdf sha256: " << urdf_sha256 << "\n";
  return 0;
}
