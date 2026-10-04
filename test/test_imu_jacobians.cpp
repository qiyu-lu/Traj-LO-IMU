/**
 * Numeric verification of the IMU preintegration factor:
 *  1. analytic Jacobians vs central finite differences on random states
 *  2. noise-free consistency: residual of a predicted state is ~0
 *  3. stationary case: gravity handling keeps a standing state in place
 *
 * Exit code 0 iff all checks pass (used as the M3 gate).
 */

#include <trajlo/core/imu_preintegration.h>

#include <iostream>
#include <random>

using namespace traj;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

static std::mt19937 rng(42);

static double urand(double lo, double hi) {
  return std::uniform_real_distribution<double>(lo, hi)(rng);
}

static Vec3 vrand(double scale) {
  return Vec3(urand(-scale, scale), urand(-scale, scale), urand(-scale, scale));
}

int main() {
  int failures = 0;
  const Vec3 g_w(0.13, -0.27, -9.79);  // deliberately not axis-aligned

  // ---------------------------------------------------------------- test 1
  // analytic vs numeric Jacobians
  for (int trial = 0; trial < 20; trial++) {
    const Vec3 bg_lin = vrand(0.02);
    const Vec3 ba_lin = vrand(0.2);
    ImuPreintegration pre(bg_lin, ba_lin, 1e-3, 1e-2);

    // ~10 samples at 500 Hz (one 40ms segment, slightly jittered stamps)
    int64_t t = 1000000000;
    Vec3 gyr = vrand(1.5);
    Vec3 acc = vrand(3.0) - g_w;
    for (int k = 0; k < 11; k++) {
      ImuData d;
      d.t_ns = t;
      d.gyr = gyr;
      d.acc = acc;
      pre.integrate(d);
      t += (int64_t)(2e6 + urand(-2e5, 2e5));
      gyr += vrand(0.1);
      acc += vrand(0.2);
    }

    PoseVelBiasState<double> s0(
        900000000, Sophus::SE3d(Sophus::SO3d::exp(vrand(1.0)), vrand(2.0)),
        vrand(1.0), bg_lin + vrand(1e-3), ba_lin + vrand(1e-2));
    PoseVelBiasState<double> s1(
        t, Sophus::SE3d(Sophus::SO3d::exp(vrand(1.0)), vrand(2.0)), vrand(1.0),
        s0.bias_gyro, s0.bias_accel);

    Eigen::Matrix<double, 9, 1> res;
    Eigen::Matrix<double, 9, 15> J0, J1;
    pre.evaluate(s0, s1, g_w, res, &J0, &J1);

    const double eps = 1e-6;
    for (int side = 0; side < 2; side++) {
      const auto& J = side == 0 ? J0 : J1;
      for (int k = 0; k < 15; k++) {
        Eigen::Matrix<double, 15, 1> inc;

        inc.setZero();
        inc(k) = eps;
        PoseVelBiasState<double> sp = side == 0 ? s0 : s1;
        sp.applyInc(inc);
        Eigen::Matrix<double, 9, 1> res_p;
        side == 0 ? pre.evaluate(sp, s1, g_w, res_p)
                  : pre.evaluate(s0, sp, g_w, res_p);

        inc(k) = -eps;
        PoseVelBiasState<double> sm = side == 0 ? s0 : s1;
        sm.applyInc(inc);
        Eigen::Matrix<double, 9, 1> res_m;
        side == 0 ? pre.evaluate(sm, s1, g_w, res_m)
                  : pre.evaluate(s0, sm, g_w, res_m);

        const Eigen::Matrix<double, 9, 1> fd = (res_p - res_m) / (2 * eps);
        const double err = (fd - J.col(k)).cwiseAbs().maxCoeff();
        // the bias->rotation column uses the standard first-order
        // approximation, allow a looser tolerance there
        const double tol = (side == 0 && k >= 9 && k < 12) ? 5e-4 : 1e-5;
        if (err > tol) {
          std::cout << "[FAIL] trial " << trial << " J" << side << " col " << k
                    << " max err " << err << std::endl;
          failures++;
        }
      }
    }
  }
  std::cout << "test 1 (analytic vs numeric Jacobians): "
            << (failures == 0 ? "PASS" : "FAIL") << std::endl;

  // ---------------------------------------------------------------- test 2
  // noise-free consistency: integrate ground-truth dynamics with the same
  // midpoint scheme, build s1 from truth, residual must vanish
  {
    const Vec3 bg_true = vrand(0.02);
    const Vec3 ba_true = vrand(0.2);
    ImuPreintegration pre(bg_true, ba_true, 1e-3, 1e-2);

    Sophus::SO3d R = Sophus::SO3d::exp(vrand(1.0));
    Vec3 p = vrand(2.0), v = vrand(1.0);
    PoseVelBiasState<double> s0(0, Sophus::SE3d(R, p), v, bg_true, ba_true);

    int64_t t = 0;
    ImuData prev;
    bool first = true;
    for (int k = 0; k <= 20; k++) {
      const Vec3 w_body = Vec3(0.8 * sin(k * 0.3), -0.5, 0.3 * cos(k * 0.2));
      const Vec3 a_world = Vec3(0.5, 1.2 * sin(k * 0.4), -0.3);
      ImuData d;
      d.t_ns = t;
      d.gyr = w_body + bg_true;
      d.acc = R.inverse() * (a_world - g_w) + ba_true;
      pre.integrate(d);

      if (!first) {
        // advance ground truth with the same midpoint rule
        const double dt = (d.t_ns - prev.t_ns) * 1e-9;
        const Vec3 w_mid = 0.5 * (prev.gyr + d.gyr) - bg_true;
        const Vec3 a_mid = 0.5 * (prev.acc + d.acc) - ba_true;
        const Vec3 a_w = R * a_mid + g_w;
        p += v * dt + 0.5 * a_w * dt * dt;
        v += a_w * dt;
        R = R * Sophus::SO3d::exp(w_mid * dt);
      }
      prev = d;
      first = false;
      t += 2000000;
    }

    PoseVelBiasState<double> s1(t, Sophus::SE3d(R, p), v, bg_true, ba_true);
    Eigen::Matrix<double, 9, 1> res;
    pre.evaluate(s0, s1, g_w, res);
    const double err = res.cwiseAbs().maxCoeff();
    std::cout << "test 2 (noise-free consistency): residual " << err << " "
              << (err < 1e-9 ? "PASS" : "FAIL") << std::endl;
    if (err >= 1e-9) failures++;

    // predict() must agree with the ground truth as well
    const auto s1_pred = pre.predict(s0, g_w, t);
    const double perr =
        (s1_pred.T_w_i.translation() - p).norm() +
        (s1_pred.vel_w_i - v).norm() +
        (s1_pred.T_w_i.so3().inverse() * R).log().norm();
    std::cout << "test 2b (predict consistency): err " << perr << " "
              << (perr < 1e-9 ? "PASS" : "FAIL") << std::endl;
    if (perr >= 1e-9) failures++;
  }

  // ---------------------------------------------------------------- test 3
  // stationary: gravity-aligned standstill stays in place
  {
    const Vec3 bg_true(0.001, -0.002, 0.0005);
    ImuPreintegration pre(bg_true, Vec3::Zero(), 1e-3, 1e-2);
    Sophus::SO3d R = Sophus::SO3d::exp(vrand(0.5));
    PoseVelBiasState<double> s0(0, Sophus::SE3d(R, Vec3::Zero()),
                                Vec3::Zero(), bg_true, Vec3::Zero());
    int64_t t = 0;
    for (int k = 0; k <= 50; k++) {
      ImuData d;
      d.t_ns = t;
      d.gyr = bg_true;
      d.acc = R.inverse() * (-g_w);
      pre.integrate(d);
      t += 2000000;
    }
    const auto s1 = pre.predict(s0, g_w, t);
    const double drift = s1.T_w_i.translation().norm() + s1.vel_w_i.norm() +
                         (s1.T_w_i.so3().inverse() * R).log().norm();
    std::cout << "test 3 (stationary drift): " << drift << " "
              << (drift < 1e-10 ? "PASS" : "FAIL") << std::endl;
    if (drift >= 1e-10) failures++;
  }

  std::cout << (failures == 0 ? "ALL PASS" : "FAILURES PRESENT") << std::endl;
  return failures == 0 ? 0 : 1;
}
