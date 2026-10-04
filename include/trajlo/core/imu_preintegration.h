/**
MIT License

Preintegrated IMU measurement between two trajectory-segment endpoints, plus
the 9-dim factor residual/Jacobians and a stand-still initializer.

Conventions (must match PoseState::incPose in pose_type.h):
  - rotation increment is applied on the RIGHT: R' = R * Exp(dphi)
  - translation/velocity increments are additive in the world frame
  - per-state increment layout: [dp(3), dphi(3), dv(3), dbg(3), dba(3)]
  - residual layout: [r_R(3), r_v(3), r_p(3)]

The preintegration follows Forster et al., "On-Manifold Preintegration for
Real-Time Visual-Inertial Odometry" (TRO 2017): midpoint measurement
integration, first-order bias-correction Jacobians (no re-integration inside
Gauss-Newton), and 9x9 covariance propagation in [phi, v, p] order.
*/

#ifndef TRAJLO_IMU_PREINTEGRATION_H
#define TRAJLO_IMU_PREINTEGRATION_H

#include <trajlo/utils/common_type.h>
#include <trajlo/utils/pose_type.h>
#include <trajlo/utils/sophus_utils.hpp>

namespace traj {

class ImuPreintegration {
 public:
  using Ptr = std::shared_ptr<ImuPreintegration>;
  using Mat3 = Eigen::Matrix3d;
  using Vec3 = Eigen::Vector3d;
  using Mat9 = Eigen::Matrix<double, 9, 9>;

  ImuPreintegration(const Vec3& bg_lin, const Vec3& ba_lin, double sigma_ng,
                    double sigma_na)
      : bg_lin_(bg_lin),
        ba_lin_(ba_lin),
        var_ng_(sigma_ng * sigma_ng),
        var_na_(sigma_na * sigma_na) {
    delta_R_ = Sophus::SO3d();
    delta_v_.setZero();
    delta_p_.setZero();
    cov_.setZero();
    J_R_bg_.setZero();
    J_v_bg_.setZero();
    J_v_ba_.setZero();
    J_p_bg_.setZero();
    J_p_ba_.setZero();
    dt_total_ = 0;
    has_last_ = false;
  }

  /// Feed one sample (body/lidar frame, absolute time). The first call only
  /// records the sample; each following call integrates the midpoint of the
  /// previous and current measurement over the elapsed interval.
  void integrate(const ImuData& d) {
    if (!has_last_) {
      last_ = d;
      has_last_ = true;
      return;
    }
    const double dt = (d.t_ns - last_.t_ns) * 1e-9;
    if (dt <= 0) {
      last_ = d;
      return;
    }

    const Vec3 w = 0.5 * (last_.gyr + d.gyr) - bg_lin_;
    const Vec3 a = 0.5 * (last_.acc + d.acc) - ba_lin_;

    const Mat3 dR = Sophus::SO3d::exp(w * dt).matrix();
    Mat3 Jr;
    Sophus::rightJacobianSO3(Vec3(w * dt), Jr);

    const Mat3 R_k = delta_R_.matrix();  // pre-update DeltaR
    const Mat3 a_hat = Sophus::SO3d::hat(a);

    // covariance propagation, order [phi, v, p]
    Mat9 A = Mat9::Identity();
    A.block<3, 3>(0, 0) = dR.transpose();
    A.block<3, 3>(3, 0) = -R_k * a_hat * dt;
    A.block<3, 3>(6, 0) = -0.5 * R_k * a_hat * dt * dt;
    A.block<3, 3>(6, 3) = Mat3::Identity() * dt;

    Eigen::Matrix<double, 9, 6> B;
    B.setZero();
    B.block<3, 3>(0, 0) = Jr * dt;
    B.block<3, 3>(3, 3) = R_k * dt;
    B.block<3, 3>(6, 3) = 0.5 * R_k * dt * dt;

    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    Q.topLeftCorner<3, 3>() = Mat3::Identity() * (var_ng_ / dt);
    Q.bottomRightCorner<3, 3>() = Mat3::Identity() * (var_na_ / dt);

    cov_ = A * cov_ * A.transpose() + B * Q * B.transpose();

    // bias jacobians (p before v before R: each uses pre-update values)
    J_p_bg_ += J_v_bg_ * dt - 0.5 * R_k * a_hat * J_R_bg_ * dt * dt;
    J_p_ba_ += J_v_ba_ * dt - 0.5 * R_k * dt * dt;
    J_v_bg_ += -R_k * a_hat * J_R_bg_ * dt;
    J_v_ba_ += -R_k * dt;
    J_R_bg_ = dR.transpose() * J_R_bg_ - Jr * dt;

    // deltas (p before v before R)
    delta_p_ += delta_v_ * dt + 0.5 * R_k * a * dt * dt;
    delta_v_ += R_k * a * dt;
    delta_R_ = delta_R_ * Sophus::SO3d::exp(w * dt);

    dt_total_ += dt;
    last_ = d;
  }

  /// 9-dim residual and (optional) Jacobians of the preintegration factor
  /// linking states s0 -> s1. Gravity g_w is a fixed world-frame vector.
  /// Jacobian column layout per state: [dp, dphi, dv, dbg, dba].
  void evaluate(const PoseVelBiasState<double>& s0,
                const PoseVelBiasState<double>& s1, const Vec3& g_w,
                Eigen::Matrix<double, 9, 1>& res,
                Eigen::Matrix<double, 9, 15>* J0 = nullptr,
                Eigen::Matrix<double, 9, 15>* J1 = nullptr) const {
    const Mat3 R0 = s0.T_w_i.so3().matrix();
    const Mat3 R1 = s1.T_w_i.so3().matrix();
    const Vec3 p0 = s0.T_w_i.translation();
    const Vec3 p1 = s1.T_w_i.translation();
    const Vec3& v0 = s0.vel_w_i;
    const Vec3& v1 = s1.vel_w_i;
    const double dt = dt_total_;

    const Vec3 dbg = s0.bias_gyro - bg_lin_;
    const Vec3 dba = s0.bias_accel - ba_lin_;

    // bias-corrected deltas (first order)
    const Mat3 dR_corr =
        delta_R_.matrix() * Sophus::SO3d::exp(J_R_bg_ * dbg).matrix();
    const Vec3 dv_corr = delta_v_ + J_v_bg_ * dbg + J_v_ba_ * dba;
    const Vec3 dp_corr = delta_p_ + J_p_bg_ * dbg + J_p_ba_ * dba;

    const Mat3 E = dR_corr.transpose() * R0.transpose() * R1;
    const Vec3 r_R = Sophus::SO3d(Eigen::Quaterniond(E).normalized()).log();
    const Vec3 u_v = R0.transpose() * (v1 - v0 - g_w * dt);
    const Vec3 u_p =
        R0.transpose() * (p1 - p0 - v0 * dt - 0.5 * g_w * dt * dt);

    res.segment<3>(0) = r_R;
    res.segment<3>(3) = u_v - dv_corr;
    res.segment<3>(6) = u_p - dp_corr;

    if (J0 || J1) {
      Mat3 Jr_inv;
      Sophus::rightJacobianInvSO3(r_R, Jr_inv);

      if (J0) {
        J0->setZero();
        // r_R
        J0->block<3, 3>(0, 3) = -Jr_inv * R1.transpose() * R0;
        J0->block<3, 3>(0, 9) = -Jr_inv * E.transpose() * J_R_bg_;
        // r_v
        J0->block<3, 3>(3, 3) = Sophus::SO3d::hat(u_v);
        J0->block<3, 3>(3, 6) = -R0.transpose();
        J0->block<3, 3>(3, 9) = -J_v_bg_;
        J0->block<3, 3>(3, 12) = -J_v_ba_;
        // r_p
        J0->block<3, 3>(6, 0) = -R0.transpose();
        J0->block<3, 3>(6, 3) = Sophus::SO3d::hat(u_p);
        J0->block<3, 3>(6, 6) = -R0.transpose() * dt;
        J0->block<3, 3>(6, 9) = -J_p_bg_;
        J0->block<3, 3>(6, 12) = -J_p_ba_;
      }
      if (J1) {
        J1->setZero();
        J1->block<3, 3>(0, 3) = Jr_inv;
        J1->block<3, 3>(3, 6) = R0.transpose();
        J1->block<3, 3>(6, 0) = R0.transpose();
      }
    }
  }

  /// Information matrix of the factor (inverse of the propagated covariance,
  /// lightly regularized).
  Mat9 information() const {
    Mat9 reg = cov_;
    reg.diagonal().array() += 1e-12;
    return reg.inverse();
  }

  double dt_total() const { return dt_total_; }
  const Sophus::SO3d& delta_R() const { return delta_R_; }
  const Vec3& delta_v() const { return delta_v_; }
  const Vec3& delta_p() const { return delta_p_; }
  const Vec3& bg_lin() const { return bg_lin_; }
  const Vec3& ba_lin() const { return ba_lin_; }

  /// Predict state at the segment end from the state at the segment begin.
  PoseVelBiasState<double> predict(const PoseVelBiasState<double>& s0,
                                   const Vec3& g_w, int64_t t1_ns) const {
    const Mat3 R0 = s0.T_w_i.so3().matrix();
    const double dt = dt_total_;

    const Vec3 dbg = s0.bias_gyro - bg_lin_;
    const Vec3 dba = s0.bias_accel - ba_lin_;
    const Mat3 dR_corr =
        delta_R_.matrix() * Sophus::SO3d::exp(J_R_bg_ * dbg).matrix();
    const Vec3 dv_corr = delta_v_ + J_v_bg_ * dbg + J_v_ba_ * dba;
    const Vec3 dp_corr = delta_p_ + J_p_bg_ * dbg + J_p_ba_ * dba;

    PoseVelBiasState<double> s1;
    s1.t_ns = t1_ns;
    s1.T_w_i.so3() = Sophus::SO3d(Eigen::Quaterniond(R0 * dR_corr));
    s1.T_w_i.translation() = s0.T_w_i.translation() + s0.vel_w_i * dt +
                             0.5 * g_w * dt * dt + R0 * dp_corr;
    s1.vel_w_i = s0.vel_w_i + g_w * dt + R0 * dv_corr;
    s1.bias_gyro = s0.bias_gyro;
    s1.bias_accel = s0.bias_accel;
    return s1;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  Vec3 bg_lin_, ba_lin_;  // bias linearization point
  double var_ng_, var_na_;

  Sophus::SO3d delta_R_;
  Vec3 delta_v_, delta_p_;
  Mat9 cov_;
  Mat3 J_R_bg_, J_v_bg_, J_v_ba_, J_p_bg_, J_p_ba_;
  double dt_total_;

  ImuData last_;
  bool has_last_;
};

/// Stand-still initializer: running mean/variance of gyro and accel (samples
/// already expressed in the body/lidar frame). Gravity is aligned to the mean
/// specific force, gyro bias to the mean rate, accel bias to zero.
class StaticInitializer {
 public:
  using Vec3 = Eigen::Vector3d;

  void add(const ImuData& d) {
    if (n_ == 0) t_first_ns_ = d.t_ns;
    t_last_ns_ = d.t_ns;
    n_++;
    // Welford running statistics
    const Vec3 d_gyr = d.gyr - mean_gyr_;
    mean_gyr_ += d_gyr / n_;
    m2_gyr_ += d_gyr.cwiseProduct(d.gyr - mean_gyr_);
    const Vec3 d_acc = d.acc - mean_acc_;
    mean_acc_ += d_acc / n_;
    m2_acc_ += d_acc.cwiseProduct(d.acc - mean_acc_);
  }

  double span() const {
    return n_ < 2 ? 0.0 : (t_last_ns_ - t_first_ns_) * 1e-9;
  }
  size_t count() const { return n_; }

  bool ready(double init_time_s) const {
    return n_ >= 20 && span() >= init_time_s;
  }

  Vec3 gyro_bias() const { return mean_gyr_; }
  /// gravity with the sensor's own measured magnitude: consumer-grade
  /// accelerometers have percent-level scale error, and forcing 9.81 here
  /// pushes that error into the accel bias (and transients into vel/pos)
  Vec3 gravity() const { return -mean_acc_; }
  Vec3 gyro_std() const {
    return n_ < 2 ? Vec3(Vec3::Zero()) : Vec3((m2_gyr_ / (n_ - 1)).cwiseSqrt());
  }
  Vec3 acc_std() const {
    return n_ < 2 ? Vec3(Vec3::Zero()) : Vec3((m2_acc_ / (n_ - 1)).cwiseSqrt());
  }

 private:
  size_t n_ = 0;
  int64_t t_first_ns_ = 0, t_last_ns_ = 0;
  Vec3 mean_gyr_ = Vec3::Zero(), m2_gyr_ = Vec3::Zero();
  Vec3 mean_acc_ = Vec3::Zero(), m2_acc_ = Vec3::Zero();
};

}  // namespace traj

#endif  // TRAJLO_IMU_PREINTEGRATION_H
