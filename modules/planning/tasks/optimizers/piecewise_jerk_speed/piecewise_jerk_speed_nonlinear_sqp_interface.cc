#include "piecewise_jerk_speed_nonlinear_sqp_interface.h"

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

PiecewiseJerkSpeedNLP::PiecewiseJerkSpeedNLP(
    const double s_init, const double s_dot_init, const double s_ddot_init,
    const double delta_t, const int num_of_points, const double s_max,
    const double s_dot_max, const double s_ddot_min, const double s_ddot_max,
    const double s_dddot_min, const double s_dddot_max)
    : curvature_curve_(0.0, 0.0, 0.0),
      v_bound_func_(0.0, 0.0, 0.0),
      s_init_(s_init),
      s_dot_init_(s_dot_init),
      s_ddot_init_(s_ddot_init),
      delta_t_(delta_t),
      num_of_points_(num_of_points),
      s_max_(s_max),
      s_dot_max_(s_dot_max),
      s_ddot_min_(-std::abs(s_ddot_min)),
      s_ddot_max_(s_ddot_max),
      s_dddot_min_(-std::abs(s_dddot_min)),
      s_dddot_max_(s_dddot_max),
      v_offset_(num_of_points),
      a_offset_(num_of_points * 2) {}

void PiecewiseJerkSpeedNLP::set_warm_start(
    const std::vector<std::vector<double>> &speed_profile) {
  x_warm_start_ = speed_profile;
}

void PiecewiseJerkSpeedNLP::set_curvature_curve(
    const PiecewiseJerkTrajectory1d &curvature_curve) {
  curvature_curve_ = curvature_curve;
}

void PiecewiseJerkSpeedNLP::get_optimization_results(
    std::vector<double> *ptr_opt_s, std::vector<double> *ptr_opt_v,
    std::vector<double> *ptr_opt_a) {
  *ptr_opt_s = opt_s_;
  *ptr_opt_v = opt_v_;
  *ptr_opt_a = opt_a_;
}

void PiecewiseJerkSpeedNLP::set_end_state_target(const double s_target,
                                                 const double v_target,
                                                 const double a_target) {
  s_target_ = s_target;
  v_target_ = v_target;
  a_target_ = a_target;
  has_end_state_target_ = true;
}

void PiecewiseJerkSpeedNLP::set_reference_speed(const double v_ref) {
  v_ref_ = v_ref;
}

void PiecewiseJerkSpeedNLP::set_reference_spatial_distance(
    const std::vector<double> &s_ref) {
  s_ref_ = s_ref;
}

void PiecewiseJerkSpeedNLP::set_speed_limit_curve(
    const PiecewiseJerkTrajectory1d &v_bound_f) {
  v_bound_func_ = v_bound_f;
  use_v_bound_ = true;
}

void PiecewiseJerkSpeedNLP::set_safety_bounds(
    const std::vector<std::pair<double, double>> &safety_bounds) {
  safety_bounds_ = safety_bounds;
}

void PiecewiseJerkSpeedNLP::set_soft_safety_bounds(
    const std::vector<std::pair<double, double>> &soft_safety_bounds) {
  soft_safety_bounds_ = soft_safety_bounds;
  use_soft_safety_bound_ = true;
}

void PiecewiseJerkSpeedNLP::set_w_target_state(const double w_target_s,
                                               const double w_target_v,
                                               const double w_target_a) {
  w_target_s_ = w_target_s;
  w_target_v_ = w_target_v;
  w_target_a_ = w_target_a;
}

void PiecewiseJerkSpeedNLP::set_w_overall_a(const double w_overall_a) {
  w_overall_a_ = w_overall_a;
}

void PiecewiseJerkSpeedNLP::set_w_overall_j(const double w_overall_j) {
  w_overall_j_ = w_overall_j;
}

void PiecewiseJerkSpeedNLP::set_w_overall_centripetal_acc(
    const double w_overall_centripetal_acc) {
  w_overall_centripetal_acc_ = w_overall_centripetal_acc;
}

void PiecewiseJerkSpeedNLP::set_w_reference_speed(
    const double w_reference_speed) {
  w_ref_v_ = w_reference_speed;
}

void PiecewiseJerkSpeedNLP::set_w_reference_spatial_distance(
    const double w_ref_s) {
  w_ref_s_ = w_ref_s;
}

void PiecewiseJerkSpeedNLP::set_w_soft_s_bound(const double w_soft_s_bound) {
  w_soft_s_bound_ = w_soft_s_bound;
}

void PiecewiseJerkSpeedNLP::get_optimization_results(
    const Vector &x, std::vector<double> *ptr_opt_s,
    std::vector<double> *ptr_opt_v, std::vector<double> *ptr_opt_a) {
  ptr_opt_s->resize(num_of_points_);
  ptr_opt_v->resize(num_of_points_);
  ptr_opt_v->resize(num_of_points_);
  for (int i = 0; i < num_of_points_; i++) {
    ptr_opt_s->at(i) = x[i];
    ptr_opt_v->at(i) = x[v_offset_ + i];
    ptr_opt_a->at(i) = x[a_offset_ + i];
  }
}

void PiecewiseJerkSpeedNLP::init_nlp() {
  num_var_ = num_of_points_ * 3;

  if (use_soft_safety_bound_) {
    // complementary slack variable for soft lower and upper s bound
    num_var_ += num_of_points_ * 2;

    lower_s_slack_offset_ = num_of_points_ * 3;
    upper_s_slack_offset_ = num_of_points_ * 4;
  }

  // var bound constraints
  num_constr_ = num_var_;

  // s monotone constraints s_i+1 - s_i >= 0
  num_constr_ += num_of_points_ - 1;

  // jerk bound constraints
  // |s_ddot_i+1 - s_ddot_i| / delta_t <= s_dddot_max
  num_constr_ += num_of_points_ - 1;

  // position equality constraints
  // s_i+1 - s_i - s_dot_i * delta_t - 1/3 * s_ddot_i * delta_t^2 - 1/6 *
  // s_ddot_i+1 * delta_t^2
  num_constr_ += num_of_points_ - 1;

  // velocity equality constraints
  // s_dot_i+1 - s_dot_i - 0.5 * s_ddot_i * delta_t - 0.5 * s_ddot_i+1 * delta_t
  num_constr_ += num_of_points_ - 1;

  if (use_v_bound_) {
    // speed limit constraints
    // s_dot_i - v_bound_func_(s_i) <= 0
    num_constr_ += num_of_points_;
  }

  if (use_soft_safety_bound_) {
    // soft safety boundary constraints
    // s_i - soft_lower_s_i + lower_slack_i >= 0.0
    num_constr_ += num_of_points_;

    // s_i - soft_upper_s_i - upper_slack_i <= 0.0
    num_constr_ += num_of_points_;
  }

  AINFO << "number of point is: " << num_of_points_;
  AINFO << "number of var is: " << num_var_;
  AINFO << "number of constraints is: " << num_constr_;
}

void PiecewiseJerkSpeedNLP::get_starting_point(Vector &x0, Vector &y0) {
  // x0
  x0.resize(num_var_);

  if (!x_warm_start_.empty()) {
    for (int i = 0; i < num_of_points_; i++) {
      x0[i] = x_warm_start_[i][0];
      x0[v_offset_ + i] = x_warm_start_[i][1];
      x0[a_offset_ + i] = x_warm_start_[i][2];
    }
  }

  if (use_soft_safety_bound_) {
    for (int i = 0; i < num_of_points_; i++) {
      x0[lower_s_slack_offset_ + i] = 0.0;
      x0[upper_s_slack_offset_ + i] = 0.0;
    }
  }

  // y0
  y0.resize(num_constr_);
  for (int i = 0; i < num_constr_; i++) {
    y0[i] = 0.0;
  }
}

void PiecewiseJerkSpeedNLP::objective(const Vector &x, Scalar &obj) {
  obj = 0;
  // difference between ref spatial distance
  for (int i = 0; i < num_of_points_; i++) {
    double s_diff = x[i] - s_ref_[i];
    obj += s_diff * s_diff * w_ref_s_;
  }

  // differnce between ref speed
  for (int i = 0; i < num_of_points_; i++) {
    double v_diff = x[v_offset_ + i] - v_ref_;
    obj += v_diff * v_diff * w_ref_v_;
  }

  // acceleration obj
  for (int i = 0; i < num_of_points_; i++) {
    double a = x[a_offset_ + i];
    obj += a * a * w_overall_a_;
  }

  // jerk obj
  for (int i = 0; i + 1 < num_of_points_; i++) {
    double jerk = (x[a_offset_ + i + 1] - x[a_offset_ + i]) / delta_t_;
    obj += jerk * jerk * w_overall_j_;
  }

  // centripetal acceleration obj
  for (int i = 0; i < num_of_points_; i++) {
    double v = x[v_offset_ + i];
    double s = x[i];
    double kappa = curvature_curve_.Evaluate(0, s);
    double a_lat = v * v * kappa;
    obj += a_lat * a_lat * w_overall_centripetal_acc_;
  }

  if (has_end_state_target_) {
    double s_diff = x[num_of_points_ - 1] - s_target_;
    obj += s_diff * s_diff * w_target_s_;

    double v_diff = x[v_offset_ + num_of_points_ - 1] - v_target_;
    obj += v_diff * v_diff * w_target_v_;

    double a_diff = x[a_offset_ + num_of_points_ - 1] - a_target_;
    obj += a_diff * a_diff * w_target_a_;
  }

  if (use_soft_safety_bound_) {
    for (int i = 0; i < num_of_points_; i++) {
      obj += x[lower_s_slack_offset_ + i] * w_soft_s_bound_;
      obj += x[upper_s_slack_offset_ + i] * w_soft_s_bound_;
    }
  }
}

void PiecewiseJerkSpeedNLP::grad_f(const Vector &x, Vector &grad) {
  // init grad
  grad.resize(num_var_);
  for (int i = 0; i < num_var_; i++) {
    grad[i] = 0;
  }

  // reference spatial distance objective
  for (int i = 0; i < num_of_points_; i++) {
    double s_diff = x[i] - s_ref_[i];
    grad[i] += 2.0 * s_diff * w_ref_s_;
  }

  // reference speed objective
  for (int i = 0; i < num_of_points_; i++) {
    double v_diff = x[v_offset_ + i] - v_ref_;
    grad[v_offset_ + i] += 2.0 * v_diff * w_ref_v_;
  }

  // acc objective
  for (int i = 0; i < num_of_points_; i++) {
    double a = x[a_offset_ + i];
    grad[a_offset_ + i] += 2.0 * a * w_overall_a_;
  }

  // jerk objective
  double c = 2.0 / (delta_t_ * delta_t_) * w_overall_j_;
  grad[a_offset_] += -c * (x[a_offset_ + 1] - x[a_offset_]);
  for (int i = 1; i + i < num_of_points_; i++) {
    double a_pre = x[a_offset_ + i - 1];
    double a = x[a_offset_ + i];
    double a_nex = x[a_offset_ + i + 1];
    grad[a_offset_ + i] += c * (2.0 * a - a_pre - a_nex);
  }
  grad[a_offset_ + num_of_points_ - 1] +=
      c *
      (x[a_offset_ + num_of_points_ - 1] - x[a_offset_ + num_of_points_ - 2]);

  // lateral acceleration objective
  for (int i = 0; i < num_of_points_; i++) {
    double v = x[v_offset_ + i];
    double v2 = v * v;
    double v3 = v2 * v;
    double v4 = v3 * v;

    double s = x[i];
    double kappa = curvature_curve_.Evaluate(0, s);
    double kappa_dot = curvature_curve_.Evaluate(1, s);

    grad[i] += 2.0 * w_overall_centripetal_acc_ * v4 * kappa * kappa_dot;
    grad[v_offset_ + i] +=
        4.0 * w_overall_centripetal_acc_ * v3 * kappa * kappa;
  }

  if (has_end_state_target_) {
    double s_diff = x[num_of_points_ - 1] - s_target_;
    grad[num_of_points_ - 1] += 2.0 * s_diff * w_target_s_;

    double v_diff = x[v_offset_ + num_of_points_ - 1] - v_target_;
    grad[v_offset_ + num_of_points_ - 1] += 2.0 * v_diff * w_target_v_;

    double a_diff = x[a_offset_ + num_of_points_ - 1] - a_target_;
    grad[a_offset_ + num_of_points_ - 1] += 2.0 * a_diff * w_target_a_;
  }

  if (use_soft_safety_bound_) {
    for (int i = 0; i < num_of_points_; ++i) {
      grad[lower_s_slack_offset_ + i] += w_soft_s_bound_;
      grad[upper_s_slack_offset_ + i] += w_soft_s_bound_;
    }
  }
}

void PiecewiseJerkSpeedNLP::objective_linearized(const Vector &x, Vector &grad,
                                                 Scalar &obj) {
  objective(x, obj);
  grad_f(x, grad);
}

void PiecewiseJerkSpeedNLP::constraint(const Vector &x, Vector &c, Vector &l,
                                       Vector &u) {
  double INF = 1.0e19;
  // init
  c.resize(num_constr_);
  l.resize(num_constr_);
  u.resize(num_constr_);
  int offset = 0;

  // var bounds constraints
  // s
  int s_bound_offset = offset;
  c[0] = x[0];
  l[0] = s_init_;
  u[0] = s_init_;
  for (int i = 1; i < num_of_points_; i++) {
    c[s_bound_offset + i] = x[i];
    l[s_bound_offset + i] = safety_bounds_[i].first;
    u[s_bound_offset + i] = safety_bounds_[i].second;
  }
  offset += num_of_points_;

  // s_dot
  int s_dot_bound_offset = offset;
  c[s_dot_bound_offset] = x[v_offset_];
  l[s_dot_bound_offset] = s_dot_init_;
  u[s_dot_bound_offset] = s_dot_init_;
  for (int i = 1; i < num_of_points_; i++) {
    c[s_dot_bound_offset + i] = x[v_offset_ + i];
    l[s_dot_bound_offset + i] = 0.0;
    u[s_dot_bound_offset + i] = s_dot_max_;
  }
  offset += num_of_points_;

  // s_ddot
  int s_ddot_bound_offset = offset;
  c[s_ddot_bound_offset] = x[a_offset_];
  l[s_ddot_bound_offset] = s_ddot_init_;
  u[s_ddot_bound_offset] = s_ddot_init_;
  for (int i = 1; i < num_of_points_; i++) {
    c[s_ddot_bound_offset + i] = x[a_offset_ + i];
    l[s_ddot_bound_offset + i] = s_ddot_min_;
    u[s_ddot_bound_offset + i] = s_ddot_max_;
  }
  offset += num_of_points_;

  if (use_soft_safety_bound_) {
    // lower_s_slack
    int lower_s_slack_bound_offset = offset;
    for (int i = 0; i < num_of_points_; i++) {
      c[lower_s_slack_bound_offset + i] = x[lower_s_slack_offset_ + i];
      l[lower_s_slack_bound_offset + i] = 0.0;
      u[lower_s_slack_bound_offset + i] = INF;
    }
    offset += num_of_points_;

    // upper_s_slack
    int upper_s_slack_bound_offset = offset;
    for (int i = 0; i < num_of_points_; i++) {
      c[upper_s_slack_bound_offset + i] = x[upper_s_slack_offset_ + i];
      l[upper_s_slack_bound_offset + i] = 0.0;
      u[upper_s_slack_bound_offset + i] = INF;
    }
    offset += num_of_points_;
  }

  int s_mono_offset = offset;
  offset += num_of_points_ - 1;

  int jerk_bound_offset = offset;
  offset += num_of_points_ - 1;

  int pos_equal_offset = offset;
  offset += num_of_points_ - 1;

  int vel_equal_offset = offset;
  offset += num_of_points_ - 1;

  double t = delta_t_;
  double t2 = t * t;
  double t3 = t2 * t;

  for (int i = 0; i + 1 < num_of_points_; ++i) {
    double s0 = x[i];
    double s1 = x[i + 1];

    double v0 = x[v_offset_ + i];
    double v1 = x[v_offset_ + i + 1];

    double a0 = x[a_offset_ + i];
    double a1 = x[a_offset_ + i + 1];

    double j = (a1 - a0) / t;

    // s monotone constraints s_i+1 - s_i >= 0
    c[s_mono_offset + i] = s1 - s0;
    l[s_mono_offset + i] = 0;
    u[s_mono_offset + i] = s_dot_max_ * t;

    // jerk bound constraints
    // |s_ddot_i+1 - s_ddot_i| / delta_t <= s_dddot_max
    c[jerk_bound_offset + i] = j;
    l[jerk_bound_offset + i] = s_dddot_min_;
    u[jerk_bound_offset + i] = s_dddot_max_;

    // position equality constraints
    // s_i+1 = s_i + s_dot_i * delta_t + 0.5 * s_ddot_i * delta_t^2 + 1/6 *
    // s_dddot_i * delta_t^3 or s_i+1 - s_i - s_dot_i * delta_t - 1/3 * s_ddot_i
    // * delta_t^2 - 1/6 * s_ddot_i+1 * delta_t^2 = 0
    c[pos_equal_offset + i] =
        s1 - (s0 + v0 * t + 0.5 * a0 * t2 + 1.0 / 6.0 * j * t3);
    l[pos_equal_offset + i] = 0.0;
    u[pos_equal_offset + i] = 0.0;

    // velocity equality constraints
    // s_dot_i+1 - s_dot_i - 0.5 * s_ddot_i * delta_t - 0.5 * s_ddot_i+1 *
    // delta_t
    c[vel_equal_offset + i] = v1 - (v0 + a0 * t + 0.5 * j * t2);
    l[vel_equal_offset + i] = 0.0;
    u[vel_equal_offset + i] = 0.0;
  }

  if (use_v_bound_) {
    // speed limit constraints
    // s_dot_i - v_bound_func_(s_i) <= 0
    int speed_limit_offset = offset;
    for (int i = 0; i < num_of_points_; i++) {
      double s = x[i];
      double s_dot = x[v_offset_ + i];
      c[speed_limit_offset + i] = s_dot - v_bound_func_.Evaluate(0, s);
      l[speed_limit_offset + i] = -INF;
      u[speed_limit_offset + i] = 0.0;
    }
    offset += num_of_points_;
  }

  if (use_soft_safety_bound_) {
    // soft safety boundary constraints
    // s_i - soft_lower_s_i + lower_slack_i >= 0.0
    int soft_lower_s_offset = offset;
    for (int i = 0; i < num_of_points_; i++) {
      double s = x[i];
      double lower_s_slack = x[lower_s_slack_offset_ + i];
      c[soft_lower_s_offset + i] =
          s - soft_safety_bounds_[i].first + lower_s_slack;
      l[soft_lower_s_offset + i] = 0.0;
      u[soft_lower_s_offset + i] = INF;
    }
    offset += num_of_points_;

    // s_i - soft_upper_s_i - upper_slack_i <= 0.0
    int soft_upper_s_offset = offset;
    for (int i = 0; i < num_of_points_; i++) {
      double s = x[i];
      double upper_s_slack = x[upper_s_slack_offset_ + i];
      c[soft_upper_s_offset + i] =
          s - soft_safety_bounds_[i].second - upper_s_slack;
      l[soft_upper_s_offset + i] = -INF;
      u[soft_upper_s_offset + i] = 0.0;
    }
    offset += num_of_points_;
  }
}

void PiecewiseJerkSpeedNLP::jac_g(const Vector &x, Matrix &Jc) {
  // init
  // TODO Jc init
  Jc.resize(num_constr_, num_var_);
  for (int i = 0; i < num_constr_; i++) {
    for (int j = 0; j < num_var_; j++) {
      Jc(i, j) = 0.0;
    }
  }
  int offset = 0;

  // var bounds constraints
  // s
  int s_bound_offset = offset;
  for (int i = 0; i < num_of_points_; i++) {
    Jc(s_bound_offset + i, i) = 1.0;
  }
  offset += num_of_points_;

  // s_dot
  int s_dot_bound_offset = offset;
  for (int i = 0; i < num_of_points_; i++) {
    Jc(s_dot_bound_offset + i, v_offset_ + i) = 1.0;
  }
  offset += num_of_points_;

  // s_ddot
  int s_ddot_bound_offset = offset;
  for (int i = 0; i < num_of_points_; i++) {
    Jc(s_ddot_bound_offset + i, a_offset_ + i) = 1.0;
  }
  offset += num_of_points_;

  if (use_soft_safety_bound_) {
    // lower_s_slack
    int lower_s_slack_bound_offset = offset;
    for (int i = 0; i < num_of_points_; i++) {
      Jc(lower_s_slack_bound_offset + i, lower_s_slack_offset_ + i) = 1.0;
    }
    offset += num_of_points_;

    // upper_s_slack
    int upper_s_slack_bound_offset = offset;
    for (int i = 0; i < num_of_points_; i++) {
      Jc(upper_s_slack_bound_offset + i, upper_s_slack_offset_ + i) = 1.0;
    }
    offset += num_of_points_;
  }

  // s monotone constraints
  // s_i+1 - s_i
  int s_mono_offset = offset;
  for (int i = 0; i + 1 < num_of_points_; i++) {
    Jc(s_mono_offset + i, i) = -1.0;
    Jc(s_mono_offset + i, i + 1) = 1.0;
  }
  offset += num_of_points_ - 1;

  // jerk bound constraints
  // |s_ddot_i+1 - s_ddot_i| / delta_t <= s_dddot_max
  int jerk_bound_offset = offset;
  for (int i = 0; i + 1 < num_of_points_; i++) {
    Jc(jerk_bound_offset + i, a_offset_ + i) = -1.0;
    Jc(jerk_bound_offset + i, a_offset_ + i + 1) = 1.0;
  }
  offset += num_of_points_ - 1;

  // position equality constraints
  // s_i+1 = s_i + s_dot_i * delta_t + 0.5 * s_ddot_i * delta_t^2 + 1/6 *
  // s_dddot_i * delta_t^3 or s_i+1 - s_i - s_dot_i * delta_t - 1/3 * s_ddot_i
  // * delta_t^2 - 1/6 * s_ddot_i+1 * delta_t^2 = 0
  int pos_equal_offset = offset;
  double t = delta_t_;
  double t2 = t * t;
  for (int i = 0; i + 1 < num_of_points_; i++) {
    Jc(pos_equal_offset + i, i) = -1.0;
    Jc(pos_equal_offset + i, i + 1) = 1.0;
    Jc(pos_equal_offset + i, v_offset_ + i) = -t;
    Jc(pos_equal_offset + i, a_offset_ + i) = -1.0 / 3.0 * t2;
    Jc(pos_equal_offset + i, a_offset_ + i + 1) = -1.0 / 6.0 * t2;
  }
  offset += num_of_points_ - 1;

  // velocity equality constraints
  // s_dot_i+1 - s_dot_i - 0.5 * s_ddot_i * delta_t - 0.5 * s_ddot_i+1 *
  // delta_t
  int vel_equal_offset = offset;
  for (int i = 0; i + 1 < num_of_points_; i++) {
    Jc(vel_equal_offset + i, v_offset_ + i) = -1.0;
    Jc(vel_equal_offset + i, v_offset_ + i + 1) = 1.0;
    Jc(vel_equal_offset + i, a_offset_ + i) = -0.5 * t;
    Jc(vel_equal_offset + i, a_offset_ + i + 1) = -0.5 * t;
  }
  offset += num_of_points_ - 1;

  if (use_v_bound_) {
    // speed limit constraints
    // s_dot_i - v_bound_func_(s_i) <= 0
    int v_bound_offset = offset;
    for (int i = 0; i < num_of_points_; i++) {
      double s = x[i];
      Jc(v_bound_offset + i, i) = -1.0 * v_bound_func_.Evaluate(1, s);
      Jc(v_bound_offset + i, v_offset_ + i) = 1.0;
    }
    offset += num_of_points_;
  }

  if (use_soft_safety_bound_) {
    // soft safety boundary constraints
    // s_i - soft_lower_s_i + lower_slack_i >= 0.0
    for (int i = 0; i < num_of_points_; i++) {
      Jc(offset + i, i) = 1.0;
      Jc(offset + i, lower_s_slack_offset_ + i) = 1.0;
    }
    offset += num_of_points_;

    // s_i - soft_upper_s_i - upper_slack_i <= 0.0
    for (int i = 0; i < num_of_points_; i++) {
      Jc(offset + i, i) = 1.0;
      Jc(offset + i, upper_s_slack_offset_ + i) = -1.0;
    }
    offset += num_of_points_;
  }
}

void PiecewiseJerkSpeedNLP::constraint_linearized(const Vector &x, Matrix &Jc,
                                                  Vector &c, Vector &l,
                                                  Vector &u) {
  constraint(x, c, l, u);
  jac_g(x, Jc);
}

}  // namespace planning
}  // namespace apollo