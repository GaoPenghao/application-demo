#pragma once

#include "modules/planning/common/trajectory1d/piecewise_jerk_trajectory1d.h"
#include "modules/planning/math/sqp_method/sqp.h"

namespace apollo {
namespace planning {

using sqp::NonLinearProblem;

class PiecewiseJerkSpeedNLP : public NonLinearProblem<double> {
 public:
  using Vector = NonLinearProblem<double>::Vector;
  using Matrix = NonLinearProblem<double>::Matrix;

  const Scalar infinity = std::numeric_limits<Scalar>::infinity();

  PiecewiseJerkSpeedNLP(const double s_init, const double s_dot_init,
                        const double s_ddot_init, const double delta_t,
                        const int num_of_points, const double s_max,
                        const double s_dot_max, const double s_ddot_min,
                        const double s_ddot_max, const double s_dddot_min,
                        const double s_dddot_max);

  virtual ~PiecewiseJerkSpeedNLP() = default;

  void set_warm_start(const std::vector<std::vector<double>>& speed_profile);

  void set_curvature_curve(const PiecewiseJerkTrajectory1d& curvature_curve);

  void get_optimization_results(std::vector<double>* ptr_opt_s,
                                std::vector<double>* ptr_opt_v,
                                std::vector<double>* ptr_opt_a);

  void set_end_state_target(const double s_target, const double v_target,
                            const double a_target);

  void set_reference_speed(const double s_dot_ref);

  void set_reference_spatial_distance(const std::vector<double>& s_ref);

  void set_speed_limit_curve(const PiecewiseJerkTrajectory1d& v_bound_f);

  void set_safety_bounds(
      const std::vector<std::pair<double, double>>& safety_bounds);

  void set_soft_safety_bounds(
      const std::vector<std::pair<double, double>>& soft_safety_bounds);

  void set_w_target_state(const double w_target_s, const double w_target_v,
                          const double w_target_a);

  void set_w_overall_a(const double w_overall_a);

  void set_w_overall_j(const double w_overall_j);

  void set_w_overall_centripetal_acc(const double w_overall_centripetal_acc);

  void set_w_reference_speed(const double w_reference_speed);

  void set_w_reference_spatial_distance(const double w_ref_s);

  void set_w_soft_s_bound(const double w_soft_s_bound);

  void get_optimization_results(const Vector& x, std::vector<double>* ptr_opt_s,
                                std::vector<double>* ptr_opt_v,
                                std::vector<double>* ptr_opt_a);

  void init_nlp() override;

  void get_starting_point(Vector& x0, Vector& y0) override;

  void objective(const Vector& x, Scalar& obj) override;

  void grad_f(const Vector& x, Vector& grad) override;

  void objective_linearized(const Vector& x, Vector& grad,
                            Scalar& obj) override;

  void constraint(const Vector& x, Vector& c, Vector& l, Vector& u) override;

  void jac_g(const Vector& x, Matrix& Jc) override;

  void constraint_linearized(const Vector& x, Matrix& Jc, Vector& c, Vector& l,
                             Vector& u) override;

 private:
  PiecewiseJerkTrajectory1d curvature_curve_;

  bool use_v_bound_ = false;

  bool use_soft_safety_bound_ = false;

  PiecewiseJerkTrajectory1d v_bound_func_;

  const double s_init_;

  const double s_dot_init_;

  const double s_ddot_init_;

  const double delta_t_;

  const int num_of_points_;

  const double s_max_;

  const double s_dot_max_;

  const double s_ddot_min_;

  const double s_ddot_max_;

  const double s_dddot_min_;

  const double s_dddot_max_;

  const int v_offset_;

  const int a_offset_;

  int lower_s_slack_offset_ = 0;

  int upper_s_slack_offset_ = 0;

  double w_target_s_ = 10000.0;

  double w_target_v_ = 10000.0;

  double w_target_a_ = 10000.0;

  double w_ref_v_ = 1.0;

  double w_ref_s_ = 1.0;

  double w_overall_a_ = 100.0;

  double w_overall_j_ = 10.0;

  double w_overall_centripetal_acc_ = 500.0;

  double w_soft_s_bound_ = 0.0;

  double v_max_ = 0.0;

  double s_target_ = 0.0;

  double v_target_ = 0.0;

  double a_target_ = 0.0;

  double v_ref_ = 0.0;

  std::vector<std::pair<double, double>> safety_bounds_;

  std::vector<std::pair<double, double>> soft_safety_bounds_;

  bool has_end_state_target_ = false;

  std::vector<double> opt_s_;

  std::vector<double> opt_v_;

  std::vector<double> opt_a_;

  std::vector<std::vector<double>> x_warm_start_;

  std::vector<double> s_ref_;
};

}  // namespace planning
}  // namespace apollo