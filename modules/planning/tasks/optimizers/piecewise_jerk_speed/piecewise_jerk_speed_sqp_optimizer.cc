#include "modules/planning/tasks/optimizers/piecewise_jerk_speed/piecewise_jerk_speed_sqp_optimizer.h"

#include <string>
#include <utility>
#include <vector>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed_profile_generator.h"
#include "modules/planning/common/st_graph_data.h"
#include "modules/planning/common/util/print_debug_info.h"
#include "modules/planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;

PiecewiseJerkSpeedSQPOptimizer::PiecewiseJerkSpeedSQPOptimizer(
    const TaskConfig& config)
    : SpeedOptimizer(config) {
  // TODO update config
  ACHECK(config_.has_piecewise_jerk_speed_sqp_optimizer_config());
}

Status PiecewiseJerkSpeedSQPOptimizer::Process(
    const PathData& path_data, const TrajectoryPoint& init_point,
    SpeedData* const speed_data) {
  AINFO << "using sqp method ...";
  if (reference_line_info_->ReachedDestination()) {
    return Status::OK();
  }

  ACHECK(speed_data != nullptr);
  const auto original_speed_data = *speed_data;
  AINFO << "original init speed point s :" << speed_data->front().s();

  if (path_data.discretized_path().empty()) {
    const std::string msg = "Empty path data";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  StGraphData& st_graph_data = *reference_line_info_->mutable_st_graph_data();

  const auto& veh_param =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  std::array<double, 3> init_s = {0.0, st_graph_data.init_point().v(),
                                  st_graph_data.init_point().a()};
  double delta_t = 0.1;
  double total_length = st_graph_data.path_length();
  double total_time = st_graph_data.total_time_by_conf();
  int num_of_knots = static_cast<int>(total_time / delta_t) + 1;

  // dense speed data for warm start
  AINFO << "before clear, speed_data size: " << speed_data->size();
  speed_data->clear();
  AINFO << "after clear, speed_data size: " << speed_data->size();
  for (int i = 0; i < num_of_knots; ++i) {
    double curr_t = i * delta_t;
    // get path_s
    SpeedPoint sp;
    original_speed_data.EvaluateByTime(curr_t, &sp);
    speed_data->push_back(sp);
  }
  AINFO << "after update, speed_data size: " << speed_data->size();

  PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t,
                                                   init_s);

  const auto& config = config_.piecewise_jerk_speed_sqp_optimizer_config();
  piecewise_jerk_problem.set_weight_ddx(config.acc_weight());
  piecewise_jerk_problem.set_weight_dddx(config.jerk_weight());

  piecewise_jerk_problem.set_x_bounds(0.0, total_length);
  piecewise_jerk_problem.set_dx_bounds(
      0.0, std::fmax(FLAGS_planning_upper_speed_limit,
                     st_graph_data.init_point().v()));
  piecewise_jerk_problem.set_ddx_bounds(veh_param.max_deceleration(),
                                        veh_param.max_acceleration());
  piecewise_jerk_problem.set_dddx_bound(FLAGS_longitudinal_jerk_lower_bound,
                                        FLAGS_longitudinal_jerk_upper_bound);

  piecewise_jerk_problem.set_dx_ref(config.ref_v_weight(),
                                    reference_line_info_->GetCruiseSpeed());
  PrintCurves print_debug;
  // Update STBoundary
  std::vector<std::pair<double, double>> s_bounds;
  for (int i = 0; i < num_of_knots; ++i) {
    double curr_t = i * delta_t;
    double s_lower_bound = 0.0;
    double s_upper_bound = total_length;
    for (const STBoundary* boundary : st_graph_data.st_boundaries()) {
      double s_lower = 0.0;
      double s_upper = 0.0;
      if (!boundary->GetUnblockSRange(curr_t, &s_upper, &s_lower)) {
        continue;
      }
      switch (boundary->boundary_type()) {
        case STBoundary::BoundaryType::STOP:
        case STBoundary::BoundaryType::YIELD:
          s_upper_bound = std::fmin(s_upper_bound, s_upper);
          break;
        case STBoundary::BoundaryType::FOLLOW:
          // TODO(Hongyi): unify follow buffer on decision side
          s_upper_bound = std::fmin(s_upper_bound, s_upper - 8.0);
          break;
        case STBoundary::BoundaryType::OVERTAKE:
          s_lower_bound = std::fmax(s_lower_bound, s_lower);
          break;
        default:
          break;
      }
    }
    if (s_lower_bound > s_upper_bound) {
      const std::string msg =
          "s_lower_bound larger than s_upper_bound on STGraph";
      AERROR << msg;
      speed_data->clear();
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
    print_debug.AddPoint("st_bounds_lower", curr_t, s_lower_bound);
    print_debug.AddPoint("st_bounds_upper", curr_t, s_upper_bound);
    s_bounds.emplace_back(s_lower_bound, s_upper_bound);
  }
  piecewise_jerk_problem.set_x_bounds(std::move(s_bounds));

  // Update SpeedBoundary and ref_s
  std::vector<double> x_ref;
  std::vector<double> penalty_dx;
  std::vector<std::pair<double, double>> s_dot_bounds;
  const SpeedLimit& speed_limit = st_graph_data.speed_limit();
  for (int i = 0; i < num_of_knots; ++i) {
    // get path_s
    SpeedPoint sp = speed_data->at(i);
    const double path_s = sp.s();
    x_ref.emplace_back(path_s);
    // get curvature
    PathPoint path_point = path_data.GetPathPointWithPathS(path_s);
    penalty_dx.push_back(std::fabs(path_point.kappa()) *
                         config.kappa_penalty_weight());
    // get v_upper_bound
    const double v_lower_bound = 0.0;
    double v_upper_bound = FLAGS_planning_upper_speed_limit;
    v_upper_bound = speed_limit.GetSpeedLimitByS(path_s);
    s_dot_bounds.emplace_back(v_lower_bound, std::fmax(v_upper_bound, 0.0));
  }
  piecewise_jerk_problem.set_x_ref(config.ref_s_weight(), std::move(x_ref));
  piecewise_jerk_problem.set_penalty_dx(penalty_dx);
  piecewise_jerk_problem.set_dx_bounds(std::move(s_dot_bounds));

  // set primal warm start
  std::vector<c_float> primal_warm_start;
  piecewise_jerk_problem.SetPrimalWarmStart(*speed_data, &primal_warm_start);

  // calculate kernel
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;
  piecewise_jerk_problem.CalculateKernel(&P_data, &P_indices, &P_indptr);

  // Calculate offset
  std::vector<c_float> q;
  piecewise_jerk_problem.CalculateOffset(&q);

  // calculate affine constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  piecewise_jerk_problem.CalculateAffineConstraint(
      &A_data, &A_indices, &A_indptr, &lower_bounds, &upper_bounds);

  // Load matrices and vectors into OSQPData
  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  CHECK_EQ(lower_bounds.size(), upper_bounds.size());

  std::size_t kernel_dim = static_cast<std::size_t>(num_of_knots) * 3;
  std::size_t num_affine_constraint = lower_bounds.size();

  data->n = kernel_dim;
  data->m = num_affine_constraint;
  data->P = csc_matrix(data->n, data->n, P_data.size(), P_data.data(),
                       P_indices.data(), P_indptr.data());
  data->q = q.data();
  data->A = csc_matrix(data->m, data->n, A_data.size(), A_data.data(),
                       A_indices.data(), A_indptr.data());
  data->l = lower_bounds.data();
  data->u = upper_bounds.data();

  // Define osqp solver settings
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->polish = true;
  settings->verbose = FLAGS_enable_osqp_debug;
  settings->scaled_termination = true;
  settings->max_iter = 4000;  // TODO

  // Define osqp workspace
  OSQPWorkspace* work = nullptr;
  work = osqp_setup(data, settings);

  // Initial solution
  const auto qp_start = std::chrono::system_clock::now();
  bool initial_solve_res =
      piecewise_jerk_problem.OptimizeWithWarmStart(primal_warm_start, &work);
  const auto qp_end = std::chrono::system_clock::now();
  std::chrono::duration<double> qp_diff = qp_end - qp_start;
  AINFO << "initial qp optimization takes " << qp_diff.count() * 1000.0
        << " ms";

  if (!initial_solve_res) {
    // clean osqp work
    osqp_cleanup(work);
    c_free(data->A);
    c_free(data->P);
    c_free(data);
    c_free(settings);
    // return status
    const std::string msg = "Piecewise jerk speed optimizer failed!";
    AERROR << msg;
    speed_data->clear();
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // update speed data
  const std::vector<double>& s = piecewise_jerk_problem.opt_x();
  const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
  const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();
  speed_data->clear();
  speed_data->AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
  for (int i = 1; i < num_of_knots; ++i) {
    speed_data->AppendSpeedPoint(s[i], delta_t * i, ds[i], dds[i],
                                 (dds[i] - dds[i - 1]) / delta_t);
  }

  // speed bound constraint check
  bool v_bound_check = true;
  for (int i = 0; i < num_of_knots; i++) {
    auto sp = speed_data->at(i);
    const double path_s = sp.s();
    const double v_upper_bound = speed_limit.GetSpeedLimitByS(path_s);
    if (sp.v() > v_upper_bound) {
      v_bound_check = false;
      break;
    }
  }
  if (v_bound_check) {
    AINFO << "initial_solve_res meet the constraints";
  } else {
    AINFO << "initial_solve_res don't meet the constraints";
  }

  // Sequential solution
  int pen_itr = 0;
  while (!v_bound_check && pen_itr < 10) {
    AINFO << "pen itr: " << pen_itr;
    // Update SpeedBoundary and ref_s
    // std::vector<double> x_ref;
    penalty_dx.clear();
    s_dot_bounds.clear();
    for (int i = 0; i < num_of_knots; ++i) {
      // get path_s
      SpeedPoint sp = speed_data->at(i);
      const double path_s = sp.s();
      // x_ref.emplace_back(path_s);
      // get curvature
      PathPoint path_point = path_data.GetPathPointWithPathS(path_s);
      penalty_dx.push_back(std::fabs(path_point.kappa()) *
                           config.kappa_penalty_weight());
      // get v_upper_bound
      const double v_lower_bound = 0.0;
      double v_upper_bound = FLAGS_planning_upper_speed_limit;
      v_upper_bound = speed_limit.GetSpeedLimitByS(path_s);
      s_dot_bounds.emplace_back(v_lower_bound, std::fmax(v_upper_bound, 0.0));
    }
    // piecewise_jerk_problem.set_x_ref(config.ref_s_weight(),
    // std::move(x_ref));
    piecewise_jerk_problem.set_penalty_dx(penalty_dx);
    piecewise_jerk_problem.set_dx_bounds(std::move(s_dot_bounds));
    // update work
    piecewise_jerk_problem.SetPrimalWarmStart(*speed_data, &primal_warm_start);
    piecewise_jerk_problem.CalculateKernel(&P_data, &P_indices, &P_indptr);
    // piecewise_jerk_problem.CalculateOffset(&q);
    piecewise_jerk_problem.CalculateAffineConstraint(
        &A_data, &A_indices, &A_indptr, &lower_bounds, &upper_bounds);
    osqp_update_P(work, P_data.data(), OSQP_NULL, P_data.size());
    // osqp_update_lin_cost(work, q.data());
    osqp_update_upper_bound(work, upper_bounds.data());
    // solve
    const auto sqp_start = std::chrono::system_clock::now();
    bool iterative_solve_res =
        piecewise_jerk_problem.OptimizeWithWarmStart(primal_warm_start, &work);
    const auto sqp_end = std::chrono::system_clock::now();
    std::chrono::duration<double> sqp_diff = sqp_end - sqp_start;
    AINFO << pen_itr << "st qp optimization takes " << sqp_diff.count() * 1000.0
          << " ms";
    if (!iterative_solve_res) {
      // clean osqp work
      osqp_cleanup(work);
      c_free(data->A);
      c_free(data->P);
      c_free(data);
      c_free(settings);
      // return status
      AINFO << pen_itr << "st QP optimizer failed!";
      break;
    }
    // update speed data
    const std::vector<double>& s = piecewise_jerk_problem.opt_x();
    const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
    const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();
    speed_data->clear();
    speed_data->AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
    for (int i = 1; i < num_of_knots; ++i) {
      speed_data->AppendSpeedPoint(s[i], delta_t * i, ds[i], dds[i],
                                   (dds[i] - dds[i - 1]) / delta_t);
    }
    // speed bound constraint check
    v_bound_check = true;
    for (int i = 0; i < num_of_knots; i++) {
      auto sp = speed_data->at(i);
      const double path_s = sp.s();
      const double v_upper_bound = speed_limit.GetSpeedLimitByS(path_s);
      if (sp.v() > v_upper_bound) {
        v_bound_check = false;
        break;
      }
    }
    if (v_bound_check) {
      AINFO << pen_itr << "st solve_res meet the constraints";
    } else {
      AINFO << pen_itr << "st solve_res don't meet the constraints";
    }

    ++pen_itr;
  }

  int new_size = -1;
  for (int i = 0; i < num_of_knots; ++i) {
    // Avoid the very last points when already stopped
    if (ds[i] <= 0.0) {
      new_size = i + 1;
      break;
    }
  }
  if (new_size > -1) {
    speed_data->resize(new_size);
  }
  AINFO << "after smoothed, the size of speed data is: " << speed_data->size();
  AINFO << "smoothed init speed point s: " << speed_data->front().s();

  SpeedProfileGenerator::FillEnoughSpeedPoints(speed_data);
  RecordDebugInfo(*speed_data, st_graph_data.mutable_st_graph_debug());
  print_debug.PrintToLog();
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
