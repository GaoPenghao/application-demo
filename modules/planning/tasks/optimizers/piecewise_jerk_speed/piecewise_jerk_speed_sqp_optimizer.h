#pragma once

#include "modules/planning/tasks/optimizers/speed_optimizer.h"

namespace apollo {
namespace planning {

class PiecewiseJerkSpeedSQPOptimizer : public SpeedOptimizer {
 public:
  explicit PiecewiseJerkSpeedSQPOptimizer(const TaskConfig& config);

  virtual ~PiecewiseJerkSpeedSQPOptimizer() = default;

 private:
  common::Status Process(const PathData& path_data,
                         const common::TrajectoryPoint& init_point,
                         SpeedData* const speed_data) override;
};

}  // namespace planning
}  // namespace apollo