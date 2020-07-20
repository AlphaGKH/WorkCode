#include "modules/planning/planning.h"

#include <iostream>

int main() {

  std::unique_ptr<dharma::planning::Planning> planning_ptr_;

  planning_ptr_ = std::make_unique<dharma::planning::Planning>();

  planning_ptr_->Init();

  while (true) {
    // 获取感知预测，定位，底盘信息
    dharma::prediction::PredictionObstacles pred_obs;
    pred_obs.mutable_header()->set_module_name("predict");
    dharma::localization::Localization loca;
    loca.mutable_header()->set_module_name("localization");
    dharma::canbus::Chassis chassis;
    chassis.mutable_header()->set_module_name("canbus");

    dharma::planning::LocalView local_view;

    local_view.prediction_obstacles =
        std::make_shared<dharma::prediction::PredictionObstacles>(pred_obs);
    local_view.localization =
        std::make_shared<dharma::localization::Localization>(loca);
    local_view.chassis = std::make_shared<dharma::canbus::Chassis>(chassis);

    // 获取一组允许行驶的reference_lines
    std::list<dharma::planning::ReferenceLine> ref_lines;

    // 本周期规划结果traj
    dharma::planning::ADCTrajectory traj;
    // 返回值res为true表示本周期规划成功
    bool res = planning_ptr_->RunOnce(local_view, ref_lines, &traj);
  }

  return 0;
}
