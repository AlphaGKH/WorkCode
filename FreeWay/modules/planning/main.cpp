#include "modules/planning/planning.h"

#include <iostream>

int main() {

//  auto p1 = dharma::common::math::Vec2d(2.0, 3.0);
//  auto p2 = dharma::common::math::Vec2d(5.0, 7.0);

//  auto aab = dharma::common::math::AABox2d(p1, p2);

//  std::cout << aab.DebugString() << std::endl;

  //  std::unique_ptr<dharma::planning::Planning> planning_ptr_;

  //  planning_ptr_ = std::make_unique<dharma::planning::Planning>();

  //  planning_ptr_->Init();

  //  while (true) {
  //    // 获取感知预测，定位，底盘信息
  //    dharma::perception::PerceptionObstacle pred_obs;
  //    dharma::localization::Localization loca;
  //    dharma::canbus::Chassis chassis;

  //    dharma::planning::LocalView local_view;
  //    local_view.prediction_obstacles->CopyFrom(pred_obs);
  //    local_view.localization->CopyFrom(loca);
  //    local_view.chassis->CopyFrom(chassis);

  //    // 获取一组允许行驶的reference_lines
  //    std::list<dharma::planning::ReferenceLine> ref_lines;

  //    // 本周期规划结果traj
  //    dharma::planning::ADCTrajectory traj;
  //    // 返回值res为true表示本周期规划成功
  //    bool res = planning_ptr_->RunOnce(local_view, ref_lines, &traj);
  //  }

  return 0;
}
