#include "modules/planning/planning.h"

#include "spider/time/time.h"

#include <iostream>

#include <fstream>

int main() {

  std::unique_ptr<dharma::planning::Planning> planning_ptr_;

  planning_ptr_ = std::make_unique<dharma::planning::Planning>();

  planning_ptr_->Init();

  double time_stamp = spider::Time::Now().ToSecond();

  //  while (true) {
  // 获取感知预测，定位，底盘信息
  dharma::prediction::PredictionObstacles pred_obs;

  dharma::perception::PerceptionObstacle ob;
  ob.set_id(1);
  ob.mutable_position()->set_x(0.0);
  ob.mutable_position()->set_y(60.0);
  ob.mutable_position()->set_z(0.85);
  ob.set_theta(M_PI / 2.0);
  ob.set_length(5.0);
  ob.set_width(1.0);
  ob.set_height(1.7);

  dharma::prediction::PredictionObstacle pred_ob;
  pred_ob.mutable_perception_obstacle()->CopyFrom(ob);

  pred_obs.mutable_header()->set_module_name("predict");
  pred_obs.mutable_header()->set_sequence_num(1);
  pred_obs.mutable_header()->set_timestamp_sec(time_stamp);
  auto pred = pred_obs.add_prediction_obstacle();
  pred->CopyFrom(pred_ob);

  dharma::localization::Localization loca;
  loca.set_measurement_time(time_stamp);

  dharma::localization::Pose p;
  p.mutable_position()->set_x(0);
  p.mutable_position()->set_y(0);
  p.mutable_position()->set_z(1.0);

  p.mutable_orientation()->set_qw(1);
  p.mutable_orientation()->set_qx(0.0);
  p.mutable_orientation()->set_qy(0.0);
  p.mutable_orientation()->set_qz(0.0);

  p.mutable_linear_acceleration_vrf()->set_x(0);
  p.mutable_linear_acceleration_vrf()->set_y(0);
  p.mutable_linear_acceleration_vrf()->set_z(0);

  p.mutable_angular_velocity_vrf()->set_x(0);
  p.mutable_angular_velocity_vrf()->set_y(0);
  p.mutable_angular_velocity_vrf()->set_z(0);

  loca.mutable_pose()->CopyFrom(p);
  loca.mutable_header()->set_module_name("localization");
  loca.mutable_header()->set_sequence_num(1);
  loca.mutable_header()->set_timestamp_sec(time_stamp + 0.01);

  dharma::canbus::Chassis chassis;
  chassis.set_speed_mps(3.0);

  chassis.mutable_header()->set_module_name("canbus");
  chassis.mutable_header()->set_sequence_num(1);
  chassis.mutable_header()->set_timestamp_sec(time_stamp + 0.03);

  dharma::planning::LocalView local_view;

  local_view.prediction_obstacles =
      std::make_shared<dharma::prediction::PredictionObstacles>(pred_obs);
  local_view.localization =
      std::make_shared<dharma::localization::Localization>(loca);
  local_view.chassis = std::make_shared<dharma::canbus::Chassis>(chassis);

  // 获取一组允许行驶的reference_lines
  std::list<dharma::planning::ReferenceLine> ref_lines;
  for (int i = 0; i < 3; i++) {
    std::vector<dharma::planning::ReferencePoint> rps;
    rps.clear();
    rps.reserve(210);
    for (int j = -10; j < 200; j++) {
      dharma::hdmap::MapPathPoint mp;
      mp.set_x(-4 + i * 4);
      mp.set_y(j);
      mp.set_heading(M_PI / 2.0);
      dharma::planning::ReferencePoint rp(mp, 0.0, 0.0);
      rps.emplace_back(rp);
    }
    ref_lines.emplace_back(rps);

    if (0) {
      std::string fname = "reference_line" + std::to_string(i) + ".csv";
      std::fstream openfile(fname, std::ios::ate | std::ios::out);
      for (const auto &point : rps) {
        openfile << point.x() << "," << point.y() << "," << point.heading()
                 << "\n";
      }
      openfile.close();
    }
  }

  // 本周期规划结果traj
  dharma::planning::ADCTrajectory traj;
  // 返回值res为true表示本周期规划成功
  bool res = planning_ptr_->RunOnce(local_view, ref_lines, &traj);

  //  if (res) {
  //    std::cout << "Planning Success!" << std::endl;
  //    for (const auto &point : traj.trajectory_point()) {
  //      std::cout << point.path_point().x() << " " << point.path_point().y()
  //                << " " << std::endl;
  //    }
  //  }

  //  }

  return 0;
}
