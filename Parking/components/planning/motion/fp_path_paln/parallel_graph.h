#ifndef PLANNING_MOTION_FP_PATH_PLAN_PARALLEL_GRAPH_H_
#define PLANNING_MOTION_FP_PATH_PLAN_PARALLEL_GRAPH_H_

#include <list>

#include "common/proto/pnc_point.pb.h"
#include "common/proto/vehicle_param.pb.h"
#include "common/math/angle.h"

#include "planning/proto/fp_path_plan_config.pb.h"
#include "planning/reference_line/reference_line.h"
#include "planning/common/path/path_data.h"
#include "planning/motion/fp_path_paln/comparable_cost.h"

#include "perception/proto/ogm_config.pb.h"

#include "perception/GridMap.hpp"

namespace planning {

class ParallelGraph
{
public:
    explicit ParallelGraph(const FpPathPlanConfig& config, const perception::OgmConfig& ogm_config);

    bool FindPathTunnel(const common::TrajectoryPoint& init_point,
                        const common::VehicleState adc_state,
                        const ReferenceLine& ref_line,
                        const perception::GridMap& grid_map,
                        PathData* min_cost_path);

private:
    bool ConvertReferenceLine2VehicleCoor(const ReferenceLine& raw_ref_line,
                                          const common::VehicleState& vehicle_pos,
                                          ReferenceLine* ref_line);

    bool SampleParallelLines(const ReferenceLine& local_ref_line, std::list<ReferenceLine>* parallel_lines);

    bool KinematicSample(const ReferenceLine& ref_line, PathData* smooth_path);

    bool ExtractMinCostPath(const std::list<PathData>& path_list, PathData* min_cost_path);

    bool ConvertPathPoints2WorldCoor(const PathData& raw_path, PathData* world_path);

private:
    template <typename U, typename V>
    void ConvertPoint2Another(U* point, const V& another) {
        double sin_theta = common::math::sin(common::math::Angle16::from_rad(another.theta()));
        double cos_theta = common::math::cos(common::math::Angle16::from_rad(another.theta()));

        double delta_x = point->x() - another.x();
        double delta_y = point->y() - another.y();

        double nx = delta_x * sin_theta - delta_y * cos_theta;
        double ny = delta_x * cos_theta + delta_y * sin_theta;
        double ntheta = common::math::NormalizeAngle(M_PI / 2.0 + point->theta() - another.theta());
        point->set_x(nx);
        point->set_y(ny);
        point->set_theta(ntheta);
    }

    double PurePursuit(const double& delta_x, const double& Ld);

    double Stanley(const double& delta_theta, const double& delta_distance, const double& velocity);

private:
    // collision analysis
    bool CollisionAnalysis(const common::PathPoint& point) const;

    void GetGridMapCoordinate(const double& x, const double& y, int32_t* x_cell, int32_t* y_cell) const;

    bool IsOccupied(const int32_t& x_cell, const int32_t& y_cell) const;

private:
    // smooth cost : central_cost + lateral_cost
    void CalculateSmoothCost(const std::list<PathData>& path_list,
                             std::vector<ComparableCost>* smooth_costs);

    void CalculateSafetyCost(const std::list<PathData>& path_list,
                             std::vector<ComparableCost>* safety_costs);

private:

    std::unique_ptr<common::TrajectoryPoint> init_point_;

    std::unique_ptr<common::VehicleState> adc_state_;

    ReferenceLine center_local_ref_line_;

    std::unique_ptr<perception::GridMap> grid_map_;

private:
    FpPathPlanConfig fp_config_;
    perception::OgmConfig ogm_config_;
};

}



#endif
