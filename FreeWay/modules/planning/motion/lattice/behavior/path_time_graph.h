#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/proto/geometry.pb.h"

#include "modules/common/math/polygon2d.h"

#include "modules/planning/obstacle/obstacle.h"
#include "modules/planning/frame/frame.h"
#include "modules/planning/reference_line/reference_line_info.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/proto/sl_boundary.pb.h"

#include "modules/planning/speed/st_point.h"
#include "modules/planning/speed/st_boundary.h"

namespace dharma {
namespace planning {

class PathTimeGraph {
public:
    PathTimeGraph(const std::vector<const Obstacle*>& obstacles,
                  const std::vector<common::PathPoint>& discretized_ref_points,
                  const ReferenceLineInfo* ptr_reference_line_info,
                  const double s_start, const double s_end, const double t_start,
                  const double t_end, const std::array<double, 3>& init_d);

    const std::vector<STBoundary>& GetPathTimeObstacles() const;

    bool GetPathTimeObstacle(const std::string& obstacle_id,
                             STBoundary* path_time_obstacle);

    std::vector<std::pair<double, double>> GetPathBlockingIntervals(
            const double t) const;

    std::vector<std::vector<std::pair<double, double>>> GetPathBlockingIntervals(
            const double t_start, const double t_end, const double t_resolution);

    std::pair<double, double> get_path_range() const;

    std::pair<double, double> get_time_range() const;

    std::vector<STPoint> GetObstacleSurroundingPoints(
            const std::string& obstacle_id, const double s_dist,
            const double t_density) const;

    bool IsObstacleInGraph(const std::string& obstacle_id);

    std::vector<std::pair<double, double>> GetLateralBounds(
            const double s_start, const double s_end, const double s_resolution);

private:
    void SetupObstacles(
            const std::vector<const Obstacle*>& obstacles,
            const std::vector<common::PathPoint>& discretized_ref_points);

    SLBoundary ComputeObstacleBoundary(
            const std::vector<common::math::Vec2d>& vertices,
            const std::vector<common::PathPoint>& discretized_ref_points) const;

    STPoint MakePathTimePoint(const double s, const double t) const;

    void SetStaticObstacle(
            const Obstacle* obstacle,
            const std::vector<common::PathPoint>& discretized_ref_points);

    void SetDynamicObstacle(
            const Obstacle* obstacle,
            const std::vector<common::PathPoint>& discretized_ref_points);

    void UpdateLateralBoundsByObstacle(
            const SLBoundary& sl_boundary,
            const std::vector<double>& discretized_path, const double s_start,
            const double s_end, std::vector<std::pair<double, double>>* const bounds);

private:
    std::pair<double, double> time_range_;
    std::pair<double, double> path_range_;
    const ReferenceLineInfo* ptr_reference_line_info_;
    std::array<double, 3> init_d_;

    std::unordered_map<std::string, STBoundary> path_time_obstacle_map_;
    std::vector<STBoundary> path_time_obstacles_;
    std::vector<SLBoundary> static_obs_sl_boundaries_;
};

}  // namespace planning
}  // namespace dharma
