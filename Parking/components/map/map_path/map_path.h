#ifndef MAP_MAP_PATH_MAP_PATH_H_
#define MAP_MAP_PATH_MAP_PATH_H_

#include "common/math/vec2d.h"
#include "common/math/line_segment2d.h"
#include "common/math/aaboxkdtree2d.h"
#include "common/proto/vehicle_state.pb.h"

#include "map/common/gflags_map.h"

namespace map {

class MapPathPoint : public common::math::Vec2d
{
public:
    MapPathPoint() = default;
    MapPathPoint(const MapPathPoint&) = default;

    MapPathPoint(const common::math::Vec2d& point, const double& theta)
        : common::math::Vec2d(point.x(), point.y()),
          theta_(theta){}

    MapPathPoint(const double& x, const double& y, const double& theta)
        : common::math::Vec2d(x, y),
          theta_(theta){}
    MapPathPoint(const double& x, const double& y, const double& theta, const double& lane_left_width,
                 const double& lane_right_width)
        : common::math::Vec2d(x, y),
          theta_(theta),
          lane_left_width_(lane_left_width),
          lane_right_width_(lane_right_width){}

public:
    void set_x(const double& x){
        x_ = x;
    }

    void set_y(const double& y){
        y_ = y;
    }

    void set_theta(const double& theta){
        theta_ = theta;
    }

    void set_lane_left_width(const double& llw){
        lane_left_width_ = llw;
    }

    void set_lane_right_width(const double& lrw){
        lane_right_width_ = lrw;
    }


    const double& theta() const {
        return theta_;
    }
    const double& lane_left_width() const {
        return lane_left_width_;
    }
    const double& lane_right_width() const {
        return lane_right_width_;
    }

protected:
    double theta_ = 0.0;
    double lane_left_width_ = 2.0; //default lane_left_width
    double lane_right_width_ = 2.0; // default lane_right_width

};

/**
  * @brief : InterpolatedIndex has two elements(id and offset), id is the index of map_path_point from map file, and
  * offset is the distance between Interpolated point and map_path_point.eg:
  *
  * map_path_points   : |---------------|--------------------------------------|----------|
  *                index:0         index:1                               index:2      index:3
  *
  * interpolated_points:x---x---x---x---x---x---x---x---x---x---x---x---x---x---x---x---x-x
  *                         |                                   |               |
  *                  (index:0,offset:4)                (index:1,offset:24)   (index:2,offset:1)
  *
  */

class InterpolatedIndex {
public:
    InterpolatedIndex(size_t index, double offset) : index(index), offset(offset) {}
    size_t index = 0;
    double offset = 0.0;
};

class MapPath
{
public:
    MapPath() = default;
    explicit MapPath(const std::vector<MapPathPoint>& map_path_points);
    explicit MapPath(std::vector<MapPathPoint>&& map_path_points);

public:
    const std::vector<MapPathPoint>& points() const {return points_;};

    const size_t& num_points() const { return num_points_; }

    const size_t& num_segments() const {return num_segments_;}

    const std::vector<common::math::LineSegment2d>& segments() const {return segments_;}

    const std::vector<double>& accumulated_s() const {return accumulated_s_;}

    const double& length() const { return length_; }

    const std::vector<double>& lane_left_widths() const {return lane_left_widths_;}

    const std::vector<double>& lane_right_widths() const {return lane_right_widths_;}

public:
    bool GetProjection(const common::math::Vec2d& point, double* accumulate_s,
                       double* lateral, double* min_distance) const;

    bool GetProjection(const common::math::Vec2d& point, double* accumulate_s,
                       double* lateral) const;

    InterpolatedIndex GetIndexFromS(double s) const;

    MapPathPoint GetSmoothPoint(const InterpolatedIndex& index) const;

    void Clear();

private:
    void Init();
    void InitPoints();

private:
    std::vector<MapPathPoint> points_;
    size_t num_points_ = 0;

    std::vector<double> accumulated_s_;

    std::vector<common::math::LineSegment2d> segments_;
    size_t num_segments_ = 0;

    std::vector<common::math::Vec2d> unit_directions_;

    double length_;

    std::vector<double> lane_left_widths_;
    std::vector<double> lane_right_widths_;

};



}


#endif
