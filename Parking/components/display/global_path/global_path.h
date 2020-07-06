#ifndef DISPLAY_GLOBAL_PATH_GLOBAL_PATH_H_
#define DISPLAY_GLOBAL_PATH_GLOBAL_PATH_H_

#include <memory>
#include <string>
#include <limits>
#include <opencv2/opencv.hpp>

#include "common/proto/vehicle_state.pb.h"
#include "common/proto/vehicle_param.pb.h"

#include "map/map_path/map_path.h"
#include "map/road_map/road_map.h"
#include "map/map_path/map_path.h"

#include "planning/reference_line/reference_point.h"

#include "display/common/gflags_display.h"
#include "display/proto/display_config.pb.h"

namespace display {

class GlobalPath
{
public:
    explicit GlobalPath(map::RoadMap* const road_map);

public:
    bool Init(const GlobalViewConfig& config);

    void Show(const common::VehicleState& adc_state, const std::vector<map::MapPathPoint>& map_path_points,
              const std::vector<planning::ReferencePoint>& ref_points);

    void Show(const common::VehicleState& adc_state);

private:

    bool ShowVehicleState(const common::VehicleState& vehicle_state, cv::Mat img);

    bool ShowMapPath(const std::vector<map::MapPathPoint>& map_points, cv::Mat img);

    bool ShowReferenceLine(const std::vector<planning::ReferencePoint>& ref_ponits, cv::Mat img);


    bool ShowGlobalPath(const std::vector<map::MapPathPoint>& points);

    void ShowGlobalPathBoundary(const std::vector<map::MapPathPoint>& points);

    template <typename Iterator>
    void PaintPath(const Iterator begin, const Iterator end, cv::Mat img,
                   const cv::Scalar& color = cv::Scalar(0,255,0),
                   int line_thickness = 1, int lineType = cv::LINE_8,
                   int cycle_thickness = 1, int cycle_radius = 3){
        cv::Point2d pos;
        cv::Point2d next_pos;
        auto it = begin;
        if(begin != end){
            for(it = begin; it != end; it++){
                CalcImagePointPos(it->x(),it->y(),&pos.x, &pos.y);

                auto next_it = it;
                next_it ++;
                if(next_it == end){
                    break;
                }
                CalcImagePointPos(next_it->x(),next_it->y(),&next_pos.x, &next_pos.y);
                cv::line(img, pos, next_pos, color, line_thickness, lineType);
                cv::circle(img, pos, cycle_radius, color, cycle_thickness);
            }

            CalcImagePointPos(it->x(), it->y(),&pos.x, &pos.y);
            cv::circle(img, pos, cycle_radius, color, cycle_thickness);
        }


    }

    void CalcImagePointPos(const double& x, const double& y, double* nx, double* ny){
        *nx = (x - 0.5 * (min_x_ + max_x_)) * scale_ + config_.map_width() / 2;
        *ny = config_.map_height() - (y - 0.5 * (min_y_ + max_y_)) * scale_ - config_.map_height() / 2;
    }

private:
    std::unique_ptr<cv::Mat> image_;
    int scale_ = 1;
    double min_x_ = std::numeric_limits<double>::infinity();
    double max_x_ = 0.0;
    double min_y_ = std::numeric_limits<double>::infinity();
    double max_y_ = 0.0;

private:
    map::RoadMap* const road_map_;

    const std::string global_path_name_ = "GlobalPath";

    GlobalViewConfig config_;

};



}

#endif
