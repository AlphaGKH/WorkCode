#ifndef DISPLAY_LOCAL_VIEW_LOCAL_VIEW_H_
#define DISPLAY_LOCAL_VIEW_LOCAL_VIEW_H_

#include <opencv2/opencv.hpp>

#include "display/common/gflags_display.h"
#include "display/proto/display_config.pb.h"

#include "perception/GridMap.hpp"

#include "planning/LcmTrajectory.hpp"

namespace display {

class LocalView
{
public:
    LocalView() = default;

public:
    bool Init(const LocalViewConfig& config);

    void Show(const perception::GridMap& grid_map);

private:
    void ShowGridMap(const perception::GridMap& grid_map, cv::Mat img);

    void ShowTrajectory(const planning::LcmTrajectory& traj, cv::Mat img);

private:
    bool ShowGrid();

    bool ShowVehicleBox();

    bool ShowCoordinateLines();

private:
    template <typename Iterator>
    void PaintLine(const Iterator begin, const Iterator end, cv::Mat img,
                   const cv::Scalar& color = cv::Scalar(0,255,0),
                   int line_thickness = 1, int lineType = cv::LINE_8,
                   int cycle_thickness = 1, int cycle_radius = 3){
        cv::Point2d pos;
        cv::Point2d next_pos;
        auto it = begin;
        if(begin != end){
            for(it = begin; it != end; it++){
                CalcImagePose(it->x(),it->y(),&pos.x, &pos.y);

                auto next_it = it;
                next_it ++;
                if(next_it == end){
                    break;
                }
                CalcImagePose(next_it->x(),next_it->y(),&next_pos.x, &next_pos.y);
                cv::line(img, pos, next_pos, color, line_thickness, lineType);
                cv::circle(img, pos, cycle_radius, color, cycle_thickness);
            }

            CalcImagePose(it->x(), it->y(),&pos.x, &pos.y);
            cv::circle(img, pos, cycle_radius, color, cycle_thickness);
        }

    }

    void CalcImagePose(const double &x, const double &y, double *nx, double *ny){
            *nx = local_config_.local_view_width() / 2.0 + x * scale_;

            *ny = local_config_.local_view_height() * 1.0 - local_config_.local_view_back_length() - y * scale_;
            return;
        }

private:
    std::unique_ptr<cv::Mat> image_;
    LocalViewConfig local_config_;

    std::string local_view_name_ = "LocalView";

    int scale_;

};

}


#endif
