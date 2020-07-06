#ifndef COMMON_VISUAL_HANDLE_VISUAL_HANDLE_H_
#define COMMON_VISUAL_HANDLE_VISUAL_HANDLE_H_

#include <opencv2/opencv.hpp>
#include <list>
#include <vector>

#include "common/macro.h"
#include "common/gflags_common.h"

#include "map/common/gflags_map.h"

#include "perception/proto/ogm_config.pb.h"
#include "perception/GridMap.hpp"

#include "planning/reference_line/reference_line.h"
#include "planning/common/path/path_data.h"

namespace common {

class VisualHandle
{
public:

    static void Init(const perception::OgmConfig& ogm_config);

    static void Visualize(const std::list<planning::ReferenceLine>& lines,
                          const std::list<planning::PathData>& paths);
    static void Visualize(const std::list<planning::ReferenceLine>& lines,
                          const std::list<planning::PathData>& paths,
                          const perception::GridMap& grid_map);
    static void Visualize(const std::list<planning::ReferenceLine>& lines,
                          const std::list<planning::PathData>& paths,
                          const perception::GridMap& grid_map,
                          const planning::PathData& path);

private:
    static void ShowGrid();

    static void ShowVehicleBox();

    static void ShowCoordinateLines();

    static void VisualizeGridMap(const perception::GridMap& grid_map, cv::Mat img);

    static void VisualizeReferenceLine(const planning::ReferenceLine& ref_line, cv::Mat img);

    static void VisualizeReferenceLines(const std::list<planning::ReferenceLine>& lines, cv::Mat img);

    static void VisualizePath(const planning::PathData& path, cv::Mat img);

    static void VisualizePaths(const std::list<planning::PathData>& paths, cv::Mat img);

private:
    template <typename Iterator>
    static void PaintLine(const Iterator begin, const Iterator end, cv::Mat img,
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

    static void CalcImagePose(const double &x, const double &y, double *nx, double *ny){
        *nx = visual_image_width / 2.0 + x * scale_;

        *ny = visual_image_height * 1.0 - visual_image_back_length - y * scale_;
        return;
    }

private:
    static std::unique_ptr<cv::Mat> image_;
    static int scale_;
    static perception::OgmConfig ogm_config_;
    static bool inited_;
    static uint32_t visual_image_width;
    static uint32_t visual_image_height;
    static uint32_t visual_image_back_length;
    DECLARE_SINGLETON(VisualHandle);
};

}


#endif
