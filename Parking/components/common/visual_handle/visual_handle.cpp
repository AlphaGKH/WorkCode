#include "common/visual_handle/visual_handle.h"

#include "common/util/file.h"
#include "common/vehicle_param_handle/vehicle_param_handle.h"

#include "perception/common/gflags_perception.h"
#include "planning/common/gflags_planning.h"

namespace common {

std::unique_ptr<cv::Mat> VisualHandle::image_ = nullptr;

int VisualHandle::scale_ = 25;

bool VisualHandle::inited_ = false;

perception::OgmConfig VisualHandle::ogm_config_;

uint32_t VisualHandle::visual_image_width = 500;
uint32_t VisualHandle::visual_image_height = 1000;
uint32_t VisualHandle::visual_image_back_length = 300;

VisualHandle::VisualHandle(){}


void VisualHandle::Init(const perception::OgmConfig& ogm_config){
    if(!inited_){

        ogm_config_ = ogm_config;

        double wight_scale = (visual_image_width * 0.8) / ogm_config_.width();
        double height_scale = (visual_image_height * 0.8) / ogm_config_.length();

        scale_ = cvRound(std::min(wight_scale, height_scale));

        image_ = std::make_unique<cv::Mat>(visual_image_height, visual_image_width, CV_8UC1);

        cv::cvtColor(*image_,*image_,CV_GRAY2BGR);

        ShowGrid();

        ShowVehicleBox();

        ShowCoordinateLines();

        inited_ = true;

//        cv::imshow("DebugView", *image_);
//        cv::waitKey(planning::FLAGS_default_planning_cycle);
    }
    return;
}

void VisualHandle::Visualize(const std::list<planning::ReferenceLine> &lines,
                             const std::list<planning::PathData> &paths){
    if(!image_){
        image_ = std::make_unique<cv::Mat>(visual_image_height, visual_image_width, CV_8UC1);
        cv::cvtColor(*image_,*image_,CV_GRAY2BGR);
    }

    if(image_->empty()){
        AERROR << "Visual Image create failed!";
        return;
    }

    cv::Mat tmp_img = image_->clone();

    VisualizeReferenceLines(lines, tmp_img);
    VisualizePaths(paths, tmp_img);

    cv::imshow("DebugView", tmp_img);
    cv::waitKey(FLAGS_planning_cycle);
}

void VisualHandle::Visualize(const std::list<planning::ReferenceLine> &lines,
                             const std::list<planning::PathData> &paths,
                             const perception::GridMap &grid_map){
    if(!image_){
        image_ = std::make_unique<cv::Mat>(visual_image_height, visual_image_width, CV_8UC1);
        cv::cvtColor(*image_,*image_,CV_GRAY2BGR);
    }

    if(image_->empty()){
        AERROR << "Visual Image create failed!";
        return;
    }

    cv::Mat tmp_img = image_->clone();

    VisualizeReferenceLines(lines, tmp_img);
    VisualizePaths(paths, tmp_img);
    VisualizeGridMap(grid_map, tmp_img);

    cv::imshow("DebugView", tmp_img);
    cv::waitKey(FLAGS_planning_cycle);
}

void VisualHandle::Visualize(const std::list<planning::ReferenceLine> &lines,
                             const std::list<planning::PathData> &paths,
                             const perception::GridMap &grid_map, const planning::PathData &path){
    if(!image_){
        image_ = std::make_unique<cv::Mat>(visual_image_height, visual_image_width, CV_8UC1);
        cv::cvtColor(*image_,*image_,CV_GRAY2BGR);
    }

    if(image_->empty()){
        AERROR << "Visual Image create failed!";
        return;
    }

    cv::Mat tmp_img = image_->clone();

    VisualizeReferenceLines(lines, tmp_img);
    VisualizePaths(paths, tmp_img);
    VisualizeGridMap(grid_map, tmp_img);
    VisualizePath(path, tmp_img);


    cv::imshow("DebugView", tmp_img);
    cv::waitKey(FLAGS_planning_cycle);
}

void VisualHandle::ShowGrid(){
    size_t lateral_points_number = static_cast<uint32_t>(ogm_config_.width() / ogm_config_.width_resolution()) + 1;

    for(size_t i = 0; i < lateral_points_number; ++i){
        cv::Point2d up_point, low_point;

        CalcImagePose(-ogm_config_.left_width() + i * ogm_config_.width_resolution(),
                      ogm_config_.length() - ogm_config_.back_lenght(), &up_point.x, &up_point.y);

        CalcImagePose(-ogm_config_.left_width() + i * ogm_config_.width_resolution(),
                      -ogm_config_.back_lenght(), &low_point.x, &low_point.y);

        cv::line(*image_,up_point,low_point,cv::Scalar(30,30,30),1,CV_AA);

    }

    size_t longitudinal_points_number =
            static_cast<uint32_t>(ogm_config_.length() / ogm_config_.length_resolution()) + 1;

    for(size_t i = 0; i < longitudinal_points_number; ++i){
        cv::Point2d left_point, right_point;

        CalcImagePose(-ogm_config_.left_width(), -ogm_config_.back_lenght() + i * ogm_config_.length_resolution(),
                      &left_point.x, &left_point.y);

        CalcImagePose(ogm_config_.width() - ogm_config_.left_width(), -ogm_config_.back_lenght() + i * ogm_config_.length_resolution(),
                      &right_point.x, &right_point.y);

        cv::line(*image_,left_point,right_point,cv::Scalar(30,30,30),1,CV_AA); //86,87,88
    }

    return;
}

void VisualHandle::ShowVehicleBox(){
    cv::Point2d vehicle_pos;

    CalcImagePose(0, 0, &vehicle_pos.x, &vehicle_pos.y);

    cv::Point2d left_front_pos;
    left_front_pos.x = vehicle_pos.x - common::VehicleParamHandle::GetParam().right_edge_to_center() * scale_;
    left_front_pos.y = vehicle_pos.y - common::VehicleParamHandle::GetParam().front_edge_to_center() * scale_;

    cv::Rect2d vehicle_rect(left_front_pos.x, left_front_pos.y,
                            common::VehicleParamHandle::GetParam().width() * scale_,
                            common::VehicleParamHandle::GetParam().length() * scale_);
    cv::rectangle(*image_, vehicle_rect, cv::Scalar(0, 0, 255),-1, cv::LINE_8,0);

    return;
}

void VisualHandle::ShowCoordinateLines(){
    double width_resolution = visual_image_width / 5.0;
    double length_resolution = visual_image_height / 10.0;

    double up_x = width_resolution;

    for(; up_x < visual_image_width;){
        cv::Point2d up_point, low_point;
        up_point.x = up_x;
        up_point.y = 0;

        low_point.x = up_x;
        low_point.y = visual_image_height;

        up_x += width_resolution;

        cv::line(*image_,up_point,low_point,cv::Scalar(86,87,88),1,CV_AA);

    }

    double left_y = length_resolution;

    for(; left_y < visual_image_height;){
        cv::Point2d left_point, rigth_point;
        left_point.x = 0;
        left_point.y = left_y;

        rigth_point.x = visual_image_width;
        rigth_point.y = left_y;

        cv::line(*image_,left_point,rigth_point,cv::Scalar(86,87,88),1,CV_AA);

        left_y += length_resolution;
    }

    return;
}

void VisualHandle::VisualizeGridMap(const perception::GridMap &grid_map, cv::Mat img){
    cv::Point2d grid_pos;
    for(int32_t i = 0; i < grid_map.num_rows; ++i){
        for(int32_t j = 0; j < grid_map.num_cols; ++j){
            if(grid_map.array[i][j] == 0){

                double front_left_x = ogm_config_.width_resolution() * j - ogm_config_.left_width();
                double front_left_y = ogm_config_.length() - i * ogm_config_.length_resolution() - ogm_config_.back_lenght();

                CalcImagePose(front_left_x, front_left_y, &grid_pos.x, &grid_pos.y);
                cv::Rect2d rect(grid_pos.x, grid_pos.y,
                                ogm_config_.width_resolution() * scale_, ogm_config_.length_resolution() * scale_);
                cv::rectangle(img, rect, cv::Scalar(255, 255, 255),-1, cv::LINE_8,0);

            }
        }
    }

    return;
}

void VisualHandle::VisualizeReferenceLine(const planning::ReferenceLine &ref_line, cv::Mat img){
    PaintLine(ref_line.reference_points().begin(), ref_line.reference_points().end(), img,
              cv::Scalar(255,255,255), 1, CV_AA, 1, 2);
}

void VisualHandle::VisualizeReferenceLines(const std::list<planning::ReferenceLine> &lines, cv::Mat img){
    auto it = lines.begin();
    for(it = lines.begin(); it != lines.end(); it++){
        PaintLine(it->reference_points().begin(),it->reference_points().end(), img,
                  cv::Scalar(255,255,255), 1, CV_AA, 1, 2);
    }
}

void VisualHandle::VisualizePath(const planning::PathData& path, cv::Mat img){
    PaintLine(path.discretized_path().begin(), path.discretized_path().end(), img,
              cv::Scalar(255,0,0), 1, CV_AA, 1, 2);
}

void VisualHandle::VisualizePaths(const std::list<planning::PathData> &paths, cv::Mat img){
    auto it = paths.begin();
    for(it = paths.begin(); it != paths.end(); it++){
        PaintLine(it->discretized_path().begin(), it->discretized_path().end(), img,
                  cv::Scalar(0,255,0), 1, CV_AA, 1, 2);
    }
}

}
