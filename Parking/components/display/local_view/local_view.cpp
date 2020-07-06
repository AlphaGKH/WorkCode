#include "display/local_view/local_view.h"

#include <vector>

#include "common/util/file.h"
#include "common/vehicle_param_handle/vehicle_param_handle.h"

namespace display {

bool LocalView::Init(const LocalViewConfig& config){
    local_config_ = config;
    double wight_scale = (local_config_.local_view_width() * 0.8) / local_config_.ogm_width();
    double height_scale = (local_config_.local_view_height() * 0.8) / local_config_.ogm_length();

    scale_ = cvRound(std::min(wight_scale, height_scale));

    image_ = std::make_unique<cv::Mat>(local_config_.local_view_height(),
                                       local_config_.local_view_width(), CV_8UC1);

    cv::cvtColor(*image_,*image_,CV_GRAY2BGR);

    if(!ShowGrid()){
        return false;
    }

    if(!ShowVehicleBox()){
        return false;
    }

    if(!ShowCoordinateLines()){
        return false;
    }

    cv::imshow(local_view_name_,*image_);
    cv::waitKey(FLAGS_display_cycle);

    return true;

}

void LocalView::Show(const perception::GridMap &grid_map){
    if(image_->empty()){
        AERROR << "LocalView Image is not created!";
        return;
    }

    cv::Mat tmp_img = image_->clone();

    ShowGridMap(grid_map, tmp_img);

    cv::imshow(local_view_name_,tmp_img);
    cv::waitKey(FLAGS_display_cycle);

    return;
}

bool LocalView::ShowGrid(){
    size_t lateral_points_number = static_cast<uint32_t>(local_config_.ogm_width() / local_config_.ogm_width_resolution()) + 1;

    for(size_t i = 0; i < lateral_points_number; ++i){
        cv::Point2d up_point, low_point;

        CalcImagePose(-local_config_.ogm_left_width() + i * local_config_.ogm_width_resolution(),
                      local_config_.ogm_length() - local_config_.ogm_back_lenght(), &up_point.x, &up_point.y);

        CalcImagePose(-local_config_.ogm_left_width() + i * local_config_.ogm_width_resolution(),
                      -local_config_.ogm_back_lenght(), &low_point.x, &low_point.y);

        cv::line(*image_,up_point,low_point,cv::Scalar(30,30,30),1,CV_AA);

    }

    size_t longitudinal_points_number =
            static_cast<uint32_t>(local_config_.ogm_length() / local_config_.ogm_length_resolution()) + 1;

    for(size_t i = 0; i < longitudinal_points_number; ++i){
        cv::Point2d left_point, right_point;

        CalcImagePose(-local_config_.ogm_left_width(), -local_config_.ogm_back_lenght() + i * local_config_.ogm_length_resolution(),
                      &left_point.x, &left_point.y);

        CalcImagePose(local_config_.ogm_width() - local_config_.ogm_left_width(),
                      -local_config_.ogm_back_lenght() + i * local_config_.ogm_length_resolution(),
                      &right_point.x, &right_point.y);

        cv::line(*image_,left_point,right_point,cv::Scalar(30,30,30),1,CV_AA); //86,87,88
    }

    return true;
}

bool LocalView::ShowVehicleBox(){
    cv::Point2d vehicle_pos;

    CalcImagePose(0, 0, &vehicle_pos.x, &vehicle_pos.y);

    cv::Point2d left_front_pos;
    left_front_pos.x = vehicle_pos.x - common::VehicleParamHandle::GetParam().right_edge_to_center() * scale_;
    left_front_pos.y = vehicle_pos.y - common::VehicleParamHandle::GetParam().front_edge_to_center() * scale_;

    cv::Rect2d vehicle_rect(left_front_pos.x, left_front_pos.y,
                            common::VehicleParamHandle::GetParam().width() * scale_,
                            common::VehicleParamHandle::GetParam().length() * scale_);
    cv::rectangle(*image_, vehicle_rect, cv::Scalar(0, 0, 255),-1, cv::LINE_8,0);

    return true;
}

bool LocalView::ShowCoordinateLines(){
    double width_resolution = local_config_.local_view_width() / 5.0;
    double length_resolution = local_config_.local_view_height() / 10.0;

    double up_x = width_resolution;

    for(; up_x < local_config_.local_view_width();){
        cv::Point2d up_point, low_point;
        up_point.x = up_x;
        up_point.y = 0;

        low_point.x = up_x;
        low_point.y = local_config_.local_view_height();

        up_x += width_resolution;

        cv::line(*image_,up_point,low_point,cv::Scalar(86,87,88),1,CV_AA);

    }

    double left_y = length_resolution;

    for(; left_y < local_config_.local_view_height();){
        cv::Point2d left_point, rigth_point;
        left_point.x = 0;
        left_point.y = left_y;

        rigth_point.x = local_config_.local_view_width();
        rigth_point.y = left_y;

        cv::line(*image_,left_point,rigth_point,cv::Scalar(86,87,88),1,CV_AA);

        left_y += length_resolution;
    }

    return true;
}

void LocalView::ShowGridMap(const perception::GridMap &grid_map, cv::Mat img){
    cv::Point2d grid_pos;
    for(int32_t i = 0; i < grid_map.num_rows; ++i){
        for(int32_t j = 0; j < grid_map.num_cols; ++j){
            if(grid_map.array[i][j] == 0){

                double front_left_x = local_config_.ogm_width_resolution() * j - local_config_.ogm_left_width();
                double front_left_y = local_config_.ogm_length() - i * local_config_.ogm_length_resolution()
                        - local_config_.ogm_back_lenght();

                CalcImagePose(front_left_x, front_left_y, &grid_pos.x, &grid_pos.y);
                cv::Rect2d rect(grid_pos.x, grid_pos.y,
                              local_config_.ogm_width_resolution() * scale_,
                              local_config_.ogm_length_resolution() * scale_);
                cv::rectangle(img, rect, cv::Scalar(255, 255, 255),-1, cv::LINE_8,0);

            }
        }
    }

    return;
}

void LocalView::ShowTrajectory(const planning::LcmTrajectory &traj, cv::Mat img){
    return;
}

}
