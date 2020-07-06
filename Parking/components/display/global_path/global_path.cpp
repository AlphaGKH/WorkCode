#include "display/global_path/global_path.h"

#include "common/util/file.h"
#include "common/vehicle_param_handle/vehicle_param_handle.h"

namespace display {

GlobalPath::GlobalPath(map::RoadMap* const road_map)
    : road_map_(road_map){}

bool GlobalPath::Init(const GlobalViewConfig& config){
    config_.CopyFrom(config);

    if(!road_map_->Init()){
        AERROR << "Display Not get road_map!";
        return false;
    }

    const auto& points = road_map_->map_path_points();

    for (auto it =  points.begin(); it!= points.end(); it++) {
        min_x_ = std::min(min_x_, it->x());
        max_x_ = std::max(max_x_, it->x());
        min_y_ = std::min(min_y_, it->y());
        max_y_ = std::max(max_y_, it->y());
    }

    double wight_scale = (config_.map_width() * 0.8) / (max_x_ - min_x_);
    double height_scale = (config_.map_height() * 0.8) / (max_y_ - min_y_);

    scale_ = cvRound(std::min(wight_scale,height_scale));

    image_ = std::make_unique<cv::Mat>(config_.map_height(), config_.map_width(), CV_8UC1);

    cv::cvtColor(*image_,*image_,CV_GRAY2BGR);

    if(!ShowGlobalPath(points)){
        return false;
    }

    return true;
}

bool GlobalPath::ShowGlobalPath(const std::vector<map::MapPathPoint>& points){
    if(image_->empty()){
        AERROR << "GlobalPath Image is not created!";
        return false;
    }

    if(points.size() < 2){
        AERROR << "GlobalPath must has two points at least!";
        return false;
    }

    PaintPath(points.begin(), points.end(), *image_, cv::Scalar(0,255,0), 1, CV_AA, 1, 1);
    if(config_.lane_buondary_display())
    {
        ShowGlobalPathBoundary(points);
    }

    cv::imshow(global_path_name_,*image_);
    cv::waitKey(FLAGS_display_cycle);

    return true;
}

void GlobalPath::Show(const common::VehicleState &adc_state,
                      const std::vector<map::MapPathPoint> &map_path_points,
                      const std::vector<planning::ReferencePoint> &ref_points){
    if(image_->empty()){
        AERROR << "GlobalPath Image is not created!";
        return;
    }

    cv::Mat tmp_img = image_->clone();
    ShowVehicleState(adc_state, tmp_img);
    ShowMapPath(map_path_points, tmp_img);
    ShowReferenceLine(ref_points, tmp_img);

    cv::imshow(global_path_name_,tmp_img);
    cv::waitKey(FLAGS_display_cycle);

    return;
}

void GlobalPath::Show(const common::VehicleState &adc_state){
    if(image_->empty()){
        AERROR << "GlobalPath Image is not created!";
        return;
    }

    cv::Mat tmp_img = image_->clone();
    ShowVehicleState(adc_state, tmp_img);

    cv::imshow(global_path_name_,tmp_img);
    cv::waitKey(FLAGS_display_cycle);

    return;
}

bool GlobalPath::ShowVehicleState(const common::VehicleState& vehicle_state, cv::Mat img){

    cv::Point2d vehicle_pos;

    CalcImagePointPos(vehicle_state.x(), vehicle_state.y(),&vehicle_pos.x, &vehicle_pos.y);

    double theta = -vehicle_state.theta();
    cv::Point2d theta_pos;
    theta_pos.x = vehicle_pos.x +
            common::VehicleParamHandle::GetParam().front_edge_to_center() * scale_ * cos(theta);
    theta_pos.y = vehicle_pos.y +
            common::VehicleParamHandle::GetParam().front_edge_to_center() * scale_ * sin(theta);


    cv::Point2d left_front_pos;
    left_front_pos.x = vehicle_pos.x
            + common::VehicleParamHandle::GetParam().front_edge_to_center() * scale_ * cos(theta)
            - common::VehicleParamHandle::GetParam().left_edge_to_center() * scale_ * sin(theta);
    left_front_pos.y = vehicle_pos.y
            + common::VehicleParamHandle::GetParam().front_edge_to_center() * scale_ * sin(theta)
            + common::VehicleParamHandle::GetParam().left_edge_to_center() * scale_ * cos(theta);

    cv::Point2d left_back_pos;
    left_back_pos.x = vehicle_pos.x
            - common::VehicleParamHandle::GetParam().back_edge_to_center() * scale_ * cos(theta)
            - common::VehicleParamHandle::GetParam().left_edge_to_center() * scale_ * sin(theta);
    left_back_pos.y = vehicle_pos.y
            - common::VehicleParamHandle::GetParam().back_edge_to_center() * scale_ * sin(theta)
            + common::VehicleParamHandle::GetParam().left_edge_to_center() * scale_ * cos(theta);

    cv::Point2d right_front_pos;
    right_front_pos.x = vehicle_pos.x
            + common::VehicleParamHandle::GetParam().front_edge_to_center() * scale_ * cos(theta)
            + common::VehicleParamHandle::GetParam().right_edge_to_center() * scale_ * sin(theta);
    right_front_pos.y = vehicle_pos.y
            + common::VehicleParamHandle::GetParam().front_edge_to_center() * scale_ * sin(theta)
            - common::VehicleParamHandle::GetParam().right_edge_to_center() * scale_ * cos(theta);

    cv::Point2d right_back_pos;
    right_back_pos.x = vehicle_pos.x
            - common::VehicleParamHandle::GetParam().back_edge_to_center() * scale_ * cos(theta)
            + common::VehicleParamHandle::GetParam().right_edge_to_center() * scale_ * sin(theta);
    right_back_pos.y = vehicle_pos.y
            - common::VehicleParamHandle::GetParam().back_edge_to_center() * scale_ * sin(theta)
            - common::VehicleParamHandle::GetParam().right_edge_to_center() * scale_ * cos(theta);



    //    cv::line(img,vehicle_pos,theta_pos,cv::Scalar(0,0,255),2,CV_AA);
    cv::line(img,left_front_pos,left_back_pos,cv::Scalar(0,0,255),2,CV_AA);
    cv::line(img,left_front_pos,right_front_pos,cv::Scalar(0,0,255),2,CV_AA);
    cv::line(img,right_back_pos,right_front_pos,cv::Scalar(0,0,255),2,CV_AA);
    cv::line(img,right_back_pos,left_back_pos,cv::Scalar(0,0,255),2,CV_AA);
    cv::line(img,right_back_pos,left_front_pos,cv::Scalar(0,0,255),2,CV_AA);
    cv::line(img,left_back_pos,right_front_pos,cv::Scalar(0,0,255),2,CV_AA);


    return true;
}

bool GlobalPath::ShowMapPath(const std::vector<map::MapPathPoint>& map_points, cv::Mat img){


    PaintPath(map_points.begin(), map_points.end(), img,
              cv::Scalar(255,255,255), 1, CV_AA, 2, 3);

    return true;
}

bool GlobalPath::ShowReferenceLine(const std::vector<planning::ReferencePoint>& ref_ponits, cv::Mat img){

    PaintPath(ref_ponits.begin(), ref_ponits.end(), img,
              cv::Scalar(0,255,255), 2, CV_AA, 1, 2);

    return true;

}

void GlobalPath::ShowGlobalPathBoundary(const std::vector<map::MapPathPoint> &points){

    constexpr double kThod = 1e-2;

    const size_t points_size = points.size();

    std::vector<common::math::Vec2d> left_points;
    std::vector<common::math::Vec2d> right_points;

    if(points_size == 2){
        for(const auto& p : points){
            double left_x = p.x() - p.lane_left_width() * sin(p.theta());
            double left_y = p.y() + p.lane_left_width() * cos(p.theta());

            double right_x = p.x() + p.lane_right_width() * sin(p.theta());
            double right_y = p.y() - p.lane_right_width() * cos(p.theta());

            left_points.emplace_back(left_x, left_y);
            right_points.emplace_back(right_x, right_y);
        }

    }

    std::vector<common::math::LineSegment2d> left_segments;
    std::vector<common::math::LineSegment2d> right_segments;

    for(size_t i = 0; i + 1 < points_size; ++i){
        const auto curr_point = points[i];
        const auto next_point = points[i + 1];
        common::math::LineSegment2d seg(curr_point, next_point);

        double curr_left_x = curr_point.x() - curr_point.lane_left_width() * sin(seg.theta());
        double curr_left_y = curr_point.y() + curr_point.lane_left_width() * cos(seg.theta());

        double curr_right_x = curr_point.x() + curr_point.lane_right_width() * sin(seg.theta());
        double curr_right_y = curr_point.y() - curr_point.lane_right_width() * cos(seg.theta());

        double next_left_x = next_point.x() - next_point.lane_left_width() * sin(seg.theta());
        double next_left_y = next_point.y() + next_point.lane_left_width() * cos(seg.theta());

        double next_right_x = next_point.x() + next_point.lane_right_width() * sin(seg.theta());
        double next_right_y = next_point.y() - next_point.lane_right_width() * cos(seg.theta());


        left_segments.emplace_back(common::math::Vec2d(curr_left_x, curr_left_y),
                                   common::math::Vec2d(next_left_x, next_left_y));

        right_segments.emplace_back(common::math::Vec2d(curr_right_x, curr_right_y),
                                    common::math::Vec2d(next_right_x, next_right_y));
    }

    auto extract_points_from_segments =
            [this] (const std::vector<common::math::LineSegment2d>& segments,
            std::vector<common::math::Vec2d>* points){
        CHECK_NOTNULL(points);
        const size_t segments_size = segments.size();
        points->emplace_back(segments.front().start());

        common::math::Vec2d intersect_point;

        for(size_t j = 0; j + 1 < segments_size; ++j){
            const auto curr_seg = segments[j];
            const auto next_seg = segments[j + 1];

            if(curr_seg.GetIntersect(next_seg,&intersect_point)){
                points->emplace_back(intersect_point);
            }
            else {
                points->emplace_back(curr_seg.end());
                points->emplace_back(next_seg.start());
            }
        }

        points->emplace_back(segments.back().end());
    };

    extract_points_from_segments(left_segments, &left_points);
    extract_points_from_segments(right_segments, &right_points);

    PaintPath(left_points.begin(), left_points.end(), *image_, cv::Scalar(255,255,255), 2, CV_AA, 2, 2);
    PaintPath(right_points.begin(), right_points.end(), *image_, cv::Scalar(255,255,255), 2, CV_AA, 2, 2);

}

}
