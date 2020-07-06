#include "map/road_map/road_map.h"
#include "map/common/gflags_map.h"

#include "common/util/file.h"
#include "common/gflags_common.h"
#include "map/proto/created_map.pb.h"

#include <fstream>

namespace map {

bool RoadMap::Init(){

    if(common::util::PathExists(FLAGS_road_map_pb_file)){
        LoadRoadMapProto();
    }
    else if (common::util::PathExists(FLAGS_road_map_file)) {
        LoadRoadMap();
    }
    else {
        AERROR << FLAGS_road_map_pb_file  << " And " << FLAGS_road_map_file << " all not exist!";
        return false;
    }


    if( (map_path_points_.size() < 2) || !Revise()){
        AERROR << "RoadMap Failed Get Path from: " << FLAGS_road_map_file << " or " << FLAGS_road_map_pb_file;
        return false;
    }

    AINFO << "RoadMap Path Loaded Successfully!";
    return true;
}

void RoadMap::LoadRoadMap(){
    map_path_points_.clear();

    std::ifstream in_file(FLAGS_road_map_file.c_str(), std::ifstream::in);
    std::string line;

    int id = 0;
    double x = 0;
    double y = 0;
    double theta = 0;

    while (getline(in_file,line)) {
        std::istringstream iss(line);
        iss >> id;
        iss >> x;
        iss >> y;
        iss >> theta;
        map_path_points_.emplace_back(x,y,theta);
    }

    return;
}

void RoadMap::LoadRoadMapProto(){
    map_path_points_.clear();

    CreatedMap map;

    CHECK(common::util::GetProtoFromFile(FLAGS_road_map_pb_file, &map))
            << "Unable to parse road map proto file " << FLAGS_road_map_pb_file;

    for(const auto& p : map.points()){
        map_path_points_.emplace_back(p.x(), p.y(), p.theta(), p.lane_left_width(), p.lane_right_width());
    }

    return;
}

bool RoadMap::Revise(){
    const auto map_points_size = map_path_points_.size();
    MapPathPoint* curr_point = &map_path_points_.front();
    for(size_t i = 1; i < map_points_size; ++i){
        MapPathPoint* next_point = &map_path_points_.at(i);
        common::math::LineSegment2d line_segment
                ({curr_point->x(),curr_point->y()},{next_point->x(),next_point->y()});
        curr_point->set_theta(line_segment.theta());
        curr_point = next_point;
    }
    const auto last_second_point = map_path_points_.at(map_points_size - 2);
    curr_point->set_theta(last_second_point.theta());

    return true;
}

}
