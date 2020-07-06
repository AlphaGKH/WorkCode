#include "perception/ogm_perception/ogm_perception.h"

#include <math.h>

#include "common/util/file.h"
#include "common/math/math_utils.h"
#include "common/vehicle_param_handle/vehicle_param_handle.h"

namespace perception {

bool OgmPerception::Init(const OgmConfig& config){

    ogm_config_ = config;

    cols_ = static_cast<int32_t>(ogm_config_.width() / ogm_config_.width_resolution());
    rows_ = static_cast<int32_t>(ogm_config_.length() / ogm_config_.length_resolution());
    angle_resolution_ = ogm_config_.lidar2d_config().scaningangle() / (ogm_config_.lidar2d_config().linesnumber() - 1);

    return true;
}

bool OgmPerception::Process(const Lidar2D& lidar2d, GridMap *grid_map){

    if(ogm_config_.enable_ogm_fill()){
        return FilledOGM(lidar2d, grid_map);
    }
    else if (ogm_config_.enable_ogm_expand()) {
        return ExpandedOGM(lidar2d, grid_map);
    }
    else {
        return SimpleOGM(lidar2d, grid_map);
    }
}

bool OgmPerception::SimpleOGM(const Lidar2D &lidar2d, GridMap *grid_map){
    int32_t number = lidar2d.distances.size();
    if(number < 1)
        return false;

    grid_map->num_rows = rows_;
    grid_map->num_cols = cols_;

    std::vector<int8_t> temp(cols_,1);
    grid_map->array.resize(rows_,temp);

    for (int32_t i = 0 ; i < number; i++){
        const auto& point = lidar2d.distances.at(i);
        const auto& angle = (-5 + i*angle_resolution_)/180*M_PI;
        const auto& x = point * std::cos(angle);
        const auto& y = point * std::sin(angle) + 4;

        if(((x < ogm_config_.width() - ogm_config_.left_width()) && (x > - ogm_config_.left_width())) &&
                ((y < ogm_config_.length() - ogm_config_.back_lenght()) && (y > - ogm_config_.back_lenght()))){

            int32_t x_cell = floor((x + ogm_config_.left_width())/ ogm_config_.width_resolution());
            int32_t y_cell = floor((y + ogm_config_.back_lenght()) / ogm_config_.length_resolution());

            x_cell = (x_cell < cols_ ? x_cell : cols_ - 1);
            x_cell = (x_cell < 0 ? 0 : x_cell);
            y_cell = (y_cell < rows_ ? y_cell : rows_ - 1);
            y_cell = (y_cell < 0 ? 0 : y_cell);

            grid_map->array[rows_ - y_cell][x_cell] = 0;
        }
    }

    grid_map->timestamp = lidar2d.timestamp;

    return true;
}

bool OgmPerception::FilledOGM(const Lidar2D &lidar2d, GridMap *grid_map){
    int32_t number = lidar2d.distances.size();
    if(number < 1)
        return false;

    grid_map->num_rows = rows_;
    grid_map->num_cols = cols_;

    std::vector<int8_t> temp(cols_,2);
    grid_map->array.resize(rows_,temp);

    for (int32_t i = 0 ; i < number; i++){
        const auto& point = lidar2d.distances.at(i);
        const auto& angle = (-5 + i*angle_resolution_)/180*M_PI;
        const auto& x = point * std::cos(angle);
        const auto& y = point * std::sin(angle) + 4;

        if(((x < ogm_config_.width() - ogm_config_.left_width()) && (x > - ogm_config_.left_width())) &&
                ((y < ogm_config_.length() - ogm_config_.back_lenght()) && (y > - ogm_config_.back_lenght()))){

            int32_t x_cell = floor((x + ogm_config_.left_width())/ ogm_config_.width_resolution());
            int32_t y_cell = floor((y + ogm_config_.back_lenght()) / ogm_config_.length_resolution());

            x_cell = (x_cell < cols_ ? x_cell : cols_ - 1);
            x_cell = (x_cell < 0 ? 0 : x_cell);
            y_cell = (y_cell < rows_ ? y_cell : rows_ - 1);
            y_cell = (y_cell < 0 ? 0 : y_cell);

            grid_map->array[rows_ - y_cell][x_cell] = 0;
        }
        else {
            double r = 0.5;
            double nx = 0.0, ny = 0.0;
            while (true) {
                nx = r * std::cos(angle);
                ny = r * std::sin(angle) + 4;
                if(((nx <= ogm_config_.width() - ogm_config_.left_width()) && (nx >= - ogm_config_.left_width())) &&
                        ((ny <= ogm_config_.length() - ogm_config_.back_lenght()) && (ny >= - ogm_config_.back_lenght()))){

                    int32_t nx_cell = common::math::RoundNum((nx + ogm_config_.left_width())/ ogm_config_.width_resolution());
                    int32_t ny_cell = common::math::RoundNum((ny + ogm_config_.back_lenght()) / ogm_config_.length_resolution());

                    nx_cell = (nx_cell < cols_ ? nx_cell : cols_ - 1);
                    nx_cell = (nx_cell < 0 ? 0 : nx_cell);
                    ny_cell = (ny_cell < rows_ ? ny_cell : rows_ - 1);
                    ny_cell = (ny_cell < 0 ? 0 : ny_cell);


                    grid_map->array[rows_ - ny_cell][nx_cell] = 1;
                }
                else {
                    break;
                }
                r += 0.5;
            }
        }
    }

    for(int c = 0; c < grid_map->num_cols; ++c){
        int r = grid_map->num_rows - 1;
        while (r >= 0) {
            if(r > (ogm_config_.length() - ogm_config_.back_lenght())/ogm_config_.length_resolution()){
                grid_map->array[r][c] = 1;
                r--;
                continue;
            }

            if(grid_map->array[r][c] == 2){
                grid_map->array[r][c] = grid_map->array[r+1][c];

            }
            r--;
        }
    }

    grid_map->timestamp = lidar2d.timestamp;

    return true;
}

bool OgmPerception::ExpandedOGM(const Lidar2D &lidar2d, GridMap *grid_map){
    int32_t number = lidar2d.distances.size();
    if(number < 1)
        return false;

    grid_map->num_rows = rows_;
    grid_map->num_cols = cols_;

    std::vector<int8_t> temp(cols_,1);
    grid_map->array.resize(rows_,temp);

    for (int32_t i = 0 ; i < number; i++){
        const auto& point = lidar2d.distances.at(i);
        const auto& angle = (-5 + i*angle_resolution_)/180*M_PI;
        const auto& x = point * std::cos(angle);
        const auto& y = point * std::sin(angle) + 4;

        if(((x < ogm_config_.width() - ogm_config_.left_width()) && (x > - ogm_config_.left_width())) &&
                ((y < ogm_config_.length() - ogm_config_.back_lenght()) && (y > - ogm_config_.back_lenght()))){

            int32_t y_cell = floor((y + ogm_config_.back_lenght()) / ogm_config_.length_resolution());
            y_cell = (y_cell < rows_ ? y_cell : rows_ - 1);
            y_cell = (y_cell < 0 ? 0 : y_cell);

            double expand_x = x - common::VehicleParamHandle::GetParam().left_edge_to_center();
            double right_x = x + common::VehicleParamHandle::GetParam().right_edge_to_center();

            while(expand_x <= right_x){

                int32_t expand_x_cell = floor((expand_x + ogm_config_.left_width())/ ogm_config_.width_resolution());

                expand_x_cell = (expand_x_cell < cols_ ? expand_x_cell : cols_ - 1);
                expand_x_cell = (expand_x_cell < 0 ? 0 : expand_x_cell);
                grid_map->array[rows_ - y_cell][expand_x_cell] = 0;

                expand_x += ogm_config_.width_resolution();
            }
        }
    }

    grid_map->timestamp = lidar2d.timestamp;

    return true;
}

}
