#ifndef MAP_PNC_PATH_PNC_PATH_H_
#define MAP_PNC_PATH_PNC_PATH_H_

#include <memory>

#include "map/road_map/road_map.h"
#include "map/map_path/map_path.h"
#include "map/proto/pnc_path_config.pb.h"

namespace map {

template <class GeoObject>
class ObjectWithAABox {
public:
    ObjectWithAABox(const common::math::AABox2d &aabox,
                    const GeoObject *geo_object,
                    const int index)
        : aabox_(aabox),
          geo_object_(geo_object),
          index_(index) {}
    ~ObjectWithAABox() {}

    const common::math::AABox2d &aabox() const { return aabox_; }

    double DistanceTo(const common::math::Vec2d &point) const {
        return geo_object_->DistanceTo(point);
    }

    double DistanceSquareTo(const common::math::Vec2d &point) const {
        return geo_object_->DistanceSquareTo(point);
    }

    const GeoObject *geo_object() const { return geo_object_; }

    int index() const { return index_; }

private:
    common::math::AABox2d aabox_;
    const GeoObject *geo_object_;
    int index_;
};

using LaneSegmentBox = ObjectWithAABox<common::math::LineSegment2d>;
using LaneSegmentKDTree = common::math::AABoxKDTree2d<LaneSegmentBox>;

class PncPath
{
public:
    explicit PncPath(RoadMap* const road_map);

    virtual ~PncPath() = default;

public:
    bool Init();

    bool stop_for_destination() const {return stop_for_destination_;}

    bool GetNearbyPoints(const common::VehicleState& vehicle_state,
                         std::vector<MapPathPoint>* points);

private:
    bool InitLaneSegmentKDTree();

    template <class KDTree>
    static bool SearchObjects(const common::math::Vec2d& center, const double radius,
                              const KDTree& kdtree, std::vector<int>* const result_indices);

    /**
      * @brief : nearest_index is the sequence number of nearest point in points_, in other words, the
      * nearest_index represents the original MapPathPoint rather than the interpolation point
      */
    bool GetNearestPointWithTheta(const common::math::Vec2d& point, const double& central_theta,
                                  const double& max_distance, const double& max_theta_diff,
                                  int* nearest_index, double* s, double* l) const;

    bool ExtendSegments(const int& central_index, const double& start_s,
                        const double& end_s, std::vector<MapPathPoint>* points) const;

    MapPathPoint LinearInterpolation(const double& prev_s, const MapPathPoint& prev_point,
                                     const double& next_s, const MapPathPoint& next_point,
                                     const double& target_s) const;

private:
    RoadMap *road_map_ = nullptr;
    std::unique_ptr<MapPath> map_path_;

private:
    std::vector<LaneSegmentBox> lane_segment_boxes_;
    std::unique_ptr<LaneSegmentKDTree> lane_segment_kdtree_;

    bool stop_for_destination_ = false;
private:
    PncPathConfig config_;


};

}


#endif

