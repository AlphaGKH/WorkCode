#ifndef PLANNING_REFERENCE_LINE_REFERENCE_LINE_PROVIDER_H_
#define PLANNING_REFERENCE_LINE_REFERENCE_LINE_PROVIDER_H_

#include <thread>

#include "common/proto/vehicle_state.pb.h"

#include "map/pnc_path/pnc_path.h"

#include "planning/common/gflags_planning.h"
#include "planning/reference_line/reference_line.h"
#include "planning/reference_line/reference_line_smoother.h"

namespace planning {

class ReferenceLineProvider
{
public:
    explicit ReferenceLineProvider(map::RoadMap* const road_map, const ReferenceLineProviderConfig& config);
    ~ReferenceLineProvider();

public:

    bool Start();

    void UpdateVehicleState(const common::VehicleState& adc_state);

    bool GetReferenceLine(ReferenceLine* ref_line);

    void Stop();

private:
    void ReferenceLineProviderThread();

    bool CreatNearbyPoints(const common::VehicleState &vehicle_state,
                           std::vector<map::MapPathPoint>* points);

    bool CreatReferenceLine(ReferenceLine* reference_line);

    bool SmoothReferenceLine(const ReferenceLine& raw_reference_line, ReferenceLine* reference_line);

    bool Shrink(const common::SLPoint& sl, ReferenceLine* reference_line);

    void UpdateReferenceLine(const ReferenceLine& reference_line);

    // for smooth reference_line
private:
    void GetAnchorPoints(const ReferenceLine &reference_line,
                         std::vector<AnchorPoint> *anchor_points) const;

    AnchorPoint GetAnchorPoint(const ReferenceLine& reference_line,
                               double s) const;

    bool IsReferenceLineSmoothValid(
            const ReferenceLine &raw, const ReferenceLine &smoothed) const;

private:
    bool is_inited_ = false;
    bool is_stop_ = true;

    std::unique_ptr<map::PncPath> pnc_path_;

    std::mutex vehicle_state_mutex_;
    common::VehicleState vehicle_state_;
    bool vehicle_state_update_ = false;


    std::unique_ptr<std::thread> reference_line_provider_thread_ptr_;

    std::mutex reference_line_mutex_;
    ReferenceLine reference_line_;
    ReferenceLine last_reference_line_;

    double last_calculation_time_ = 0.0;

    int update_counter_ = 10;

// smoother
private:
    std::unique_ptr<ReferenceLineSmoother> smoother_;
private:
    ReferenceLineProviderConfig provider_config_;

};

}

#endif
