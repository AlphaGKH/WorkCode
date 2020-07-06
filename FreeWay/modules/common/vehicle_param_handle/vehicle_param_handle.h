#pragma once

#include <string>

#include "modules/common/proto/vehicle_param.pb.h"
#include "cyber/common/macros.h"

namespace dharma {

namespace common {

class VehicleParamHandle
{
public:

    static void Init();

    static void Init(const VehicleParam &param);

    static void Init(const std::string &param_file);

    static const VehicleParam& GetParam();

    /**
      * @brief Get the safe turning radius when the vehicle is turning with
      * maximum steering angle.
      *
      * The calculation is described by the following figure.
      *  <pre>
      *
      *
      *    front of car
      * A +----------+ B
      *   |          |
      *   /          / turn with maximum steering angle
      *   |          |
      *   |          |
      *   |          |
      *   |    X     |                                       O
      *   |<-->.<----|-------------------------------------->* (turn center)
      *   |          |   VehicleParam.min_turn_radius()
      *   |          |
      * D +----------+ C
      *    back of car
      *
      *  </pre>
      *
      *  In the above figure, The four corner points of the vehicle is A, B, C, and
      * D. XO is VehicleParam.min_turn_radius(), X to AD is left_edge_to_center,
      * X to AB is VehicleParam.front_edge_to_center(). Then
      *     AO = sqrt((XO +  left_edge_to_center) ^2 + front_edge_to_center^2).
      * @return AO in the above figure, which is the maximum turn radius when the
      * vehicle turns with maximum steering angle
      */

    static double MinSafeTurnRadius();

private:
    static VehicleParam vehicle_param_;
    static bool is_init_;
    DECLARE_SINGLETON(VehicleParamHandle);

};

}

}
