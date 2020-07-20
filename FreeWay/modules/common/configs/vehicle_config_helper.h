#pragma once

#include <string>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/proto/vehicle_config.pb.h"

#include "modules/common/math/box2d.h"
#include "spider/common/macros.h"

/**
 * @namespace dharma::common
 * @brief dharma::common
 */
namespace dharma {
namespace common {

/**
 * @class VehicleConfigHelper
 *
 * @Brief This is a helper class that can load vehicle configurations. The
 * vehicle configurations are
 * defined modules/common/configs/proto/vehicle_config.proto
 */
class VehicleConfigHelper {
public:
  /**
   * @brief Initialize vehicle configurations with default configuration file
   * pointed by gflags FLAGS_vehicle_config_path. The code will crash if
   * FLAGS_vehicle_config_path does not exist or it points to a file with
   * invalid format.
   */
  static void Init();

  /**
   * @brief Initialize vehicle configurations with \p config
   * @param config A VehicleConfig class instance. The VehicleConfig class is
   * defined by modules/common/configs/proto/vehicle_config.proto.
   */
  static void Init(const VehicleConfig &config);

  /**
   * @brief Initialize vehicle configurations with \p config_file.
   * The code will crash if \p config_file does not exist or \p config_file has
   * invalid format.
   * @param config_file The configuration file path. The format of the file is
   * defined by protobuf file
   * modules/common/configs/proto/vehicle_config.proto.
   */
  static void Init(const std::string &config_file);

  /**
   * @brief Get the current vehicle configuration.
   * @return the current VehicleConfig instance reference.
   */
  static const VehicleConfig &GetConfig();

  /**
   * @brief Get the box (four corners: ABCD) of the vehicle.
   * @param path_point of a vehicle (which contains point X and heading).
   * @return a box2d which contains the ABCD points info.
   */
  static common::math::Box2d
  GetBoundingBox(const common::PathPoint &path_point);

private:
  static VehicleConfig vehicle_config_;
  static bool is_init_;
  DECLARE_SINGLETON(VehicleConfigHelper)
};

} // namespace common
} // namespace dharma
