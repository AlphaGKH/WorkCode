/**
  * @projectName : MociusAuto
  * @brief       :
  * @author      : guokonghui
  * @date        : 2019-10-12
  */

/**
 * @file
 * @brief Exports the SIN_TABLE, used by the Angle class.
 */

#ifndef COMMON_MATH_SIN_TABLE_H_
#define COMMON_MATH_SIN_TABLE_H_


namespace common {
namespace math {

//! Used by Angle class to speed-up computation of trigonometric functions.
#define SIN_TABLE_SIZE 16385
extern const float SIN_TABLE[SIN_TABLE_SIZE];

}  // namespace math
}  // namespace common

#endif /* MODULES_COMMON_MATH_SIN_TABLE_H_ */
