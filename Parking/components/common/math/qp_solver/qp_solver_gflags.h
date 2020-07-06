#ifndef COMMON_MATH_QP_SOLVER_QP_SOLVER_GFLAGS_H_
#define COMMON_MATH_QP_SOLVER_QP_SOLVER_GFLAGS_H_

#include "gflags/gflags.h"

// math : active set solver
DECLARE_double(default_active_set_eps_num);
DECLARE_double(default_active_set_eps_den);
DECLARE_double(default_active_set_eps_iter_ref);
DECLARE_bool(default_enable_active_set_debug_info);
DECLARE_int32(default_qp_iteration_num);

#endif /* COMMON_MATH_QP_SOLVER_QP_SOLVER_GFLAGS_H_ */
