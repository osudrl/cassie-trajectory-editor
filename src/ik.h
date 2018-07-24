#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "main.h"
#include "mujoco.h"
#include <inttypes.h>

#define QPOS_TRANSFORM_FULL 1

void ik_default_fill_solver_params(ik_solver_params_t* params);
int ik_iterative_better_body_optimizer(
    traj_info_t* traj_info,
    ik_solver_params_t* params,
    double* xyz_xpos_target, 
    int body_id_end,
    int frameoffset,
    int count);

#endif
