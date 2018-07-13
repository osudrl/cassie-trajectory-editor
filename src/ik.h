#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#include "main.h"
#include "mujoco.h"

double ik_better_body_optimizer(
    traj_info_t* traj_info,
    double* xyz_xpos_target, 
    int body_id_end);
void ik_iterative_better_body_optimizer(
    traj_info_t* traj_info,
    double* xyz_xpos_target, 
    int body_id_end,
    int frame,
    int count);

#endif
