
#ifndef DECORATE_H
#define DECORATE_H

#include "main.h"
#include "mujoco.h"
#include <stdbool.h>

#include "decor.h"

void decor_single_sphere(traj_info_t* traj_info, mjvScene* scn, int num);
void decor_showdecor(traj_info_t* traj_info, mjvScene* scn);
void decor_reset(traj_info_t* traj_info);
bool decor_has_init(traj_info_t* traj_info);
void decor_positional_init(traj_info_t* traj_info, double* pos);
void decor_positional_addto(traj_info_t* traj_info, double* pos);
void decor_joint_init(traj_info_t* traj_info, double* pos);
void decor_joint_addto(traj_info_t* traj_info, double* pos);

#endif
