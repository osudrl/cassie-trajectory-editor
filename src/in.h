#ifndef IN_H
#define IN_H

#include "mujoco.h"
#include "glfw3.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

struct __attribute__ ((packed)) _full_traj_state_t_
{
    double time;
    double qpos[35];
    double qvel[32];
    double torque[10];
    double mpos[10];
    double mvel[10];
};
typedef struct _full_traj_state_t_ full_traj_state_t;

void in_my_qposes(traj_info_t* traj_info);

#endif

