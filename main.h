#include "mujoco.h"
#include "glfw3.h"
#include <stdio.h>

#define CASSIE_QPOS_SIZE 34

void traj_foreach_frame(mjModel* m, mjData* d, mjvPerturb* pert);
