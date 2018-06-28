#include "mujoco.h"
#include "glfw3.h"
#include <stdio.h>

#define CASSIE_QPOS_SIZE 34

struct _traj_info_
{
	mjModel* m;
	mjData* d;
	mjvPerturb* pert;
};
typedef struct _traj_info_ traj_info_t;

void traj_foreach_frame(traj_info_t* traj_info);
