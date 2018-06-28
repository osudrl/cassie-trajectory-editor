#include "mujoco.h"
#include "glfw3.h"
#include <stdio.h>

#define CASSIE_QPOS_SIZE 34

struct _traj_required_info_
{
	mjModel* m;
	mjData* d;
	mjvPerturb* pert;
};
typedef struct _traj_required_info_ traj_required_info_t;


void traj_foreach_frame(traj_required_info_t* traj_info);
