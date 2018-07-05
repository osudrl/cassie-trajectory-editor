#ifndef MAIN_H
#define MAIN_H

#include "mujoco.h"
#include "glfw3.h"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/wait.h>
#include <stdlib.h>
#include <string.h>

#define CASSIE_QPOS_SIZE 34

struct _traj_info_
{
	mjModel* m;
	mjData* d;
	mjvPerturb* pert;
	mjtNum timer;
};
typedef struct _traj_info_ traj_info_t;

#include "out.h"
#include "in.h"

void traj_foreach_frame(traj_info_t* traj_info);


#endif
