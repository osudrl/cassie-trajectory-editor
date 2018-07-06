#ifndef IN_H
#define IN_H

#include "mujoco.h"
#include "glfw3.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

void in_other_qposes(double* arr);
void in_my_qposes(traj_info_t* traj_info);

#endif

