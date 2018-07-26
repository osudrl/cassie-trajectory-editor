
#ifndef CONTROL_H
#define CONTROL_H


#include "glfw3.h"
#include "mujoco.h"
#include "main.h"
#include <stdlib.h>

void control_key_event(traj_info_t* traj_info, int key, int mods);

#endif

