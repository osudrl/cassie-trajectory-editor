#ifndef NODE_H
#define NODE_H

#include "mujoco.h"
#include "glfw3.h"
#include "main.h"
#include <stdio.h>

void node_dropped(traj_info_t* traj_info, int selected_cassie_body_id, int selected_node_body_id);
void traj_position_nodes(traj_info_t* traj_info, int selectedbody);
void move_body_to_pert_refpos(traj_info_t* traj_info, int joint_start_index);
int node_body_index_to_joint_index(int bodyindex);

#endif

