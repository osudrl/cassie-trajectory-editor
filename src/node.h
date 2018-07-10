#ifndef NODE_H
#define NODE_H

#include "mujoco.h"
#include "glfw3.h"
#include "main.h"
#include <stdio.h>

struct _cassie_body_id_t_
{
    uint8_t id;
};
typedef struct _cassie_body_id_t_ cassie_body_id_t;

struct _node_body_id_t_
{
    uint8_t id;
};
typedef struct _cassie_body_id_t_ node_body_id_t;

typedef mjtNum* v3_t;

void node_dropped(traj_info_t* traj_info, int selected_cassie_body_id, int selected_node_body_id);
void node_position_initial_using_cassie_body(traj_info_t* traj_info,  cassie_body_id_t body_id);
void move_body_to_pert_refpos(traj_info_t* traj_info, int joint_start_index);
int node_body_index_to_joint_index(int bodyindex);

#endif

