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
typedef struct _node_body_id_t_ node_body_id_t;



typedef mjtNum* v3_t;

node_body_id_t node_get_body_id_from_node_index(int index);
node_body_id_t node_get_body_id_from_real_body_id(int real);
cassie_body_id_t node_get_cassie_id_from_index(int i);
v3_t node_get_qpos_by_node_id(traj_info_t* traj_info, node_body_id_t id);
v3_t node_get_xpos_by_node_id(traj_info_t* traj_info, node_body_id_t id);
v3_t node_get_body_xpos_curr(traj_info_t* traj_info, cassie_body_id_t id);
v3_t node_get_body_xpos_by_frame(traj_info_t* traj_info, timeline_t* timeline, int frame, cassie_body_id_t id);
void node_position_initial_using_cassie_body(traj_info_t* traj_info, cassie_body_id_t body_id);
double gaussian_distrobution(double r, double s);
void node_refine_pert(
    traj_info_t* traj_info,
    ik_solver_params_t* params);
void node_perform_pert(
    traj_info_t* traj_info,
    ik_solver_params_t* params,
    v3_t grabbed_node_transformation,
    cassie_body_id_t body_id,
    int rootframe);
void node_dropped(traj_info_t* traj_info, cassie_body_id_t body_id, node_body_id_t node_id);
void node_position_scale_visually(
    traj_info_t* traj_info,
    cassie_body_id_t body_id,
    node_body_id_t node_id);
double node_calculate_filter_from_frame_offset(double frame_offset, double sigma, double nodeheight);
void node_calclate_global_target_using_transformation_type(
    traj_info_t* traj_info,
    timeline_t* timeline,
    v3_t global_body_init_xpos_at_rootframe,
    v3_t global_body_target_xpos, 
    v3_t rootframe_transform_vector,
    int rootframe,
    int frame_offset,
    cassie_body_id_t body_id);

#endif

