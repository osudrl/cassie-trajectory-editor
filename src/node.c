
#include "node.h"

node_body_id_t node_get_body_id_from_node_index(int index)
{
    node_body_id_t id;
    id.id = index + 27;
    return id;
}

v3_t node_get_qpos_by_node_id(traj_info_t* traj_info, node_body_id_t id)
{
    return traj_info->d->qpos + CASSIE_QPOS_SIZE + (NON_NODE_COUNT * 3) + ((id.id - 27) * 3);
}

v3_t node_get_xpos_by_node_id(traj_info_t* traj_info, node_body_id_t id)
{
    return traj_info->d->xpos + (id.id * 3);
}

v3_t node_get_body_xpos_curr(traj_info_t* traj_info, cassie_body_id_t id)
{
    return traj_info->d->xpos + (id.id * 3);
}

v3_t node_get_body_xpos_by_frame(traj_info_t* traj_info, int frame, cassie_body_id_t id)
{
    timeline_set_qposes_to_pose_frame(traj_info, frame);
    mj_forward(traj_info->m, traj_info->d);
    return node_get_body_xpos_curr(traj_info, id);
}

void node_position_initial_using_cassie_body(traj_info_t* traj_info, cassie_body_id_t body_id)
{
    int i;
    int frame;
    v3_t node_qpos;
    v3_t body_xpos;

    if(!traj_info->timeline.init)
        timeiline_init_from_input_file(traj_info);
    
    for (i = 0; i < NODECOUNT; i++)
    {
        frame = (TIMELINE_SIZE / NODECOUNT) * i;
        node_qpos = node_get_qpos_by_node_id(traj_info, node_get_body_id_from_node_index(i) );
        body_xpos = node_get_body_xpos_by_frame(traj_info, frame, body_id);
        mju_copy3(node_qpos, body_xpos);
    }
    mj_forward(traj_info->m, traj_info->d);
}

double gaussian_distrobution(double r, double s)
{
    s *= 2;
    return (mju_exp(-(r*r)/s))/(mjPI * s) * 2;
}

int node_body_index_to_joint_index(int bodyindex)
{
    bodyindex -= 28;
    bodyindex *= 3;
    bodyindex += 41;
    return bodyindex;
}

void move_body_to_pert_refpos(traj_info_t* traj_info, int joint_start_index)
{
    traj_info->d->qpos[joint_start_index + 0] = traj_info->pert->refpos[0];
    traj_info->d->qpos[joint_start_index + 1] = traj_info->pert->refpos[1];
    traj_info->d->qpos[joint_start_index + 2] = traj_info->pert->refpos[2];
}

void nodeframe_ik_transform(traj_info_t* traj_info, int selected_cassie_body_id, int frame, double* target)
{
    timeline_set_qposes_to_pose_frame(traj_info, frame); // should be repetitive
    ik_iterative_better_body_optimizer(traj_info, target, selected_cassie_body_id, 25);
    timeline_overwrite_frame_using_curr_pose(traj_info, frame);
}

int safe_add_rootframe_to_frame_offset(int rootframe, int frame_offset)
{
    rootframe += TIMELINE_SIZE;
    rootframe += frame_offset;
    rootframe %= TIMELINE_SIZE;
    return rootframe;
}

void scale_target_using_frame_offset(
    traj_info_t* traj_info,
    double* ik_body_target, 
    double* transform_vector,
    int rootframe,
    int frame_offset,
    int selected_cassie_body_id)
{
    double filter;
    double scaled_diff_only[3];

    filter = gaussian_distrobution(frame_offset/100.0, 1) *(1/0.318310);
   
    scaled_diff_only[0] = transform_vector[0] * filter;
    scaled_diff_only[1] = transform_vector[1] * filter;
    scaled_diff_only[2] = transform_vector[2] * filter;

    timeline_set_qposes_to_pose_frame(traj_info, 
        safe_add_rootframe_to_frame_offset(rootframe ,frame_offset));

    mj_forward(traj_info->m,traj_info->d);

    mju_add3(ik_body_target, traj_info->d->xpos + (selected_cassie_body_id*3), scaled_diff_only);
}

int get_frame_from_node_body_id(int selected_node_body_id)
{
    return (TIMELINE_SIZE / NODECOUNT) * (selected_node_body_id - 27); // or maybe 28
}

void calculate_node_dropped_transformation_vector(
    traj_info_t* traj_info, 
    double* transform_vector,
    int selected_cassie_body_id, 
    int selected_node_body_id)
{
    int rootframe;
    double body_init_xpos[3];
    double node_now_xpos[3];

    rootframe = get_frame_from_node_body_id(selected_node_body_id);
    timeline_set_qposes_to_pose_frame(traj_info, rootframe);
    mj_forward(traj_info->m,traj_info->d);

    body_init_xpos[0] = traj_info->d->xpos[selected_cassie_body_id*3 + 0];
    body_init_xpos[1] = traj_info->d->xpos[selected_cassie_body_id*3 + 1];
    body_init_xpos[2] = traj_info->d->xpos[selected_cassie_body_id*3 + 2];

    node_now_xpos[0] = traj_info->d->xpos[selected_node_body_id*3 + 0];
    node_now_xpos[1] = traj_info->d->xpos[selected_node_body_id*3 + 1];
    node_now_xpos[2] = traj_info->d->xpos[selected_node_body_id*3 + 2];


    mju_sub3(transform_vector, node_now_xpos, body_init_xpos);

}

void node_dropped(traj_info_t* traj_info, int selected_cassie_body_id, int selected_node_body_id)
{
    int rootframe;
    int frame_offset;
    double transform_vector[3];
    double ik_body_target[3];

    rootframe = get_frame_from_node_body_id(selected_node_body_id);
    calculate_node_dropped_transformation_vector(
        traj_info, 
        transform_vector, 
        selected_cassie_body_id, 
        selected_node_body_id);

    scale_target_using_frame_offset(
        traj_info,
        ik_body_target, 
        transform_vector,
        rootframe,
        0,
        selected_cassie_body_id);

    nodeframe_ik_transform(traj_info, selected_cassie_body_id, rootframe, ik_body_target);

    for(frame_offset = 1; frame_offset < 400; frame_offset++)
    {
        if(frame_offset % 5 == 0)
        {
            printf("Solving inverse kinematics... %.2f percent \n",(frame_offset+0.0) / 4);
        }
        scale_target_using_frame_offset(
            traj_info,
            ik_body_target, 
            transform_vector,
            rootframe,
            frame_offset,
            selected_cassie_body_id);
        nodeframe_ik_transform(
            traj_info, 
            selected_cassie_body_id, 
            safe_add_rootframe_to_frame_offset(rootframe ,frame_offset), 
            ik_body_target);

        scale_target_using_frame_offset(
            traj_info,
            ik_body_target, 
            transform_vector,
            rootframe,
            -frame_offset,
            selected_cassie_body_id);
        nodeframe_ik_transform(
            traj_info, 
            selected_cassie_body_id, 
            safe_add_rootframe_to_frame_offset(rootframe ,-frame_offset), 
            ik_body_target);

        // timeline_index = (frame_offset + rootframe) % TIMELINE_SIZE;
        // traj_info->timeline.qposes[timeline_index].q[2] += filter;
        // printf("filter %.5f\n", filter);

        // timeline_index = (rootframe + TIMELINE_SIZE - frame_offset) % TIMELINE_SIZE;
        // traj_info->timeline.qposes[timeline_index].q[2] += filter;
    }
}