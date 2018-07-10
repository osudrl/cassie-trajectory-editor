#include "main.h"

/*void traj_fill_single_body_xpos(traj_info_t* traj_info, double* body_xposes, int selectedbody, int currframe)
{
    timeline_set_qposes_to_pose_frame(traj_info, currframe);
    mj_forward(traj_info->m, traj_info->d);
    body_xposes[0] = traj_info->d->xpos[selectedbody * 3 + 0] - traj_info->d->xpos[1* 3 + 0];
    body_xposes[1] = traj_info->d->xpos[selectedbody * 3 + 1] - traj_info->d->xpos[1* 3 + 1];
    body_xposes[2] = traj_info->d->xpos[selectedbody * 3 + 2] - traj_info->d->xpos[1* 3 + 2];
}
*/
void traj_fill_single_body_xpos(traj_info_t* traj_info, double* body_xposes, int selectedbody, int currframe)
{
    timeline_set_qposes_to_pose_frame(traj_info, currframe);
    mj_forward(traj_info->m, traj_info->d);
    body_xposes[0] = traj_info->d->xpos[selectedbody * 3 + 0];
    body_xposes[1] = traj_info->d->xpos[selectedbody * 3 + 1];
    body_xposes[2] = traj_info->d->xpos[selectedbody * 3 + 2];
}

void traj_fill_body_xposes(traj_info_t* traj_info, double* body_xposes, int selectedbody)
{
    int i;
    int startframe;
    int currframe;

    if(!traj_info->timeline.init)
        in_init_timeline(traj_info);

    startframe = timeline_get_frame_from_time(traj_info);

    for (i = 0; i < NODECOUNT; i++)
    {
        currframe = (TIMELINE_SIZE / NODECOUNT) * i;
        traj_fill_single_body_xpos(traj_info, body_xposes + (3 * i), selectedbody, currframe);
    }

    timeline_set_qposes_to_pose_frame(traj_info, startframe);
    
}

void traj_position_nodes(traj_info_t* traj_info, int selectedbody)
{
    double body_xposes[NODECOUNT * 3];
    int i;

    if(!traj_info->timeline.init)
        in_init_timeline(traj_info);

    traj_fill_body_xposes(traj_info, body_xposes, selectedbody);

    for (i = 0; i < NODECOUNT; i++)
    {
        traj_info->d->qpos[CASSIE_QPOS_SIZE + (NODE_OFFSET * 3) + (i * 3) + 0] = body_xposes[i * 3 + 0];
        traj_info->d->qpos[CASSIE_QPOS_SIZE + (NODE_OFFSET * 3) + (i * 3) + 1] = body_xposes[i * 3 + 1];
        traj_info->d->qpos[CASSIE_QPOS_SIZE + (NODE_OFFSET * 3) + (i * 3) + 2] = body_xposes[i * 3 + 2];
    }
    mj_forward(traj_info->m, traj_info->d);
}

int traj_last_select_id = 0;
int traj_last_non_node_select_id = 0;
int traj_last_activenum = 0;

double traj_gauss(double r, double s)
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

    filter = traj_gauss(frame_offset/100.0, 1) *(1/0.318310);
   
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

int allow_pelvis_to_be_grabbed_and_moved(traj_info_t* traj_info, double* xyz_ref)
{
    if (traj_info->pert->select != traj_last_select_id &&
            traj_info->pert->select > 0 &&
            traj_info->pert->select <= 25) //notanode
    {
        traj_position_nodes(traj_info, traj_info->pert->select);
        traj_last_non_node_select_id = traj_info->pert->select;        
    }

    if(traj_info->pert->active) 
    {
        traj_last_select_id = traj_info->pert->select;
        traj_last_activenum = traj_info->pert->active;

        if(traj_info->pert->select == 1)
        {
            move_body_to_pert_refpos(traj_info, 0);
            return 0;
        }
        else if (traj_info->pert->select > 25) //isanode
        {
            move_body_to_pert_refpos(traj_info, node_body_index_to_joint_index(traj_info->pert->select));
            return 0;
        }
        else
        {
            xyz_ref[0] = traj_info->pert->refpos[0];
            xyz_ref[1] = traj_info->pert->refpos[1];
            xyz_ref[2] = traj_info->pert->refpos[2];
            return 1;
        }
    }
    else if (traj_last_activenum == 1 && traj_last_select_id > 25)
    {
        node_dropped(traj_info, traj_last_non_node_select_id, traj_last_select_id);

        traj_last_select_id = traj_info->pert->select;
        traj_last_activenum = traj_info->pert->active;
    }
    return 0;
}

uint64_t traj_time_in_micros()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return 1000000 * tv.tv_sec + tv.tv_usec;
}

uint64_t traj_calculate_runtime_micros(traj_info_t* traj_info)
{
    uint64_t val;
    if (!(*traj_info->paused))
    {
        val = traj_time_in_micros() - traj_info->time_start;
        traj_info->time_frozen = val;
        return val;
    }
    else
    {
        traj_info->time_start = traj_time_in_micros() - traj_info->time_frozen;
        return traj_info->time_frozen;
    }
}

int traj_foreach_frame_lastmod = 0;

void traj_foreach_frame(traj_info_t* traj_info)
{
    double xyz_xpos_target[3];
    int mod;
    double x = 0;
    mod = allow_pelvis_to_be_grabbed_and_moved(traj_info,xyz_xpos_target);
    if(mod && mod != traj_foreach_frame_lastmod)
    {
        traj_foreach_frame_lastmod = mod;
    }
    else if(!mod)
        traj_foreach_frame_lastmod = mod;

    for(int z = 0; mod && z < 20; z++)
    {
        ik_better_body_optimizer(traj_info,
            xyz_xpos_target,
            traj_info->pert->select);
    }

    in_my_qposes(traj_info);
    mj_forward(traj_info->m, traj_info->d);
}

