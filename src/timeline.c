#include "timeline.h"

uint32_t timeline_fill_full_traj_state_array(traj_info_t* traj_info, full_traj_state_t* buf, int bufcount)
{
    FILE* fd_in;
    uint32_t result;

    fd_in = fopen(traj_info->filename_step_data, "r");
    result = fread(buf, sizeof(full_traj_state_t), bufcount, fd_in);

    fclose(fd_in);
    return result;
}

void timeline_copy_a_full_traj_state_to_qpos(qpos_t* qpos, full_traj_state_t* state)
{
    int i;
    
    /* 
    Uncomment for walk in place
    qpos->q[0] = 0;
    qpos->q[1] = 0; 
    */

    for (i = 0; i < CASSIE_QPOS_SIZE; i++)
        qpos->q[i] = state->qpos[i];
}

void timeiline_init_from_input_file(traj_info_t* traj_info)
{
    int i;
    full_traj_state_t buf[TIMELINE_SIZE];

    timeline_fill_full_traj_state_array(traj_info, buf, TIMELINE_SIZE);

    for(i = 0; i < TIMELINE_SIZE; i++)
        timeline_copy_a_full_traj_state_to_qpos(traj_info->timeline.qposes+i, buf+i); //pointer arithmatic

    traj_info->timeline.init = 1;
}

void timeline_set_mj_qpose(traj_info_t* traj_info, qpos_t* desired)
{
    int i;

    for(i = 0; i < CASSIE_QPOS_SIZE; i++)
    {
        traj_info->d->qpos[i] = desired->q[i];
    }
}

int timeline_make_frame_safe(int frame)
{
    while(frame < 0)
        frame += TIMELINE_SIZE;
    frame %= TIMELINE_SIZE;
    return frame;
}

void timeline_set_qposes_to_pose_frame(traj_info_t* traj_info, int frame)
{   
    if(!traj_info->timeline.init)
        timeiline_init_from_input_file(traj_info);

    frame = timeline_make_frame_safe(frame);

    timeline_set_mj_qpose(traj_info, traj_info->timeline.qposes + frame);
}

void timeline_overwrite_frame_using_curr_pose(traj_info_t* traj_info, int frame)
{
    int i;

    if(!traj_info->timeline.init)
        timeiline_init_from_input_file(traj_info);
    
    frame = timeline_make_frame_safe(frame);

    for (i = 0; i < CASSIE_QPOS_SIZE; i++)
        traj_info->timeline.qposes[frame].q[i] = traj_info->d->qpos[i];

}

int timeline_get_frame_from_time(traj_info_t* traj_info)
{
    return mju_round( traj_calculate_runtime_micros(traj_info) / (1000 * 10));
}

void timeline_update_mj_poses_from_realtime(traj_info_t* traj_info)
{
    int frame;

    if(!traj_info->timeline.init)
        timeiline_init_from_input_file(traj_info);

    frame = timeline_get_frame_from_time(traj_info);
    timeline_set_qposes_to_pose_frame(traj_info, frame);
}

