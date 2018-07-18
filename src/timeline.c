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

void timeiline_init_from_input_file(traj_info_t* traj_info)
{
    int i;
    full_traj_state_t buf[TIMELINE_SIZE];

    timeline_fill_full_traj_state_array(traj_info, buf, TIMELINE_SIZE);

    for(i = 0; i < TIMELINE_SIZE; i++)
        mju_copy(traj_info->timeline.qposes[i].q, buf[i].qpos, CASSIE_QPOS_SIZE);

    traj_info->timeline.init = 1;
}

void timeline_set_mj_qpose(traj_info_t* traj_info, qpos_t* desired)
{
    mju_copy(traj_info->d->qpos, desired->q, CASSIE_QPOS_SIZE);
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
    if(!traj_info->timeline.init)
        timeiline_init_from_input_file(traj_info);
    
    frame = timeline_make_frame_safe(frame);

    mju_copy(traj_info->timeline.qposes[frame].q, traj_info->d->qpos, CASSIE_QPOS_SIZE);
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

