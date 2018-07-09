#include "in.h"

uint32_t in_fill_full_traj_state_array(full_traj_state_t* buf, int bufcount)
{
    FILE* fd_in;
    uint32_t result;

    fd_in = fopen("stepdata.bin", "r");
    result = fread(buf, sizeof(full_traj_state_t), bufcount, fd_in);

    fclose(fd_in);
    return result;
}

void in_copy_single_full_traj_state_to_qpos(qpos_t* qpos, full_traj_state_t* state)
{
    int i;
    // qpos->q[0] = 0;
    // qpos->q[1] = 0;
    for (i = 0; i < CASSIE_QPOS_SIZE; i++)
        qpos->q[i] = state->qpos[i];
}

void in_copy_all_full_traj_states_to_qposes(qpos_t* qpos, full_traj_state_t* state, int bufcount)
{
    int i;

    for(i = 0; i<bufcount; i++)
        in_copy_single_full_traj_state_to_qpos(qpos+i, state+i);
}

void in_init_timeline(traj_info_t* traj_info)
{
    full_traj_state_t buf[TIMELINE_SIZE];
    in_fill_full_traj_state_array(buf, TIMELINE_SIZE);
    in_copy_all_full_traj_states_to_qposes(traj_info->timeline.qposes, buf, TIMELINE_SIZE);
    traj_info->timeline.init = 1;
}

void in_set_mj_qpose(traj_info_t* traj_info, qpos_t* desired)
{
    int i;

    for(i = 0; i < CASSIE_QPOS_SIZE; i++)
    {
        traj_info->d->qpos[i] = desired->q[i];
    }
}

void timeline_set_qposes_to_pose_frame(traj_info_t* traj_info, int frame)
{
    if(!traj_info->timeline.init)
        in_init_timeline(traj_info);

    in_set_mj_qpose(traj_info, traj_info->timeline.qposes + frame);
}

int timeline_get_frame_from_time(traj_info_t* traj_info)
{
    int frame = mju_round( traj_calculate_runtime_micros(traj_info) / (1000 * 5));
    frame = (frame % TIMELINE_SIZE);
    return frame;
}

void in_my_qposes(traj_info_t* traj_info)
{
    int frame;

    if(!traj_info->timeline.init)
        in_init_timeline(traj_info);

    frame = timeline_get_frame_from_time(traj_info);
    timeline_set_qposes_to_pose_frame(traj_info, frame);
}

