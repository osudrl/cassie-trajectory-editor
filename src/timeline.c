#include "timeline.h"

uint32_t timeline_fill_full_traj_state_array(traj_info_t* traj_info, uint8_t** buf)
{
    FILE* fd_in;
    uint32_t result;
    uint32_t size;
    int readsofar = 0;

    fd_in = fopen(traj_info->filename_step_data, "r");
    
    size = 131072;
    *buf = malloc(size);

    while (1)
    {
        result = fread(*buf + readsofar, 1, 131072, fd_in);
        readsofar += result;
        if(result < 131072)
        {
            break;
        }
        else
        {
            size += 131072;
            *buf = realloc(*buf , size);
        }
    }

    fclose(fd_in);
    return readsofar;
}

void timeiline_init_from_input_file(traj_info_t* traj_info)
{
    int i;
    int bytecount;
    full_traj_state_t* fulls;

    bytecount = timeline_fill_full_traj_state_array(traj_info, (uint8_t**) &fulls);
    bytecount /= sizeof(full_traj_state_t);

    printf("Read %d poses \n", bytecount);

    if(!traj_info->timeline)
        traj_info->timeline = malloc(sizeof(timeline_t));

    traj_info->timeline->qposes = malloc(sizeof(qpos_t) * bytecount);

    for(i = 0; i < bytecount; i++)
        mju_copy(traj_info->timeline->qposes[i].q, fulls[i].qpos, CASSIE_QPOS_SIZE);

    free(fulls);

    traj_info->timeline->numposes = bytecount;
    traj_info->timeline->init = 1;
    traj_info->timeline->next = NULL;
}

void timeline_set_mj_qpose(traj_info_t* traj_info, qpos_t* desired)
{
    mju_copy(traj_info->d->qpos, desired->q, CASSIE_QPOS_SIZE);
}

int timeline_make_frame_safe(int frame, int numposes)
{
    while(frame < 0)
        frame += numposes;
    frame %= numposes;
    return frame;
}

void timeline_set_qposes_to_pose_frame(traj_info_t* traj_info, int frame)
{   
    if(!traj_info->timeline || !traj_info->timeline->init)
        timeiline_init_from_input_file(traj_info);

    frame = timeline_make_frame_safe(frame, traj_info->timeline->numposes);

    timeline_set_mj_qpose(traj_info, traj_info->timeline->qposes + frame);
}

void timeline_overwrite_frame_using_curr_pose(traj_info_t* traj_info, int frame)
{
    if(!traj_info->timeline || !traj_info->timeline->init)
        timeiline_init_from_input_file(traj_info);
    
    frame = timeline_make_frame_safe(frame, traj_info->timeline->numposes);

    mju_copy(traj_info->timeline->qposes[frame].q, traj_info->d->qpos, CASSIE_QPOS_SIZE);
}

int timeline_get_frame_from_time(traj_info_t* traj_info)
{
    return mju_round( traj_calculate_runtime_micros(traj_info) / (1000 * 10));
}

void timeline_update_mj_poses_from_realtime(traj_info_t* traj_info)
{
    int frame;

    if(!traj_info->timeline || !traj_info->timeline->init)
        timeiline_init_from_input_file(traj_info);

    frame = timeline_get_frame_from_time(traj_info);
    timeline_set_qposes_to_pose_frame(traj_info, frame);
}

