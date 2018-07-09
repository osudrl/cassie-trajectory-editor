#include "main.h"


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

void traj_position_nodes(traj_info_t* traj_info)
{
    double body_xposes[NODECOUNT * 3];
    int i;

    if(!traj_info->timeline.init)
        in_init_timeline(traj_info);

    traj_fill_body_xposes(traj_info, body_xposes, traj_info->pert->select);

    for (i = 0; i < NODECOUNT; i++)
    {
        traj_info->d->qpos[35 + (i * 3) + 0] = body_xposes[i * 3 + 0];
        traj_info->d->qpos[35 + (i * 3) + 1] = body_xposes[i * 3 + 1];
        traj_info->d->qpos[35 + (i * 3) + 2] = body_xposes[i * 3 + 2];
    }
    mj_forward(traj_info->m, traj_info->d);
}

int traj_last_select_id = 0;

int allow_pelvis_to_be_grabbed_and_moved(traj_info_t* traj_info, double* xyz_ref)
{
    if (traj_info->pert->select != traj_last_select_id &&
            traj_info->pert->select > 0)
        traj_position_nodes(traj_info);

    traj_last_select_id = traj_info->pert->select;

    if(traj_info->pert->active) 
    {
        // printf("selected: %d\n", traj_info->pert->select);
        if(traj_info->pert->select == 1)
        {
            traj_info->d->qpos[0] = traj_info->pert->refpos[0];
            traj_info->d->qpos[1] = traj_info->pert->refpos[1];
            traj_info->d->qpos[2] = traj_info->pert->refpos[2];
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



    // printf("%ld size div ints %.2f\n", sizeof(full_traj_state_t), 1318688.0/ sizeof(full_traj_state_t ));
    
    in_my_qposes(traj_info);
    mj_forward(traj_info->m, traj_info->d);
}

