#include "main.h"

int traj_last_select_id = 0;
int traj_last_non_node_select_id = 0;
int traj_last_activenum = 0;
int traj_foreach_frame_lastmod = 0;

cassie_body_id_t g(int i)
{
    cassie_body_id_t id;
    id.id = i;
    return id;
}

int allow_pelvis_to_be_grabbed_and_moved(traj_info_t* traj_info, double* xyz_ref)
{
    if (traj_info->pert->select != traj_last_select_id &&
            traj_info->pert->select > 0 &&
            traj_info->pert->select <= 25) //notanode
    {
        node_position_initial_using_cassie_body(traj_info, g(traj_info->pert->select));
        traj_last_non_node_select_id = traj_info->pert->select;        
    }

    if(traj_info->pert->active) 
    {
        traj_last_select_id = traj_info->pert->select;
        traj_last_activenum = traj_info->pert->active;

        if(traj_info->pert->select == 1 )
        {
            mju_copy3(traj_info->d->qpos,traj_info->pert->refpos);
        }
        else if( traj_info->pert->select > 25)
        {
            v3_t dqpos = node_get_qpos_by_node_id(traj_info, 
                node_get_body_id_from_real_body_id(traj_info->pert->select));
            mju_copy3(dqpos,traj_info->pert->refpos);
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

    timeline_update_mj_poses_from_realtime(traj_info);
    mj_forward(traj_info->m, traj_info->d);
}

