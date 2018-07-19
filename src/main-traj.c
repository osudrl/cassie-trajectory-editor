#include "main.h"


void allow_pelvis_to_be_grabbed_and_moved(traj_info_t* traj_info, double* xyz_ref)
{
    if (traj_info->pert->select != traj_info->id_last_body_select &&  //made a new selection
            traj_info->pert->select > 0 && //body is on cassie not a node
            traj_info->pert->select <= 25)
    {
        node_position_initial_using_cassie_body(traj_info, 
        	node_get_cassie_id_from_index(traj_info->pert->select));
        traj_info->id_last_non_node_select = traj_info->pert->select;        
    }

    if(traj_info->pert->active) 
    {
        traj_info->id_last_body_select = traj_info->pert->select;
        traj_info->id_last_pert_activenum = traj_info->pert->active;

        if( traj_info->pert->select > 25)
        {
            v3_t dqpos = node_get_qpos_by_node_id(traj_info, 
                node_get_body_id_from_real_body_id(traj_info->pert->select));
            mju_copy3(dqpos,traj_info->pert->refpos);
            
        }
    }
    else if (traj_info->id_last_pert_activenum == 1 && traj_info->id_last_body_select > 25)
    {
        node_dropped(traj_info, 
            node_get_cassie_id_from_index(traj_info->id_last_non_node_select), 
            node_get_body_id_from_real_body_id(traj_info->id_last_body_select));

        traj_info->id_last_body_select = traj_info->pert->select;
        traj_info->id_last_pert_activenum = traj_info->pert->active;
    }
    if(traj_info->pert->select > 25)
    {
    	node_position_scale_visually(traj_info, 
                node_get_cassie_id_from_index(traj_info->id_last_non_node_select), 
                node_get_body_id_from_real_body_id(traj_info->pert->select)); 
    }
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

    allow_pelvis_to_be_grabbed_and_moved(traj_info,xyz_xpos_target);
    
    timeline_update_mj_poses_from_realtime(traj_info);
    mj_forward(traj_info->m, traj_info->d);
}

