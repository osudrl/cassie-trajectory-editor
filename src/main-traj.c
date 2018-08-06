#include "main.h"

void allow_node_transformations(traj_info_t* traj_info)
{
    if (traj_info->pert->select > 0 && 
        traj_info->pert->select <= 25 &&
        (
            traj_info->pert->select != traj_info->selection.id_last_body_select ||
            traj_info->selection.node_type != NODE_POSITIONAL
        ))
    {
        node_position_initial_using_cassie_body(traj_info, 
        	node_get_cassie_id_from_index(traj_info->pert->select));
        traj_info->selection.id_last_non_node_select = traj_info->pert->select;
        traj_info->selection.id_last_body_select = traj_info->pert->select;    
    }

    if( traj_info->pert->active &&
        traj_info->selection.node_type == NODE_POSITIONAL) 
    {
        traj_info->selection.id_last_body_select = traj_info->pert->select;
        traj_info->selection.id_last_pert_activenum = traj_info->pert->active;

        if( traj_info->pert->select > 25)
        {
            v3_t dqpos = node_get_qpos_by_node_id(traj_info, 
                node_get_body_id_from_real_body_id(traj_info->pert->select));
            mju_copy3(dqpos,traj_info->pert->refpos);
            node_position_scale_visually(traj_info, 
                node_get_cassie_id_from_index(traj_info->selection.id_last_non_node_select), 
                node_get_body_id_from_real_body_id(traj_info->pert->select)); 
            
        }
    }
    else if (traj_info->selection.id_last_pert_activenum == 1 && 
        traj_info->selection.id_last_body_select > 25 &&
        traj_info->selection.node_type == NODE_POSITIONAL)
    {
        node_dropped(traj_info, 
            node_get_cassie_id_from_index(traj_info->selection.id_last_non_node_select), 
            node_get_body_id_from_real_body_id(traj_info->selection.id_last_body_select));

        traj_info->selection.id_last_body_select = traj_info->pert->select;
        traj_info->selection.id_last_pert_activenum = traj_info->pert->active;
    }
}

uint64_t traj_time_in_micros()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return 1000000 * tv.tv_sec + tv.tv_usec;
}

int64_t traj_calculate_runtime_micros(traj_info_t* traj_info)
{
    int64_t val;
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

void nodes_recolor(traj_info_t* traj_info)
{
    int i;

    for(i = 35; i < traj_info->m->ngeom && traj_info->selection.node_type == NODE_POSITIONAL; i++)
    {
        traj_info->m->geom_rgba[i*4 + 0] = .2;
        traj_info->m->geom_rgba[i*4 + 1] = .6;
        traj_info->m->geom_rgba[i*4 + 2] = .2;
        traj_info->m->geom_rgba[i*4 + 3] = 0;
        traj_info->m->geom_size[i*3 + 0] = .015;
        traj_info->m->geom_size[i*3 + 1] = .015;
        traj_info->m->geom_size[i*3 + 2] = .015;
    }

    for(i = 35; i < traj_info->m->ngeom && traj_info->selection.node_type != NODE_POSITIONAL; i++)
    {
        traj_info->m->geom_rgba[i*4 + 0] = .1;
        traj_info->m->geom_rgba[i*4 + 1] = .1;
        traj_info->m->geom_rgba[i*4 + 2] = .8;
        traj_info->m->geom_size[i*3 + 0] = .010;
        traj_info->m->geom_size[i*3 + 1] = .010;
        traj_info->m->geom_size[i*3 + 2] = .010;
    }
}

void traj_foreach_frame(traj_info_t* traj_info)
{
    nodes_recolor(traj_info);

    allow_node_transformations(traj_info);
    
    timeline_update_mj_poses_from_realtime(traj_info);

    mj_forward(traj_info->m, traj_info->d);
}

