#include "main.h"


#define TP traj_info->pert
#define SEL traj_info->selection

void allow_node_transformations(traj_info_t* traj_info)
{
    if (TP->select > 0 && 
        TP->select <= 25 &&
        TP->select != SEL.id_last_body_select )
    {
        node_position_initial_using_cassie_body(traj_info, 
        	node_get_cassie_id_from_index(TP->select));
        SEL.id_last_non_node_select = TP->select;
        SEL.id_last_body_select = TP->select; 

        mju_copy3(SEL.localpos, TP->localpos);  
    }

    if( TP->active &&
        TP->select > 25 ) 
    {
        SEL.id_last_body_select = TP->select;
        SEL.id_last_pert_activenum = TP->active;
    
        if(SEL.node_type == NODE_POSITIONAL)
        {
           node_scale_visually_positional(traj_info, 
               node_get_cassie_id_from_index(SEL.id_last_non_node_select), 
               node_get_body_id_from_real_body_id(TP->select));  
        }
        else if (SEL.node_type == NODE_JOINTMOVE)
        {
            node_scale_visually_jointmove(traj_info, 
                node_get_cassie_id_from_index(SEL.id_last_non_node_select), 
                node_get_body_id_from_real_body_id(TP->select));  
        }           
    }
    else if (
        !TP->active &&
        SEL.id_last_pert_activenum == 1 && 
        SEL.id_last_body_select > 25)
    {
        if (SEL.node_type == NODE_POSITIONAL)
        {
            node_dropped_positional(traj_info, 
                node_get_cassie_id_from_index(SEL.id_last_non_node_select), 
                node_get_body_id_from_real_body_id(SEL.id_last_body_select));
        }
        else if (SEL.node_type == NODE_JOINTMOVE)
        {
            node_dropped_jointmove(traj_info, 
                node_get_cassie_id_from_index(SEL.id_last_non_node_select), 
                node_get_body_id_from_real_body_id(SEL.id_last_body_select));
        }


        SEL.id_last_body_select = TP->select;
        SEL.id_last_pert_activenum = TP->active;
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
    traj_info->decor.pos[0] = 0;
    traj_info->decor.pos[1] = 0;
    traj_info->decor.pos[2] = 1;

    traj_info->decor.size[0] = 0.02;
    traj_info->decor.size[1] = 0.02;
    traj_info->decor.size[2] = 0.02;

    traj_info->decor.color[0] = .5;
    traj_info->decor.color[1] = .5;
    traj_info->decor.color[2] = .5;
    traj_info->decor.color[3] = 1;

    traj_info->decor.count = 1;

    allow_node_transformations(traj_info);
    
    timeline_update_mj_poses_from_realtime(traj_info);

    mj_forward(traj_info->m, traj_info->d);
}

