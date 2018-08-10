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

void f_copy(float* to, float* from, int num)
{
    int i;
    for (i = 0; i < num; i++)
    {
        to[i] = from[i];
    }
}

void nodes_recolor(traj_info_t* traj_info)
{
    int i;

    for(i = 35; i < traj_info->m->ngeom && traj_info->selection.node_type == NODE_POSITIONAL; i++)
    {
        f_copy(traj_info->m->geom_rgba + i*4,
            traj_info->decor.rgba_default_positional,
            4);
        mju_copy(traj_info->m->geom_size + i*3,
            traj_info->decor.size_default_positional,
            3);
    }

    for(i = 35; i < traj_info->m->ngeom && traj_info->selection.node_type != NODE_POSITIONAL; i++)
    {
        f_copy(traj_info->m->geom_rgba + i*4,
            traj_info->decor.rgba_default_joint,
            4);
        mju_copy(traj_info->m->geom_size + i*3,
            traj_info->decor.size_default_joint,
            3);
    }
}

void traj_foreach_frame(traj_info_t* traj_info)
{
    allow_node_transformations(traj_info);
    
    timeline_update_mj_poses_from_realtime(traj_info);

    mj_forward(traj_info->m, traj_info->d);
}

