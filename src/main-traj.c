#include "main.h"



void do_the_sphere_thing(traj_info_t* traj_info, int count, int joint_id)
{
    int i;
    int n;
    int sign;
    double diff;
    double init;

    n = count;


    if(traj_info->spherestage == 0)
    {
        init = traj_info->d->qpos[joint_id];
        for(i = 0; i < n; i++)
        {
            sign = (n-i) % 2 == 1 ? -1 : 1;
            diff = ((n-i)/2);
            diff *= sign;
            diff = diff * .1;

            traj_info->d->qpos[joint_id] = init + diff;
            mj_forward(traj_info->m, traj_info->d);
            mj_local2Global(
                traj_info->d,
                traj_info->sphere_poses + (i*3),
                NULL,
                traj_info->pert->localpos,
                traj_info->d->xquat+(4*traj_info->pert->select),
                traj_info->pert->select);
        }
    }

    traj_info->displayqspheres = 1;

}



void allow_node_transformations(traj_info_t* traj_info)
{
    if (traj_info->pert->select != traj_info->id_last_body_select &&  //made a new selection
            traj_info->pert->select > 0 && //body is on cassie not a node
            traj_info->pert->select <= 25)
    {
        node_position_initial_using_cassie_body(traj_info, 
        	node_get_cassie_id_from_index(traj_info->pert->select));
        traj_info->id_last_non_node_select = traj_info->pert->select;
        traj_info->id_last_body_select = traj_info->pert->select;    
    }

    if(!traj_info->pert->active && 
        traj_info->pert->select > 0 && 
        traj_info->pert->select <= 25)
    {
        do_the_sphere_thing(traj_info, 20, traj_info->jointnum);
    }
    else
    {
        traj_info->displayqspheres = 0;
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
            node_position_scale_visually(traj_info, 
                node_get_cassie_id_from_index(traj_info->id_last_non_node_select), 
                node_get_body_id_from_real_body_id(traj_info->pert->select)); 
            
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


void traj_foreach_frame(traj_info_t* traj_info)
{
    allow_node_transformations(traj_info);
    
    timeline_update_mj_poses_from_realtime(traj_info);


    // if(traj_info->pert->select > 0)
    // {
    //     printf("%.3f %.3f %.3f \n",
    //         traj_info->pert->localpos[0],
    //         traj_info->pert->localpos[1],
    //         traj_info->pert->localpos[2]
    //         );
    // }

    mj_forward(traj_info->m, traj_info->d);
}

