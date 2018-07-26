
#include "control.h"


void load_pert(traj_info_t* traj_info)
{
    int body_id;
    int rootframe;
    double grabbed_node_transform[3];
    FILE* pfile = fopen("last.pert", "r");
    char buf[2048];
    char* result;
    ik_solver_params_t params;

    ik_default_fill_solver_params(&params);

    if(pfile)
    {
        result = fgets(buf,2048,pfile);
        if(!result)
            return;
        body_id = strtol(buf, NULL, 10);
        result = fgets(buf,2048,pfile);
        if(!result)
            return;
        rootframe = strtol(buf, NULL, 10);
        result = fgets(buf,2048,pfile);
        if(!result)
            return;
        traj_info->nodesigma = strtod(buf, NULL);

        result = fgets(buf,2048,pfile);
        if(!result)
            return;
        grabbed_node_transform[0] = strtod(buf, NULL);
        result = fgets(buf,2048,pfile);
        if(!result)
            return;
        grabbed_node_transform[1] = strtod(buf, NULL);
        result = fgets(buf,2048,pfile);
        if(!result)
            return;
        grabbed_node_transform[2] = strtod(buf, NULL);

        fclose(pfile);

        node_perform_pert(
            traj_info,
            &params,
            grabbed_node_transform,
            node_get_cassie_id_from_index(body_id),
            rootframe
            );

        traj_info->id_last_non_node_select = body_id;
    }
}

ik_solver_params_t* globparams = NULL;

void refine_pert(traj_info_t* traj_info)
{
    if(!globparams)
    {
        globparams = malloc(sizeof (ik_solver_params_t));
        ik_default_fill_solver_params(globparams);
    }
    
    globparams->ik_accuracy_cutoff /= 2;
    globparams->seedoption = IK_NEVER_SEED_LASTSOLN;
    globparams->pd_k_regular = 10000;
    globparams->pd_b_regular = 10;

    node_refine_pert(
        traj_info,
        globparams);
}

void undo_pert(traj_info_t* traj_info)
{
    if(!traj_info->timeline->next)
        return;

    traj_info->timeline = traj_info->timeline->next;

    if(traj_info->id_last_non_node_select > 0 && traj_info->id_last_non_node_select <= 25)
        node_position_initial_using_cassie_body(traj_info,  node_get_cassie_id_from_index(traj_info->id_last_non_node_select));
}

void redo_pert(traj_info_t* traj_info)
{
    if(!traj_info->timeline->prev)
        return;

    traj_info->timeline = traj_info->timeline->prev;

    if(traj_info->id_last_non_node_select > 0 && traj_info->id_last_non_node_select <= 25)
        node_position_initial_using_cassie_body(traj_info,  node_get_cassie_id_from_index(traj_info->id_last_non_node_select));
}

void control_key_event(traj_info_t* traj_info, int key, int mods)
{
    if (*(traj_info->paused))
    {
        if(key == GLFW_KEY_RIGHT)
            traj_info->time_frozen += 50000;
        if(key == GLFW_KEY_LEFT)
            traj_info->time_frozen -= 50000;
        if(key == GLFW_KEY_DOWN)
            traj_info->time_frozen -= 500000;
        if (key == GLFW_KEY_UP)
            traj_info->time_frozen += 500000;
    }
     if( mods & GLFW_MOD_CONTROL )
     {
        if( key==GLFW_KEY_A )
            traj_info->nodesigma *= .95;
        else if( key==GLFW_KEY_D)
            traj_info->nodesigma *= 1.05;
        else if( key==GLFW_KEY_P)
            load_pert(traj_info);
        else if( key==GLFW_KEY_R)
            refine_pert(traj_info);
        else if( key==GLFW_KEY_Z &&  !(mods & GLFW_MOD_SHIFT) )
            undo_pert(traj_info);
        else if( key==GLFW_KEY_Y || (
            key==GLFW_KEY_Z &&  (mods & GLFW_MOD_SHIFT)  ))
            redo_pert(traj_info);
    }
}



