
#include "node.h"

node_body_id_t node_get_body_id_from_node_index(int index)
{
    node_body_id_t id;
    id.id = index + 26;

    return id;
}

node_body_id_t node_get_body_id_from_real_body_id(int real)
{
    node_body_id_t id;
    id.id = real;
    
    return id;
}

cassie_body_id_t node_get_cassie_id_from_index(int i)
{
    cassie_body_id_t id;
    id.id = i;
    return id;
}

v3_t node_get_qpos_by_node_id(traj_info_t* traj_info, node_body_id_t id)
{
    if(id.id < 26)
        return 0;
    else
        return traj_info->d->mocap_pos + ((id.id - 26) * 3);
}

v3_t node_get_xpos_by_node_id(traj_info_t* traj_info, node_body_id_t id)
{
    return traj_info->d->mocap_pos + ((id.id-26) * 3);
}

v3_t node_get_body_xpos_curr(traj_info_t* traj_info, cassie_body_id_t id)
{
    return traj_info->d->xpos + (id.id * 3);
}

v3_t node_get_body_xpos_by_frame(traj_info_t* traj_info, timeline_t* timeline, int frame, cassie_body_id_t id)
{
    timeline_set_qposes_to_pose_frame(traj_info, timeline, frame);
    mj_forward(traj_info->m, traj_info->d);
    return node_get_body_xpos_curr(traj_info, id);
}


void node_position_initial_using_cassie_body(traj_info_t* traj_info, cassie_body_id_t body_id)
{
    int i;
    int frame;
    v3_t node_qpos;
    v3_t body_xpos;
    

    


    for (i = 0; i < NODECOUNT; i++)
    {
        frame = (traj_info->timeline->numposes / NODECOUNT) * i;
        node_qpos = node_get_qpos_by_node_id(traj_info, node_get_body_id_from_node_index(i) );
        body_xpos = node_get_body_xpos_by_frame(traj_info, traj_info->timeline, frame, body_id);
        mju_copy3(node_qpos, body_xpos);
    }



    //if issues uncomment
    // mj_forward(traj_info->m, traj_info->d);
}

double gaussian_distrobution(double r, double s)
{
    s *= 2;
    return (mju_exp(-(r*r)/s))/(mjPI * s) * 2;
}

void nodeframe_ik_transform(
    traj_info_t* traj_info, 
    timeline_t* overwrite,
    ik_solver_params_t* params,
    cassie_body_id_t body_id, 
    int frame, 
    int frameoffset, 
    v3_t target,
    double* ik_iter_total)
{
    *ik_iter_total += ik_iterative_better_body_optimizer(
        traj_info, 
        params,
        target, 
        body_id.id, 
        frameoffset, 
        1500);
    timeline_overwrite_frame_using_curr_pose(traj_info, overwrite, frame);
}

void scale_target_using_frame_offset(
    traj_info_t* traj_info,
    timeline_t* timeline,
    v3_t ik_body_target_xpos, 
    v3_t grabbed_node_transformation,
    int rootframe,
    int frame_offset,
    cassie_body_id_t body_id)
{
    double dragvect[3];
    double filter;
    v3_t body_init_xpos;

    filter = node_calculate_filter_from_frame_offset(frame_offset, traj_info->nodesigma, traj_info->nodeheight);

    body_init_xpos = node_get_body_xpos_by_frame(traj_info, timeline, rootframe + frame_offset, body_id);
    
    if (traj_info->pert_type == PERT_TARGET)
    {
        mju_sub3(dragvect, grabbed_node_transformation, body_init_xpos);
        mju_addScl3(ik_body_target_xpos, body_init_xpos, dragvect, filter);
    }
    else if (traj_info->pert_type == PERT_TRANSLATION)
    {
        mju_addScl3(ik_body_target_xpos, body_init_xpos, grabbed_node_transformation, filter);
    }
}

int get_frame_from_node_body_id(traj_info_t* traj_info, timeline_t* timeline, node_body_id_t node_id)
{
    return (traj_info->timeline->numposes / NODECOUNT) * (node_id.id - 26); // or maybe 28
}

void calculate_node_dropped_transformation_vector(
    traj_info_t* traj_info, 
    timeline_t* timeline,
    v3_t grabbed_node_transformation,
    cassie_body_id_t body_id, 
    node_body_id_t node_id)
{
    int rootframe;
    v3_t body_init_xpos;
    v3_t node_final_xpos;

    rootframe = get_frame_from_node_body_id(traj_info, timeline, node_id);
    body_init_xpos = node_get_body_xpos_by_frame(traj_info, timeline, rootframe, body_id);
    node_final_xpos = node_get_xpos_by_node_id(traj_info, node_id);

    if (traj_info->pert_type == PERT_TARGET)
    {
        mju_copy3(grabbed_node_transformation, node_final_xpos);
    }
    else if (traj_info->pert_type == PERT_TRANSLATION)
    {
        mju_sub3(grabbed_node_transformation, node_final_xpos, body_init_xpos);
    }
}

double normalCFD(double value)
{
   return 0.5 * erfc(-value * M_SQRT1_2);
}

double percent(int frame_offset, int iterations, double sigma)
{
    return 200 *((normalCFD(frame_offset/sigma) - normalCFD(0) ) / normalCFD((iterations+1) / sigma));
}

void node_refine_pert(
    traj_info_t* traj_info,
    ik_solver_params_t* params)
{
    int i;
    int frame;
    uint64_t init_time;
    uint64_t iktimedelta;
    double ik_iter_total = 0;
    int outcount = 0;
    timeline_t* timeline_old;
    timeline_t* timeline_new;
    cassie_body_id_t body_id;
    int rootframe;

    rootframe = traj_info->refine_rootframe;
    body_id = node_get_cassie_id_from_index(traj_info->refine_body);
    timeline_old = traj_info->timeline;
    timeline_new = timeline_duplicate(timeline_old);

    init_time = traj_calculate_runtime_micros(traj_info);

    for(i = 0; i < traj_info->target_list_size; i++)
    {
        frame = rootframe + traj_info->target_list[i].frame_offset;
        timeline_set_qposes_to_pose_frame(
            traj_info, timeline_old, frame );
        nodeframe_ik_transform(
            traj_info,
            timeline_new,
            params, 
            body_id, 
            frame, 
            traj_info->target_list[i].frame_offset,
            traj_info->target_list[i].target,
            &ik_iter_total);
        if(((int) (20.0*i)/traj_info->target_list_size) > outcount)
        {
            printf("Refining... %.1f%%\n", (100.0*i)/traj_info->target_list_size);
            outcount++;
        }
    }

    iktimedelta = traj_calculate_runtime_micros(traj_info) - init_time;

    // printf("Finished solving IK for %d poses in %.1f seconds\n", 
    //     1+iterations*2, 
    //     (iktimedelta/1000000.0));

    printf("Refined: accurate to %.5fmm\n", 1000.0*params->ik_accuracy_cutoff);

    timeline_new->next = timeline_old;
    timeline_old->prev = timeline_new;
    traj_info->timeline = timeline_new;
    traj_info->time_start += iktimedelta;
}

void node_perform_pert(
    traj_info_t* traj_info,
    ik_solver_params_t* params,
    v3_t grabbed_node_transformation,
    cassie_body_id_t body_id,
    int rootframe)
{
    int frame_offset;
    double ik_body_target_xpos[3];
    int iterations;
    uint64_t init_time;
    double ik_iter_total = 0;
    long iktimedelta;
    int outcount = 0;
    int target_list_index = 0;
    timeline_t* timeline_old;
    timeline_t* timeline_new;

    timeline_old = traj_info->timeline;
    timeline_new = timeline_duplicate(timeline_old);

    init_time = traj_calculate_runtime_micros(traj_info);

    scale_target_using_frame_offset(
        traj_info,
        timeline_old,
        ik_body_target_xpos, 
        grabbed_node_transformation,
        rootframe,
        0,
        body_id);

    timeline_set_qposes_to_pose_frame(traj_info, timeline_old, rootframe);

    nodeframe_ik_transform(
        traj_info,
        timeline_new,
        params, 
        body_id, 
        rootframe, 
        0, 
        ik_body_target_xpos,
        &ik_iter_total);

    //this is toomuch
    iterations = 3.491 * traj_info->nodesigma;

    if(traj_info->target_list)
    {
        free(traj_info->target_list);
        traj_info->target_list = NULL;
    }

    if(!traj_info->target_list)
    {
        traj_info->target_list_size = (iterations*2 + 1);
        traj_info->target_list = malloc(sizeof(target_t) * traj_info->target_list_size);
        traj_info->target_list[target_list_index].frame_offset = 0;
        mju_copy3(traj_info->target_list[target_list_index++].target, ik_body_target_xpos);
    }

    // printf("math= %.3f\n", 
    //     inv_norm(0.0005/mju_norm(grabbed_node_transformation, 3)) * traj_info->nodesigma);

    for(frame_offset = 1; frame_offset <= iterations; frame_offset++)
    {
        if( ((int) (.2 * percent(frame_offset, iterations, traj_info->nodesigma))) > outcount)
        {
            outcount++;
            iktimedelta = traj_calculate_runtime_micros(traj_info) - init_time;
            printf("Solving IK (%2.0f%%,%3ds) @ %4d simulation steps per pose ...\n", 
                percent(frame_offset, iterations, traj_info->nodesigma),
                (int) (iktimedelta/1000000.0),
                (int) (ik_iter_total/(1+frame_offset*2)));
        }
        scale_target_using_frame_offset(
            traj_info,
            timeline_old,
            ik_body_target_xpos, 
            grabbed_node_transformation,
            rootframe,
            frame_offset,
            body_id);

        if(traj_info->target_list)
        {
            traj_info->target_list[target_list_index].frame_offset = frame_offset;
            mju_copy3(traj_info->target_list[target_list_index++].target, ik_body_target_xpos);
        }

        nodeframe_ik_transform( 
            traj_info,
            timeline_new,
            params, 
            body_id, 
            rootframe + frame_offset,
            frame_offset,
            ik_body_target_xpos,
            &ik_iter_total);        

        scale_target_using_frame_offset(
            traj_info,
            timeline_old,
            ik_body_target_xpos, 
            grabbed_node_transformation,
            rootframe,
            -frame_offset,
            body_id);

        if(traj_info->target_list)
        {
            traj_info->target_list[target_list_index].frame_offset = -frame_offset;
            mju_copy3(traj_info->target_list[target_list_index++].target, ik_body_target_xpos);
        }

        nodeframe_ik_transform(
            traj_info,
            timeline_new, 
            params, 
            body_id, 
            rootframe - frame_offset, 
            -frame_offset,
            ik_body_target_xpos,
            &ik_iter_total);
    }

    iktimedelta = traj_calculate_runtime_micros(traj_info) - init_time;

    printf("Finished solving IK for %d poses in %.1f seconds, accurate to %.5fmm\n", 
        1+iterations*2, 
        (iktimedelta/1000000.0),
        1000.0*params->ik_accuracy_cutoff);

    timeline_new->next = timeline_old;
    timeline_old->prev = timeline_new;
    traj_info->timeline = timeline_new;

    traj_info->refine_rootframe = rootframe;
    traj_info->refine_body = body_id.id;
    traj_info->time_start += iktimedelta;
    node_position_initial_using_cassie_body(traj_info,  body_id);
}

void node_dropped(traj_info_t* traj_info, cassie_body_id_t body_id, node_body_id_t node_id)
{
    FILE* pfile;
    int rootframe;
    double grabbed_node_transformation[3];
    ik_solver_params_t params;

    ik_default_fill_solver_params(&params);

    rootframe = get_frame_from_node_body_id(traj_info, traj_info->timeline, node_id);
    calculate_node_dropped_transformation_vector(
        traj_info, 
        traj_info->timeline,
        grabbed_node_transformation, 
        body_id, 
        node_id);

    node_perform_pert(
        traj_info, 
        &params ,
         grabbed_node_transformation, 
         body_id,
         rootframe);

    pfile = fopen("last.pert", "w");
    if(pfile)
    {
        fprintf(pfile, "%d\n%d\n%.5f\n%.10f\n%.10f\n%.10f\n",
         body_id.id,
         rootframe,
         traj_info->nodesigma,
         grabbed_node_transformation[0],
         grabbed_node_transformation[1],
         grabbed_node_transformation[2]
         );
        fclose(pfile);
    }
}


void node_position_scale_visually(
    traj_info_t* traj_info,
    cassie_body_id_t body_id,
    node_body_id_t node_id)
{
    double grabbed_node_transformation[3];
    double filter;
    int rootframe;
    int frame_offset;
    int currframe;
    int i;
    v3_t node_qpos;
    v3_t body_xpos;

   

    calculate_node_dropped_transformation_vector(
        traj_info, 
        traj_info->timeline,
        grabbed_node_transformation,
        body_id,
        node_id);


    rootframe = get_frame_from_node_body_id(traj_info, traj_info->timeline, node_id);

    for (i = 0; i < NODECOUNT; i++)
    {
        //skips the node currently being dragged
        if(node_get_body_id_from_node_index(i).id == node_id.id)
            continue;

        currframe = (traj_info->timeline->numposes / NODECOUNT) * i;
        frame_offset = currframe - rootframe;

        node_qpos = node_get_qpos_by_node_id(traj_info, node_get_body_id_from_node_index(i) );
        scale_target_using_frame_offset(traj_info, 
            traj_info->timeline,
            node_qpos,
            grabbed_node_transformation,
            rootframe,
            frame_offset,
            body_id);

        // filter = node_calculate_filter_from_frame_offset(frame_offset, traj_info->nodesigma);
        // body_xpos = node_get_body_xpos_by_frame(traj_info, traj_info->timeline, currframe, body_id);
        // mju_addScl3(node_qpos, body_xpos, grabbed_node_transformation, filter);
    }

}

double node_calculate_filter_from_frame_offset(double frame_offset, double sigma, double nodeheight)
{
    return mju_min(mju_max(nodeheight,1) * gaussian_distrobution(frame_offset/sigma, 1) *(1/0.318310),1);
}

