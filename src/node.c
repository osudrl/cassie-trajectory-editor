
#include "node.h"

#define SEL traj_info->selection
#define GETJOINTNUM traj_info->selection.joint_cycle_list[traj_info->selection.joint_cycle_list_index]


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

v3_t node_get_body_xpos_curr(traj_info_t* traj_info, cassie_body_id_t id)
{
    return traj_info->d->xpos + (id.id * 3);
}

v3_t node_get_body_xpos_by_frame(traj_info_t* traj_info,
    timeline_t* timeline,
    int frame, 
    cassie_body_id_t id)
{
    timeline_set_qposes_to_pose_frame(traj_info, timeline, frame);
    mj_forward(traj_info->m, traj_info->d);
    return node_get_body_xpos_curr(traj_info, id);
}


double gaussian_distrobution(double r)
{
    return (mju_exp(-(r*r)));
}

void node_perform_ik_on_xpos_transformation(
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
        IK_STEP_CUTOFF);
    timeline_overwrite_frame_using_curr_pose(traj_info, overwrite, frame);
}

void node_calculate_arbitrary_target_using_scale_type(
    traj_info_t* traj_info,
    double* final_curr,
    double* root_transformation,
    double* init_curr,
    double* init_root,
    int vector_size,
    double scalefactor)
{
    int stack_mark;
    double* final_root;
    double* init_curr_to_final_root;

    stack_mark = traj_info->d->pstack;

    final_root = mj_stackAlloc(traj_info->d, vector_size);
    init_curr_to_final_root = mj_stackAlloc(traj_info->d, vector_size);

    if (SEL.scale_type == SCALING_B)
    {
        mju_add(
            final_root, 
            root_transformation, 
            init_root,
            vector_size);
        mju_sub(
            init_curr_to_final_root,
            final_root,
            init_curr,
            vector_size);
        mju_addScl(
            final_curr,
            init_curr,
            init_curr_to_final_root,
            scalefactor,
            vector_size);
    }
    else if (SEL.scale_type == SCALING_A)
    {
        mju_addScl(
            final_curr,
            init_curr,
            root_transformation,
            scalefactor,
            vector_size);
    }

    traj_info->d->pstack = stack_mark;
}

void node_gimme_target_friendly_rf_init_body_xpos(
    traj_info_t* traj_info,
    timeline_t* timeline,
    v3_t fixed_body_init_xpos_at_rootframe, 
    v3_t global_body_init_xpos_at_rootframe,
    int rootframe,
    int frame_offset,
    cassie_body_id_t body_id)
{
    double pelvis_start[3];
    double pelvis_end[3];
    double pelvis_delta[3];
    int numdeltas;

    numdeltas = mju_round((frame_offset + 0.0) / (timeline->numnoloopframes + 0.0));
    mju_copy3(pelvis_start, node_get_body_xpos_by_frame(
        traj_info,
        timeline,
        0,
        node_get_cassie_id_from_index(1)));
    mju_copy3(pelvis_end, node_get_body_xpos_by_frame(
        traj_info,
        timeline,
        timeline->numnoloopframes-1,
        node_get_cassie_id_from_index(1)));
    mju_sub3(pelvis_delta, pelvis_end, pelvis_start);
    mju_addScl3(fixed_body_init_xpos_at_rootframe,
        global_body_init_xpos_at_rootframe,
        pelvis_delta, 
        numdeltas);
}

void node_calclate_global_target_using_transformation_type(
    traj_info_t* traj_info,
    timeline_t* timeline,
    v3_t global_body_init_xpos_at_rootframe,
    v3_t global_body_target_xpos, 
    v3_t rootframe_transform_vector,
    int rootframe,
    int frame_offset,
    cassie_body_id_t body_id)
{
    double filter;
    v3_t body_init_xpos;
    int i;
    double temp;
    double fixed_body_init_xpos_at_rootframe[3];

    filter = node_calculate_filter_from_frame_offset(
        frame_offset, 
        SEL.nodesigma, 
        SEL.nodeheight);

    for(i = 0; i * traj_info->timeline->numnoloopframes < traj_info->timeline->numframes; i++)
    {
        temp = node_calculate_filter_from_frame_offset(
            frame_offset + i * traj_info->timeline->numnoloopframes,
            SEL.nodesigma, 
            SEL.nodeheight);
        filter = mju_max(temp,filter);
        temp = node_calculate_filter_from_frame_offset(
            frame_offset - i * traj_info->timeline->numnoloopframes,
            SEL.nodesigma, 
            SEL.nodeheight);
        filter = mju_max(temp,filter);        
    }

    node_gimme_target_friendly_rf_init_body_xpos(
        traj_info,
        timeline,
        fixed_body_init_xpos_at_rootframe,
        global_body_init_xpos_at_rootframe,
        rootframe,
        frame_offset,
        body_id);

    body_init_xpos = node_get_body_xpos_by_frame(
        traj_info, 
        timeline, 
        rootframe + frame_offset, 
        body_id);    
    
    node_calculate_arbitrary_target_using_scale_type(
        traj_info,
        global_body_target_xpos,
        rootframe_transform_vector,
        body_init_xpos,
        fixed_body_init_xpos_at_rootframe,
        3,
        filter);
}

int get_frame_from_node_body_id(traj_info_t* traj_info, 
    timeline_t* timeline, 
    node_body_id_t node_id)
{
    int offset;
    int frame;
    offset = SEL.frame_offset;
    while(offset < 0)
        offset += timeline->numframes;
    offset = SEL.frame_offset % timeline->numframes;
    frame = (timeline->numframes / NODECOUNT) * (node_id.id - 26); // or maybe 28
    frame += offset;
    frame %= timeline->numframes;
    return frame;
}

void node_calculate_rootframe_transformation_vector(
    traj_info_t* traj_info, 
    timeline_t* timeline,
    v3_t rootframe_transform_vector,
    cassie_body_id_t body_id, 
    node_body_id_t node_id)
{
    int rootframe;
    v3_t body_init_xpos;
    v3_t node_final_xpos;

    rootframe = get_frame_from_node_body_id(traj_info, timeline, node_id);
    body_init_xpos = node_get_body_xpos_by_frame(traj_info, timeline, rootframe, body_id);
    node_final_xpos = node_get_qpos_by_node_id(traj_info, node_id);

    mju_sub3(rootframe_transform_vector, node_final_xpos, body_init_xpos);
}

double normalCFD(double value)
{
   return 0.5 * erfc(-value * M_SQRT1_2);
}

double percent(int frame_offset, int iterations, double sigma)
{
    return 200 *(
        (normalCFD(frame_offset/sigma) - normalCFD(0) )
            / normalCFD((iterations+1) / sigma)
    );
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
    rootframe = timeline_make_frame_safe(rootframe, timeline_old->numframes);
    for(i = 0; i < traj_info->target_list_size; i++)
    {
        frame = rootframe + traj_info->target_list[i].frame_offset;
        timeline_set_qposes_to_pose_frame(
            traj_info, timeline_old, frame );
        node_perform_ik_on_xpos_transformation(
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

    timeline_safe_link(timeline_new, timeline_old);
    timeline_new->node_type = NODE_POSITIONAL;
    traj_info->timeline = timeline_new;
    traj_info->time_start += iktimedelta;
}

void node_calc_frame_lowhigh(
    int* low_frame,
    int* high_frame,
    int rootframe,
    int numframes,
    traj_info_t* traj_info)
{
    int i;
    double filter;

    for(i = 0; 1; i++)
    {
        filter = node_calculate_filter_from_frame_offset(
            i,
            SEL.nodesigma,
            SEL.nodeheight);
        if(filter < 0.001)
            break;
    }
    
    if(!SEL.loop_enabled)
    {
        *low_frame = mju_max(0, rootframe - i);
        *high_frame = mju_min(numframes-1, rootframe + i);
    }
    else
    {
       *low_frame = rootframe - i;
       *high_frame = rootframe + i; 
   }    
}

void node_perform_pert(
    traj_info_t* traj_info,
    ik_solver_params_t* params,
    v3_t rootframe_transform_vector,
    cassie_body_id_t body_id,
    int rootframe)
{
    int frame_offset;
    double global_body_init_xpos_at_rootframe[3];
    double global_body_target_xpos[3];
    int iterations;
    uint64_t init_time;
    double ik_iter_total = 0;
    long iktimedelta;
    int outcount = 0;
    int target_list_index = 0;
    int low_frame;
    int high_frame;
    int loopcount;
    timeline_t* timeline_old;
    timeline_t* timeline_new;
    timeline_t* timeline_final;
    bool failed;

    failed = 0;
    timeline_old = traj_info->timeline;
    timeline_new = timeline_noloop(timeline_old);
    rootframe = timeline_make_frame_safe(rootframe, timeline_new->numframes);
    node_calc_frame_lowhigh(
        &low_frame,
        &high_frame,
        rootframe,
        timeline_new->numframes,
        traj_info);

    init_time = traj_calculate_runtime_micros(traj_info);
    mju_copy3(
        global_body_init_xpos_at_rootframe,
        node_get_body_xpos_by_frame(traj_info, timeline_new, rootframe, body_id));

    node_calclate_global_target_using_transformation_type(
        traj_info,
        timeline_new,
        global_body_init_xpos_at_rootframe,
        global_body_target_xpos, 
        rootframe_transform_vector,
        rootframe,
        0,
        body_id);

    node_perform_ik_on_xpos_transformation(
        traj_info,
        timeline_new,
        params, 
        body_id, 
        rootframe, 
        0, 
        global_body_target_xpos,
        &ik_iter_total);

    iterations = high_frame-low_frame+1;

    if(traj_info->target_list)
    {
        free(traj_info->target_list);
        traj_info->target_list = NULL;
    }
    
    traj_info->target_list_size = (iterations);
    traj_info->target_list = malloc(sizeof(target_t) * traj_info->target_list_size);
    traj_info->target_list[target_list_index].frame_offset = 0;
    mju_copy3(traj_info->target_list[target_list_index++].target, global_body_target_xpos);
    
    loopcount = mju_max(high_frame-rootframe, rootframe-low_frame); 

    for(frame_offset = 1; frame_offset <= loopcount; frame_offset++)
    {
        if(((int)(.2 * percent(frame_offset, iterations, SEL.nodesigma))) > outcount)
        {
            outcount++;
            iktimedelta = traj_calculate_runtime_micros(traj_info) - init_time;
            printf("Solving IK (%2.0f%%,%3ds) @ %4d simulation steps per pose ...\n", 
                percent(frame_offset*2, iterations, SEL.nodesigma),
                (int) (iktimedelta/1000000.0),
                (int) (ik_iter_total/(1+frame_offset*2)));
        }

        if(rootframe + frame_offset <= high_frame)
        {
        node_calclate_global_target_using_transformation_type(
            traj_info,
            timeline_new,
            global_body_init_xpos_at_rootframe,
            global_body_target_xpos, 
            rootframe_transform_vector,
            rootframe,
            frame_offset,
            body_id);

        if(traj_info->target_list)
        {
            traj_info->target_list[target_list_index].frame_offset = frame_offset;
            mju_copy3(traj_info->target_list[target_list_index++].target, global_body_target_xpos);
        }

        node_perform_ik_on_xpos_transformation( 
            traj_info,
            timeline_new,
            params, 
            body_id, 
            rootframe + frame_offset,
            frame_offset,
            global_body_target_xpos,
            &ik_iter_total);        
        }
        if(rootframe - frame_offset >= low_frame)
        {

        node_calclate_global_target_using_transformation_type(
            traj_info,
            timeline_new,
            global_body_init_xpos_at_rootframe,
            global_body_target_xpos, 
            rootframe_transform_vector,
            rootframe,
            -frame_offset,
            body_id);

        if(traj_info->target_list)
        {
            traj_info->target_list[target_list_index].frame_offset = -frame_offset;
            mju_copy3(
                traj_info->target_list[target_list_index++].target,
                global_body_target_xpos);
        }

        node_perform_ik_on_xpos_transformation(
            traj_info,
            timeline_new, 
            params, 
            body_id, 
            rootframe - frame_offset, 
            -frame_offset,
            global_body_target_xpos,
            &ik_iter_total);
        }

        if(frame_offset > 1 && ik_iter_total > (.95 * IK_STEP_CUTOFF * (2*frame_offset + 1)))
        {
            printf("TOO HARD. ABORTING. \n");
            failed = 1;
            break;
        }
    }

    iktimedelta = traj_calculate_runtime_micros(traj_info) - init_time;

    if(!failed)
    {
        printf("Finished solving IK for %d poses in %.1f seconds, accurate to %.5fmm\n", 
            1+iterations*2, 
            (iktimedelta/1000000.0),
            1000.0*params->ik_accuracy_cutoff);

        timeline_final = timeline_loop(
            timeline_new,
            mju_round(timeline_old->numframes/timeline_old->numnoloopframes));
        timeline_free(timeline_new);
        timeline_new = timeline_final;
        timeline_safe_link(timeline_new, timeline_old);
        traj_info->timeline = timeline_new;
        timeline_new->node_type = NODE_POSITIONAL;
    }
    else
    {
        timeline_free(timeline_new);
    }

    traj_info->refine_rootframe = rootframe;
    traj_info->refine_body = body_id.id;
    traj_info->time_start += iktimedelta;
    node_position_initial_using_cassie_body(traj_info,  body_id);
}


void node_dropped_jointmove(
    traj_info_t* traj_info,
    cassie_body_id_t body_id,
    node_body_id_t node_id)
{
    int frame;
    int rootframe;
    double rootframe_init;
    double filter;
    double jointdiff;
    timeline_t* timeline_old;
    timeline_t* timeline_new;

    timeline_old = traj_info->timeline;
    timeline_new = timeline_duplicate(timeline_old);

    rootframe = get_frame_from_node_body_id(traj_info,
        timeline_new,
        node_id);

    timeline_set_qposes_to_pose_frame(
        traj_info,
        timeline_old,
        rootframe);

    rootframe_init = traj_info->d->qpos[GETJOINTNUM];

    mj_forward(traj_info->m, traj_info->d);
    jointdiff = node_caluclate_jointdiff(traj_info,
        node_get_body_xpos_curr(traj_info, body_id));

    for (frame = 0; frame < timeline_new->numframes; frame++)
    {
        filter = node_calculate_filter_from_frame_offset(
            frame - rootframe, 
            SEL.nodesigma, 
            SEL.nodeheight);

        node_calculate_arbitrary_target_using_scale_type(
            traj_info,
            timeline_new->qposes[frame].q + GETJOINTNUM,
            &jointdiff,
            timeline_old->qposes[frame].q + GETJOINTNUM,
            &rootframe_init,
            1,
            filter);
    }

    timeline_safe_link(timeline_new, timeline_old);
    traj_info->timeline = timeline_new;

    node_position_initial_using_cassie_body(traj_info,  body_id);
}

void node_dropped_positional(
    traj_info_t* traj_info,
    cassie_body_id_t body_id,
    node_body_id_t node_id)
{
    FILE* pfile;
    int rootframe;
    double rootframe_transform_vector[3];
    ik_solver_params_t params;

    ik_default_fill_solver_params(&params);

    rootframe = get_frame_from_node_body_id(traj_info, traj_info->timeline, node_id);
    node_calculate_rootframe_transformation_vector(
        traj_info, 
        traj_info->timeline,
        rootframe_transform_vector, 
        body_id, 
        node_id);

    node_perform_pert(
        traj_info, 
        &params ,
         rootframe_transform_vector, 
         body_id,
         rootframe);

    pfile = fopen("last.pert", "w");
    if(pfile)
    {
        fprintf(pfile, "%d\n%d\n%.5f\n%.10f\n%.10f\n%.10f\n",
         body_id.id,
         rootframe,
         SEL.nodesigma,
         rootframe_transform_vector[0],
         rootframe_transform_vector[1],
         rootframe_transform_vector[2]
         );
        fclose(pfile);
    }
}

void node_position_jointmove(
    traj_info_t* traj_info, 
    cassie_body_id_t body_id,
    int rootframe,
    double jointdiff)
{
    int i;
    int currframe;
    int frame_offset;
    v3_t node_qpos;
    double rootframe_init;
    double filter;
    double temp_new_qpos_val;
    bool update_decor;

    timeline_set_qposes_to_pose_frame(
        traj_info,
        traj_info->timeline,
        rootframe);
    rootframe_init = traj_info->d->qpos[GETJOINTNUM];

    update_decor = !decor_has_init(traj_info);

    for (i = 0; i < NODECOUNT; i++)
    {
        currframe = get_frame_from_node_body_id(traj_info, 
            traj_info->timeline,
            node_get_body_id_from_node_index(i));
        node_qpos = node_get_qpos_by_node_id(traj_info, 
            node_get_body_id_from_node_index(i));
        
        timeline_set_qposes_to_pose_frame(traj_info, traj_info->timeline, currframe);

        node_compare_looped_filters(traj_info,
            rootframe,
            &currframe,
            &frame_offset);

        filter = node_calculate_filter_from_frame_offset(
            frame_offset, 
            SEL.nodesigma, 
            SEL.nodeheight);

        node_calculate_arbitrary_target_using_scale_type(
            traj_info,
            &temp_new_qpos_val,
            &jointdiff,
            traj_info->d->qpos + GETJOINTNUM,
            &rootframe_init,
            1,
            filter);

        traj_info->d->qpos[GETJOINTNUM] = temp_new_qpos_val;

        mj_forward(traj_info->m, traj_info->d);

        mj_local2Global(
            traj_info->d,
            node_qpos,
            NULL,
            SEL.localpos,
            traj_info->d->xquat + (4*body_id.id),
            body_id.id
            );
        if(update_decor && i == 0)
            decor_joint_init(traj_info, node_qpos);
        else if (update_decor)
            decor_joint_addto(traj_info, node_qpos);

    }
}

double node_caluclate_jointdiff(
    traj_info_t* traj_info,
    v3_t body_init_xpos)
{
    double rootframe_transform_vector[3];

    mju_sub3(rootframe_transform_vector,
        SEL.joint_move_ref,
        body_init_xpos);

    return rootframe_transform_vector[2];
}

void node_scale_visually_jointmove(
    traj_info_t* traj_info,
    cassie_body_id_t body_id,
    node_body_id_t node_id)
{
    double qpos_cache[CASSIE_QPOS_SIZE];
    double jointdiff;
    int rootframe;
    v3_t body_init_xpos;

    mju_copy(qpos_cache, traj_info->d->qpos, CASSIE_QPOS_SIZE);   
    rootframe = get_frame_from_node_body_id(traj_info,
        traj_info->timeline,
        node_id);

    body_init_xpos = node_get_body_xpos_by_frame(
        traj_info,
        traj_info->timeline,
        rootframe,
        body_id);

    mju_copy3(SEL.joint_move_ref,
        traj_info->pert->refpos);

    jointdiff = node_caluclate_jointdiff(traj_info, body_init_xpos);
 
    node_position_jointmove(traj_info,
        body_id,
        rootframe,
        jointdiff); 

    mju_copy(traj_info->d->qpos, qpos_cache, CASSIE_QPOS_SIZE);
    mj_forward(traj_info->m, traj_info->d);
}

void node_compare_looped_filters(
    traj_info_t* traj_info,
    int rootframe,
    int* currframe, 
    int* frame_offset)
{
    int oldcurrframe = *currframe;
    double filter;

    filter = node_calculate_filter_from_frame_offset(
        oldcurrframe - rootframe,
        SEL.nodesigma,
        SEL.nodeheight);

    if(SEL.loop_enabled && 
        filter < node_calculate_filter_from_frame_offset(
            oldcurrframe - rootframe - traj_info->timeline->numframes,
            SEL.nodesigma,
            SEL.nodeheight))
    {
        *currframe = oldcurrframe - traj_info->timeline->numframes;
        filter = node_calculate_filter_from_frame_offset(
            oldcurrframe - rootframe - traj_info->timeline->numframes,
            SEL.nodesigma,
            SEL.nodeheight);
    }
    if(SEL.loop_enabled &&
        filter < node_calculate_filter_from_frame_offset(
            oldcurrframe - rootframe + traj_info->timeline->numframes,
            SEL.nodesigma,
            SEL.nodeheight))
    {
        *currframe = oldcurrframe + traj_info->timeline->numframes;
    }

    *frame_offset = *currframe - rootframe;
}


void node_scale_visually_positional(
    traj_info_t* traj_info,
    cassie_body_id_t body_id,
    node_body_id_t node_id)
{
    double global_body_init_xpos_at_rootframe[3];
    double rootframe_transform_vector[3];
    int rootframe;
    int frame_offset;
    int currframe;
    int i;
    v3_t node_qpos;
    bool decor_update = 1;


    node_qpos = node_get_qpos_by_node_id(traj_info, node_id);
    mju_copy3(node_qpos, traj_info->pert->refpos);

    node_calculate_rootframe_transformation_vector(
        traj_info, 
        traj_info->timeline,
        rootframe_transform_vector,
        body_id,
        node_id);


    rootframe = get_frame_from_node_body_id(traj_info,
        traj_info->timeline,
        node_id);
    mju_copy3(global_body_init_xpos_at_rootframe,
        node_get_body_xpos_curr(traj_info, body_id));

    if(decor_has_init(traj_info))
        decor_update = 0;

    if(decor_update)
        decor_positional_init(traj_info, global_body_init_xpos_at_rootframe);

    for (i = 0; i < NODECOUNT; i++)
    {
        //skips the node currently being dragged
        if(node_get_body_id_from_node_index(i).id == node_id.id)
            continue;

        currframe = get_frame_from_node_body_id(traj_info, 
            traj_info->timeline,
            node_get_body_id_from_node_index(i));
        
        node_compare_looped_filters(
            traj_info,
            rootframe,
            &currframe,
            &frame_offset);

        node_qpos = node_get_qpos_by_node_id(traj_info,
            node_get_body_id_from_node_index(i));
        node_calclate_global_target_using_transformation_type(
            traj_info, 
            traj_info->timeline,
            global_body_init_xpos_at_rootframe,
            node_qpos,
            rootframe_transform_vector,
            rootframe,
            frame_offset,
            body_id);

        if(decor_update)
            decor_positional_addto(traj_info, node_get_body_xpos_curr(traj_info, body_id));
    }

}

void node_position_initial_positional(
    traj_info_t* traj_info,
    cassie_body_id_t body_id)
{
    int i;
    int frame;
    v3_t node_qpos;
    v3_t body_xpos;
   
    for (i = 0; i < NODECOUNT; i++)
    {
        frame = get_frame_from_node_body_id(traj_info, 
            traj_info->timeline,
            node_get_body_id_from_node_index(i));
        node_qpos = node_get_qpos_by_node_id(traj_info, node_get_body_id_from_node_index(i) );
        body_xpos = node_get_body_xpos_by_frame(traj_info, traj_info->timeline, frame, body_id);
        mju_copy3(node_qpos, body_xpos);
    }    
}

void node_position_jointid(
    traj_info_t* traj_info,
    cassie_body_id_t body_id)
{
    int i;
    double init;
    double diff;
    v3_t node_qpos;

    init = traj_info->d->qpos[GETJOINTNUM];
    for (i = 0; i < NODECOUNT; i++)
    {
        node_qpos = node_get_qpos_by_node_id(traj_info, node_get_body_id_from_node_index(i));
        diff = (i - NODECOUNT/2) * 1.0/(NODECOUNT/2);
        traj_info->d->qpos[GETJOINTNUM] = init + diff;
        mj_forward(traj_info->m, traj_info->d);
        mj_local2Global(
            traj_info->d,
            node_qpos,
            NULL,
            traj_info->pert->localpos,
            traj_info->d->xquat + (4*body_id.id),
            body_id.id
            );
    }
}

void node_position_initial_using_cassie_body(
    traj_info_t* traj_info,
    cassie_body_id_t body_id)
{
    double qpos_cache[CASSIE_QPOS_SIZE];

    decor_reset(traj_info);
    mju_copy(qpos_cache, traj_info->d->qpos, CASSIE_QPOS_SIZE);

    if(SEL.node_type == NODE_POSITIONAL)
        node_position_initial_positional(traj_info, body_id);
    else if (SEL.node_type == NODE_JOINTMOVE)
    {
        node_position_jointmove(traj_info, body_id, 1, 0);
    }
    else if (SEL.node_type == NODE_JOINTID)
        node_position_jointid(traj_info, body_id);

    mju_copy(traj_info->d->qpos, qpos_cache, CASSIE_QPOS_SIZE);
    mj_forward(traj_info->m, traj_info->d);
}

double node_calculate_filter_from_frame_offset(
    double frame_offset,
    double sigma,
    double nodeheight)
{
    return mju_min(
        mju_max(nodeheight,1) * 
        gaussian_distrobution(frame_offset/sigma)
        ,1);
}

