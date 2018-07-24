
#include "node.h"

node_body_id_t node_get_body_id_from_node_index(int index)
{
    node_body_id_t id;
    id.id = index + 27;

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
    if(id.id < 27)
        return 0;
    else
        return traj_info->d->qpos + CASSIE_QPOS_SIZE + (NON_NODE_COUNT * 3) + ((id.id - 27) * 3);
}

v3_t node_get_xpos_by_node_id(traj_info_t* traj_info, node_body_id_t id)
{
    return traj_info->d->xpos + (id.id * 3);
}

v3_t node_get_body_xpos_curr(traj_info_t* traj_info, cassie_body_id_t id)
{
    return traj_info->d->xpos + (id.id * 3);
}

v3_t node_get_body_xpos_by_frame(traj_info_t* traj_info, int frame, cassie_body_id_t id)
{
    timeline_set_qposes_to_pose_frame(traj_info, frame);
    mj_forward(traj_info->m, traj_info->d);
    return node_get_body_xpos_curr(traj_info, id);
}

void node_position_initial_using_cassie_body(traj_info_t* traj_info, cassie_body_id_t body_id)
{
    int i;
    int frame;
    v3_t node_qpos;
    v3_t body_xpos;

    if(!traj_info->timeline.init)
        timeiline_init_from_input_file(traj_info);
    
    for (i = 0; i < NODECOUNT; i++)
    {
        frame = (TIMELINE_SIZE / NODECOUNT) * i;
        node_qpos = node_get_qpos_by_node_id(traj_info, node_get_body_id_from_node_index(i) );
        body_xpos = node_get_body_xpos_by_frame(traj_info, frame, body_id);
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
        150000);
    timeline_overwrite_frame_using_curr_pose(traj_info, frame);
}

void scale_target_using_frame_offset(
    traj_info_t* traj_info,
    v3_t ik_body_target_xpos, 
    v3_t grabbed_node_transformation,
    int rootframe,
    int frame_offset,
    cassie_body_id_t body_id)
{
    double filter;
    v3_t body_init_xpos;

    filter = node_calculate_filter_from_frame_offset(frame_offset, traj_info->nodesigma);

    body_init_xpos = node_get_body_xpos_by_frame(traj_info, rootframe + frame_offset, body_id);
    
    /*
    mju_addScl3
    void mju_addScl3(mjtNum res[3], const mjtNum vec1[3], const mjtNum vec2[3], mjtNum scl);
    Set res = vec1 + vec2*scl.
    */

    mju_addScl3(ik_body_target_xpos, body_init_xpos, grabbed_node_transformation, filter);
}

int get_frame_from_node_body_id(node_body_id_t node_id)
{
    return (TIMELINE_SIZE / NODECOUNT) * (node_id.id - 27); // or maybe 28
}

void calculate_node_dropped_transformation_vector(
    traj_info_t* traj_info, 
    v3_t grabbed_node_transformation,
    cassie_body_id_t body_id, 
    node_body_id_t node_id)
{
    int rootframe;
    v3_t body_init_xpos;
    v3_t node_final_xpos;

    rootframe = get_frame_from_node_body_id(node_id);
    body_init_xpos = node_get_body_xpos_by_frame(traj_info, rootframe, body_id);
    node_final_xpos = node_get_xpos_by_node_id(traj_info, node_id);

    mju_sub3(grabbed_node_transformation, node_final_xpos, body_init_xpos);
}

double normalCFD(double value)
{
   return 0.5 * erfc(-value * M_SQRT1_2);
}

double percent(int frame_offset, int iterations, double sigma)
{
    return 200 *((normalCFD(frame_offset/sigma) - normalCFD(0) ) / normalCFD((iterations+1) / sigma));
}

// double inv_norm(double t)
// {
//     const double c[] = {2.515517, 0.802853, 0.010328};
//     const double d[] = {1.432788, 0.189269, 0.001308};
//     return t - ((c[2]*t + c[1])*t + c[0]) /
//            (((d[2]*t + d[1])*t + d[0])*t + 1.0);
// }

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

    init_time = traj_calculate_runtime_micros(traj_info);

    scale_target_using_frame_offset(
        traj_info,
        ik_body_target_xpos, 
        grabbed_node_transformation,
        rootframe,
        0,
        body_id);

    timeline_set_qposes_to_pose_frame(traj_info, rootframe);

    nodeframe_ik_transform(traj_info,
        params, 
        body_id, 
        rootframe, 
        0, 
        ik_body_target_xpos,
        &ik_iter_total);

    //this is toomuch
    iterations = 3.491 * traj_info->nodesigma;

    // printf("math= %.3f\n", 
    //     inv_norm(0.0005/mju_norm(grabbed_node_transformation, 3)) * traj_info->nodesigma);

    for(frame_offset = 1; frame_offset < iterations; frame_offset++)
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
            ik_body_target_xpos, 
            grabbed_node_transformation,
            rootframe,
            frame_offset,
            body_id);
        nodeframe_ik_transform( 
            traj_info,
            params, 
            body_id, 
            rootframe + frame_offset,
            frame_offset,
            ik_body_target_xpos,
            &ik_iter_total);

        scale_target_using_frame_offset(
            traj_info,
            ik_body_target_xpos, 
            grabbed_node_transformation,
            rootframe,
            -frame_offset,
            body_id);
        nodeframe_ik_transform(
            traj_info, 
            params, 
            body_id, 
            rootframe - frame_offset, 
            -frame_offset,
            ik_body_target_xpos,
            &ik_iter_total);
    }

    iktimedelta = traj_calculate_runtime_micros(traj_info) - init_time;

    printf("Finished solving IK for %d poses in %.1f seconds\n", 
        1+iterations*2, 
        (iktimedelta/1000000.0));

    traj_info->time_start += iktimedelta;
    node_position_initial_using_cassie_body(traj_info,  body_id);
}

void node_dropped(traj_info_t* traj_info, cassie_body_id_t body_id, node_body_id_t node_id)
{
    FILE* pfile;
    int rootframe;
    double grabbed_node_transformation[3];
    ik_solver_params_t params;

    rootframe = get_frame_from_node_body_id(node_id);
    calculate_node_dropped_transformation_vector(
        traj_info, 
        grabbed_node_transformation, 
        body_id, 
        node_id);

    ik_default_fill_solver_params(&params);
    node_perform_pert(traj_info, &params, grabbed_node_transformation, body_id, rootframe);

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
        grabbed_node_transformation,
        body_id,
        node_id);

    rootframe = get_frame_from_node_body_id(node_id);

    for (i = 0; i < NODECOUNT; i++)
    {
        //skips the node currently being dragged
        if(node_get_body_id_from_node_index(i).id == node_id.id)
            continue;

        currframe = (TIMELINE_SIZE / NODECOUNT) * i;
        frame_offset = currframe - rootframe;
        filter = node_calculate_filter_from_frame_offset(frame_offset, traj_info->nodesigma);
        node_qpos = node_get_qpos_by_node_id(traj_info, node_get_body_id_from_node_index(i) );
        body_xpos = node_get_body_xpos_by_frame(traj_info, currframe, body_id);
        mju_addScl3(node_qpos, body_xpos, grabbed_node_transformation, filter);
    }
}

double node_calculate_filter_from_frame_offset(double frame_offset, double sigma)
{
    return gaussian_distrobution(frame_offset/sigma, 1) *(1/0.318310);
}

