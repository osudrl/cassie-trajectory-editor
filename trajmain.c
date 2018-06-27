#include "trajmain.h"

double body_get_apos(mjData* data, int body_id, int cord_id)
{
    return data->xpos[body_id*3 + cord_id];
}

double body_get_xpos(mjData* data, int body_id)
{
    return body_get_apos(data, body_id, CORD_ID_X);
}

double body_get_ypos(mjData* data, int body_id)
{
    return body_get_apos(data, body_id, CORD_ID_Y);
}

double body_get_zpos(mjData* data, int body_id)
{
    return body_get_apos(data, body_id, CORD_ID_Z);
}

void body_pos_difference(mjData* data, double* xyz_arr, int body_id_end, int body_id_root)
{
    xyz_arr[0] = body_get_xpos(data, body_id_end) - body_get_xpos(data, body_id_root);
    xyz_arr[1] = body_get_ypos(data, body_id_end) - body_get_ypos(data, body_id_root);
    xyz_arr[2] = body_get_zpos(data, body_id_end) - body_get_zpos(data, body_id_root);
}

double body_pos_difference_mag(mjData* data, int body_id_end, int body_id_root)
{
    double xyz_arr[3];
    body_pos_difference(data, xyz_arr, body_id_end, body_id_root);
    return mju_norm(xyz_arr, 3);
}

float goon_abs(float f)
{
    return f > 0 ? f : -f;
}

void vector3_subtraction(double* xyz_result, double* xyz_curr_end_to_root, double* xyz_target_end_to_root)
{
    xyz_result[0] = xyz_target_end_to_root[0] - xyz_curr_end_to_root[0];
    xyz_result[1] = xyz_target_end_to_root[1] - xyz_curr_end_to_root[1];
    xyz_result[2] = xyz_target_end_to_root[2] - xyz_curr_end_to_root[2];
}

double norm_of_vector3_subtraction(double* xyz_curr_end_to_root, double* xyz_target_end_to_root)
{
    double xyz_result[3];
    vector3_subtraction(xyz_result, xyz_curr_end_to_root, xyz_target_end_to_root);
    return mju_norm(xyz_result, 3);
}

double fwd_kinematics_observe(
    mjModel* m, 
    mjData* d, 
    double* xyz_target_end_to_root, 
    int body_id_end, 
    int body_id_root)
{
    double xyz_observed_dist[3];

    mj_kinematics(m,d);
    body_pos_difference(d, xyz_observed_dist, body_id_end, body_id_root);
    // observed_diff = norm_of_vector3_subtraction(xyz_observed_dist, xyz_target_end_to_root);
    double my[3] = {
        body_get_xpos(d, body_id_end),
        body_get_ypos(d, body_id_end),
        body_get_zpos(d, body_id_end),
    };

    // observed_diff = norm_of_vector3_subtraction(xyz_observed_dist, xyz_target_end_to_root);
    return norm_of_vector3_subtraction(my, xyz_target_end_to_root);
}

void fwd_kinematics_compare_result(
    mjModel* m, 
    mjData* d, 
    double* xyz_target_end_to_root, 
    int body_id_end, 
    int body_id_root,
    double* best_diff,
    int* best_qpos_index,
    int* positive_axis,
    int current_index,
    int is_positive)
{
    double observed_diff;

    observed_diff = fwd_kinematics_observe(m,d,xyz_target_end_to_root,body_id_end,body_id_root);

    if(observed_diff < *best_diff) 
    {
        *best_diff = observed_diff;
        *best_qpos_index = current_index;
        *positive_axis = is_positive;
    }
}

void better_body_optimizer(
    mjModel* m, 
    mjData* d, 
    double* xyz_target_end_to_root, 
    int body_id_end, 
    int body_id_root)
{
    int i;
    double best_diff;
    double pos_val_before_dx;
    int best_qpos_index = -1;
    int positive_axis ;
    double dx;

    best_diff = fwd_kinematics_observe(m,d,xyz_target_end_to_root,body_id_end,body_id_root);
    dx = 1.93 * 0.01 * best_diff + 0.0005;
    for(i = 7 ; i < CASSIE_QPOS_SIZE; i++)
    {
        pos_val_before_dx = d->qpos[i];

        d->qpos[i] = pos_val_before_dx + dx;

        fwd_kinematics_compare_result(
            m, 
            d, 
            xyz_target_end_to_root, 
            body_id_end, 
            body_id_root,
            &best_diff,
            &best_qpos_index,
            &positive_axis,
            i,
            1);

        d->qpos[i] = pos_val_before_dx - dx;

        fwd_kinematics_compare_result(
            m, 
            d, 
            xyz_target_end_to_root, 
            body_id_end, 
            body_id_root,
            &best_diff,
            &best_qpos_index,
            &positive_axis,
            i,
            0);

        d->qpos[i] = pos_val_before_dx;
    }

    if(!positive_axis)
        dx = -dx;
    if(best_qpos_index > 2)
        d->qpos[best_qpos_index] += dx;
    //dist = body_pos_difference_mag(d, body_id_end, body_id_root);
    printf("i=%d diff=%.5f \n",
        best_qpos_index, best_diff);
}

void random_body_optimizer(mjModel* m, mjData* d, double targetval, int body_id_end, int body_id_root)
{
    int qpos_max_size = 34;
    int i = -1;
    double dx = 0.005;
    double init_dist;
    double dist;
    double init_pos_val;
    double bestdiff = 1000000;
    int index;
    int pos ;

    init_dist = body_pos_difference_mag(d, body_id_end, body_id_root);

    for(i = 0 ; i < 34; i++)
    {
        init_pos_val = d->qpos[i];
        d->qpos[i] = init_pos_val +0.005 ;
        mj_kinematics(m,d);
        dist = body_pos_difference_mag(d, body_id_end, body_id_root);
        if(goon_abs(targetval-dist) < bestdiff) 
        {
            bestdiff = goon_abs(dist - targetval);
            index = i;
            pos = 1;
        }

        d->qpos[i] = init_pos_val -0.005 ;

        mj_kinematics(m,d);
        dist = body_pos_difference_mag(d, body_id_end, body_id_root);
        if(goon_abs(targetval-dist)  < bestdiff) 
        {
            bestdiff = goon_abs(dist - targetval);
            index = i;
            pos = 0;
        }

        d->qpos[i] = init_pos_val;
    }

    if(!pos)
        dx = -0.005;
    if(index > 2)
        d->qpos[index] += dx;
    //dist = body_pos_difference_mag(d, body_id_end, body_id_root);
    //printf("i=%d dist=%.5f diff=%.5f bestdif=%.5f\n", index, dist, init_dist - dist, bestdiff);
}

int allow_pelvis_to_be_grabbed_and_moved(mjData* d, mjvPerturb* pert, double* xyz_ref)
{
    if(pert->active) 
    {
        printf("pert.select = %d\n",pert->select);
        if(pert->select == 1)
        {
            d->qpos[0] = pert->refpos[0];
            d->qpos[1] = pert->refpos[1];
            d->qpos[2] = pert->refpos[2];
            return 0;
        }
        else
        {
            xyz_ref[0] = pert->refpos[0];
            xyz_ref[1] = pert->refpos[1];
            xyz_ref[2] = pert->refpos[2];
            return 1;
        }
    }
    return 0;
}

int countup = 0;

int ksimcounter = 0;
int oldksimcounter = -1;
int kflip = 0;
double oldpos;

void traj_foreach_frame(mjModel* m, mjData* d, mjvPerturb* pert)
{
    int mod;

    countup++;
    countup %= 100;
    float thing = countup; // 0 to 1000
    thing -= 50; // -500 to 500
    thing = thing > 0 ? thing : -thing;

    if( !kflip && thing < 1)
    {
        ksimcounter++;
        ksimcounter %= CASSIE_QPOS_SIZE;
        kflip = 1;
    } 
    else if (!(thing < 1))
    {
        kflip = 0;
    }

    if(ksimcounter != oldksimcounter)
    {
        printf("ksimcounter=%d\n",ksimcounter);
        double temp = oldpos;
        oldpos = d->qpos[ksimcounter];
        d->qpos[oldksimcounter] = temp;
        oldksimcounter = ksimcounter;
    }

    // d->qpos[2] = 1.5;
    d->qpos[ksimcounter] = thing/10;
    // d->qpos[16] = thing/100;
    
    double xyz_target_end_to_root[3] = {0,0,0};
    mod = 0;
    mod = allow_pelvis_to_be_grabbed_and_moved(d,pert,xyz_target_end_to_root);

    /*
    mj_kinematics(m,d);
    printf("%.5f \n",body_pos_difference_mag(d, 13, 1));
    
    */
    if(!mod)
    {
    d->qpos[ksimcounter] = thing/10;

    }
    for(int z = 0; mod && z < 100; z++)
        better_body_optimizer(m,d,xyz_target_end_to_root,pert->select, pert->select);
        // better_body_optimizer(m,d,xyz_target_end_to_root,13,25);
    // random_body_optimizer(m,d,0,13,25);
    mj_forward(m, d);
}

