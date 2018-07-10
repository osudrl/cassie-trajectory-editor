#include "ik.h"

double ik_fwd_kinematics_score(
    traj_info_t* traj_info,
    double* xyz_xpos_target, 
    int body_id_end)
{
    double* xyz_xpos_curr_end;

    mj_kinematics(traj_info->m,traj_info->d);   
    xyz_xpos_curr_end = traj_info->d->xpos + body_id_end*3;
    return vectors_norm_of_vector3_subtraction(xyz_xpos_curr_end, xyz_xpos_target);
}

double ik_better_body_optimizer(
    traj_info_t* traj_info,
    double* xyz_xpos_target, 
    int body_id_end)
{
    int i;
    double best_diff;
    double pos_val_before_dx;
    int best_qpos_index = -1;
    int positive_axis = 0;
    double dx;
    double observed_diff;

    best_diff = ik_fwd_kinematics_score(traj_info,xyz_xpos_target,body_id_end);
    dx = 1.93 * .25 * best_diff + 0.0001;
    for(i = 7 ; i < CASSIE_QPOS_SIZE; i++)
    {
        pos_val_before_dx = traj_info->d->qpos[i];

        traj_info->d->qpos[i] = pos_val_before_dx + dx;

        observed_diff = ik_fwd_kinematics_score(traj_info,xyz_xpos_target,body_id_end);
        if(observed_diff < best_diff) 
        {
            best_diff = observed_diff;
            best_qpos_index = i;
            positive_axis = 1;
        }

        traj_info->d->qpos[i] = pos_val_before_dx - dx;

        observed_diff = ik_fwd_kinematics_score(traj_info,xyz_xpos_target,body_id_end);
        if(observed_diff < best_diff) 
        {
            best_diff = observed_diff;
            best_qpos_index = i;
            positive_axis = 0;
        }

        traj_info->d->qpos[i] = pos_val_before_dx;
    }

    if(!positive_axis)
        dx = -dx;
    if(best_qpos_index > 2)
        traj_info->d->qpos[best_qpos_index] += dx;
    
    return observed_diff;
}

void ik_iterative_better_body_optimizer(
    traj_info_t* traj_info,
    double* xyz_xpos_target, 
    int body_id_end,
    int count)
{
    double observed_diff;
    int i;

    observed_diff = 500; //bignumber
    for (i = 0; i < count && observed_diff > 0.0015; i++)
    {
        observed_diff = ik_better_body_optimizer(traj_info, xyz_xpos_target, body_id_end);
    }    
}

