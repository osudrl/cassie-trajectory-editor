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

void fill_joint_array(int* arr, int body_id_end)
{
    int i= 0;

    
    
    if(body_id_end >= 14 && body_id_end <= 25)
    {
        arr[i++] = 30;
        arr[i++] = 29;
        arr[i++] = 28;
        arr[i++] = 23;
        arr[i++] = 22;
        arr[i++] = 21;
    }
    else if (body_id_end >= 2 && body_id_end <= 13)
    {
        arr[i++] = 16;
        arr[i++] = 15;
        arr[i++] = 14;
        arr[i++] = 9;
        arr[i++] = 8;
        arr[i++] = 7;
    }
    else
        for(i = 0; i < CASSIE_QPOS_SIZE - 7; i++)
            arr[i] = i+7;

    arr[i] = -1;
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
    double observed_diff = 1;
    int joint_array[CASSIE_QPOS_SIZE];
    int index;

    fill_joint_array(joint_array,body_id_end);
    best_diff = ik_fwd_kinematics_score(traj_info,xyz_xpos_target,body_id_end);
    dx = 1.93 * .35 * best_diff + 0.00001;
    for(i = 0; i < CASSIE_QPOS_SIZE && joint_array[i] >= 0; i++)
    {
        index = joint_array[i];

        pos_val_before_dx = traj_info->d->qpos[index];

        traj_info->d->qpos[index] = pos_val_before_dx + dx;

        observed_diff = ik_fwd_kinematics_score(traj_info,xyz_xpos_target,body_id_end);
        if(observed_diff < best_diff) 
        {
            best_diff = observed_diff;
            best_qpos_index = index;
            positive_axis = 1;
        }

        traj_info->d->qpos[index] = pos_val_before_dx - dx;

        observed_diff = ik_fwd_kinematics_score(traj_info,xyz_xpos_target,body_id_end);
        if(observed_diff < best_diff) 
        {
            best_diff = observed_diff;
            best_qpos_index = index;
            positive_axis = 0;
        }

        traj_info->d->qpos[index] = pos_val_before_dx;
    }

    if(!positive_axis)
        dx = -dx;
    if(best_qpos_index >= 0)
        traj_info->d->qpos[best_qpos_index] += dx;
    
    return best_diff;
}

double apply_pd_controller(double* forces, double* xcurr, double* vcurr, double* xtarget)
{
    double xdelta[3];
    double vdelta[3];
    double vtarget[3];
    double norm;

    mju_zero3(vtarget);
    mju_sub3(xdelta, xtarget, xcurr);
    norm =  mju_norm(xdelta,3);
    mju_sub3(vdelta, vtarget, vcurr);
    mju_scl3(xdelta,xdelta,4);
    mju_scl3(vdelta,vdelta,.5);
    mju_add3(forces, xdelta, vdelta);

    return norm;
}

void ik_iterative_better_body_optimizer(
    traj_info_t* traj_info,
    double* xyz_xpos_target, 
    int body_id_end,
    int count)
{
    double best_diff;
    double curr_diff;
    int i;
    double initqposes[CASSIE_QPOS_SIZE];
    double bestqposes[CASSIE_QPOS_SIZE];

    for(int x = 0; x < CASSIE_QPOS_SIZE; x++)
    {
        initqposes[x] = traj_info->d->qpos[x];
    }

    best_diff = 500; //bignumber
    for (i = 0; i < count && best_diff > .1; i++)
    {
        // for(int x = 0; x < 7; x++)
        // {
        //     traj_info->d->qpos[x] = initqposes[x];
        // }

        // best_diff = apply_pd_controller(
        //         traj_info->d->xfrc_applied + 25*6,
        //         traj_info->d->xpos + 25*3,
        //         traj_info->d->cvel+ 25*6 + 3,
        //         xyz_xpos_target);

        
        
        mj_step(traj_info->m, traj_info->d);
    }
    if(i == count)
    {
        printf("MAY BE IMPOSSIBLE\n");
    }
    for(int x = 0; x < CASSIE_QPOS_SIZE; x++)
    {
        traj_info->d->qpos[x] = bestqposes[x];
    }
}

