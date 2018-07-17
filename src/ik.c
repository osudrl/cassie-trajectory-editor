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

void cheat(pdikdata_t* ik)
{
    int bestindex;
    double* my_pelvis;
    double* my_foot;
    double bestdiff = 10000000;
    double reffoot[3];
    double footdiff[3];
    double tempnorm;

    my_pelvis = ik->target_pelvis;
    my_foot = ik->target_body;
    for (int i = 0; i < 5000000; i+=71)
    {
        mju_add3(reffoot, my_pelvis, ik->lib[i].v_pelvis_to_foot);
        mju_sub3(footdiff, my_foot, reffoot);
        tempnorm = mju_norm(footdiff,3);
        if(tempnorm < bestdiff)
        {
            bestdiff = tempnorm;
            bestindex = i;
        }
    }

    mju_copy(ik->d->qpos + 21, ik->lib[bestindex].curr_qposes + 21, CASSIE_QPOS_SIZE - 22);
}

void ik_iterative_better_body_optimizer(
    traj_info_t* traj_info,
    double* xyz_xpos_target, 
    int body_id_end,
    int frame,
    int count)
{
    traj_info->ik.maxiter = 100000;
    traj_info->ik.doik = 100000;
    traj_info->ik.lowscore = 500000;

    // mj_forward(traj_info->m, traj_info->d); //should be unnessesary
    mju_copy3(traj_info->ik.target_body, xyz_xpos_target);
    mju_copy3(traj_info->ik.target_pelvis, traj_info->d->xpos + (3*1));
    QuatToEuler(traj_info->d->xquat+4, traj_info->ik.target_pelvis_euler);
    mju_copy3(traj_info->ik.target_other, traj_info->d->xpos + (3*13));

    traj_info->ik.frame = frame;

    cheat(&traj_info->ik);   

    for (int i = 0; i < 3; ++i) 
    {
        traj_info->m->jnt_stiffness[i] = 1000000;
        traj_info->m->dof_damping[i] = 100000;
        traj_info->m->qpos_spring[i] = traj_info->d->qpos[i];
    }

    for (int i = 3; i < 7; ++i)
        traj_info->m->dof_damping[i] = 500;

    for (int i = 0; i < traj_info->m->nv; i++)
        traj_info->d->qvel[i] = 0;

    while(traj_info->ik.doik > 0 && traj_info->ik.lowscore > .01)
    {
        mju_zero(traj_info->d->xfrc_applied, 6*traj_info->m->nbody);
        mj_step(traj_info->m,traj_info->d);
    }

    // if(traj_info->ik.doik == 0)
    // {
    //     printf("MAY BE IMPOSSIBLE\n");
    // }

    traj_info->ik.doik = 0;
}

