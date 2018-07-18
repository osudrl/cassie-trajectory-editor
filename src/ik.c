#include "ik.h"

void ik_reset_collateral_qpos_damage(traj_info_t* traj_info, double* initqpos, int body_id)
{
    if(body_id >= 14 && body_id <= 25)
        for (int i = 7; i <= 20; i++)
            traj_info->d->qpos[i] = initqpos[i];
    if(body_id >= 2 && body_id <= 13)
        for (int i = 20; i <= 34; i++)
            traj_info->d->qpos[i] = initqpos[i];
}

void ik_set_pelvis_springs(traj_info_t* traj_info)
{
    for (int i = 0; i < 3; ++i) 
    {
        traj_info->m->jnt_stiffness[i] = 1000000;
        traj_info->m->dof_damping[i] = 100000;
        traj_info->m->qpos_spring[i] = traj_info->d->qpos[i];
    }

    for (int i = 3; i < 7; ++i)
        traj_info->m->dof_damping[i] = 500;
}

void ik_zero_velocities(traj_info_t* traj_info)
{
    for (int i = 0; i < traj_info->m->nv; i++)
         traj_info->d->qvel[i] = 0;
}

int ik_iterative_better_body_optimizer(
    traj_info_t* traj_info,
    double* xyz_xpos_target, 
    int body_id_end,
    int frameoffset,
    int count)
{
    double initqpos[CASSIE_QPOS_SIZE];
    int returnvalue;

    mju_copy(initqpos, traj_info->d->qpos, CASSIE_QPOS_SIZE);
    traj_info->ik.max_doik = count;
    traj_info->ik.doik = count;
    traj_info->ik.lowscore = 500000; // just a big number

    traj_info->ik.frame = frameoffset;

    traj_info->ik.body_id = body_id_end;
    mju_copy3(traj_info->ik.target_body, xyz_xpos_target);

    ik_set_pelvis_springs(traj_info);
    ik_zero_velocities(traj_info);    

    while(traj_info->ik.doik > 0 && traj_info->ik.lowscore > .001)
    {
        mju_zero(traj_info->d->xfrc_applied, 6*traj_info->m->nbody);
        mj_step(traj_info->m,traj_info->d);
    }

    if(traj_info->ik.doik == 0)
    {
        printf("Relative frame %d maxed out %d iterations!\n", frameoffset, traj_info->ik.max_doik);
    }

    returnvalue = traj_info->ik.max_doik - traj_info->ik.doik;
    traj_info->ik.doik = 0;

    ik_reset_collateral_qpos_damage(traj_info, initqpos, body_id_end);

    return returnvalue;
}

