#include "ik.h"

void ik_set_left_leg(traj_info_t* traj_info, double* ref)
{
    for (int i = 7; i <= 20; i++)
        traj_info->d->qpos[i] = ref[i];
}

void ik_set_right_leg(traj_info_t* traj_info, double* ref)
{
    for (int i = 20; i <= 34; i++)
        traj_info->d->qpos[i] = ref[i];
}

void ik_set_opposite_leg(traj_info_t* traj_info, double* ref, int body_id)
{
    if(body_id >= 14 && body_id <= 25)
        ik_set_left_leg(traj_info, ref);
        
    if(body_id >= 2 && body_id <= 13)
        ik_set_right_leg(traj_info, ref);
}

void ik_set_selected_leg(traj_info_t* traj_info, double* ref, int body_id)
{
    if(body_id >= 14 && body_id <= 25)
        ik_set_right_leg(traj_info, ref);
        
    if(body_id >= 2 && body_id <= 13)
        ik_set_left_leg(traj_info, ref);
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

double ik_positive_keyed_qposes[CASSIE_QPOS_SIZE];
double ik_negative_keyed_qposes[CASSIE_QPOS_SIZE];

void ik_basic_setup(traj_info_t* traj_info)
{ 
    traj_info->ik.pd_k = 480;
    traj_info->ik.pd_b = 30;   
}

void ik_cheater_setup(traj_info_t* traj_info, int frameoffset, int body_id)
{
    if (frameoffset > 0)
        ik_set_selected_leg(traj_info, ik_positive_keyed_qposes, body_id);
    if (frameoffset < 0)
        ik_set_selected_leg(traj_info, ik_negative_keyed_qposes, body_id);

    traj_info->ik.pd_k = 50000;
    traj_info->ik.pd_b = 10;
}

FILE* ikoutfile = NULL;

void outit(traj_info_t* traj_info, int frameoffset, double* initqpos, int returnvalue)
{
    double d_frameoffset;
    double d_returnvalue;

    if(!ikoutfile)
        ikoutfile = fopen("fool.bin", "w");

    d_frameoffset = frameoffset * 1.0;
    d_returnvalue = returnvalue * 1.0;

    fwrite(&d_frameoffset, sizeof(double), 1, ikoutfile);
    fwrite(&d_returnvalue, sizeof(double), 1, ikoutfile);
    fwrite(initqpos, sizeof(double) * CASSIE_QPOS_SIZE, 1, ikoutfile);
    fwrite(traj_info->d->qpos, sizeof(double) * CASSIE_QPOS_SIZE, 1, ikoutfile);
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

    if((frameoffset + 10000 )% 100 == 0)
        ik_basic_setup(traj_info);
    else
        ik_cheater_setup(traj_info, frameoffset, body_id_end);

    ik_set_pelvis_springs(traj_info);
    ik_zero_velocities(traj_info);

    while(traj_info->ik.doik > 0 && traj_info->ik.lowscore > .0001)
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

    ik_set_opposite_leg(traj_info, initqpos, body_id_end);

    if(frameoffset >= 0)
        mju_copy(ik_positive_keyed_qposes, traj_info->d->qpos, CASSIE_QPOS_SIZE);
    if(frameoffset <= 0)
        mju_copy(ik_negative_keyed_qposes, traj_info->d->qpos, CASSIE_QPOS_SIZE);

    outit(traj_info, frameoffset, initqpos, returnvalue);

    return returnvalue;
}

