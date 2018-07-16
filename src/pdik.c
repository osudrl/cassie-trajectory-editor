#include "pdik.h"

void QuatToEuler( double *quat, double *rotx,  double *roty, double *rotz)
{
    double ysqr = quat[2] * quat[2];
    double t0 = -2.0f * (ysqr + quat[3] * quat[3]) + 1.0f;
    double t1 = +2.0f * (quat[1] * quat[2] - quat[0] * quat[3]);
    double t2 = -2.0f * (quat[1] * quat[3] + quat[0] * quat[2]);
    double t3 = +2.0f * (quat[2] * quat[3] - quat[0] * quat[1]);
    double t4 = -2.0f * (quat[1] * quat[1] + ysqr) + 1.0f;

    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;

    *roty = mju_asin(t2);
    *rotx = mju_atan2(t3, t4);
    *rotz = mju_atan2(t1, t0);
}


void reset_pdikdata(pdikdata_t* ik, mjModel* m, mjData* d)
{
    FILE* infile = fopen("dropdata.bin","r");


    ik->m = m;
    ik->d = d;
    ik->doik = IK_ITER;
    ik->lowscore = 1000000;
 
    fread(ik->initqposes, sizeof(mjtNum), CASSIE_QPOS_SIZE, infile);
    fread(ik->target_body, sizeof(mjtNum), 3, infile);
    fclose(infile);

    for(int i = 0; i < CASSIE_QPOS_SIZE; i++)
    {
        d->qpos[i] = ik->initqposes[i];
    }

    mj_forward(m, d);

    for (int i = 0; i < 3; ++i) 
    {
        m->jnt_stiffness[i] = 1000000;
        m->dof_damping[i] = 100000;
        m->qpos_spring[i] = d->qpos[i];
    }

    m->jnt_stiffness[22] = 100000;
    // m->dof_damping[22] = 1;
    m->qpos_spring[22] = d->qpos[22];

    for (int i = 3; i < 7; ++i)
        m->dof_damping[i] = 500;

    QuatToEuler(ik->d->xquat+4, ik->target_pelvis_euler, ik->target_pelvis_euler+1, ik->target_pelvis_euler+2);

    //set target to "itself?"
    // mju_copy(ik->target_body, ik->d->xpos + 25*3, 3);

    mju_copy(ik->target_pelvis, ik->d->xpos + 3*1, 3);
    mju_copy(ik->target_other, ik->d->xpos + 13*3, 3);

    for(int i = 0 ; i < ik->m->nv; i++)
    {
        ik->d->qvel[i] = 0;
    }

    ik->frame = 777;
    // if(ik->outfile)
    //     fclose(ik->outfile);
    ik->outfile = fopen("iksolvedata.bin", "w");
    if (!ik->outfile)
        printf("yo wat\n");
}

double apply_pd_controller(double k1, double k2, double* forces, double* xcurr, double* vcurr, double* xtarget)
{
    double xdelta[3];
    double vdelta[3];
    double vtarget[3];
    double norm;

    mju_zero3(vtarget);
    mju_sub3(xdelta, xtarget, xcurr);
    norm =  mju_norm(xdelta,3);
    mju_sub3(vdelta, vtarget, vcurr);
    mju_scl3(xdelta,xdelta,k1);
    mju_scl3(vdelta,vdelta,k2);
    mju_add3(forces, xdelta, vdelta);

    return norm;
}


void pdik_per_step_control(pdikdata_t* ik)
{
    double xscale = 1000;
    double vscale = 50;
    double closenorm;
    double res[3];
    double ones[3];
    double f[3];
    ikoutdata_t od;

    // d->xfrc_applied[6+0] += 1000*(0-d->xpos[6+0]);
    // d->xfrc_applied[6+1] += 1000*(0-d->xpos[6+1]);
    // double vel[6];

    // mj_objectVelocity(m,d,mjOBJ_BODY,1,vel,0);
    // printf("z %.5f \n", d->xpos[6+2]);
    // d->xfrc_applied[6+0] += 10*(2-d->xpos[6+0]) + 100 * (0-d->cvel[6+3]);
    // d->xfrc_applied[6+1] += 10*(2-d->xpos[6+1]) + 100 * (0-d->cvel[6+4]);
    // d->xfrc_applied[6+2] += 10*(2-d->xpos[6+2]) + 100 * (0-d->cvel[6+5]);

    if (ik->doik > 0 && ik->doik < IK_ITER - 0)
    {
        od.iter = IK_ITER - ik->doik;
        // od.off_pelvis = apply_pd_controller(
        //         10*xscale,
        //         10*vscale,
        //         ik->d->qfrc_applied,
        //         ik->d->xpos + 3,
        //         ik->d->cvel+ 6 + 3,
        //         ik->target_pelvis );
        // QuatToEuler(ik->d->xquat+4, res, res+1, res+2);
        // // printf("%.2f %.2f %.2f    ", res[0], res[1], res[2]);
        // od.off_orientation = apply_pd_controller(
        //         -2*xscale,
        //         vscale*2,
        //         f,
        //         res,
        //         ik->d->cvel+ 6 ,
        //         ik->target_pelvis_euler);

        // ik->d->qfrc_applied[3] = f[0];
        // ik->d->qfrc_applied[4] = f[1];
        // ik->d->qfrc_applied[5] = f[2];
        // ik->d->qfrc_applied[4] = .1;
        // ik->d->qfrc_applied[3] = f[0];
        // ik->d->qfrc_applied[4] = f[2];
        // ik->d->qfrc_applied[5] = f[0];
        // ik->d->qfrc_applied[4] = f[1];
        // ik->d->qfrc_applied[5] = f[1];
        // ik->d->qfrc_applied[4] = f[1];
        // ik->d->qfrc_applied[5] = f[2];
        // ik->d->qfrc_applied[4] = f[2];
        // ik->d->qfrc_applied[4] = f[1];

        
        closenorm = apply_pd_controller(
            15000 * (1.0/ik->lowscore > 500 ? 500 : 1.0/ik->lowscore),
            1.0/ik->lowscore > 1000 ? 100 : 1,
            ik->d->xfrc_applied + 25*6,
            ik->d->xpos + 25*3,
            ik->d->cvel+ 25*6 + 3,
            ik->target_body);

        

        od.off_rfoot = closenorm;

        // printf("close %.5f\n", closenorm);

        if(closenorm < ik->lowscore)
        {
            for(int i = 0; i < 35; i++)
                ik->bestqposes[i] = ik->d->qpos[i];
            ik->lowscore = closenorm;
        }

        od.best_rfoot_off = ik->lowscore;

        for(int i = 13; i <= 13; i++)
        {
            // od.off_lfoot = apply_pd_controller(
            //      0*.5*xscale,
            //      0*1*vscale,
            //     ik->d->xfrc_applied + i*6,
            //     ik->d->xpos + i*3,
            //     ik->d->cvel+ i*6 + 3,
            //     ik->target_other );
            // printf("%d\n",i);
        // d->xfrc_applied[i*6 + 0] = 5*(initxposes[i*3 + 0] - d->xpos[i*3 + 0])  + 1 *(0-d->cvel[i*6 + 3]);
        // d->xfrc_applied[i*6 + 1] = 5*(initxposes[i*3 + 1] - d->xpos[i*3 + 1])  + 1 *(0-d->cvel[i*6 + 4]);
        // d->xfrc_applied[i*6 + 2] = 5*(initxposes[i*3 + 2] - d->xpos[i*3 + 2])  + 1 *(0-d->cvel[i*6 + 5]);
        }
        mju_copy(od.curr_qposes, ik->d->qpos, CASSIE_QPOS_SIZE);
        if (ik->outfile)
            fwrite(&od, sizeof(ikoutdata_t), 1, ik->outfile);
    }
    ik->doik--;
}

