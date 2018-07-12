#include "pdik.h"


void reset_pdikdata(pdikdata_t* ik, mjModel* m, mjData* d)
{
    FILE* infile = fopen("dropdata.bin","r");
    
    ik->m = m;
    ik->d = d;
    ik->doik = 10000;
    ik->lowscore = 1000000;

    fread(ik->initqposes, sizeof(mjtNum), CASSIE_QPOS_SIZE, infile);
    fread(ik->target_body, sizeof(mjtNum), 3, infile);
    fclose(infile);

    for(int i = 0; i < CASSIE_QPOS_SIZE; i++)
    {
        d->qpos[i] = ik->initqposes[i];
    }

    mj_forward(m, d);

    mju_copy(ik->target_pelvis, ik->d->xpos + 3*1, 3);
    mju_copy(ik->target_other, ik->d->xpos + 13*3, 3);
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
    mju_scl3(xdelta,xdelta,100);
    mju_scl3(vdelta,vdelta,.5);
    mju_add3(forces, xdelta, vdelta);

    return norm;
}

void pdik_per_step_control(pdikdata_t* ik)
{
    double closenorm;
    // d->xfrc_applied[6+0] += 1000*(0-d->xpos[6+0]);
    // d->xfrc_applied[6+1] += 1000*(0-d->xpos[6+1]);
    // double vel[6];

    // mj_objectVelocity(m,d,mjOBJ_BODY,1,vel,0);
    // printf("z %.5f \n", d->xpos[6+2]);
    // d->xfrc_applied[6+0] += 10*(2-d->xpos[6+0]) + 100 * (0-d->cvel[6+3]);
    // d->xfrc_applied[6+1] += 10*(2-d->xpos[6+1]) + 100 * (0-d->cvel[6+4]);
    // d->xfrc_applied[6+2] += 10*(2-d->xpos[6+2]) + 100 * (0-d->cvel[6+5]);

    if (ik->doik > 0)
    {
        for(int i = 0; i < 7; i++)
        {
            ik->d->qpos[i] = ik->initqposes[i];
        }

        closenorm = apply_pd_controller(
            ik->d->xfrc_applied + 25*6,
            ik->d->xpos + 25*3,
            ik->d->cvel+ 25*6 + 3,
            ik->target_body);

        printf("close %.5f\n", closenorm);

        if(closenorm < ik->lowscore)
        {
            for(int i = 0; i < 35; i++)
                ik->bestqposes[i] = ik->d->qpos[i];
            ik->lowscore = closenorm;
        }

        for(int i = 13; i <= 13; i++)
        {
            apply_pd_controller(
                ik->d->xfrc_applied + i*6,
                ik->d->xpos + i*3,
                ik->d->cvel+ i*6 + 3,
                ik->target_other );
        // d->xfrc_applied[i*6 + 0] = 5*(initxposes[i*3 + 0] - d->xpos[i*3 + 0])  + 1 *(0-d->cvel[i*6 + 3]);
        // d->xfrc_applied[i*6 + 1] = 5*(initxposes[i*3 + 1] - d->xpos[i*3 + 1])  + 1 *(0-d->cvel[i*6 + 4]);
        // d->xfrc_applied[i*6 + 2] = 5*(initxposes[i*3 + 2] - d->xpos[i*3 + 2])  + 1 *(0-d->cvel[i*6 + 5]);
        }
        ik->doik--;
    }
}

