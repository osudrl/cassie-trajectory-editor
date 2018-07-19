#include "pdik.h"

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
    double closenorm;

    if (ik->doik > 0)
    {
        closenorm = apply_pd_controller(
            ik->pd_k,
            ik->pd_b,
            ik->d->xfrc_applied + ik->body_id*6,
            ik->d->xpos + ik->body_id*3,
            ik->d->cvel+ ik->body_id*6 + 3,
            ik->target_body);

        if(closenorm < ik->lowscore)
            ik->lowscore = closenorm;

        ik->doik--;
    }
}

