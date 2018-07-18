#include "pdik.h"

void QuatToEuler( double *quat, double* euler)
{
    double ysqr = quat[2] * quat[2];
    double t0 = -2.0f * (ysqr + quat[3] * quat[3]) + 1.0f;
    double t1 = +2.0f * (quat[1] * quat[2] - quat[0] * quat[3]);
    double t2 = -2.0f * (quat[1] * quat[3] + quat[0] * quat[2]);
    double t3 = +2.0f * (quat[2] * quat[3] - quat[0] * quat[1]);
    double t4 = -2.0f * (quat[1] * quat[1] + ysqr) + 1.0f;

    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;

    euler[1] = mju_asin(t2);
    euler[0] = mju_atan2(t3, t4);
    euler[2] = mju_atan2(t1, t0);
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
    double closenorm;

    if (ik->doik > 0)
    {
        closenorm = apply_pd_controller(
            400,
            30,
            ik->d->xfrc_applied + ik->body_id*6,
            ik->d->xpos + ik->body_id*3,
            ik->d->cvel+ ik->body_id*6 + 3,
            ik->target_body);

        if(closenorm < ik->lowscore)
            ik->lowscore = closenorm;

        ik->doik--;
    }
}

