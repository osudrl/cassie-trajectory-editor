#ifndef MAIN_H
#define MAIN_H

#include "mujoco.h"
#include "glfw3.h"
#include <stdio.h>
#include <sys/time.h>

#define CASSIE_QPOS_SIZE 35
#define TIMELINE_SIZE 23

struct __attribute__ ((packed)) _q_pos_t_
{
    double qpos[CASSIE_QPOS_SIZE];
};
typedef struct _q_pos_t_ q_pos_t;

struct __attribute__ ((packed)) _traj_pt_t_
{
    double time;
    q_pos_t qpos;
    double qvel[32];
    double torque[10];
    double mpos[10];
    double mvel[10];
};
typedef struct _traj_pt_t_ traj_pt_t;

struct __attribute__ ((packed)) _timeline_t_
{
    traj_pt_t qposlist[TIMELINE_SIZE];
};
typedef struct _timeline_t_ timeline_t;

struct _traj_info_
{
    mjModel* m;
    mjData* d;
    mjvPerturb* pert;
    timeline_t* timeline;
};
typedef struct _traj_info_ traj_info_t;

#include "out.h"
#include "in.h"
#include "vectors.h"
#include "ik.h"

uint64_t traj_time_in_micros();
void traj_foreach_frame(traj_info_t* traj_info);

#endif
