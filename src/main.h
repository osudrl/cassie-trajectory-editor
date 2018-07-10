#ifndef MAIN_H
#define MAIN_H

#include "mujoco.h"
#include "glfw3.h"
#include <stdio.h>
#include <sys/time.h>
#include <stdbool.h>

#define CASSIE_QPOS_SIZE 35
#define TIMELINE_SIZE 1682
#define NODECOUNT 30
#define NODE_OFFSET 1
#define FILENAME_STEP_DATA "stepdata.bin" //used in simulate.c : reset_traj_info()

struct _qpos_t_
{
    double q[CASSIE_QPOS_SIZE];
};
typedef struct _qpos_t_ qpos_t;

struct _timeline_t_
{
    uint8_t init;
    qpos_t qposes[TIMELINE_SIZE]; // this may want to be dynamically allocated
};
typedef struct _timeline_t_ timeline_t;

struct _traj_info_
{
    mjModel* m;
    mjData* d;
    mjvPerturb* pert;
    timeline_t timeline;
    uint64_t time_start;
    uint64_t time_frozen;
    bool* paused;
    char* filename_step_data;
};
typedef struct _traj_info_ traj_info_t;

#include "out.h"
#include "timeline.h"
#include "vectors.h"
#include "ik.h"
#include "node.h"

uint64_t traj_time_in_micros();
void traj_foreach_frame(traj_info_t* traj_info);
uint64_t traj_calculate_runtime_micros(traj_info_t* traj_info);

#endif
