#ifndef MAIN_H
#define MAIN_H

#include "mujoco.h"
#include "glfw3.h"
#include <stdio.h>
#include <sys/time.h>
#include <stdbool.h>

#define CASSIE_QPOS_SIZE 35
#define NODECOUNT 30
#define NON_NODE_COUNT 1
#define FILENAME_STEP_DATA "stepdata.bin" //used in simulate.c : reset_traj_info()

struct _qpos_t_
{
    double q[CASSIE_QPOS_SIZE];
};
typedef struct _qpos_t_ qpos_t;

struct _timeline_t_
{
    bool init;
    int numposes;
    qpos_t* qposes;
};
typedef struct _timeline_t_ timeline_t;

#include "pdik.h"

enum ik_seed_option_e
{
    IK_NEVER_SEED_LASTSOLN,
    IK_ALWAYS_SEED_LASTSOLN,
    IK_MOSTLY_SEED_LASTSOLN
};

struct _ik_solver_params_t_
{
    double ik_accuracy_cutoff;
    double pd_k_regular;
    double pd_b_regular;
    double pd_k_lastsoln;
    double pd_b_lastsoln;
    double lastsoln_merge_scale;
    enum ik_seed_option_e seedoption;
    uint32_t frame_mostly_seed_frequency;
    uint32_t width_frame_noseed_around_rootframe;
};
typedef struct _ik_solver_params_t_ ik_solver_params_t;

struct _target_t_
{
    int frame_offset;
    double target[3];
};
typedef struct _target_t_ target_t;

struct _traj_info_
{
    mjModel* m;
    mjData* d;
    mjvPerturb* pert;
    target_t* target_list;

    pdikdata_t ik;
    timeline_t* timeline;
    
    uint64_t time_start;
    uint64_t time_frozen;
    bool* paused;
    char* filename_step_data;
    int id_last_body_select;
    int id_last_non_node_select;
    int id_last_pert_activenum;
    double nodesigma;
    int target_list_size;
};
typedef struct _traj_info_ traj_info_t;

#include "timeline.h"
#include "ik.h"
#include "node.h"

uint64_t traj_time_in_micros();
void traj_foreach_frame(traj_info_t* traj_info);
uint64_t traj_calculate_runtime_micros(traj_info_t* traj_info);

#endif
