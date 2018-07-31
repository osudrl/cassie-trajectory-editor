#ifndef MAIN_H
#define MAIN_H

#include "mujoco.h"
#include "glfw3.h"
#include <stdio.h>
#include <sys/time.h>
#include <stdbool.h>

#define CASSIE_QPOS_SIZE 35
#define NODECOUNT 100
#define NON_NODE_COUNT 0
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
    struct _timeline_t_* prev;
    struct _timeline_t_* next;
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

enum node_type_e
{
    NODES_POSITIONAL,
    NODES_JOINTID,
    NODES_JOINTTRANSFORM
};

enum pert_type_e
{
    PERT_TRANSLATION,
    PERT_TARGET
};

struct _selection_t_
{
    int id_last_body_select;
    int id_last_non_node_select;
    int id_last_pert_activenum;

    enum node_type_e node_type;
    enum pert_type_e pert_type;
    double nodesigma;
    double nodeheight;
    int jointnum;
};
typedef struct _selection_t_ selection_t;

struct _traj_info_
{
    mjModel* m;
    mjData* d;
    mjvPerturb* pert;
    target_t* target_list;

    int refine_body;
    int refine_rootframe;

    pdikdata_t ik;
    selection_t selection;
    timeline_t* timeline;
    
    int64_t time_start;
    int64_t time_frozen;
    bool* paused;
    char* filename_step_data;

    int target_list_size;    
};
typedef struct _traj_info_ traj_info_t;

#include "timeline.h"
#include "ik.h"
#include "node.h"

uint64_t traj_time_in_micros();
void traj_foreach_frame(traj_info_t* traj_info);
int64_t traj_calculate_runtime_micros(traj_info_t* traj_info);

#endif
