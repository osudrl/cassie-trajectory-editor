#ifndef MAIN_H
#define MAIN_H

#include "mujoco.h"
#include "glfw3.h"
#include <stdio.h>
#include <sys/time.h>
#include <stdbool.h>

#define CASSIE_QPOS_SIZE 35
#define XMLNODECOUNT 50
#define NODECOUNT (traj_info->selection.nodecount)
#define FILENAME_STEP_DATA "stepdata.bin" //used in simulate.c : reset_traj_info()
#define DECOR_BUF_SIZE 400
#define IK_STEP_CUTOFF 1500

struct _qpos_t_
{
    double q[CASSIE_QPOS_SIZE];
};
typedef struct _qpos_t_ qpos_t;

struct _timeline_t_
{
    int numposes;
    double duration;
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
    NODE_POSITIONAL = 0,
    NODE_JOINTID = 1,
    NODE_JOINTMOVE = 2
};

#define NODE_TYPE_E_COUNT 3

enum scale_type_e
{
    SCALING_A = 0,
    SCALING_B = 1,
};

#define SCALE_TYPE_E_COUNT 2

struct _decor_t_
{
    int count;

    mjtNum size_default_positional[3];
    mjtNum size_default_joint[3];
    float rgba_default_positional[4];
    float rgba_default_joint[4];

    mjtNum pos[3 * DECOR_BUF_SIZE];
    mjtNum size[3 * DECOR_BUF_SIZE];
    float rgba[4 * DECOR_BUF_SIZE];
};
typedef struct _decor_t_ decor_t;

struct _selection_t_
{
    int id_last_body_select;
    int id_last_non_node_select;
    int id_last_pert_activenum;

    int frame_offset;
    enum node_type_e node_type;
    enum scale_type_e scale_type;
    int nodecount;
    double nodesigma;
    double nodeheight;
    int jointnum;

    double localpos[3];
    double joint_move_ref[3];
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
    decor_t decor;
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
#include "decor.h"
#include "overlay.h"

void f_copy(float* to, float* from, int num);
uint64_t traj_time_in_micros();
void traj_foreach_frame(traj_info_t* traj_info);
void nodes_recolor(traj_info_t* traj_info);
int64_t traj_calculate_runtime_micros(traj_info_t* traj_info);

#endif
