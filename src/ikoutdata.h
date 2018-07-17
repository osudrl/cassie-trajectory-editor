#ifndef IKOUTDATA
#define IKOUTDATA

#ifndef CASSIE_QPOS_SIZE
#define CASSIE_QPOS_SIZE 35
#endif

#include <inttypes.h>


struct _iklibe_t_
{
    double norm_pelvis_to_foot;
    double v_pelvis_to_foot[3];
    double curr_qposes[CASSIE_QPOS_SIZE];
};
typedef struct _iklibe_t_ iklibe_t;

struct _ikoutdata_t_
{
    int64_t frame; //0
    int64_t iter; //1
    double off_pelvis; //2
    double off_orientation; //3
    double off_rfoot; //4
    double off_lfoot; //5
    double best_rfoot_off; //6
    double curr_qposes[CASSIE_QPOS_SIZE];
};
typedef struct _ikoutdata_t_ ikoutdata_t;

#endif
