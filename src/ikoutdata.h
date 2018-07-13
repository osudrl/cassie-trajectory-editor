#ifndef IKOUTDATA
#define IKOUTDATA

#ifndef CASSIE_QPOS_SIZE
#define CASSIE_QPOS_SIZE 35
#endif

#include <inttypes.h>

struct _ikoutdata_t_
{
	int64_t frame;
	int64_t iter;
	double off_pelvis;
	double off_orientation;
	double off_rfoot;
	double off_lfoot;
	double best_rfoot_off;
	double curr_qposes[CASSIE_QPOS_SIZE];
};
typedef struct _ikoutdata_t_ ikoutdata_t;

#endif
