#ifndef IN_H
#define IN_H

#include "mujoco.h"
#include "glfw3.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

struct __attribute__ ((packed)) _full_traj_state_t_
{
    double time;
    double qpos[35];
    double qvel[32];
    double torque[10];
    double mpos[10];
    double mvel[10];
};
typedef struct _full_traj_state_t_ full_traj_state_t;

void timeline_free(timeline_t* ref);
timeline_t* timeline_deep_copy(timeline_t* ref);
void timeline_set_qposes_to_pose_frame(traj_info_t* traj_info, timeline_t* timeline, int frame);
void timeline_overwrite_frame_using_curr_pose(traj_info_t* traj_info, timeline_t* timeline, int frame);
void timeline_update_mj_poses_from_realtime(traj_info_t* traj_info);

#endif

