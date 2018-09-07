#ifndef IN_H
#define IN_H

#include "mujoco.h"
#include "glfw3.h"
#include "main.h"
#include <stdio.h>
#include <time.h>
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

#include "timeline.h"

void timeline_collapse(timeline_t* ref);
void timeiline_init_from_input_file(traj_info_t* traj_info);
timeline_t* timeline_duplicate(timeline_t* ref);
void timeline_export_to_file(traj_info_t* traj_info, full_traj_state_t* fulls, int numframes);
void timeline_export(traj_info_t* traj_info, timeline_t* timeline);
uint32_t timeline_fill_full_traj_state_array(traj_info_t* traj_info, uint8_t** buf);
void timeline_free(timeline_t* ref);
int timeline_get_frame_from_time(traj_info_t* traj_info);
qpos_t* timeline_get_qposes_from_frame(timeline_t* timeline, int frame);
timeline_t* timeline_init_with_single_pose(qpos_t* qpos, timeline_t* xcopy);
timeline_t* timeline_loop(timeline_t* ref, int loopcount);
int timeline_make_frame_safe(int frame, int numframes);
timeline_t* timeline_noloop(timeline_t* ref);
void timeline_overwrite_frame_using_curr_pose(traj_info_t* traj_info, timeline_t* timeline, int frame);
void timeline_safe_link(timeline_t* next, timeline_t* prev);
void timeline_set_mj_qpose(traj_info_t* traj_info, qpos_t* desired);
void timeline_set_qposes_to_pose_frame(traj_info_t* traj_info, timeline_t* timeline, int frame);
void timeline_update_mj_poses_from_realtime(traj_info_t* traj_info);


#endif

