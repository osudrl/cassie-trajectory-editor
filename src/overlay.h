#ifndef OVERLAY_H
#define OVERLAY_H

#include "main.h"

void overlay_set_time_and_frame(traj_info_t* traj_info, int frame);
char* overlay_get_info_string();
void overlay_fill_selection_type_buf(traj_info_t* traj_info, char* buf);
void overlay_fill_info_status_buf(
    char* buf,
    traj_info_t* traj_info,
    char* camerastr,
    double fps);
void overlay_update_urr(traj_info_t* traj_info);

#endif


