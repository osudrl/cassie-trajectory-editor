#ifndef OVERLAY_H
#define OVERLAY_H

#include "main.h"

char* overlay_get_info_string();
char* overlay_get_selection_type_name(traj_info_t* traj_info);
void overlay_fill_selection_status_buf(char* buf, traj_info_t* traj_info);
char* overlay_get_selection_string();
void overlay_update_urr(traj_info_t* traj_info);
void overlay_set_time_and_frame(traj_info_t* traj_info, int frame);
void overlay_fill_info_status_buf(
    char* buf,
    traj_info_t* traj_info,
    char* camerastr,
    double fps);

#endif


