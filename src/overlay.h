#ifndef OVERLAY_H
#define OVERLAY_H

#include "main.h"

char* overlay_get_info_string();
void overlay_fill_info_status_buf(
    char* buf,
    traj_info* traj_info,
    char* camerastr);


#endif


