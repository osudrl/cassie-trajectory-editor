
#include "overlay.h"

#define OV traj_info->info_overlay
#define SEL traj_info->selection

char* overlay_get_info_string()
{
    return "Paused\n\
Time\n\
Frame\n\
FPS\n\
N StdDev\n\
N Height\n\
Undo\n\
Redo\n\
Refine\n\
Camera";
}

void overlay_set_time_and_frame(traj_info_t* traj_info, int frame)
{
    float result;

    frame = timeline_make_frame_safe(frame, traj_info->timeline->numposes);

    result = frame;
    result /= traj_info->timeline->numposes;
    result *= traj_info->timeline->duration;

    OV.sec = result;
    OV.frame = frame;
}

void overlay_fill_info_status_buf(
    char* buf,
    traj_info_t* traj_info,
    char* camerastr,
    double fps)
{
    int offset = 0;

    offset += sprintf(buf + offset, "%s\n", !(*(traj_info->paused)) ? "FALSE" : "TRUE");
    offset += sprintf(buf + offset, "%.3f seconds\n", OV.sec);
    offset += sprintf(buf + offset, "%d\n", OV.frame);
    offset += sprintf(buf + offset, "%.1f\n", fps);
    offset += sprintf(buf + offset, "%.2f\n", SEL.nodesigma);
    offset += sprintf(buf + offset, "%.2f\n", SEL.nodeheight);
    offset += sprintf(buf + offset, "%s\n", OV.canundo == 0 ? "no" : "READY");
    offset += sprintf(buf + offset, "%s\n", OV.canredo == 0 ? "no" : "READY");
    offset += sprintf(buf + offset, "%s\n", OV.canrefine == 0 ? "no" : "READY");
    offset += sprintf(buf + offset, "%s", camerastr);
}
