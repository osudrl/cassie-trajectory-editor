
#include "overlay.h"

char* overlay_get_info_string()
{
    return "Paused\n\
Time\n\
Frame\n\
N StdDev\n\
N Height\n\
Undo\n\
Redo\n\
Refine\n\
Camera";
}

void overlay_fill_info_status_buf(
    char* buf,
    traj_info* traj_info,
    char* camerastr)
{
    
}
