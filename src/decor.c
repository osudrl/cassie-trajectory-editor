#include "decor.h"

void decor_single_sphere(traj_info_t* traj_info, mjvScene* scn, int num)
{
    mjvGeom* sphere = scn->geoms + scn->ngeom++;
    
    mjv_initGeom(sphere,
        2,
        traj_info->decor.size + 3*num,
        traj_info->decor.pos + 3*num,
        NULL,
        traj_info->decor.color + 4*num);

}


void decor_showdecor(traj_info_t* traj_info, mjvScene* scn)
{
    int i;

    for (i = 0; i < traj_info->decor.count && i < DECOR_BUF_SIZE; i++)
    {
        decor_single_sphere(traj_info, scn, i);
    }
}






