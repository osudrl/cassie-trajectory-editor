#include "decor.h"

#define DEC traj_info->decor
#define AT3 (+ 3*)

void decor_single_sphere(traj_info_t* traj_info, mjvScene* scn, int num)
{
    mjvGeom* sphere = scn->geoms + scn->ngeom++;
    
    mjv_initGeom(sphere,
        2,
        DEC.size + 3*num,
        DEC.pos + 3*num,
        NULL,
        DEC.color + 4*num);

}


void decor_showdecor(traj_info_t* traj_info, mjvScene* scn)
{
    int i;

    for (i = 0; i < DEC.count && i < DECOR_BUF_SIZE; i++)
    {
        decor_single_sphere(traj_info, scn, i);
    }
}

void decor_reset(traj_info_t* traj_info)
{
    DEC.count = -1;
}

bool decor_has_init(traj_info_t* traj_info)
{
    return DEC.count >= 0;
}

void decor_positional_rootframe(traj_info_t* traj_info, v3_t pos)
{
    DEC.count = 0;
    (DEC.pos AT3 DEC.count)[0] = 0;
}

void decor_positional_addto(traj_info_t* traj_info, v3_t pos)
{

}





