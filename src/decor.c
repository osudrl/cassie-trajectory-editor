#include "decor.h"

#define DEC traj_info->decor

void decor_single_sphere(traj_info_t* traj_info, mjvScene* scn, int num)
{
    mjvGeom* sphere = scn->geoms + scn->ngeom++;
    
    mjv_initGeom(sphere,
        2,
        DEC.size + 3*num,
        DEC.pos + 3*num,
        NULL,
        DEC.rgba + 4*num);

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

void decor_positional_rootframe(traj_info_t* traj_info, double* pos)
{
    DEC.count = 0;
    DEC.pos[DEC.count * 3 + 0] = pos[0];
    DEC.pos[DEC.count * 3 + 1] = pos[1];
    DEC.pos[DEC.count * 3 + 2] = pos[2];
    mju_copy(DEC.size + DEC.count * 3, DEC.size_default_positional, 3);
    f_copy(DEC.rgba + DEC.count * 4, DEC.rgba_default_positional, 4);
    DEC.rgba[DEC.count * 4 + 3] = 0.25;
    DEC.count++;
}

void decor_positional_addto(traj_info_t* traj_info, double* pos)
{
    DEC.pos[DEC.count * 3 + 0] = pos[0];
    DEC.pos[DEC.count * 3 + 1] = pos[1];
    DEC.pos[DEC.count * 3 + 2] = pos[2];
    mju_copy(DEC.size + DEC.count * 3, DEC.size_default_positional, 3);
    f_copy(DEC.rgba + DEC.count * 4, DEC.rgba_default_positional, 4);
    DEC.rgba[DEC.count * 4 + 3] = 0.25;
    DEC.count++;
}





