#include "out.h"

FILE* out_c_outfile = NULL;
int out_c_frame = 0;

void out_my_qposes(traj_info_t* traj_info)
{
    char buf[400000];
    int offset = 0;
    int i;

    if(!out_c_outfile)
    {
        out_c_outfile = fopen("out-traj.csv","w");
        offset += sprintf(buf+offset,"Frame");
        for (i = 0; i < CASSIE_QPOS_SIZE; i++)
            offset += sprintf(buf+offset,",qpos%i", i);
        fprintf(out_c_outfile,"%s\n",buf);
        offset = 0;
    }
    offset += sprintf(buf+offset,"%d",(out_c_frame++));
    for (i = 0; i < CASSIE_QPOS_SIZE; i++)
            offset += sprintf(buf+offset,",%.25f",traj_info->d->qpos[i]);

    fprintf(out_c_outfile,"%s\n",buf);
}

