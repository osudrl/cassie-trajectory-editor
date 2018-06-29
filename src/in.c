#include "in.h"

FILE* outfile = NULL;
int frame = 0;

void in_my_qposes(traj_info_t* traj_info)
{
    char buf[2048];
    int offset = 0;
    int i;

    if(!outfile)
    {
        outfile = fopen("out-traj.csv","w");
        offset += sprintf(buf+offset,"Frame");
        for (i = 0; i < CASSIE_QPOS_SIZE; i++)
            offset += sprintf(buf+offset,",qpos%i", i);
        fprintf(outfile,"%s\n",buf);
        offset = 0;
    }
    offset += sprintf(buf+offset,"%d",(frame++));
    for (i = 0; i < CASSIE_QPOS_SIZE; i++)
            offset += sprintf(buf+offset,",%.10f",traj_info->d->qpos[i]);

    fprintf(outfile,"%s\n",buf);
}

