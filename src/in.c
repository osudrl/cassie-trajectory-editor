#include "in.h"

FILE* in_c_infile = NULL;

void in_other_qposes(double* arr)
{
    char buf[4000000];
    char* strs[CASSIE_QPOS_SIZE];
    int strs_index = 0;
    char* annoy = 0;

    if(!in_c_infile)
    {
        in_c_infile = fopen("out-traj.csv","r");
        annoy = fgets(buf,4000000,in_c_infile);
    }
    annoy = fgets(buf,4000000,in_c_infile);

    if(!annoy)
        return;
    
    strs[strs_index] = strtok(buf,",");
    while(strs_index+1 < CASSIE_QPOS_SIZE && strs[strs_index] )
    {
        strs_index++;
        strs[strs_index] = strtok(NULL, ",");
    }

    strs_index = 1;
    while ( strs_index < CASSIE_QPOS_SIZE && strs[strs_index])
    {
        arr[strs_index-1] = strtod(strs[strs_index], NULL);
        // printf("strsindex %d %.5f \n",strs_index, arr[strs_index]);

        strs_index++;
    }
    // printf("strs_index %d \n", strs_index);
}

void in_my_qposes(traj_info_t* traj_info)
{
    in_other_qposes(traj_info->d->qpos);
}

