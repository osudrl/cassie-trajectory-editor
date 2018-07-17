#include <stdio.h>
#include "ikoutdata.h"


int main()
{   
    iklibe_t lib;
    FILE* infile = fopen("iksolvedata.bin", "r");
    int i = 0;

    while(fread(&lib, sizeof(iklibe_t), 1, infile) > 0)
    {
        i++;
        printf("%.5f",lib.norm_pelvis_to_foot);
        printf(",%.5f",lib.v_pelvis_to_foot[0]);
        printf(",%.5f",lib.v_pelvis_to_foot[1]);
        printf(",%.5f",lib.v_pelvis_to_foot[2]);
        for(int i = 0; i < CASSIE_QPOS_SIZE; i++)
        {
            //printf(",%.5f",lib.curr_qposes[i]);
        }
        printf("\n");
    }
    printf("%d\n",i);
}


