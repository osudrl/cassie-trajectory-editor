#include <stdio.h>
#include "ikoutdata.h"


int main()
{
    ikoutdata_t od;
    FILE* infile = fopen("iksolvedata.bin", "r");

    while(fread(&od, sizeof(ikoutdata_t), 1, infile) > 0)
    {
        printf("%ld,%ld",od.frame, od.iter);
        printf(",%.5f",od.off_pelvis);
        printf(",%.5f",od.off_orientation);
        printf(",%.5f",od.off_rfoot);
        printf(",%.5f",od.off_lfoot);
        printf(",%.5f",od.best_rfoot_off);
        for(int i = 0; i < CASSIE_QPOS_SIZE; i++)
        {
            printf(",%.5f",od.curr_qposes[i]);
        }
        printf("\n");
    }
}


