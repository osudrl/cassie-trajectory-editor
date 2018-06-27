#include "main.h"

int countup = 0;

int ksimcounter = 0;
int oldksimcounter = -1;
int kflip = 0;
double oldpos;

void traj_foreach_frame(mjModel* m, mjData* d, mjvPerturb* pert)
{
    countup++;
    countup %= 100;
    float thing = countup; // 0 to 1000
    thing -= 50; // -500 to 500
    thing = thing > 0 ? thing : -thing;

    if( !kflip && thing < 1)
    {
        ksimcounter++;
        ksimcounter %= CASSIE_QPOS_SIZE;
        kflip = 1;
    } 
    else if (!(thing < 1))
    {
        kflip = 0;
    }

    if(ksimcounter != oldksimcounter)
    {
        printf("moving qpos index: %d\n",ksimcounter);
        double temp = oldpos;
        oldpos = d->qpos[ksimcounter];
        d->qpos[oldksimcounter] = temp;
        oldksimcounter = ksimcounter;
    }

    d->qpos[ksimcounter] = thing/10;   
    
    mj_forward(m, d);
}

