#include "interpolate.h"

#define INT_POS_BUFFER_SIZE 2
#define FRAMES_PER 10

void interpolate_linear(
    double* result,
    double* q1,
    double* q2,
    double weight)
{
    int i;

    for (i = 0; i < CASSIE_QPOS_SIZE; i++)
        result[i] = q1[i] * (1-weight) + q2[i] * (weight);
}

double interpolate_c_qposes[CASSIE_QPOS_SIZE * INT_POS_BUFFER_SIZE];
int interpolate_c_valids[INT_POS_BUFFER_SIZE];
int interpolate_c_lastindex = -1;

long int interpolate_c_counter = 0;

double* interpolate_read_from_buf(int desiredindex)
{
    double* arrstart;
    arrstart = interpolate_c_qposes + (desiredindex * CASSIE_QPOS_SIZE);
    // printf("desiredindex %d result %p \n", desiredindex, arrstart);
    if(interpolate_c_valids[desiredindex] == 0)
    {
        in_other_qposes(arrstart);
        interpolate_c_valids[desiredindex] = 1;
    }
    return arrstart;
}

void interpolate_fill_qposes(double* result)
{
    int i;
    int index;
    double weight;

    if (interpolate_c_counter == 0)
    {
        for(i = 0; i < INT_POS_BUFFER_SIZE; i++)
            interpolate_c_valids[i] = 0;
    }
    weight = ((double) (interpolate_c_counter % FRAMES_PER)) / ((double) FRAMES_PER);
    index = (interpolate_c_counter / FRAMES_PER) % INT_POS_BUFFER_SIZE;
    interpolate_c_counter++;

    // printf("index %d weight %.2f\n",index,weight);

    if(index != interpolate_c_lastindex)
    {
        interpolate_c_valids[interpolate_c_lastindex] = 0;
        interpolate_c_lastindex = index;
    }

    interpolate_linear(result,
        interpolate_read_from_buf(index),
        interpolate_read_from_buf((index+1)%INT_POS_BUFFER_SIZE),
        weight);
}

