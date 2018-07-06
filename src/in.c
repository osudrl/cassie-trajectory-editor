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

uint32_t in_fill_full_traj_state_array(full_traj_state_t* buf, int bufcount)
{
    FILE* fd_in;
    uint32_t result;

    fd_in = fopen("step_in_place_trajectory", "r");
    result = fread(buf, sizeof(full_traj_state_t), bufcount, fd_in);
    printf("read %d bytes\n", result);

    fclose(fd_in);
    return result;
}

void in_copy_single_full_traj_state_to_qpos(qpos_t* qpos, full_traj_state_t* state)
{
    int i;

    printf("time = %.5f\n", state->time);
    for (i = 0; i < CASSIE_QPOS_SIZE; i++)
        qpos->q[i] = state->qpos[i];
}

void in_copy_all_full_traj_states_to_qposes(qpos_t* qpos, full_traj_state_t* state, int bufcount)
{
    int i;

    for(i = 0; i<bufcount; i++)
        in_copy_single_full_traj_state_to_qpos(qpos+i, state+i);
}

void in_init_timeline(traj_info_t* traj_info)
{
    full_traj_state_t buf[TIMELINE_SIZE];
    in_fill_full_traj_state_array(buf, TIMELINE_SIZE);
    in_copy_all_full_traj_states_to_qposes(traj_info->timeline.qposes, buf, TIMELINE_SIZE);
    traj_info->timeline.init = 1;
}

void in_set_mj_qpose(traj_info_t* traj_info, qpos_t* desired)
{
    int i;

    for(i = 0; i < CASSIE_QPOS_SIZE; i++)
    {
        traj_info->d->qpos[i] = desired->q[i];
        printf("qpos%d=%.3f ", i, desired->q[i]);
    }
    printf("\n");
}

void in_my_qposes(traj_info_t* traj_info)
{
    int frame;
    qpos_t* desired;

    if(!traj_info->timeline.init)
        in_init_timeline(traj_info);

    frame = mju_round( traj_calculate_runtime_micros(traj_info) / (1000 * 1000.0));
    desired = traj_info->timeline.qposes + (frame % TIMELINE_SIZE);
    in_set_mj_qpose(traj_info, desired);
}

