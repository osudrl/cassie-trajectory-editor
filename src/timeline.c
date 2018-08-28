#include "timeline.h"

uint32_t timeline_fill_full_traj_state_array(traj_info_t* traj_info, uint8_t** buf)
{
    FILE* fd_in;
    uint32_t result;
    uint32_t size;
    int readsofar = 0;

    fd_in = fopen(traj_info->filename_step_data, "r");
    
    size = 131072;
    *buf = malloc(size);

    while (1)
    {
        result = fread(*buf + readsofar, 1, 131072, fd_in);
        readsofar += result;
        if(result < 131072)
        {
            break;
        }
        else
        {
            size += 131072;
            *buf = realloc(*buf , size);
        }
    }

    fclose(fd_in);
    return readsofar;
}

int timeline_make_frame_safe(int frame, int numposes)
{
    while(frame < 0)
        frame += numposes;
    frame %= numposes;
    return frame;
}

void timeiline_init_from_input_file(traj_info_t* traj_info)
{
    int i;
    int bytecount;
    full_traj_state_t* fulls;
    int big;
    int start;
    int loopcount;

    loopcount = 1;

    bytecount = timeline_fill_full_traj_state_array(traj_info, (uint8_t**) &fulls);
    bytecount /= sizeof(full_traj_state_t);

    // printf("Read %d poses \n", bytecount);

    if(!traj_info->timeline)
        traj_info->timeline = malloc(sizeof(timeline_t));

    traj_info->timeline->qposes = malloc(sizeof(qpos_t) * bytecount * loopcount);

    for(i = 0; i < bytecount * loopcount; i++)
    {
        mju_copy(traj_info->timeline->qposes[i].q,
            fulls[i % bytecount].qpos,
            CASSIE_QPOS_SIZE);
        // traj_info->timeline->qposes[i].q[0] = 0;
    }

    for(big = 1; big <= loopcount-1; big++)
    {
        start = bytecount * big;
        for(i = start; i < start + bytecount; i++)
        {
            mju_add(traj_info->timeline->qposes[i].q,
                traj_info->timeline->qposes[i].q,
                traj_info->timeline->qposes[start-1].q,
                1);
        }
    }

    traj_info->timeline->duration = fulls[bytecount-1].time * loopcount;

    free(fulls);

    traj_info->timeline->numposes = bytecount * loopcount;
    traj_info->timeline->next = NULL;
    traj_info->timeline->prev = NULL;
    traj_info->timeline->node_type = NODE_NONE;
}

void timeline_export_to_file(full_traj_state_t* fulls, int numposes)
{
   char filename[64];
   time_t now;
   FILE* outfile;

   now = time(NULL);
   strftime(filename, 64, "stepdata-%Y-%m-%d--%H-%M-%S.bin", localtime(&now));
   outfile = fopen(filename, "w");

   // printf("bytes: %d, doubles: %f\n", sizeof(full_traj_state_t), (sizeof(full_traj_state_t)+0.0)/sizeof(double));
   fwrite(fulls, sizeof(full_traj_state_t), numposes, outfile);
   fflush(outfile);
   fclose(outfile);
}

void timeline_export(traj_info_t* traj_info, timeline_t* timeline)
{
    int savestackptr;
    double numDubsNeeded;
    int i;
    full_traj_state_t* membuf;

    savestackptr = traj_info->d->pstack;
    numDubsNeeded = timeline->numposes * sizeof(full_traj_state_t);
    numDubsNeeded /= sizeof(double);
    membuf = (full_traj_state_t*) 
        mj_stackAlloc(traj_info->d, mju_ceil(numDubsNeeded));

    for(i = 0; i < timeline->numposes; i++)
    {
        membuf[i].time = (timeline->duration / (timeline->numposes-1)) * i;
        mju_copy(membuf[i].qpos, timeline->qposes[i].q, 35);
        mju_zero(membuf[i].qvel, 32);
        mju_zero(membuf[i].torque, 10);
        mju_zero(membuf[i].mpos, 10);
        mju_zero(membuf[i].mvel, 10);
    }

    timeline_export_to_file(membuf, timeline->numposes);

    traj_info->d->pstack = savestackptr; 
}


timeline_t* timeline_init_with_single_pose(qpos_t* qpos, timeline_t* xcopy)
{
    timeline_t* dest;
    int i;

    dest = malloc(sizeof(timeline_t));
    dest->qposes = malloc(sizeof(qpos_t) * xcopy->numposes);

    for(i = 0; i < xcopy->numposes; i++)
    {
        mju_copy(dest->qposes[i].q, qpos->q, CASSIE_QPOS_SIZE);
        mju_copy(dest->qposes[i].q, xcopy->qposes[i].q, 1);    
    }

    dest->next = NULL;
    dest->prev = NULL;
    dest->numposes = xcopy->numposes;
    dest->node_type = NODE_NONE;


    return dest;
}

timeline_t* timeline_duplicate(timeline_t* ref)
{
    timeline_t* dest;
    int qposbytecount;

    qposbytecount = sizeof(qpos_t) * ref->numposes;

    dest = malloc(sizeof(timeline_t));
    dest->qposes = malloc(qposbytecount);

    memcpy(dest->qposes, ref->qposes, qposbytecount);
    dest->next = NULL;
    dest->prev = NULL;
    dest->numposes = ref->numposes;
    dest->duration = ref->duration;
    dest->node_type = NODE_NONE;


    return dest;
}

void timeline_safe_link(timeline_t* next, timeline_t* prev)
{
    if(prev->next)
        timeline_free(prev->next);

    prev->next = next;
    next->prev = prev;
}

void timeline_free(timeline_t* ref)
{
    free(ref->qposes);
    free(ref);
}

qpos_t* timeline_get_qposes_from_frame(timeline_t* timeline, int frame)
{
    frame = timeline_make_frame_safe(frame, timeline->numposes);

    return timeline->qposes + frame ;
}

void timeline_set_mj_qpose(traj_info_t* traj_info, qpos_t* desired)
{
    mju_copy(traj_info->d->qpos, desired->q, CASSIE_QPOS_SIZE);
}

void panic()
{
    fprintf(stderr, "PANIC!\n");
    exit(1);
}

void timeline_set_qposes_to_pose_frame(traj_info_t* traj_info, timeline_t* timeline, int frame)
{   
    if(!timeline)
        panic();

    frame = timeline_make_frame_safe(frame, timeline->numposes);

    timeline_set_mj_qpose(traj_info, timeline->qposes + frame);
}

void timeline_overwrite_frame_using_curr_pose(traj_info_t* traj_info, timeline_t* timeline, int frame)
{
    qpos_t* qposes;

    if(!timeline)
       panic();
    
    qposes = timeline_get_qposes_from_frame(timeline, frame);

    mju_copy(qposes->q, traj_info->d->qpos, CASSIE_QPOS_SIZE);
}

int timeline_get_frame_from_time(traj_info_t* traj_info)
{
    return mju_round( traj_calculate_runtime_micros(traj_info) / (1000 * 10));
}

void timeline_update_mj_poses_from_realtime(traj_info_t* traj_info)
{
    int frame;

    if(!traj_info->timeline)
        timeiline_init_from_input_file(traj_info);

    frame = timeline_get_frame_from_time(traj_info);
    overlay_set_time_and_frame(traj_info, frame);
    timeline_set_qposes_to_pose_frame(traj_info, traj_info->timeline, frame);
}

