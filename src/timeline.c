#include "timeline.h"

uint32_t timeline_fill_full_traj_state_array(traj_info_t* traj_info, uint8_t** buf)
{
    FILE* fd_in;
    uint32_t result;
    uint32_t size;
    int readsofar = 0;

    fd_in = fopen(traj_info->filename_step_data, "r");

    if(!fd_in)
    {
        fprintf(stderr, "File %s cannot be opened!\n", traj_info->filename_step_data);
        exit(1);
    }
    
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

int timeline_make_frame_safe(int frame, int numframes)
{
    while(frame < 0)
        frame += numframes;
    frame %= numframes;
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

    loopcount = traj_info->visually_loop_count;

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

    traj_info->timeline->numnoloopframes = bytecount;
    traj_info->timeline->numframes = bytecount * loopcount;
    traj_info->timeline->next = NULL;
    traj_info->timeline->prev = NULL;
    traj_info->timeline->node_type = NODE_NONE;
}

void filename_replace_dots(char* filename)
{
    int stack[256];
    int stackptr = -1;
    char* strptr;

    strptr = filename;
    while(strptr && *strptr)
    {
        strptr = strchr(strptr, '.');
        if(strptr)
        {
            stack[++stackptr] = strptr - filename;
            strptr++;
        }
    }

    if(stackptr >= 0)
    {
        filename[stack[stackptr--]] = '\0';
    }
    while(stackptr >= 0)
    {
        filename[stack[stackptr--]] = '-';
    }
}

void timeline_export_to_file(traj_info_t* traj_info, full_traj_state_t* fulls, int numframes)
{
   char filename[256];
   char infilename[256];
   char timestring[256];
   time_t now;
   FILE* outfile;

   now = time(NULL);
   strncpy(infilename, traj_info->filename_step_data, 256);
   filename_replace_dots(infilename);
   strftime(timestring, 256, "%Y-%m-%d-%H-%M-%S.bin", localtime(&now));
   snprintf(filename, 256, "%s%s", infilename, timestring);
   outfile = fopen(filename, "w");

   if(!outfile )
   {
       fprintf(stderr, "File %s cannot be opened!\n", filename);
       exit(1);
   }

   // printf("bytes: %d, doubles: %f\n", sizeof(full_traj_state_t), (sizeof(full_traj_state_t)+0.0)/sizeof(double));
   fwrite(fulls, sizeof(full_traj_state_t), numframes, outfile);
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
    numDubsNeeded = timeline->numframes * sizeof(full_traj_state_t);
    numDubsNeeded /= sizeof(double);
    membuf = (full_traj_state_t*) 
        mj_stackAlloc(traj_info->d, mju_ceil(numDubsNeeded));

    for(i = 0; i < timeline->numframes; i++)
    {
        membuf[i].time = (timeline->duration / (timeline->numframes-1)) * i;
        mju_copy(membuf[i].qpos, timeline->qposes[i].q, 35);
        mju_zero(membuf[i].qvel, 32);
        mju_zero(membuf[i].torque, 10);
        mju_zero(membuf[i].mpos, 10);
        mju_zero(membuf[i].mvel, 10);
    }

    timeline_export_to_file(traj_info, membuf, timeline->numframes);

    traj_info->d->pstack = savestackptr; 
}


timeline_t* timeline_init_with_single_pose(qpos_t* qpos, timeline_t* xcopy)
{
    timeline_t* dest;
    int i;

    dest = malloc(sizeof(timeline_t));
    dest->qposes = malloc(sizeof(qpos_t) * xcopy->numframes);

    for(i = 0; i < xcopy->numframes; i++)
    {
        mju_copy(dest->qposes[i].q, qpos->q, CASSIE_QPOS_SIZE);
        mju_copy(dest->qposes[i].q, xcopy->qposes[i].q, 1);    
    }

    dest->next = NULL;
    dest->prev = NULL;
    dest->numframes = xcopy->numframes;
    dest->node_type = NODE_NONE;
    dest->numnoloopframes = xcopy->numnoloopframes;

    return dest;
}

timeline_t* timeline_loop(timeline_t* ref, int loopcount)
{
    timeline_t* dest;
    int qposbytecount;
    int i;
    int big;
    int start;

    qposbytecount = sizeof(qpos_t) * ref->numnoloopframes * loopcount;

    dest = malloc(sizeof(timeline_t));
    dest->qposes = malloc(qposbytecount);

    for(i = 0; i < ref->numnoloopframes * loopcount; i++)
    {
        mju_copy(dest->qposes[i].q, 
            ref->qposes[i % ref->numnoloopframes].q,
            CASSIE_QPOS_SIZE);
    }

    for(big = 1; big <= loopcount-1; big++)
    {
        start = ref->numnoloopframes * big;
        for(i = start; i < start + ref->numnoloopframes; i++)
        {
            mju_add(dest->qposes[i].q,
                dest->qposes[i].q,
                dest->qposes[start-1].q,
                1);
        }
    }

    dest->next = NULL;
    dest->prev = NULL;
    dest->numframes = ref->numframes * loopcount;
    dest->numnoloopframes = ref->numnoloopframes;
    dest->duration = ref->duration;
    dest->node_type = NODE_NONE;

    return dest;
}

timeline_t* timeline_noloop(timeline_t* ref)
{
    timeline_t* dest;
    int qposbytecount;

    qposbytecount = sizeof(qpos_t) * ref->numnoloopframes;

    dest = malloc(sizeof(timeline_t));
    dest->qposes = malloc(qposbytecount);

    memcpy(dest->qposes, ref->qposes, qposbytecount);
    dest->next = NULL;
    dest->prev = NULL;
    dest->numframes = ref->numnoloopframes;
    dest->numnoloopframes = ref->numnoloopframes;
    dest->duration = ref->duration;
    dest->node_type = NODE_NONE;

    return dest;
}

void timeline_collapse(timeline_t* ref)
{
    int i;

    for(i = 0; i < ref->numframes; i++)
    {
        ref->qposes[i].q[0] = 0;
    }
}

timeline_t* timeline_duplicate(timeline_t* ref)
{
    timeline_t* dest;
    int qposbytecount;

    qposbytecount = sizeof(qpos_t) * ref->numframes;

    dest = malloc(sizeof(timeline_t));
    dest->qposes = malloc(qposbytecount);

    memcpy(dest->qposes, ref->qposes, qposbytecount);
    dest->next = NULL;
    dest->prev = NULL;
    dest->numframes = ref->numframes;
    dest->numnoloopframes = ref->numnoloopframes;
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
    frame = timeline_make_frame_safe(frame, timeline->numframes);

    return timeline->qposes + frame ;
}

void timeline_set_mj_qpose(traj_info_t* traj_info, qpos_t* desired)
{
    mju_copy(traj_info->d->qpos, desired->q, CASSIE_QPOS_SIZE);
}


void timeline_set_qposes_to_pose_frame(traj_info_t* traj_info, timeline_t* timeline, int frame)
{   
    frame = timeline_make_frame_safe(frame, timeline->numframes);

    timeline_set_mj_qpose(traj_info, timeline->qposes + frame);
}

void timeline_overwrite_frame_using_curr_pose(traj_info_t* traj_info, timeline_t* timeline, int frame)
{
    qpos_t* qposes;
    
    qposes = timeline_get_qposes_from_frame(timeline, frame);

    mju_copy(qposes->q, traj_info->d->qpos, CASSIE_QPOS_SIZE);
}

int timeline_get_frame_from_time(traj_info_t* traj_info)
{
    return mju_round( traj_calculate_runtime_micros(traj_info) / traj_info->playback_time_scale);
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

