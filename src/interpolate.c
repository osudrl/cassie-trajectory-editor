#include "interpolate.h"

#define INT_POS_BUFFER_SIZE 2
#define FRAMES_PER 10

int birth = 0;
int inpipe[2];
int outpipe[2];
pid_t pid;

void birth_child()
{
    int result;

    pid = fork();

    if (pipe(inpipe) != 0) 
        perror("Failed pipe");
    if (pipe(outpipe) != 0) 
        perror("Failed pipe");

    if (pid == 0)
    {
        // printf("CHILD\n");
        if(dup2(inpipe[0], 0)<0)
            perror("issue with dup2");
        if(dup2(outpipe[1], 1) < 0)
            perror("issue with dup2");
        close(inpipe[0]);
        close(outpipe[1]);
        close(inpipe[0]);
        close(outpipe[1]);
        execlp("cat", "cat", "-", NULL);
        perror("exec failed");
        exit(-1);
    }

    printf("%d\n",inpipe[1]);
    // close(inpipe[0]);
    close(outpipe[1]);
    birth = 1;
}



/*
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
*/

/*
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
*/

void interpolate_pipe_request_pose(double timer)
{
    char buf[128];
    int result;

    result = sprintf(buf, "%.5f\n", timer);
    buf[result] = '\0';
    printf("inpipe[1] = %d\n", inpipe[1]);

    result = write(inpipe[1], buf, result);
    printf("write result = %d\n", result);
}

void interpolate_pipe_parese_pose()
{
    char buf[400000];
    char* schrresult = NULL;
    int offset = 0;
    int status;

    printf("childstate %d \n ", waitpid(pid, &status, WNOHANG));


    while(!schrresult)
    {
        offset += read(outpipe[0], buf+offset, 128);
        buf[offset] = '\0';
        schrresult = strchr("buf", '\n');
    }

    printf("%s\n",buf);
}

void interpolate_with_pipe(traj_info_t* traj_info)
{
    if (!birth)
        birth_child();

    interpolate_pipe_request_pose(traj_info->timer);
    printf("woo %.5f\n", traj_info->timer);
    interpolate_pipe_parese_pose();
}

void interpolate_fill_qposes(traj_info_t* traj_info)
{
    interpolate_with_pipe(traj_info);
}
