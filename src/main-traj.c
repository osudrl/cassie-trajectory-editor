#include "main.h"

int allow_pelvis_to_be_grabbed_and_moved(traj_info_t* traj_info, double* xyz_ref)
{
    if(traj_info->pert->active) 
    {
        // printf("selected: %d\n", traj_info->pert->select);
        if(traj_info->pert->select == 1)
        {
            traj_info->d->qpos[0] = traj_info->pert->refpos[0];
            traj_info->d->qpos[1] = traj_info->pert->refpos[1];
            traj_info->d->qpos[2] = traj_info->pert->refpos[2];
            return 0;
        }
        else
        {
            xyz_ref[0] = traj_info->pert->refpos[0];
            xyz_ref[1] = traj_info->pert->refpos[1];
            xyz_ref[2] = traj_info->pert->refpos[2];
            return 1;
        }
    }
    return 0;
}

uint64_t traj_time_in_micros()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return 1000000 * tv.tv_sec + tv.tv_usec;
}

uint64_t traj_calculate_runtime_micros(traj_info_t* traj_info)
{
    return traj_time_in_micros() - traj_info->start_time;
}

int traj_foreach_frame_lastmod = 0;

void traj_foreach_frame(traj_info_t* traj_info)
{
    // double xyz_xpos_target[3];
    // int mod;
    // mod = allow_pelvis_to_be_grabbed_and_moved(traj_info,xyz_xpos_target);
    // if(mod && mod != traj_foreach_frame_lastmod)
    // {
    //     traj_foreach_frame_lastmod = mod;
    //     traj_perform_initial_pert_caluclations(traj_info);
    // }
    // else if(!mod)
    //     traj_foreach_frame_lastmod = mod;

    // for(int z = 0; mod && z < 20; z++)
    // {
    //     better_body_optimizer(traj_info,
    //         xyz_xpos_target,
    //         traj_info->pert->select);
    // }



    // printf("%ld size div ints %.2f\n", sizeof(full_traj_state_t), 1318688.0/ sizeof(full_traj_state_t ));
    in_my_qposes(traj_info);
    mj_forward(traj_info->m, traj_info->d);
}

