#include "main.h"

double body_get_pos(traj_info_t* traj_info, int body_id, int cord_id)
{
    return traj_info->d->xpos[body_id*3 + cord_id];
}

void body_pos_difference(traj_info_t* traj_info, double* xyz_arr, int body_id_end, int body_id_root)
{
    xyz_arr[0] = body_get_pos(traj_info, body_id_end, 0)
        - body_get_pos(traj_info, body_id_root, 0);
    xyz_arr[1] = body_get_pos(traj_info, body_id_end, 1)
        - body_get_pos(traj_info, body_id_root, 1);
    xyz_arr[2] = body_get_pos(traj_info, body_id_end, 2)
        - body_get_pos(traj_info, body_id_root, 2);
}

void vector3_subtraction(double* xyz_result, double* xyz_curr_end_to_root, double* xyz_target_end_to_root)
{
    xyz_result[0] = xyz_target_end_to_root[0] - xyz_curr_end_to_root[0];
    xyz_result[1] = xyz_target_end_to_root[1] - xyz_curr_end_to_root[1];
    xyz_result[2] = xyz_target_end_to_root[2] - xyz_curr_end_to_root[2];
}

double norm_of_vector3_subtraction(double* xyz1, double* xyz2)
{
    double xyz_result[3];
    vector3_subtraction(xyz_result, xyz1, xyz2);
    return mju_norm(xyz_result, 3);
}

double fwd_kinematics_score(
    traj_info_t* traj_info,
    double* xyz_xpos_target, 
    int body_id_end)
{
    double* xyz_xpos_curr_end;

    mj_kinematics(traj_info->m,traj_info->d);   
    xyz_xpos_curr_end = traj_info->d->xpos + body_id_end*3;
    return norm_of_vector3_subtraction(xyz_xpos_curr_end, xyz_xpos_target);
}

void better_body_optimizer(
    traj_info_t* traj_info,
    double* xyz_xpos_target, 
    int body_id_end)
{
    int i;
    double best_diff;
    double pos_val_before_dx;
    int best_qpos_index = -1;
    int positive_axis = 0;
    double dx;
    double observed_diff;

    best_diff = fwd_kinematics_score(traj_info,xyz_xpos_target,body_id_end);
    dx = 1.93 * 0.03 * best_diff + 0.0005;
    for(i = 7 ; i < CASSIE_QPOS_SIZE; i++)
    {
        pos_val_before_dx = traj_info->d->qpos[i];

        traj_info->d->qpos[i] = pos_val_before_dx + dx;

        observed_diff = fwd_kinematics_score(traj_info,xyz_xpos_target,body_id_end);
        if(observed_diff < best_diff) 
        {
            best_diff = observed_diff;
            best_qpos_index = i;
            positive_axis = 1;
        }

        traj_info->d->qpos[i] = pos_val_before_dx - dx;

        observed_diff = fwd_kinematics_score(traj_info,xyz_xpos_target,body_id_end);
        if(observed_diff < best_diff) 
        {
            best_diff = observed_diff;
            best_qpos_index = i;
            positive_axis = 0;
        }

        traj_info->d->qpos[i] = pos_val_before_dx;
    }

    if(!positive_axis)
        dx = -dx;
    if(best_qpos_index > 2)
        traj_info->d->qpos[best_qpos_index] += dx;
    
}

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

double xyz_plane_normal_vect[3] = {0,0,0};

void traj_perform_initial_pert_caluclations(traj_info_t* traj_info)
{
    double* xyz_xpos_tarsus;
    double* xyz_xpos_shin;
    double* xyz_xpos_mouse;
    double xyz_tarus_to_shin[3];
    double xyz_mouse_to_tarsus[3];

    if (traj_info->pert->select == 9 ||
        traj_info->pert->select == 21)  // is tarsus
    {
        xyz_xpos_tarsus = traj_info->d->xpos + traj_info->pert->select*3;
        xyz_xpos_shin = traj_info->d->xpos + (traj_info->pert->select-1)*3;
        xyz_xpos_mouse = traj_info->pert->refpos;
        vector3_subtraction(xyz_tarus_to_shin, xyz_xpos_tarsus, xyz_xpos_shin);
        vector3_subtraction(xyz_mouse_to_tarsus, xyz_xpos_mouse, xyz_xpos_tarsus);
        mju_cross(xyz_plane_normal_vect, xyz_tarus_to_shin, xyz_mouse_to_tarsus);
        mju_normalize(xyz_mouse_to_tarsus,3);
        mju_normalize(xyz_tarus_to_shin,3);
        // printf("dot: %.5f\n",mju_acos(mju_dot(xyz_tarus_to_shin, xyz_mouse_to_tarsus,3))*(180/3.1415));
    }
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
    // out_my_qposes(traj_info);
    // in_my_qposes(traj_info);
    
    mj_forward(traj_info->m, traj_info->d);
}

