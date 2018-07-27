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
    dx = 1.93 * 0.01 * best_diff + 0.0005;
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
    double* t;
    double* f;
    double* fm;
    double tf[3];
    // double fm[3];

    if (traj_info->pert->select == 13 ||
        traj_info->pert->select == 25)  // is foot
    {
        f = traj_info->d->xpos + traj_info->pert->select*3;
        t = traj_info->d->xpos + (traj_info->pert->select-4)*3;
        fm = traj_info->pert->localpos;
        vector3_subtraction(tf, f, t);
        // mju_add(fm, f, m, 3 );
        double woo = mju_norm(tf,3);
        mju_normalize(fm,3);
        printf("\t\tdot: %.5f\n",mju_acos(mju_dot(tf, fm,3))*(180/(3.1415*woo)));
    }
}

int traj_foreach_frame_lastmod = 0;

void traj_foreach_frame(traj_info_t* traj_info)
{
    double xyz_xpos_target[3];
    int mod;
    mod = allow_pelvis_to_be_grabbed_and_moved(traj_info,xyz_xpos_target);
    if(mod && mod != traj_foreach_frame_lastmod)
    {
        traj_foreach_frame_lastmod = mod;
        traj_perform_initial_pert_caluclations(traj_info);
    }
    else if(!mod)
        traj_foreach_frame_lastmod = mod;

    for(int z = 0; mod && z < 100; z++)
        better_body_optimizer(traj_info,
            xyz_xpos_target,
            traj_info->pert->select);

    traj_info->d->qpos[20]+=0.001;
    // printf("foot %.5f\n", traj_info-a>d->qpos[20] );

    mj_forward(traj_info->m, traj_info->d);
}

