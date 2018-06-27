#include "main.h"

double body_get_pos(mjData* data, int body_id, int cord_id)
{
    return data->xpos[body_id*3 + cord_id];
}

void body_pos_difference(mjData* data, double* xyz_arr, int body_id_end, int body_id_root)
{
    xyz_arr[0] = body_get_pos(data, body_id_end, 0) - body_get_pos(data, body_id_root, 0);
    xyz_arr[1] = body_get_pos(data, body_id_end, 1) - body_get_pos(data, body_id_root, 1);
    xyz_arr[2] = body_get_pos(data, body_id_end, 2) - body_get_pos(data, body_id_root, 2);
}

void vector3_subtraction(double* xyz_result, double* xyz_curr_end_to_root, double* xyz_target_end_to_root)
{
    xyz_result[0] = xyz_target_end_to_root[0] - xyz_curr_end_to_root[0];
    xyz_result[1] = xyz_target_end_to_root[1] - xyz_curr_end_to_root[1];
    xyz_result[2] = xyz_target_end_to_root[2] - xyz_curr_end_to_root[2];
}

double norm_of_vector3_subtraction(double* xyz_curr_end_to_root, double* xyz_target_end_to_root)
{
    double xyz_result[3];
    vector3_subtraction(xyz_result, xyz_curr_end_to_root, xyz_target_end_to_root);
    return mju_norm(xyz_result, 3);
}

double fwd_kinematics_observe(
    mjModel* m, 
    mjData* d, 
    double* xyz_target_end_to_root, 
    int body_id_end, 
    int body_id_root)
{
    double xyz_observed_dist[3];

    mj_kinematics(m,d);
    body_pos_difference(d, xyz_observed_dist, body_id_end, body_id_root);
    // observed_diff = norm_of_vector3_subtraction(xyz_observed_dist, xyz_target_end_to_root);
    double my[3] = {
        body_get_pos(d, body_id_end, 0),
        body_get_pos(d, body_id_end, 1),
        body_get_pos(d, body_id_end, 2),
    };

    // observed_diff = norm_of_vector3_subtraction(xyz_observed_dist, xyz_target_end_to_root);
    return norm_of_vector3_subtraction(my, xyz_target_end_to_root);
}

void fwd_kinematics_compare_result(
    mjModel* m, 
    mjData* d, 
    double* xyz_target_end_to_root, 
    int body_id_end, 
    int body_id_root,
    double* best_diff,
    int* best_qpos_index,
    int* positive_axis,
    int current_index,
    int is_positive)
{
    double observed_diff;

    observed_diff = fwd_kinematics_observe(m,d,xyz_target_end_to_root,body_id_end,body_id_root);

    if(observed_diff < *best_diff) 
    {
        *best_diff = observed_diff;
        *best_qpos_index = current_index;
        *positive_axis = is_positive;
    }
}

void better_body_optimizer(
    mjModel* m, 
    mjData* d, 
    double* xyz_target_end_to_root, 
    int body_id_end, 
    int body_id_root)
{
    int i;
    double best_diff;
    double pos_val_before_dx;
    int best_qpos_index = -1;
    int positive_axis = 0;
    double dx;

    best_diff = fwd_kinematics_observe(m,d,xyz_target_end_to_root,body_id_end,body_id_root);
    dx = 1.93 * 0.01 * best_diff + 0.0005;
    for(i = 7 ; i < CASSIE_QPOS_SIZE; i++)
    {
        pos_val_before_dx = d->qpos[i];

        d->qpos[i] = pos_val_before_dx + dx;

        fwd_kinematics_compare_result(
            m, 
            d, 
            xyz_target_end_to_root, 
            body_id_end, 
            body_id_root,
            &best_diff,
            &best_qpos_index,
            &positive_axis,
            i,
            1);

        d->qpos[i] = pos_val_before_dx - dx;

        fwd_kinematics_compare_result(
            m, 
            d, 
            xyz_target_end_to_root, 
            body_id_end, 
            body_id_root,
            &best_diff,
            &best_qpos_index,
            &positive_axis,
            i,
            0);

        d->qpos[i] = pos_val_before_dx;
    }

    if(!positive_axis)
        dx = -dx;
    if(best_qpos_index > 2)
        d->qpos[best_qpos_index] += dx;
    
}

int allow_pelvis_to_be_grabbed_and_moved(mjData* d, mjvPerturb* pert, double* xyz_ref)
{
    if(pert->active) 
    {
        if(pert->select == 1)
        {
            d->qpos[0] = pert->refpos[0];
            d->qpos[1] = pert->refpos[1];
            d->qpos[2] = pert->refpos[2];
            return 0;
        }
        else
        {
            xyz_ref[0] = pert->refpos[0];
            xyz_ref[1] = pert->refpos[1];
            xyz_ref[2] = pert->refpos[2];
            return 1;
        }
    }
    return 0;
}

void traj_foreach_frame(mjModel* m, mjData* d, mjvPerturb* pert)
{
    double xyz_target_end_to_root[3] = {0,0,0};
    int mod = 0;
    mod = allow_pelvis_to_be_grabbed_and_moved(d,pert,xyz_target_end_to_root);

    for(int z = 0; mod && z < 100; z++)
        better_body_optimizer(m,d,xyz_target_end_to_root,pert->select, pert->select);

    mj_forward(m, d);
}

