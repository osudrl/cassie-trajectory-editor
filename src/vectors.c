#include "vectors.h"


double vectors_body_get_pos(traj_info_t* traj_info, int body_id, int cord_id)
{
    return traj_info->d->xpos[body_id*3 + cord_id];
}

void vectors_body_pos_difference(traj_info_t* traj_info, 
    double* xyz_arr, 
    int body_id_end, 
    int body_id_root)
{
    xyz_arr[0] = vectors_body_get_pos(traj_info, body_id_end, 0)
        - vectors_body_get_pos(traj_info, body_id_root, 0);
    xyz_arr[1] = vectors_body_get_pos(traj_info, body_id_end, 1)
        - vectors_body_get_pos(traj_info, body_id_root, 1);
    xyz_arr[2] = vectors_body_get_pos(traj_info, body_id_end, 2)
        - vectors_body_get_pos(traj_info, body_id_root, 2);
}

void vectors_3subtraction(double* xyz_result, 
    double* xyz_curr_end_to_root, 
    double* xyz_target_end_to_root)
{
    xyz_result[0] = xyz_target_end_to_root[0] - xyz_curr_end_to_root[0];
    xyz_result[1] = xyz_target_end_to_root[1] - xyz_curr_end_to_root[1];
    xyz_result[2] = xyz_target_end_to_root[2] - xyz_curr_end_to_root[2];
}

double vectors_norm_of_vector3_subtraction(double* xyz1, double* xyz2)
{
    double xyz_result[3];
    vectors_3subtraction(xyz_result, xyz1, xyz2);
    return mju_norm(xyz_result, 3);
}
