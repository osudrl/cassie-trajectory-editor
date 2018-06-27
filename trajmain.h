#include "mujoco.h"
#include "glfw3.h"
#include <stdio.h>

#define CORD_ID_X 0
#define CORD_ID_Y 1
#define CORD_ID_Z 2

#define CASSIE_QPOS_SIZE 34

double body_get_apos(mjData* data, int body_id, int cord_id);
double body_get_xpos(mjData* data, int body_id);
double body_get_ypos(mjData* data, int body_id);
double body_get_zpos(mjData* data, int body_id);
void body_pos_difference(mjData* data, double* xyz_arr, int body_id_end, int body_id_root);
double body_pos_difference_mag(mjData* data, int body_id_end, int body_id_root);
void random_body_optimizer(mjModel* m, mjData* d, double targetval, int body_id_end, int body_id_root);
void better_body_optimizer(
    mjModel* m, 
    mjData* d, 
    double* xyz_target_end_to_root, 
    int body_id_end, 
    int body_id_root);
void traj_foreach_frame(mjModel* m, mjData* d, mjvPerturb* pert);
