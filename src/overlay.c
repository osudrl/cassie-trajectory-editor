
#include "overlay.h"

#define OV traj_info->info_overlay
#define SEL traj_info->selection
#define GETJOINTNUM traj_info->selection.joint_cycle_list[traj_info->selection.joint_cycle_list_index]


char* overlay_get_info_string()
{
    return "Paused (Space)\n\
Time (Ctrl+Scroll)\n\
Frame\n\
FPS\n\
N StdDev (F1)\n\
N Height (F1)\n\
Loop (L)\n\
Contact (C)\n\
Undo (Ctrl+Z)\n\
Redo (Ctrl+Y)\n\
Refine (Ctrl+R)\n\
Camera ([ and ])";
}

char* overlay_get_cassie_joint_string(int joint)
{
    if(joint == 0)
        return "pelvis-x";
    else if(joint == 1)
        return "pelvis-y";
    else if(joint == 2)
        return "pelvis-z";
    else if(joint == 3)
        return "pelvis (quatw)";
    else if(joint == 4)
        return "pelvis (quatx)";
    else if(joint == 5)
        return "pelvis (quaty)";
    else if(joint == 6)
        return "pelvis (quatz)";
    else if(joint == 7)
        return "left-hip-roll ";
    else if(joint == 8)
        return "left-hip-yaw ";
    else if(joint == 9)
        return "left-hip-pitch ";
    else if(joint == 10)
        return "left-achilles-rod (quatw)";
    else if(joint == 11)
        return "left-achilles-rod (quatw)";
    else if(joint == 12)
        return "left-achilles-rod (quatw)";
    else if(joint == 13)
        return "left-achilles-rod (quatw)";
    else if(joint == 14)
        return "left-knee ";
    else if(joint == 15)
        return "left-shin ";
    else if(joint == 16)
        return "left-tarsus ";
    else if(joint == 17)
        return "left-heel-spring ";
    else if(joint == 18)
        return "left-foot-crank ";
    else if(joint == 19)
        return "left-plantar-rod ";
    else if(joint == 20)
        return "left-foot ";
    else if(joint == 21)
        return "right-hip-roll ";
    else if(joint == 22)
        return "right-hip-yaw ";
    else if(joint == 23)
        return "right-hip-pitch ";
    else if(joint == 24)
        return "right-achilles-rod (quatw)";
    else if(joint == 25)
        return "right-achilles-rod (quatx)";
    else if(joint == 26)
        return "right-achilles-rod (quaty)";
    else if(joint == 27)
        return "right-achilles-rod (quatz)";
    else if(joint == 28)
        return "right-knee ";
    else if(joint == 29)
        return "right-shin ";
    else if(joint == 30)
        return "right-tarsus ";
    else if(joint == 31)
        return "right-heel-spring ";
    else if(joint == 32)
        return "right-foot-crank ";
    else if(joint == 33)
        return "right-plantar-rod ";
    else if(joint == 34)
        return "right-foot ";
    else
        return "JOINT-UNKOWN";
}

char* overlay_get_cassie_body_string(int body)
{
    if(body == 1)
        return "cassie-pelvis";
    else if(body == 2)
        return "left-hip-roll";
    else if(body == 3)
        return "left-hip-yaw";
    else if(body == 4)
        return "left-hip-pitch";
    else if(body == 5)
        return "left-achilles-rod";
    else if(body == 6)
        return "left-knee";
    else if(body == 7)
        return "left-knee-spring";
    else if(body == 8)
        return "left-shin";
    else if(body == 9)
        return "left-tarsus";
    else if(body == 10)
        return "left-heel-spring";
    else if(body == 11)
        return "left-foot-crank";
    else if(body == 12)
        return "left-plantar-rod";
    else if(body == 13)
        return "left-foot";
    else if(body == 14)
        return "right-hip-roll";
    else if(body == 15)
        return "right-hip-yaw";
    else if(body == 16)
        return "right-hip-pitch";
    else if(body == 17)
        return "right-achilles-rod";
    else if(body == 18)
        return "right-knee";
    else if(body == 19)
        return "right-knee-spring";
    else if(body == 20)
        return "right-shin";
    else if(body == 21)
        return "right-tarsus";
    else if(body == 22)
        return "right-heel-spring";
    else if(body == 23)
        return "right-foot-crank";
    else if(body == 24)
        return "right-plantar-rod";
    else if(body == 25)
        return "right-foot";
    else
        return "BODY-UNKOWN";
}

char* overlay_get_selection_type_name(traj_info_t* traj_info)
{
    if(SEL.node_type == NODE_POSITIONAL)
        return "Positional\nDrag and drop\nnodes to perturb\nand solve IK";
    else if(SEL.node_type == NODE_JOINTID)
        return "Joint Identification\nUse PgUp/Dn to\nmake a joint\nselection";
    else if(SEL.node_type == NODE_JOINTMOVE)
        return "Single-Joint Tuning\nDrag nodes up/down\nto modify the\nseleced joint values";
    else
        return NULL;
}

void overlay_fill_selection_status_buf(char* buf, traj_info_t* traj_info)
{
    int offset = 0;

    offset += sprintf(buf + offset, "%s\n", overlay_get_selection_type_name(traj_info));
    offset += sprintf(buf + offset, "%d\n", SEL.id_last_non_node_select);
    offset += sprintf(buf + offset, "%s\n", overlay_get_cassie_body_string(SEL.id_last_non_node_select));
    offset += sprintf(buf + offset, "%d\n", GETJOINTNUM);
    offset += sprintf(buf + offset, "%s", overlay_get_cassie_joint_string(GETJOINTNUM));
}

char* overlay_get_selection_string()
{
   return 
"Tool (Enter)\n\n\n\n\
BodId (LMouse)\n\
BodName\n\
JId (PgUpDn)\n\
Joint Name";
}

void overlay_update_urr(traj_info_t* traj_info)
{
    OV.canundo = traj_info->timeline->prev != NULL;
    OV.canredo = traj_info->timeline->next != NULL;
    OV.canrefine = !OV.canredo && OV.canundo && traj_info->timeline->node_type == NODE_POSITIONAL;
}

void overlay_set_time_and_frame(traj_info_t* traj_info, int frame)
{
    float result;

    frame = timeline_make_frame_safe(frame, traj_info->timeline->numframes);

    result = frame;
    result /= traj_info->timeline->numframes;
    result *= traj_info->timeline->duration;

    OV.sec = result;
    OV.frame = frame;
}

void overlay_fill_info_status_buf(
    char* buf,
    traj_info_t* traj_info,
    char* camerastr,
    double fps)
{
    int offset = 0;

    overlay_update_urr(traj_info);

    offset += sprintf(buf + offset, "%s\n", !(*(traj_info->paused)) ? "FALSE" : "TRUE");
    offset += sprintf(buf + offset, "%.3f sec\n", OV.sec);
    offset += sprintf(buf + offset, "%d\n", OV.frame);
    offset += sprintf(buf + offset, "%.1f\n", fps);
    offset += sprintf(buf + offset, "%.2f\n", SEL.nodesigma);
    offset += sprintf(buf + offset, "%.2f\n", SEL.nodeheight);
    offset += sprintf(buf + offset, "%s\n", SEL.loop_enabled ? "Enabled" : "Disabled");
    offset += sprintf(buf + offset, "%s\n", !(traj_info->m->opt.disableflags & (mjDSBL_CONTACT)) ? "Enabled" : "Disabled");
    offset += sprintf(buf + offset, "%s\n", OV.canundo == 0 ? "not ready" : "READY");
    offset += sprintf(buf + offset, "%s\n", OV.canredo == 0 ? "not ready" : "READY");
    offset += sprintf(buf + offset, "%s\n", OV.canrefine == 0 ? "not ready" : "READY");
    offset += sprintf(buf + offset, "%s", camerastr);
}
