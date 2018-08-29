//---------------------------------//
//  This file is part of MuJoCo    //
//  Written by Emo Todorov         //
//  Copyright (C) 2017 Roboti LLC  //
//---------------------------------//


#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"

#include "main.h"
#include "control.h"
 

//-------------------------------- global variables -------------------------------------

// model
mjModel* m = 0;
mjData* d = 0;
char lastfile[1000] = "";


// user state
bool paused = false;
bool showoption = false;
bool showinfo = true;
bool showfullscreen = false;
bool slowmotion = false;
bool showsensor = false;
bool showprofiler = true;
int showhelp = 1;                   // 0: none; 1: brief; 2: full
int fontscale = mjFONTSCALE_150;    // can be 100, 150, 200
int keyreset = -1;                  // non-negative: reset to keyframe

// abstract visualization
mjvScene scn;
mjvCamera cam;
mjvOption vopt;
mjvPerturb pert;
mjvFigure figconstraint;
mjvFigure figcost;
mjvFigure figtimer;
mjvFigure figsize;
mjvFigure figsensor;
char info_status[1000] = "";
char selection_status[1000] = "";

// OpenGL rendering
int refreshrate;
mjrContext con;
float depth_buffer[5120*2880];        // big enough for 5K screen
unsigned char depth_rgb[1280*720*3];  // 1/4th of screen

// selection and perturbation
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;
double window2buffer = 1;           // framebuffersize / windowsize (for scaled video modes)

// help strings
const char help_title[] = 
"More Help\n"
"Option\n"
"Info\n"
"Full screen\n"
"Select\n"
"Center\n"
"Track\n"
"Zoom\n"
"Translate\n"
"Rotate\n"
"Perturb\n"
"Free Camera\n"
"Orientations\n"
"Label";

const char help_content[] = 
"F1\n"
"F2\n"
"F3\n"
"F5\n"
"L dblclick\n"
"R dblclick\n"
"Ctrl R dblclick\n"
"Scroll or M drag\n"
"[Shift] R drag\n"
"L drag\n"
"Ctrl [Shift] L/R drag\n"
"Esc\n"
"; '\n"
". /";

const char help2_title[] = 
"Fwd/Bk\n"
"Fwd/Bk 20\n"
"Advance Nodes\n"
"Advance Nodes x20\n"
"Playback Speed\n"
"Decrease Nodes\n"
"Increase Nodes\n"
"Autoscale\n"
"Reload\n"
"Load Last Perturbation\n"
"Expand Current Pose\n"
"Save To File"
;

const char help2_content[] = 
"R/L arrows\n"
"U/D arrows\n"
"Ctrl L/R\n"
"Ctrl U/D\n"
"Ctrl PgUp/Dn\n"
"Minus (-)\n"
"Equal (=)\n"
"Ctrl A\n"
"Ctrl L\n"
"Ctrl P\n"
"Ctrl E\n"
"Ctrl S";

char opt_title[1000] = "";
char opt_content[1000];

traj_info_t traj_info;
int firsttrajinforeset = 0;

void reset_traj_info()
{
    uint8_t joint_cycle_list[] = {
        0,1,2,
        3,4,5,6,
        7,8,9,
        14,15,16,
        20,
        21,22,23,
        28,29,30,
        34
    };

    traj_info.m = m;
    traj_info.d = d;
    traj_info.pert = &pert;
    // traj_info.timeline.init = 0;
    
    traj_info.selection.nodecount = 50;


    traj_info.time_start = traj_time_in_micros();
    traj_info.paused = &paused;
    traj_info.ik.m = m;
    traj_info.ik.d = d;
    traj_info.ik.doik = 0;
    traj_info.filename_step_data = FILENAME_STEP_DATA;
    traj_info.playback_time_scale = 10000;
    traj_info.selection.nodesigma = 100;
    traj_info.selection.nodeheight = 1;
    traj_info.selection.node_type = NODE_POSITIONAL;
    traj_info.selection.joint_cycle_list_size = sizeof(joint_cycle_list);
    traj_info.selection.joint_cycle_list_index = sizeof(joint_cycle_list)-1;
    traj_info.selection.joint_cycle_list = malloc(sizeof(joint_cycle_list));
    traj_info.selection.loop_enabled = 0;

    memcpy(traj_info.selection.joint_cycle_list,joint_cycle_list,sizeof(joint_cycle_list));

    traj_info.decor.count = 0;
    
    traj_info.decor.rgba_default_positional[0] = .2;
    traj_info.decor.rgba_default_positional[1] = .6;
    traj_info.decor.rgba_default_positional[2] = .2;
    traj_info.decor.rgba_default_positional[3] = 1;
    traj_info.decor.size_default_positional[0] = .015;
    traj_info.decor.size_default_positional[1] = .015;
    traj_info.decor.size_default_positional[2] = .015;
    traj_info.decor.rgba_default_joint[0] = .1;
    traj_info.decor.rgba_default_joint[1] = .1;
    traj_info.decor.rgba_default_joint[2] = .8;
    traj_info.decor.rgba_default_joint[3] = 1;
    traj_info.decor.size_default_joint[0] = .010;
    traj_info.decor.size_default_joint[1] = .010;
    traj_info.decor.size_default_joint[2] = .010;

    traj_info.selection.frame_offset = 0;

    if (firsttrajinforeset > 0 && traj_info.target_list)
        free(traj_info.target_list);

    traj_info.timeline = NULL;

    traj_info.target_list = NULL;
    traj_info.target_list_size = -1;

    showinfo = 1;

    for( int i=0; i<mjNRNDFLAG; i++ )
        if ( strcmp(mjRNDSTRING[i][0], "Shadow") == 0)
        {
            scn.flags[i] = 0;
        }


    firsttrajinforeset++;
}


//-------------------------------- profiler and sensor ----------------------------------

// init profiler
void profilerinit(void)
{
   
}

// show profiler
void profilerupdate(void)
{
    
} 

// show profiler
void profilershow(mjrRect rect)
{

} 

// init sensor figure
void sensorinit(void)
{
    
}

// update sensor figure
void sensorupdate(void)
{
   
}

// show sensor figure
void sensorshow(mjrRect rect)
{

}

//-------------------------------- utility functions ------------------------------------

// center and scale view
void autoscale(GLFWwindow* window)
{
    // autoscale
    cam.lookat[0] = m->stat.center[0];
    cam.lookat[1] = m->stat.center[1];
    cam.lookat[2] = m->stat.center[2];
    cam.distance = 1.5 * m->stat.extent;

    // set to free camera
    cam.type = mjCAMERA_FREE;
}


// load mjb or xml model
void loadmodel(GLFWwindow* window, const char* filename)
{
    // make sure filename is given
    if( !filename  )
        return;

    // load and compile
    char error[1000] = "could not load binary model";
    mjModel* mnew = 0;                 
    if( strlen(filename)>4 && !strcmp(filename+strlen(filename)-4, ".mjb") )
        mnew = mj_loadModel(filename, 0);
    else
        mnew = mj_loadXML(filename, 0, error, 1000);
    if( !mnew )
    {
        printf("%s\n", error);
        return;
    }

    // delete old model, assign new
    mj_deleteData(d);
    mj_deleteModel(m);
    m = mnew;
    d = mj_makeData(m);
        
    m->opt.disableflags |= 0xddc;

    m->opt.disableflags &= ~(mjDSBL_LIMIT); // comment if segfaults
    // m->opt.disableflags &= ~(mjDSBL_CONTACT); //  comment if segfaults


    mj_forward(m, d);

    // save filename for reload
    strcpy(lastfile, filename);

    // re-create custom context
    mjr_makeContext(m, &con, fontscale);

    // clear perturbation state and keyreset
    pert.active = 0;
    pert.select = 0;
    keyreset = -1;

    // center and scale view, update scene
    autoscale(window);
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    // set window title to mode name
    if( window && m->names )
        glfwSetWindowTitle(window, m->names);

    reset_traj_info();
}


// timer in milliseconds
mjtNum timer(void)
{
    // save start time
    static double starttm = 0;
    if( starttm==0 )
        starttm = glfwGetTime();

    // return time since start
    return (mjtNum)(1000 * (glfwGetTime() - starttm));
}


// clear all times
void cleartimers(mjData* d)
{
    for( int i=0; i<mjNTIMER; i++ )
    {
        d->timer[i].duration = 0;
        d->timer[i].number = 0;
    }
}

bool option_key_whitelisted(int key)
{
    return
        key == GLFW_KEY_S ||
        key == GLFW_KEY_W ||
        key == GLFW_KEY_G ||
        key == GLFW_KEY_J ||
        key == GLFW_KEY_T ||
        key == GLFW_KEY_E ||
        key == GLFW_KEY_D;
}

//--------------------------------- GLFW callbacks --------------------------------------

// keyboard
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // int n;

    // require model
    if( !m )
        return;

    // do not act on release
    if( act==GLFW_RELEASE )
        return;

    control_key_event(&traj_info, key, mods);

    switch( key )
    {
    case GLFW_KEY_F1:                   // help
        showhelp++;
        if( showhelp>3 )
            showhelp = 0;
        break;

    case GLFW_KEY_F2:                   // option
        showoption = !showoption;
        break;

    case GLFW_KEY_F3:                   // info
        showinfo = !showinfo;
        break;

    case GLFW_KEY_F5:                   // toggle full screen
        showfullscreen = !showfullscreen;
        if( showfullscreen )
            glfwMaximizeWindow(window);
        else
            glfwRestoreWindow(window);
        break;

    // case GLFW_KEY_F6:                   // stereo
    //     scn.stereo = (scn.stereo==mjSTEREO_NONE ? mjSTEREO_QUADBUFFERED : mjSTEREO_NONE);
    //     break;

    case GLFW_KEY_SPACE:                // pause
        paused = !paused;

        break;

    case GLFW_KEY_ESCAPE:               // free camera
        cam.type = mjCAMERA_FREE;
        break;

    case '[':                           // previous fixed camera or free
        if( m->ncam && cam.type==mjCAMERA_FIXED )
        {
            if( cam.fixedcamid>0 )
                cam.fixedcamid--;
            else
                cam.type = mjCAMERA_FREE;
        }
        break;

    case ']':                           // next fixed camera
        if( m->ncam )
        {
            if( cam.type!=mjCAMERA_FIXED )
            {
                cam.type = mjCAMERA_FIXED;
                cam.fixedcamid = 0;
            }
            else if( cam.fixedcamid<m->ncam-1 )
                cam.fixedcamid++;
        }
        break;

    case ';':                           // cycle over frame rendering modes
        vopt.frame = mjMAX(0, vopt.frame-1);
        break;

    case '\'':                          // cycle over frame rendering modes
        vopt.frame = mjMIN(mjNFRAME-1, vopt.frame+1);
        break;

    case '.':                           // cycle over label rendering modes
        vopt.label = mjMAX(0, vopt.label-1);
        break;

    case '/':                           // cycle over label rendering modes
        vopt.label = mjMIN(mjNLABEL-1, vopt.label+1);
        break;

    default:                            // toggle flag
        // control keys
        if( mods & GLFW_MOD_CONTROL )
        {
            if( key==GLFW_KEY_A )
                autoscale(window);
            if( key==GLFW_KEY_L && lastfile[0] )
                loadmodel(window, lastfile);          

            break;
        }

        if(option_key_whitelisted(key))
        {
            // toggle visualization flag
            for( int i=0; i<mjNVISFLAG; i++ )
                if( key==mjVISSTRING[i][2][0] )
                    vopt.flags[i] = !vopt.flags[i];

            // toggle rendering flag
            for( int i=0; i<mjNRNDFLAG; i++ )
                if( key==mjRNDSTRING[i][2][0] )
                    scn.flags[i] = !scn.flags[i];
        }
    }
}


// mouse button
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // past data for double-click detection
    static int lastbutton = 0;
    static double lastclicktm = 0;

    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // Alt: swap left and right
    if( (mods & GLFW_MOD_ALT) )
    {
        bool tmp = button_left;
        button_left = button_right;
        button_right = tmp;

        if( button==GLFW_MOUSE_BUTTON_LEFT )
            button = GLFW_MOUSE_BUTTON_RIGHT;
        else if( button==GLFW_MOUSE_BUTTON_RIGHT )
            button = GLFW_MOUSE_BUTTON_LEFT;
    }

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);

    // require model
    if( !m )
        return;

    // set perturbation
    int newperturb = 0;
    if( act==GLFW_PRESS && (mods & GLFW_MOD_CONTROL) && pert.select>0 )
    {
        // right: translate;  left: rotate
        if( button_right )
        {
            newperturb = mjPERT_TRANSLATE;
            traj_info.selection.scale_type = SCALING_A;
            traj_info.selection.nodeheight = 1;

        }
        else if (button_left)
        {
            newperturb = mjPERT_TRANSLATE;
            traj_info.selection.scale_type = SCALING_B;
            traj_info.selection.nodeheight = 1;
        }
        // perturbation onset: reset reference
        if( newperturb && !pert.active )
            mjv_initPerturb(m, d, &scn, &pert);
    }
    pert.active = newperturb;

    // detect double-click (250 msec)
    if( act==GLFW_PRESS && glfwGetTime()-lastclicktm<0.25 && button==lastbutton )
    {
        // determine selection mode
        int selmode;
        if( button==GLFW_MOUSE_BUTTON_LEFT )
            selmode = 1;
        else if( mods & GLFW_MOD_CONTROL )
            selmode = 3;
        else
            selmode = 2;

        // get current window size
        int width, height;
        glfwGetWindowSize(window, &width, &height);

        // find geom and 3D click point, get corresponding body
        mjtNum selpnt[3];
        int selgeom = mjv_select(m, d, &vopt,
                                 (mjtNum)width/(mjtNum)height, 
                                 (mjtNum)lastx/(mjtNum)width, 
                                 (mjtNum)(height-lasty)/(mjtNum)height, 
                                 &scn, selpnt);
        int selbody = (selgeom>=0 ? m->geom_bodyid[selgeom] : 0);

        // set lookat point, start tracking is requested
        if( selmode==2 || selmode==3 )
        {
            // copy selpnt if geom clicked
            if( selgeom>=0 )
                mju_copy3(cam.lookat, selpnt);

            // switch to tracking camera
            if( selmode==3 && selbody )
            {
                cam.type = mjCAMERA_TRACKING;
                cam.trackbodyid = selbody;
                cam.fixedcamid = -1;
            }
        }

        // set body selection
        else
        {
            if( selbody )
            {
                // record selection
                pert.select = selbody;

                // compute localpos
                mjtNum tmp[3];
                mju_sub3(tmp, selpnt, d->xpos+3*pert.select);
                mju_mulMatTVec(pert.localpos, d->xmat+9*pert.select, tmp, 3, 3);
            }
            else
                pert.select = 0;
        }

        // stop perturbation on select
        pert.active = 0;
    }

    // save info
    if( act==GLFW_PRESS )
    {
        lastbutton = button;
        lastclicktm = glfwGetTime();
    }
}


// mouse move
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // require model
    if( !m )
        return;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    bool mod_ctrl = (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL)==GLFW_PRESS);


    if(mod_ctrl && pert.active && button_left)
    {
        button_left = 0;
        button_right = 1;
    }


    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move perturb or camera
    if( pert.active )
        mjv_movePerturb(m, d, action, dx/height, dy/height, &scn, &pert);
    else
        mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // require model
    if( !m )
        return;

    // scroll: emulate vertical mouse motion = 5% of window height

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);
    bool mod_ctrl = (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL)==GLFW_PRESS);

    if (mod_ctrl && pert.active)
    {
        double mult = yoffset/25.0;
        mult += 1;
        if(!mod_shift)
            traj_info.selection.nodesigma *= mult;
        else
        {
            traj_info.selection.nodeheight *= mult;
            traj_info.selection.nodeheight = mju_max(traj_info.selection.nodeheight, 1);
        }


    }
    else if (mod_ctrl)
    {
        traj_info.time_frozen += yoffset*200000;
        traj_info.time_start -= yoffset*200000;
    }
    else
        mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


// drop
void drop(GLFWwindow* window, int count, const char** paths)
{
    // make sure list is non-empty
    if( count>0 )
        loadmodel(window, paths[0]);
}


//-------------------------------- simulation and rendering -----------------------------

// make option string
void makeoptionstring(const char* name, char key, char* buf)
{
    int i=0, cnt=0;

    // copy non-& characters
    while( name[i] && i<50 )
    {
        if( name[i]!='&' )
            buf[cnt++] = name[i];

        i++;
    }

    // finish
    buf[cnt] = ' ';
    buf[cnt+1] = '(';
    buf[cnt+2] = key;
    buf[cnt+3] = ')';
    buf[cnt+4] = 0;
}

// advance simulation
void simulation(void)
{
    // no model
    if( !m )
        return;

    // clear timers
    cleartimers(d);

    // paused
    if( paused )
    {
        // apply pose perturbations, run mj_forward
        if( pert.active )
        {
            mjv_applyPerturbPose(m, d, &pert, 1);      // move mocap and dynamic bodies
            mj_forward(m, d);
        }
    }
    else
    {
    }
        traj_foreach_frame(&traj_info);
}

// render
void render(GLFWwindow* window)
{
    // past data for FPS calculation
    static double lastrendertm = 0;

    // get current framebuffer rectangle
    mjrRect rect = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &rect.width, &rect.height);
    mjrRect smallrect = rect;

    // reduce rectangle when profiler is on
    if( showprofiler )
        smallrect.width = rect.width - rect.width/5;

    // no model: empty screen
    if( !m )
    {
        mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, "Drag-and-drop model file here", 0, &con);

        // swap buffers
        glfwSwapBuffers(window); 
        return;
    }

    // advance simulation
    simulation();

    // update simulation statistics
    char camstr[20];
    if( cam.type==mjCAMERA_FREE )
        strcpy(camstr, "Free");
    else if( cam.type==mjCAMERA_TRACKING )
        strcpy(camstr, "Tracking");
    else
        sprintf(camstr, "Fixed %d", cam.fixedcamid);

    overlay_fill_info_status_buf(info_status, &traj_info, camstr, 1.0/(glfwGetTime()-lastrendertm));
    overlay_fill_selection_status_buf(selection_status, &traj_info);

    // FPS timing satistics
    lastrendertm = glfwGetTime();

    // update scene
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    decor_showdecor(&traj_info, &scn);
    // render
    mjr_render(rect, &scn, &con);

    // show overlays
    if( showhelp==1 )
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, smallrect, "Help  ", "F1  ", &con);
    else if( showhelp==2 )
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, smallrect, help2_title, help2_content, &con);
    else if( showhelp==3 )
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, smallrect, help_title, help_content, &con);

    // show info
    if(showinfo )
    {
        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, smallrect, 
            overlay_get_info_string(), info_status, &con);
        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMRIGHT, rect, 
            overlay_get_selection_string(),
            selection_status, &con);
    }

    // show options
    if( showoption )
    {
        int i;
        char buf[100];

        // fill titles on first pass
        if( !opt_title[0] )
        {
            for( i=0; i<mjNRNDFLAG; i++)
            {
                if(option_key_whitelisted(mjRNDSTRING[i][2][0]))
                {
                    makeoptionstring(mjRNDSTRING[i][0], mjRNDSTRING[i][2][0], buf);
                    strcat(opt_title, buf);
                    strcat(opt_title, "\n");
                }
               
            }
            for( i=0; i<mjNVISFLAG; i++)
            {
                if(option_key_whitelisted(mjVISSTRING[i][2][0]))
                {
                    makeoptionstring(mjVISSTRING[i][0], mjVISSTRING[i][2][0], buf);
                    strcat(opt_title, buf);
                    if( i<mjNVISFLAG-1 )
                        strcat(opt_title, "\n");
                }
            }
        }

        // fill content
        opt_content[0] = 0;
        for( i=0; i<mjNRNDFLAG; i++)
        {
            if(option_key_whitelisted(mjRNDSTRING[i][2][0]))
            {
                strcat(opt_content, scn.flags[i] ? " + " : "   ");
                strcat(opt_content, "\n");
            }            
        }
        for( i=0; i<mjNVISFLAG; i++)
        {
            if(option_key_whitelisted(mjVISSTRING[i][2][0]))
            {
                strcat(opt_content, vopt.flags[i] ? " + " : "   ");
                if( i<mjNVISFLAG-1 )
                    strcat(opt_content, "\n");
            }
        }

        // show
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, smallrect, opt_title, opt_content, &con);
    }

    // show profiler
    if( showprofiler )
    {
        if( !paused )
            profilerupdate();
        profilershow(rect);
    }

    // show sensor
    if( showsensor )
    {
        if( !paused )
            sensorupdate();
        sensorshow(smallrect);
    }

    // swap buffers
    glfwSwapBuffers(window);
}


//-------------------------------- main function ----------------------------------------

void control(const mjModel* m, mjData* d)
{
    pdik_per_step_control(&traj_info.ik);    
}

int main(int argc, const char** argv)
{
    // print version, check compatibility
    printf("MuJoCo Pro library version %.2lf\n", 0.01*mj_version());
    if( mjVERSION_HEADER!=mj_version() )
        mju_error("Headers and library have different versions");

    // activate MuJoCo license
    mj_activate("mjkey.txt");
    
    // init GLFW
    if (!glfwInit())
        return 1;

    // get refreshrate
    refreshrate = glfwGetVideoMode(glfwGetPrimaryMonitor())->refreshRate;

    // multisampling
    glfwWindowHint(GLFW_SAMPLES, 4);

    // create widdow
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Simulate", NULL, NULL);
    if( !window )
    {
        glfwTerminate();
        return 1;
    }

    // make context current, disable v-sync
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // save window-to-framebuffer pixel scaling (needed for OSX scaling)
    int width, width1, height;
    glfwGetWindowSize(window, &width, &height);
    glfwGetFramebufferSize(window, &width1, &height);
    window2buffer = (double)width1 / (double)width;

    // init MuJoCo rendering, get OpenGL info
    mjv_makeScene(&scn, 1000);
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&vopt);
    mjr_defaultContext(&con);
    mjr_makeContext(m, &con, fontscale);
    profilerinit();
    sensorinit();

    // set GLFW callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
    glfwSetDropCallback(window, drop);
    glfwSetWindowRefreshCallback(window, render);

    // set MuJoCo time callback for profiling
    mjcb_time = timer;
    mjcb_control = control;


    // load model if filename given as argument
    if( argc==2 )
        loadmodel(window, argv[1]);
    else
        loadmodel(window, "model/cassie.xml");

    // main loop
    while( !glfwWindowShouldClose(window) )
    {
        // simulate and render
        render(window);

        // handle events (this calls all callbacks)
        glfwPollEvents();
    }

    // delete everything we allocated
    mj_deleteData(d);
    mj_deleteModel(m);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);

    // terminate
    glfwTerminate();
    mj_deactivate();
    return 0;
}


