#include "main.h"

void allow_node_transformations(traj_info_t* traj_info)
{
    if (traj_info->pert->select != traj_info->selection.id_last_body_select &&  //made a new selection
            traj_info->pert->select > 0 && //body is on cassie not a node
            traj_info->pert->select <= 25)
    {
        node_position_initial_using_cassie_body(traj_info, 
        	node_get_cassie_id_from_index(traj_info->pert->select));
        traj_info->selection.id_last_non_node_select = traj_info->pert->select;
        traj_info->selection.id_last_body_select = traj_info->pert->select;    
    }

    if(traj_info->pert->active) 
    {
        traj_info->selection.id_last_body_select = traj_info->pert->select;
        traj_info->selection.id_last_pert_activenum = traj_info->pert->active;

        if( traj_info->pert->select > 25)
        {
            v3_t dqpos = node_get_qpos_by_node_id(traj_info, 
                node_get_body_id_from_real_body_id(traj_info->pert->select));
            mju_copy3(dqpos,traj_info->pert->refpos);
            node_position_scale_visually(traj_info, 
                node_get_cassie_id_from_index(traj_info->selection.id_last_non_node_select), 
                node_get_body_id_from_real_body_id(traj_info->pert->select)); 
            
        }
    }
    else if (traj_info->selection.id_last_pert_activenum == 1 && traj_info->selection.id_last_body_select > 25)
    {
        node_dropped(traj_info, 
            node_get_cassie_id_from_index(traj_info->selection.id_last_non_node_select), 
            node_get_body_id_from_real_body_id(traj_info->selection.id_last_body_select));

        traj_info->selection.id_last_body_select = traj_info->pert->select;
        traj_info->selection.id_last_pert_activenum = traj_info->pert->active;
    }
}

uint64_t traj_time_in_micros()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return 1000000 * tv.tv_sec + tv.tv_usec;
}

int64_t traj_calculate_runtime_micros(traj_info_t* traj_info)
{
    int64_t val;
    if (!(*traj_info->paused))
    {
        val = traj_time_in_micros() - traj_info->time_start;
        traj_info->time_frozen = val;
        return val;
    }
    else
    {
        traj_info->time_start = traj_time_in_micros() - traj_info->time_frozen;
        return traj_info->time_frozen;
    }
}

typedef struct {
    double r;       // a fraction between 0 and 1
    double g;       // a fraction between 0 and 1
    double b;       // a fraction between 0 and 1
} rgb;

typedef struct {
    double h;       // angle in degrees
    double s;       // a fraction between 0 and 1
    double v;       // a fraction between 0 and 1
} hsv;

static hsv   rgb2hsv(rgb in);
static rgb   hsv2rgb(hsv in);

hsv rgb2hsv(rgb in)
{
    hsv         out;
    double      min, max, delta;

    min = in.r < in.g ? in.r : in.g;
    min = min  < in.b ? min  : in.b;

    max = in.r > in.g ? in.r : in.g;
    max = max  > in.b ? max  : in.b;

    out.v = max;                                // v
    delta = max - min;
    if (delta < 0.00001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0              
        // s = 0, h is undefined
        out.s = 0.0;
        out.h = NAN;                            // its now undefined
        return out;
    }
    if( in.r >= max )                           // > is bogus, just keeps compilor happy
        out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
    else
    if( in.g >= max )
        out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
    else
        out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

    out.h *= 60.0;                              // degrees

    if( out.h < 0.0 )
        out.h += 360.0;

    return out;
}


rgb hsv2rgb(hsv in)
{
    double      hh, p, q, t, ff;
    long        i;
    rgb         out;

    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;
        return out;
    }
    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }
    return out;     
}


void nodes_recolor(traj_info_t* traj_info)
{
    int i;

    for(i = 35; i < traj_info->m->ngeom && traj_info->selection.node_type == NODES_POSITIONAL; i++)
    {
        traj_info->m->geom_rgba[i*4 + 0] = .2;
        traj_info->m->geom_rgba[i*4 + 1] = .6;
        traj_info->m->geom_rgba[i*4 + 2] = .2;
        traj_info->m->geom_size[i*3 + 0] = .015;
        traj_info->m->geom_size[i*3 + 1] = .015;
        traj_info->m->geom_size[i*3 + 2] = .015;
    }

    for(i = 35; i < traj_info->m->ngeom && traj_info->selection.node_type != NODES_POSITIONAL; i++)
    {
        traj_info->m->geom_rgba[i*4 + 0] = .1;
        traj_info->m->geom_rgba[i*4 + 1] = .1;
        traj_info->m->geom_rgba[i*4 + 2] = .8;
        traj_info->m->geom_size[i*3 + 0] = .010;
        traj_info->m->geom_size[i*3 + 1] = .010;
        traj_info->m->geom_size[i*3 + 2] = .010;
    }
}

void traj_foreach_frame(traj_info_t* traj_info)
{
    nodes_recolor(traj_info);
    
    allow_node_transformations(traj_info);
    
    timeline_update_mj_poses_from_realtime(traj_info);

    mj_forward(traj_info->m, traj_info->d);
}

