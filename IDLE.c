// #include <math.h>
#include <kilombo.h>
// #include <stdbool.h>
// #include "shape.c"
#include "formation.h"


void idleState()
{
    
    if(kilo_ticks > WARMUP)
    {
        //obtain the index of the starting lattice of the current path
        mydata->path_point_index = 0;
        for(int i = 0; i < kilo_epoch; i++)
        {
            mydata->path_point_index += mydata->path_length[i];
        }

        //if the point is at the starting lattice of the current path
        if (mydata->path[mydata->path_point_index].q == mydata->hex_q && mydata->path[mydata->path_point_index].r == mydata->hex_r )
        {
            mydata->edge_followee_id = find_nearest_N_ID();
            set_bot_state(MOVE_OUT);
            set_move_type(EDGE);
            
            //start real-time localization
            mydata->localized = 0; 
            mydata->localize_cycle = 0;
            
            return;
        } 
    }

    return;


}