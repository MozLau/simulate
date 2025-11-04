#include <kilombo.h>
#include "formation.h"
#include "shape.h"



void moveOutState()
{
    int ending_point_index = 0;
    for(int i = 0; i <= kilo_epoch; i++)
    {
        ending_point_index += mydata->path_length[i];
    }

    //move to the target point
    if(omni_move_to_lattice(&(mydata->path[mydata->path_point_index]))==1)
    {
        //move to the next point
        if(mydata->path_point_index < ending_point_index - 1)
            mydata->path_point_index ++;
        else
        {
            //state to IDLE if reached the endpoint
            omni_stop();
            set_bot_state(IDLE);
            set_move_type(STOP);
            set_bot_type(NOR);

            //update epoch
            kilo_epoch ++;

            //Localizated
            mydata->localized = 1; 
        }
            
    } 

    return;
}