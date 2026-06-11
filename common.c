#include <math.h>

#include <kilombo.h>
#include "formation.h"
#include "liu_path.h"

#define COLINEAR_THRESHOLD 0.01

/* -------- swarm ----------*/





//////////////////////localization////////////////////////
int check_localizablitiy()
{
    int localized_bot_count = 0;

    for (int i = 0; i < mydata->N_Neighbors; i++)
    {
        if (mydata->neighbors[i].localized == 1)
        {
            localized_bot_count ++;
        }
    }

    if(localized_bot_count >= 3){

        //check if all points on the same line
        double dx1 = mydata->neighbors[1].x - mydata->neighbors[0].x;
        double dy1 = mydata->neighbors[1].y - mydata->neighbors[0].y;

        for (int i = 2; i < mydata->N_Neighbors; i++)
        {
            if (mydata->neighbors[i].localized != 1) continue;

            double dx2 = mydata->neighbors[i].x - mydata->neighbors[0].x;
            double dy2 = mydata->neighbors[i].y - mydata->neighbors[0].y;

            //if slope not the same
            if (fabs(dy1 * dx2 - dy2 * dx1) > COLINEAR_THRESHOLD * kilo_lattice_size * kilo_lattice_size * 0.25)
            {
                return 1;
            }
        }


        return 0;
    }
    else{

        return 0;
    }
}

double random_perturbation() {
    return ((rand() / (double)RAND_MAX) * 2 - 1) * kilo_lattice_size; // Random value in [-max_magnitude, max_magnitude]
}

void global_localization()
{
    //learning rate
    double alpha = 0.1;

    for(int i = 0; i < NUM_OF_LOC_ROUNDS; i++)
    {


        double gradX = 0.0, gradY = 0.0;

        for (int j = 0; j < mydata->N_Neighbors; j++) 
        {
            //skip moving robots
            if(mydata->neighbors[j].n_bot_move != STOP) continue;
            if (mydata->neighbors[j].localized != 1) continue;

            double dx = mydata->x - mydata->neighbors[j].x;
            double dy = mydata->y - mydata->neighbors[j].y;
            double distance = sqrt(dx * dx + dy * dy);
            double targetDist = get_dist_by_ID(mydata->neighbors[j].ID);

            double error = distance - targetDist;

            gradX += 2 * error * (dx / distance);
            gradY += 2 * error * (dy / distance);

            // if ((rand() / (double)RAND_MAX) < 0.05) {
            //     gradX += random_perturbation();
            //     gradY += random_perturbation();
            // }

        }

        // Update positions
        mydata->x -= alpha * gradX;
        mydata->y -= alpha * gradY;

    }



    mydata->localized = 1;

}

void single_localization()
{
    for(int j = 0; j < NUM_OF_LOC_ROUNDS; j++)
    {
        for (int i = 0; i < mydata->N_Neighbors; i++)
        {
            double c, vx, vy, nx, ny;
            if (mydata->neighbors[i].localized == 1)
            {
                c = sqrt(pow(mydata->x - mydata->neighbors[i].x,2) + pow(mydata->y - mydata->neighbors[i].y,2));
                vx = (mydata->x - mydata->neighbors[i].x)/c;
                vy = (mydata->y - mydata->neighbors[i].y)/c;
                nx = mydata->neighbors[i].x + get_dist_by_ID(mydata->neighbors[i].ID) *vx;
                ny = mydata->neighbors[i].y + get_dist_by_ID(mydata->neighbors[i].ID) *vy;
                mydata->x = 3.0/4 * mydata->x + 1.0/4 * nx;
                mydata->y = 3.0/4 * mydata->y + 1.0/4 * ny;
                
            }
        }                         
    }
}

void trilateration() {


    double x1 = mydata->neighbors[0].x;
    double y1 = mydata->neighbors[0].y;
    double d1 = mydata->neighbors[0].precise_dist;
    double x2 = mydata->neighbors[1].x;
    double y2 = mydata->neighbors[1].y;
    double d2 = mydata->neighbors[1].precise_dist;
    double x3;
    double y3;
    double d3;

    if(check_localizablitiy()==0) {
        printf("Not localizable.\n");
        return;
    }

    //find a noncolinear robot
    double dx1 = mydata->neighbors[1].x - mydata->neighbors[0].x;
    double dy1 = mydata->neighbors[1].y - mydata->neighbors[0].y;

    for (int i = 2; i < mydata->N_Neighbors; i++)
    {
        if (mydata->neighbors[i].localized != 1) continue;

        double dx2 = mydata->neighbors[i].x - mydata->neighbors[0].x;
        double dy2 = mydata->neighbors[i].y - mydata->neighbors[0].y;

        //if slope not the same
        if (fabs(dy1 * dx2 - dy2 * dx1) > COLINEAR_THRESHOLD * kilo_lattice_size * kilo_lattice_size * 0.25)
        {
            x3 = mydata->neighbors[i].x;
            y3 = mydata->neighbors[i].y;
            d3 = mydata->neighbors[i].precise_dist;
            break;
        }
    }



    // Define constants for the equations
    double A = 2 * (x2 - x1);
    double B = 2 * (y2 - y1);
    double C = d1 * d1 - d2 * d2 - x1 * x1 + x2 * x2 - y1 * y1 + y2 * y2;

    double D = 2 * (x3 - x1);
    double E = 2 * (y3 - y1);
    double F = d1 * d1 - d3 * d3 - x1 * x1 + x3 * x3 - y1 * y1 + y3 * y3;

    // Solve the linear equations using substitution
    double denominator = A * E - B * D;
    if (fabs(denominator) < 1e-6) {
        printf("Error: Points are collinear or distances are inconsistent.\n");
        return;
    }

    mydata->x = (C * E - B * F) / denominator;
    mydata->y = (A * F - C * D) / denominator;
}



/////////////////////broadcast system step///////////////////////

void broadcast_exists_moving_bot()
{
    mydata->exists_moving_robot = 1;
    mydata->exists_moving_robot_tick = kilo_ticks;
}

void broadcast_moving_bot_stopped()
{
    mydata->exists_moving_robot = 1;
    mydata->exists_moving_robot_tick = kilo_ticks;
}

uint8_t get_N_neighbors_in_dist(uint8_t distance)
{
    uint8_t count = 0;
    for (int i = 0; i < mydata->N_Neighbors; i++)
    {
        #if (PRECISE_LOC == 1)
            double dist = mydata->neighbors[i].precise_dist;
        #else
            uint8_t dist = mydata->neighbors[i].dist;
        #endif
        
        if (dist <= distance)
        {
            count ++;
        }
    }

    return count;

}


double get_dist_by_ID(uint16_t ID)
{
    double dist = 10000000.0;
    
    for(int i = 0; i < mydata->N_Neighbors; i++)
    {
        if(mydata->neighbors[i].ID == ID)
        {
            #if (PRECISE_LOC == 1)
                dist = mydata->neighbors[i].precise_dist;
            #else
                dist = (double) mydata->neighbors[i].dist;
            #endif

            return dist;
        }
    }
    return dist;
}

uint8_t get_gradient_by_ID(uint16_t ID)
{
    uint8_t i;
    
    for(i = 0; i < mydata->N_Neighbors; i++)
    {
        if(mydata->neighbors[i].ID == ID)
        {
            return mydata->neighbors[i].gradient;
        }
    }

    return -1;
}

double find_dist_to_next_bot() //return the distance to the robot of the same ribbon and just behind it
{
    uint8_t i;
    double curdist;

    
    for(i = 0; i < mydata->N_Neighbors; i++)
    {
        #if (PRECISE_LOC == 1)
            curdist = mydata->neighbors[i].precise_dist;
        #else
            curdist = (double) mydata->neighbors[i].dist;
        #endif

        if(mydata->neighbors[i].on_ribbon_ID == mydata->on_ribbon_ID+1 && mydata->neighbors[i].gradient == mydata->gradient_value)
        {
            return curdist;
        }
    }

    return kilo_lattice_size * 2;
}

uint16_t find_nearest_N_ID() //return the kilo_uid of the nearest kilobot in Idle, stop_in, stop_out, stop_in_again, stop_out_again state
{

    uint8_t i;
    double dist = 1000000000.0;
    double curdist;

    
    for(i = 0; i < mydata->N_Neighbors; i++)
    {
        #if (PRECISE_LOC == 1)
            curdist = mydata->neighbors[i].precise_dist;
        #else
            curdist = (double) mydata->neighbors[i].dist;
        #endif

        if(curdist < dist) 
            //&&mydata->neighbors[i].n_bot_move != EDGE 
            // &&mydata->neighbors[i].n_bot_move != SHUFFLE_MOVE
            // &&mydata->neighbors[i].n_bot_move != PAUSE) //none-moving neighbours
        {
            dist = curdist;
        }
    }
        

    for(i = 0; i < mydata->N_Neighbors; i++)
    {
        #if (PRECISE_LOC == 1)
            curdist = mydata->neighbors[i].precise_dist;
        #else
            curdist = (double) mydata->neighbors[i].dist;
        #endif

        if(fabs(curdist - dist) < 0.00001) 
        {
            return mydata->neighbors[i].ID;
        }
    }



    return 0;

    
    
}

uint16_t find_nearest_N_ID_lattice() //return the kilo_uid of the nearest kilobot, by considering the lattice point it occupied
{
    int i;
    double dist = 10000000; // a very big number
    
    uint16_t ID = 0;
    
    for(i = 0; i < mydata->N_Neighbors; i++)
    {
        struct Hex hexPosition={mydata->neighbors[i].hex_q, mydata->neighbors[i].hex_r};
        struct Cartesian rounded_postion = hex_to_Cart(hexPosition);

        double distance = sqrt(pow(mydata->x - rounded_postion.x,2) + pow(mydata->y - rounded_postion.y,2));

        

        if(distance < dist)
            // && mydata->neighbors[i].n_bot_move != EDGE 
            // && mydata->neighbors[i].n_bot_move != SHUFFLE_MOVE
            // && mydata->neighbors[i].n_bot_move != PAUSE) //none-moving neighbours
        {
            dist = distance;
            ID = mydata->neighbors[i].ID;
        }
    }
    
    return ID;

}

uint16_t find_nearest_N_ID_lattice_ribbon() //return the kilo_uid of the nearest kilobot, by considering the lattice point it occupied, only consider the parent ribbon
{
    int i;
    double dist = 10000000; // a very big number
    
    uint16_t ID = 0;
    
    for(i = 0; i < mydata->N_Neighbors; i++)
    {
        if( mydata->neighbors[i].gradient + 1 != mydata->gradient_value) continue;
        struct Hex hexPosition={mydata->neighbors[i].hex_q, mydata->neighbors[i].hex_r};
        struct Cartesian rounded_postion = hex_to_Cart(hexPosition);

        double distance = sqrt(pow(mydata->x - rounded_postion.x,2) + pow(mydata->y - rounded_postion.y,2));

        

        if(distance < dist)
            // && mydata->neighbors[i].n_bot_move != EDGE 
            // && mydata->neighbors[i].n_bot_move != SHUFFLE_MOVE
            // && mydata->neighbors[i].n_bot_move != PAUSE) //none-moving neighbours
        {
            dist = distance;
            ID = mydata->neighbors[i].ID;
        }
    }
    
    return ID;

}

uint16_t find_nearest_ribbon_N_ID_lattice() //return the kilo_uid of the nearest kilobot, by considering the lattice point it occupied
{
    int i;
    double dist = 10000000; // a very big number
    
    uint16_t ID = 0;
    
    for(i = 0; i < mydata->N_Neighbors; i++)
    {
        struct Hex hexPosition={mydata->neighbors[i].hex_q, mydata->neighbors[i].hex_r};
        struct Cartesian rounded_postion = hex_to_Cart(hexPosition);

        double distance = sqrt(pow(mydata->x - rounded_postion.x,2) + pow(mydata->y - rounded_postion.y,2));



        if(distance < dist 
            && mydata->neighbors[i].n_bot_move != EDGE 
            && mydata->neighbors[i].n_bot_move != SHUFFLE_MOVE
            && mydata->neighbors[i].n_bot_move != PAUSE
            && mydata->neighbors[i].gradient == mydata->gradient_value - 1) //none-moving neighbours
        {
            dist = distance;
            ID = mydata->neighbors[i].ID;
        }
    }
    
    return ID;

    
    
}

double get_neighbourX_by_ID(uint16_t ID)
{
    int i;
    for( i = 0; i < mydata->N_Neighbors; i++)
    {
        if(mydata->neighbors[i].ID == ID)
        {
            return mydata->neighbors[i].x;
        }
    }

    return NAN;
}

double get_neighbourY_by_ID(uint16_t ID)
{
    int i;
    for(i = 0; i < mydata->N_Neighbors; i++)
    {
        if(mydata->neighbors[i].ID == ID)
        {
            return mydata->neighbors[i].y;
        }
    }

    return NAN;
}

int16_t get_neighbour_q_by_ID(uint16_t ID)
{
    int i;
    for(i = 0; i < mydata->N_Neighbors; i++)
    {
        if(mydata->neighbors[i].ID == ID)
        {
            return mydata->neighbors[i].hex_q;
        }
    }

    return NAN;
}


int16_t get_neighbour_r_by_ID(uint16_t ID)
{
    int i;
    for(i = 0; i < mydata->N_Neighbors; i++)
    {
        if(mydata->neighbors[i].ID == ID)
        {
            return mydata->neighbors[i].hex_r;
        }
    }

    return NAN;
}

uint8_t find_nearest_N_dist(int output_selection)
{
    uint8_t i;
    uint8_t dist = 255;
    
    //check if all the neighbors are in MOVE_IN or MOVE_OUT state
    uint8_t check_flag = 1;
    
    for(i = 0; i < mydata->N_Neighbors; i++)
    {
        if(!(mydata->neighbors[i].n_bot_state == MOVE_IN
             || mydata->neighbors[i].n_bot_state == MOVE_OUT||mydata->neighbors[i].n_bot_state == JOIN_BACK_IDLE))
        {
            check_flag = 0;
        }
    }
    
    check_flag=0;
    double curdist;
    
    if (check_flag==1) //if all the neighbors are in MOVE_IN or MOVE_OUT state
    {
        for(i = 0; i < mydata->N_Neighbors; i++)
        {
            #if (PRECISE_LOC == 1)
                curdist = mydata->neighbors[i].precise_dist;
            #else
                curdist = (double) mydata->neighbors[i].dist;
            #endif

            if(curdist < dist)
            {
                dist = curdist;
            }
        }
    }
    else
    {
        for(i = 0; i < mydata->N_Neighbors; i++)
        {
            #if (PRECISE_LOC == 1)
                curdist = mydata->neighbors[i].precise_dist;
            #else
                curdist = (double) mydata->neighbors[i].dist;
            #endif

            if(curdist < dist
               && (mydata->neighbors[i].n_bot_state == IDLE
                   || mydata->neighbors[i].n_bot_state == STOP_IN
                   || mydata->neighbors[i].n_bot_state == STOP_OUT
                   || mydata->neighbors[i].n_bot_state == STOP_OUT_AGAIN
                   || mydata->neighbors[i].n_bot_state == STOP_IN_AGAIN
                   || mydata->neighbors[i].n_bot_state == RIBBON))
                   //||mydata->neighbors[i].n_bot_move == PAUSE)) //go through all none-moving neighbours
            {
                dist = curdist;
            }
        }
        

        
    }
    
    if(output_selection == 0)
    {
        return dist;
    }
    else if (output_selection == 1)
    {
        return -1;
    }
    else if (output_selection == 2)
    {
        for(i = 0; i < mydata->N_Neighbors; i++)
        {
            #if (PRECISE_LOC == 1)
                curdist = mydata->neighbors[i].precise_dist;
            #else
                curdist = (double) mydata->neighbors[i].dist;
            #endif

            if(fabs(curdist - dist) < 0.0001) //curdist == dist
                   //||mydata->neighbors[i].n_bot_move == PAUSE)) //go through all none-moving neighbours
            {
                return mydata->neighbors[i].gradient;
            }
        }
        return -1;
    }
    
    return -1;
    
    
}

/////////////////////////Stage 1////////////////////////////////////
// Function to find the number of distinct elements in an array
int countDistinct(uint16_t arr[], int size)
{
    int count = 0;

    for (int i = 0; i < size; i++) {
        int isDistinct = 1; // Assume the element is distinct

        // Check if the element has already been seen
        for (int j = 0; j < i; j++) {
            if (arr[i] == arr[j]) {
                isDistinct = 0; // Mark as not distinct
                break;
            }
        }

        if (isDistinct) {
            count++; // Increment distinct count
        }
    }
    return count;
}

bool checkExist(uint16_t arr[], int size, uint16_t num)
{

    int count = 0;

    for (int i = 0; i < size; i++) {
        if (arr[i] == num) {
            return true;
        }
    }

    return false;
}

void update_sum_ID (uint16_t ID)
{
    // search the neighbor list by ID
    if (mydata->sum_of_bots_ID[0] == ID)
    {// the kilobot is logged
        printf("skip");
    }
    else
    {
        //if exists
        if (checkExist(mydata->sum_of_bots_ID,2,ID)) return;
        
        //shift the array
        memmove(&mydata->sum_of_bots_ID[1], &mydata->sum_of_bots_ID[0], sizeof(uint16_t));

        //Update
        mydata->sum_of_bots_ID[0] = ID;
    }

}

void checkStage1Completion()
{
    // if(kilo_uid == 19)
    // {
    //     printf("Before State: %d", mydata->bot_state);
    //     printf("hex: %d %d %d %d\n", mydata->hex_q,mydata->hex_r,mydata->initialPos.q, mydata->initialPos.r);
    // }

    // //store the position at idle stage
    // if(mydata->bot_state == IDLE){
    //     mydata->initialPos.q = mydata->hex_q;
    //     mydata->initialPos.r = mydata->hex_r;
    // }

    // if(kilo_uid == 19)
    // {
    //     printf("State: %d", mydata->bot_state);
    //     printf("hex: %d %d %d %d\n", mydata->hex_q,mydata->hex_r,mydata->initialPos.q, mydata->initialPos.r);
    // }

    //store the position at idle stage
    if(mydata->initialPos.q == 0 && mydata->initialPos.r ==0){
        mydata->initialPos.q = mydata->hex_q;
        mydata->initialPos.r = mydata->hex_r;
    }

        

    //check if the bot moves one complete circle
    if(mydata->bot_state == MOVE_OUT && countDistinct(mydata->passing_by_ID, mydata->N_passing) > 3)
        if(mydata->initialPos.q == mydata->hex_q && mydata->initialPos.r == mydata->hex_r)
            mydata->stage1complete = 1;

}

void boardcastStage1Complete()
{
    //Update if any neighbor with stage 1 complete
    for (int i = 0; i < mydata->N_Neighbors; i++)
    {
        if(mydata->neighbors[i].stage1complete == 1)
            mydata->stage1complete = 1;
    }
}

//////////////////////omni movement/////////////////////////////


void edge_following_omni_clockwise_wrt_lattice()
{
    uint16_t nearestID = find_nearest_N_ID_lattice();

    // //record N_passing
    // mydata->edge_followee_id = nearestID;
    // update_passing_by_ID (mydata->edge_followee_id);

    struct Cartesian position={get_neighbourX_by_ID(nearestID),get_neighbourY_by_ID(nearestID)};
    struct Cartesian rounded_postion = hex_to_Cart(nearest_lattice(position));

    // double direction = target_direction(mydata->x, mydata->y,get_neighbourX_by_ID(nearestID),get_neighbourY_by_ID(nearestID)) + 0.5f * M_PI;

    double direction = target_direction(mydata->x, mydata->y,rounded_postion.x,rounded_postion.y) + 0.5f * M_PI;
    double distance = sqrt(pow(mydata->x - rounded_postion.x,2) + pow(mydata->y - rounded_postion.y,2));

    //double direction = 1;
    turn_to(direction);
    target_speed(0,0);

    if (fabs(kilo_direction - direction)<0.1)
    {
        if (distance > kilo_lattice_size){
            target_speed(OMNI_SPEED,-OMNI_SPEED * 0.2f);
        }else{
            target_speed(OMNI_SPEED,OMNI_SPEED * 0.2f);
        }
    }
}

void ribbon_following_omni_clockwise_wrt_lattice()
{
    uint16_t nearestID = find_nearest_ribbon_N_ID_lattice();

    // //record N_passing
    // mydata->edge_followee_id = nearestID;
    // update_passing_by_ID (mydata->edge_followee_id);

    struct Cartesian position={get_neighbourX_by_ID(nearestID),get_neighbourY_by_ID(nearestID)};
    struct Cartesian rounded_postion = hex_to_Cart(nearest_lattice(position));

    // double direction = target_direction(mydata->x, mydata->y,get_neighbourX_by_ID(nearestID),get_neighbourY_by_ID(nearestID)) + 0.5f * M_PI;

    double direction = target_direction(mydata->x, mydata->y,rounded_postion.x,rounded_postion.y) + 0.5f * M_PI;
    double distance = sqrt(pow(mydata->x - rounded_postion.x,2) + pow(mydata->y - rounded_postion.y,2));

    //double direction = 1;
    turn_to(direction);
    target_speed(0,0);

    if (fabs(kilo_direction - direction)<0.1)
    {
        if (distance > kilo_lattice_size){
            target_speed(OMNI_SPEED,-OMNI_SPEED * 0.2f);
        }else{
            target_speed(OMNI_SPEED,OMNI_SPEED * 0.2f);
        }
    }
}


void edge_following_omni_counter_clockwise_wrt_lattice()
{
    uint16_t nearestID = find_nearest_N_ID_lattice();

    struct Cartesian position={get_neighbourX_by_ID(nearestID),get_neighbourY_by_ID(nearestID)};
    struct Cartesian rounded_postion = hex_to_Cart(nearest_lattice(position));

    // double direction = target_direction(mydata->x, mydata->y,get_neighbourX_by_ID(nearestID),get_neighbourY_by_ID(nearestID)) + 0.5f * M_PI;

    double direction = target_direction(mydata->x, mydata->y,rounded_postion.x,rounded_postion.y) + 0.5f * M_PI;
    double distance = sqrt(pow(mydata->x - rounded_postion.x,2) + pow(mydata->y - rounded_postion.y,2));

    //double direction = 1;
    turn_to(direction);
    target_speed(0,0);

    if (fabs(kilo_direction - direction)<0.1)
    {
        if (distance > kilo_lattice_size){
            target_speed(-OMNI_SPEED,-OMNI_SPEED * 0.2f);
        }else{
            target_speed(-OMNI_SPEED,OMNI_SPEED * 0.2f);
        }
    }
}

void edge_following_omni_clockwise_wrt_robot()
{
    uint16_t nearestID = find_nearest_N_ID();


    double direction = target_direction(mydata->x, mydata->y,get_neighbourX_by_ID(nearestID),get_neighbourY_by_ID(nearestID)) + 0.5f * M_PI;

    //double direction = 1;
    turn_to(direction);
    target_speed(0,0);

    if (fabs(kilo_direction - direction)<0.1)
    {
        if (get_dist_by_ID(nearestID) > kilo_lattice_size){
            target_speed(OMNI_SPEED,-OMNI_SPEED * 0.2f);
        }else{
            target_speed(OMNI_SPEED,OMNI_SPEED * 0.2f);
        }
    }
}


void edge_following_omni_counter_clockwise_wrt_robot()
{
    uint16_t nearestID = find_nearest_N_ID();

    double direction = target_direction(mydata->x, mydata->y,get_neighbourX_by_ID(nearestID),get_neighbourY_by_ID(nearestID)) + 0.5f * M_PI;

    //double direction = 1;
    turn_to(direction);
    target_speed(0,0);

    if (fabs(kilo_direction - direction)<0.1)
    {
        if (get_dist_by_ID(nearestID) > kilo_lattice_size){
            target_speed(-OMNI_SPEED,-OMNI_SPEED* 0.2f);
        }else{
            target_speed(-OMNI_SPEED,OMNI_SPEED* 0.2f);
        }
    }
}

void edge_following_omni_clockwise_nonlocalizable()
{
    uint16_t nearestID = find_nearest_N_ID();

    double diff = fabs(get_dist_by_ID(nearestID) - kilo_lattice_size) / kilo_lattice_size;

    if (get_dist_by_ID(nearestID) > kilo_lattice_size  ){
        // if(mydata->previous_direction == 0){
        //     //do nothing
        // }
        // else{
        //     mydata->target_direction = kilo_direction - 0.2; //turn right
        //     mydata->previous_direction = 0;
        //     mydata->non_localizable_move_ticks = kilo_ticks;
        // }
        

        // if(abs(mydata->non_localizable_move_ticks - kilo_ticks)>5.0+(rand() % 10001) / 10000.0)
        // {
        //     mydata->target_direction = kilo_direction - 0.1; //turn right
        //     mydata->non_localizable_move_ticks = kilo_ticks;
        // }
        mydata->target_direction = kilo_direction - 0.04f * (rand() % 10001) / 10000.0; //turn right
        turn_to(mydata->target_direction);

        
    }
    else if(get_dist_by_ID(nearestID) < kilo_lattice_size  )
    {
        
        // if(mydata->previous_direction == 1){
        //     //do nothing
        // }
        // else{
        //     mydata->target_direction = kilo_direction + 0.2; //turn right
        //     mydata->previous_direction = 1;
        //     mydata->non_localizable_move_ticks = kilo_ticks;
        // }

        // if(abs(mydata->non_localizable_move_ticks - kilo_ticks)>5.0+(rand() % 10001) / 10000.0)
        // {
        //     mydata->target_direction = kilo_direction + 0.1; //turn left
        //     mydata->non_localizable_move_ticks = kilo_ticks;
        // }

        mydata->target_direction = kilo_direction + 0.04f * (rand() % 10001) / 10000.0; //+ 0.05 * (1+(rand() % 10001) / 10000.0); //turn left
        turn_to(mydata->target_direction);
    }
    else
    {

    }

    if (get_dist_by_ID(nearestID) > kilo_lattice_size){
        target_speed(OMNI_SPEED,-OMNI_SPEED * 0.7f);
    }else{
        target_speed(OMNI_SPEED,OMNI_SPEED * 0.7f);
    }
    

}

void edge_following_omni_clockwise_wrt_lattice_ribbon()
{
    uint16_t nearestID = find_nearest_N_ID_lattice_ribbon();

    // //record N_passing
    // mydata->edge_followee_id = nearestID;
    // update_passing_by_ID (mydata->edge_followee_id);

    struct Cartesian position={get_neighbourX_by_ID(nearestID),get_neighbourY_by_ID(nearestID)};
    struct Cartesian rounded_postion = hex_to_Cart(nearest_lattice(position));

    // double direction = target_direction(mydata->x, mydata->y,get_neighbourX_by_ID(nearestID),get_neighbourY_by_ID(nearestID)) + 0.5f * M_PI;

    double direction = target_direction(mydata->x, mydata->y,rounded_postion.x,rounded_postion.y) + 0.5f * M_PI;
    double distance = sqrt(pow(mydata->x - rounded_postion.x,2) + pow(mydata->y - rounded_postion.y,2));

    //double direction = 1;
    turn_to(direction);
    target_speed(0,0);

    if (fabs(kilo_direction - direction)<0.1)
    {
        if (distance > kilo_lattice_size){
            target_speed(OMNI_SPEED,-OMNI_SPEED * 0.2f);
        }else{
            target_speed(OMNI_SPEED,OMNI_SPEED * 0.2f);
        }
    }
}

void omni_stop()
{
    target_speed(0,0);
}

float angle_diff(float a, float b) {
    float d = a - b;
    while (d > M_PI) d -= 2.0f * M_PI;
    while (d < -M_PI) d += 2.0f * M_PI;
    return fabs(d);
}

void omni_move_to_nearest_lattice() {
    /*
     * Move the robot to the nearest lattice point in a hexagonal grid with a fixed turning rate and speed.
     * `lattice_size` is the distance from the center of a hex to any of its vertices.
     * `timestep` is the time increment for movement simulation.
     */

    float lattice_size = kilo_lattice_size;
    float dx =  lattice_size;   // Horizontal distance between lattice points
    float dy = sqrt(3.0f)/2.0f * lattice_size; 

    // Convert (x, y) to axial coordinates
    float r = mydata->y / dy;
    float q = mydata->x / dx - 0.5f * r;

    // Round to nearest axial coordinates
    int q_round = round(q);
    int r_round = round(r);

    // Correct for rounding errors in axial coordinates
    float q_diff = fabs(q - q_round);
    float r_diff = fabs(r - r_round);
    float s_diff = fabs(-q - r - (-q_round - r_round));

    if (q_diff > r_diff && q_diff > s_diff) {
        q_round = -r_round - (-q_round - r_round);
    } else if (r_diff > s_diff) {
        r_round = -q_round - (-q_round - r_round);
    }

    // Convert back to Cartesian coordinates to find target position
    float target_x = q_round * dx + r_round * 0.5f * dx;    
    float target_y = r_round * dy;

    

    // Calculate the direction to the target
    float target_direction = atan2(target_y - mydata->y, target_x - mydata->x);

    //double direction = 1;
    turn_to(target_direction);
    target_speed(0,0);

    //if (fabs(kilo_direction - target_direction)<0.01)
    if (angle_diff(kilo_direction, target_direction) < 0.01f)
    {
        float epsilon = 0.5; // tolerance
        float distance = sqrt(pow(mydata->x - target_x,2) + pow(mydata->y - target_y,2));

        if (distance > epsilon && distance > 15.0) {
            target_speed(OMNI_SPEED,0);
        }else if(distance > epsilon && distance <= 15.0){
            target_speed(distance*0.5f,0); //slow down when approaching
        }else{
            target_speed(0,0);
        }
    }

    return;
}
#if 0
int omni_move_to_lattice(struct Hex *point) {
    /*
     * Move the robot to the lattice point (q,r) in a hexagonal grid with a fixed turning rate and speed.
     * `lattice_size` is the distance from the center of a hex to any of its vertices.
     * `timestep` is the time increment for movement simulation.
     * return 1 if reached the target lattice point
     */

// printf("%d %d", (**point).q, (**point).r  );
    

    float lattice_size = kilo_lattice_size;
    float dx =  lattice_size;   // Horizontal distance between lattice points
    float dy = sqrt(3.0f)/2.0f * lattice_size; 
    // Convert back to Cartesian coordinates to find target position
    float target_x = (*point).q * dx + (*point).r * 0.5f * dx;    
    float target_y = (*point).r * dy;

    

    // Calculate the direction to the target
    float target_direction = atan2(target_y - mydata->y, target_x - mydata->x);

    //double direction = 1;
    turn_to(target_direction);
    target_speed(0,0);
#if 0
    if (fabs(kilo_direction - target_direction)<0.01)
    {
        float epsilon = 0.5; // tolerance
        float distance = sqrt(pow(mydata->x - target_x,2) + pow(mydata->y - target_y,2));

        if (distance > epsilon && distance > 15.0) {
            target_speed(OMNI_SPEED,0);
        }else if(distance > epsilon && distance <= 15.0){
            target_speed(distance*0.5f,0); //slow down when approaching
        }else{
            target_speed(0,0);
            return 1;
        }
    }
#else
    if (fabs(kilo_direction - target_direction)<0.01)
    {
        float epsilon = 0.5; // tolerance
        float distance = sqrt(pow(kilo_x - target_x,2) + pow(kilo_y - target_y,2));
        printf("距离目标点 %f\n",distance);

        if (distance > epsilon && distance > 15.0) {
            target_speed(OMNI_SPEED*10,0);
        
        }else if(distance > epsilon && distance <= 15.0){
            target_speed(distance,0); //slow down when approaching
        }else{
            target_speed(0,0);
            occupied++;
            return 1;
        }
    }
#endif
    return 0;
}
#else
#if 1
int omni_move_to_lattice(struct Hex *point) {
    /*
     * Move the robot to the lattice point (q,r) in a hexagonal grid with a fixed turning rate and speed.
     * `lattice_size` is the distance from the center of a hex to any of its vertices.
     * `timestep` is the time increment for movement simulation.
     * return 1 if reached the target lattice point
     */

    float lattice_size = kilo_lattice_size;
    float dx =  lattice_size;   // Horizontal distance between lattice points
    float dy = sqrt(3.0f)/2.0f * lattice_size; 
    // Convert back to Cartesian coordinates to find target position
    float target_x = (*point).q * dx + (*point).r * 0.5f * dx;    
    float target_y = (*point).r * dy;

    struct Hex current_hex = get_hex_from_cartesian(mydata->x,mydata->y);
  
    // Calculate the direction to the target
    //float target_direction = atan2(target_y - mydata->y, target_x - myda ta->x);
    float target_direction = atan2(target_y - mydata->y, target_x - mydata->x);
// 等待几帧
// 再打印一次
    //double direction = 1;
    turn_to(target_direction);
    target_speed(0,0);

    if (fabs(kilo_direction - target_direction)<0.01)
    {
        float epsilon = 0.5; // tolerance
        float distance = sqrt(pow(mydata->x - target_x,2) + pow(mydata->y - target_y,2));
        
        //printf("当前位置(%d,%d),距离目标点 (%d,%d) %f, 当前的位置是(%0.1f,%0.1f)\n",mydata->hex_q,mydata->hex_r,point->q,point->r,distance,mydata->x,mydata->y);
        
        if (distance > epsilon && distance > 15.0) {
            target_speed(OMNI_SPEED*0.5f,0);

            
        
        }else if(distance > epsilon && distance <= 15.0){
            //printf("当前速度是%0.5f\n",distance*0.5f);
            //target_speed(distance*0.5f,0); //slow down when approaching
            target_speed(distance*0.1f,0); //slow down when approaching
    
        }else{
            target_speed(0,0);
            return 1;
        }
    
    
    }

    return 0;
}
#else
int omni_move_to_lattice(struct Hex *point) {
    static bool turning_complete = false;
    static float target_x = 0, target_y = 0;
    
    // 初始化目标位置（只在第一次调用时）
    if (target_x == 0 && target_y == 0) {
        float lattice_size = kilo_lattice_size;
        float dx = lattice_size;
        float dy = sqrt(3.0f)/2.0f * lattice_size;
        
        target_x = point->q * dx + point->r * 0.5f * dx;    
        target_y = point->r * dy;
    }
    
    float target_direction = atan2(target_y - kilo_y, target_x - kilo_x);
    float distance = sqrt(pow(kilo_x - target_x, 2) + pow(kilo_y - target_y, 2));
    
    // 先完成转向
    if (!turning_complete) {
        turn_to(target_direction);
        target_speed(0, 0);
        
        if (fabs(kilo_direction - target_direction) < 0.01) {
            turning_complete = true;
            printf("转向完成，开始移动\n");
        }
    } 
    // 转向完成后开始移动
    else {
        if (distance > 15.0) {
            target_speed(OMNI_SPEED, 0);
        } else if (distance > 1.0) {
            target_speed(distance * 0.5f, 0);
        } else {
            target_speed(0, 0);
            printf("到达目标\n");
            turning_complete = false;  // 重置状态
            return 1;
        }
    }
    
    return 0;
}
    #endif
#endif

void record_current_position_for_next_lattice()
{
    //record current lattice pos
    //if(mydata->move_type == STOP)
    //if(mydata->exists_moving_robot == 0)
    if (mydata->moved_within_step == 0)
    {
        mydata->currentPos.q = mydata->hex_q;
        mydata->currentPos.r = mydata->hex_r;        
    }
}

void move_to_next_lattice_point_same_ribbon()
{

        //move to the next lattice pos
        //printf("%d %d\n", mydata->currentPos.q, mydata->currentPos.r);
  

        // if(mydata->currentPos.q == mydata->hex_q && mydata->currentPos.r == mydata->hex_r)
        // ribbon_following_omni_clockwise_wrt_lattice();
        

        //if near the next lattice point
        if((mydata->currentPos.q != mydata->hex_q || mydata->currentPos.r != mydata->hex_r) && distance_to_nearest_lattice()< kilo_lattice_size * 0.3f)
        {
            omni_move_to_nearest_lattice();
            
            // if(distance_to_nearest_lattice()>= 0.5)
            //     mydata->ribbon_com_tick = kilo_ticks;               
            // else//delay 100 ticks to restart moving to the next lattice
            // {
            //     //if(kilo_uid == 15) printf("done with distance %f", distance_to_nearest_lattice());
            //     mydata->move_to_next_lattice_same_ribbon_flag = 0;
       
            //     //keep localization
            //     mydata->localized = 2; 
            //     mydata->localize_cycle = 0;
                
            //     set_move_type(STOP);
            // }

            //reset move to next ribbon flag
            if(distance_to_nearest_lattice()< 0.5)
            {
                                      
                //if(kilo_uid == 15) printf("done with distance %f", distance_to_nearest_lattice());
                mydata->move_to_next_lattice_same_ribbon_flag = 0;
       
                //keep localization
                mydata->localized = 2; 
                mydata->localize_cycle = 0;
                
                set_move_type(STOP);
            }
        } 
        else
        {
            ribbon_following_omni_clockwise_wrt_lattice();
        }

        
}


///////////////////Hex And Cartesian//////////////


struct Hex nearest_lattice(struct Cartesian position) {
    /*
     * nearest lattice of the cartesian position
     */

    float lattice_size = kilo_lattice_size;
    float dx =  lattice_size;   // Horizontal distance between lattice points
    float dy = sqrt(3.0f)/2.0f * lattice_size; 

    // Convert (x, y) to axial coordinates
    float r = position.y / dy;
    float q = position.x / dx - 0.5f * r;

    // Round to nearest axial coordinates
    int q_round = round(q);
    int r_round = round(r);

    struct Hex hex = {q_round, r_round};

    return hex;
}

struct Hex my_nearest_lattice() {
    /*
     * nearest lattice point of {mydata->x, mydata->y}
     */

    float lattice_size = kilo_lattice_size;
    float dx =  lattice_size;   // Horizontal distance between lattice points
    float dy = sqrt(3.0f)/2.0f * lattice_size; 
#if 1
    mydata->x = kilo_x;
    mydata->y = kilo_y;
#endif
    struct Cartesian position={mydata->x,mydata->y};

    // Convert (x, y) to axial coordinates
    float r = position.y / dy;
    float q = position.x / dx - 0.5f * r;

    // Round to nearest axial coordinates
    int q_round = round(q);
    int r_round = round(r);

    struct Hex hex = {q_round, r_round};

    return hex;
}

double distance_to_lattice(struct Hex position){
    float lattice_size = kilo_lattice_size;
    float dx =  lattice_size;   // Horizontal distance between lattice points
    float dy = sqrt(3.0f)/2.0f * lattice_size; 

    // Convert back to Cartesian coordinates to find target position
    float lattice_x = position.q * dx + position.r * 0.5f * dx;    
    float lattice_y = position.r * dy;

    return sqrt(pow(lattice_x - mydata->x,2) + pow(lattice_y - mydata->y,2));
}
double distance_to_nearest_lattice(){

    float lattice_size = kilo_lattice_size;
    float dx =  lattice_size;   // Horizontal distance between lattice points
    float dy = sqrt(3.0f)/2.0f * lattice_size; 

    

    // Convert (x, y) to axial coordinates
    float r = mydata->y / dy;
    float q = mydata->x / dx - 0.5f * r;

    // Round to nearest axial coordinates
    int q_round = round(q);
    int r_round = round(r);

    struct Hex hex={q_round,r_round};

    return distance_to_lattice(hex);

    // // Convert back to Cartesian coordinates to find target position
    // float lattice_x = q_round * dx + r_round * 0.5f * dx;    
    // float lattice_y = r_round * dy;

    // return sqrt(pow(lattice_x - position.x,2) + pow(lattice_y - position.y,2));
}

struct Cartesian hex_to_Cart(struct Hex hex){
    float lattice_size = kilo_lattice_size;
    float dx =  lattice_size;   // Horizontal distance between lattice points
    float dy = sqrt(3.0f)/2.0f * lattice_size; 

    // Convert back to Cartesian coordinates to find target position
    float x = hex.q * dx + hex.r * 0.5f * dx;    
    float y = hex.r * dy;

    struct Cartesian cart = {x,y};

    return cart;
}

bool is_lattice_point_adjacent(struct Hex point1, struct Hex point2)
{
    const int DIRECTIONS[6][2] = {
    { 1, -1 }, // Down-Right
    { 0, -1 },// Down-Left
    { -1, 0 }, // Left
    { -1, 1 },  // Up-Left
    { 0, 1 },   // Up-Right
	{ 1, 0 },  // Right
    };

    int q_diff = point1.q - point2.q;
    int r_diff = point1.r - point2.r;

    for (int i = 0; i<6; i++)
    {
        if(q_diff == DIRECTIONS[i][0] && r_diff == DIRECTIONS[i][1])
            return true;
    }

    return false;
    
}


////////////////////////////shape//////////////////////////

bool is_in_shape(struct Hex hexPoint)
{
    for (int i = 0; i < mydata->lattice_shape_size; i++)
    {
        if (mydata->lattice_shape[i].q == mydata->hex_q && mydata->lattice_shape[i].r == mydata->hex_r)
        {
            return true;
        }
    }

    return false;
}

bool am_in_shape()
{
    struct Hex hex = {mydata->hex_q, mydata->hex_r};
    return is_in_shape(hex);
}

// bool is_in_shape(struct Hex hexPoint)
// {
//     for (int i = 0; i < mydata->lattice_shape_size; i++)
//     {
//         if (mydata->lattice_shape[i].q == mydata->hex_q && mydata->lattice_shape[i].r == mydata->hex_r)
//         {
//             return true;
//         }
//     }

//     return false;
// }

// bool am_in_shape()
// {
//     struct Hex hex = {mydata->hex_q, mydata->hex_r};
//     return is_in_shape(hex);
// }

// bool is_in_hole(struct Hex hexPoint)
// {
//     for (int i = 0; i < mydata->lattice_hole_size; i++)
//     {
//         if (mydata->lattice_hole[i].q == mydata->hex_q && mydata->lattice_hole[i].r == mydata->hex_r)
//         {
//             return true;
//         }
//     }

//     return false;
// }

// bool am_in_hole()
// {
//     struct Hex hex = {mydata->hex_q, mydata->hex_r};
//     return is_in_hole(hex);
// }

bool is_at_ribbon_head(struct Hex hexPoint)
{
    for (int i = 0; i < mydata->num_of_ribbon_heads; i++)
    {
        if (mydata->ribbon_heads[i].q == mydata->hex_q && mydata->ribbon_heads[i].r == mydata->hex_r)
        {
            return true;
        }
    }

    return false;
}

bool am_at_ribbon_head()
{
    struct Hex hex = {mydata->hex_q, mydata->hex_r};
    return is_at_ribbon_head(hex);
}


/////////////////Ribbon functions//////////////////

bool am_at_tail()
{
    struct Hex myHex = {mydata->hex_q, mydata->hex_r};
    

    //Go through all neighbors
    for (int i = 0; i < mydata->N_Neighbors; i++)
    {


        struct Hex adjHex = {mydata->neighbors[i].hex_q, mydata->neighbors[i].hex_r};

        //locate the adjacent robot of the same ribbon
        if(mydata->gradient_value == mydata->neighbors[i].gradient && is_lattice_point_adjacent(myHex, adjHex))
        {
            struct Hex parentHex;

            //locate the robot in parent ribbon which is adjacent to both robot
            for (int j = 0; j < mydata->N_Neighbors; j++)
            {
                parentHex.q = mydata->neighbors[j].hex_q;
                parentHex.r = mydata->neighbors[j].hex_r;

                if (is_lattice_point_adjacent(myHex, parentHex) && is_lattice_point_adjacent(adjHex,parentHex) && mydata->neighbors[j].gradient ==  mydata->gradient_value -1)
                {
                    break;
                }
            }

            //check if the adjRobHex is clockwise to myHex wrt parentHex
            struct Cartesian myCart = {mydata->x, mydata->y};
            struct Cartesian adjCart = hex_to_Cart(adjHex);
            struct Cartesian parentCart = hex_to_Cart(parentHex);

            if(crossProductZ((myCart.x - parentCart.x), (myCart.y - parentCart.y), (adjCart.x - parentCart.x),(adjCart.y - parentCart.y)) > 0)
            {
                return false;
            }
        }

        
    }

    //return true if no robot of the same ribbon is behind
    return true;
}

void step_detection()
{
    /////////////////////////halfway detection///////////////////////
    if( mydata->system_step == (int)(mydata->lattice_s_size * 0.5))
    {
        kilo_is_paused = 1;
    }

    /////////////////////////finish detection///////////////////////
    if( mydata->system_step == mydata->lattice_s_size)
    {
        kilo_is_paused = 1;
    }

}


/////////////////faulty handling//////////////////////
void shift_forward(struct Hex *lattice_s, int size) {
    if (size <= 1) return; // No shifting needed for empty or single-element arrays

    struct Hex last = lattice_s[size - 1]; // Store last element

    // Shift all elements forward
    for (int i = size - 1; i > 0; i--) {
        lattice_s[i] = lattice_s[i - 1];
    }

    printf("%d", size);
    lattice_s[0] = last; // Place last element at the beginning
}

// Function to insert a new Hex at index z
void insert_at(struct Hex *lattice_s,  int size, int z, struct Hex newElement) {

    if (z < 0 || z > size) {
        printf("Invalid index!\n");
        return;
    }

    // Shift elements to the left from index z
    for (int i = 0; i < z-1; i++) {
        lattice_s[i] = lattice_s[i + 1];
    }
    
    // printf("insert %d %d at index %d", newElement.q, newElement.r, z-1);
    // Insert new element at index z
    lattice_s[z-1] = newElement;
}

void insert_at_recycle(struct Hex *lattice_r,  int size, int z, struct Hex newElement, int *recycleFlagSeq)
{
    //insert before the first recycle robot

    // Shift elements to the left from index z

    for (int i = size-1; i > z-1; i--) {
        if(recycleFlagSeq[i]==2)
        {
            for (int j = i-1; j > z-1; j--) 
            {
                if (recycleFlagSeq[j]==2)
                {
                    lattice_r[i] = lattice_r[j];
                    break;
                }
                
            }
        }
        
    }

    // Insert new element at index z
    for (int i = z; i < size+1; i++) {
        if (recycleFlagSeq[i]==2)
        {
            lattice_r[i] = newElement;
            break;
        }
                
        
    }

}



///////////////////////////Other utility functions//////////////////////////////////////
void update_passingby_ID_gradient (uint16_t ID)
{

    if (mydata->passingby_ID_gradient[0] == ID)
    {
        // the kilobot is logged
    }
    else
    {
        //shift the array
        memmove(&mydata->passingby_ID_gradient[1], &mydata->passingby_ID_gradient[0], sizeof(uint16_t));
        //Update
        mydata->passingby_ID_gradient[0] = ID;
        if(mydata->N_passing_gradient<2) mydata->N_passing_gradient ++;
    }

}

void update_passing_by_ID (uint16_t ID)
{

    // search the neighbor list by ID
    if (mydata->passing_by_ID[0] == ID)
    {// the kilobot is logged

    }
    else
    {
        //shift the array
        memmove(&mydata->passing_by_ID[1], &mydata->passing_by_ID[0], (N_PASSING_MAX - 1)*sizeof(uint16_t));
        memmove(&mydata->timestamp[1], &mydata->timestamp[0], (N_PASSING_MAX-1)*sizeof(uint32_t));

        //Update number of followed kilobot
         if (mydata->N_passing < N_PASSING_MAX-1) mydata->N_passing ++;

        //Update
        mydata->passing_by_ID[0] = ID;
        mydata->timestamp[0] = kilo_ticks;
    }

}



double convertX16toDouble(uint16_t a)
{
    return  ((- x_range) + ((int)a / 65535.0) * 2 * x_range);
}

double convertY16toDouble(uint16_t a)
{
    return  ((- y_range) + ((int)a / 65535.0) * 2 * y_range);
}

double crossProductZ(double x1, double y1, double x2, double y2) //vector [x1,y1] cross [x2,y2]//int vect_A[], int vect_B[], int cross_P[])
 
{
    return x1 * y2 - x2 * y1;  //vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
}

float normalize_to_pi(float angle) {
    const float PI = M_PI;
    const float TWO_PI = 2.0f * PI;

    // Normalize to [-2pi, 2pi)
    angle = fmod(angle, TWO_PI);  
    
    // Ensure the result is in the range [-pi, pi]
    if (angle > PI) {
        angle -= TWO_PI;  // Wrap to [-pi, pi]
    } else if (angle < -PI) {
        angle += TWO_PI;  // Wrap to [-pi, pi]
    }
    
    return angle;
}

double target_direction(double bot_x, double bot_y, double target_x, double target_y)
{
  return atan2(target_y - bot_y, target_x - bot_x);
}




///////////////////////////////////////////robot state//////////////////////////////////


void set_bot_state(enum BOTSTATE state)
{
    mydata->bot_state = state;
}

enum BOTSTATE get_bot_state(void)
{
    return mydata->bot_state;
}

void set_move_type(enum MOVE_TYPE type)
{
    mydata->move_type = type;
}

enum MOVE_TYPE get_move_type(void)
{
    return mydata->move_type;
}

void set_bot_type(enum BOTTYPE type)
{
    mydata->bot_type = type;
}

enum BOTTYPE get_bot_type(void)
{
    return mydata->bot_type;
}

enum RIBBON_BOT_TYPE get_r_bot_type(void)
{
    return mydata->r_bot_type;
}

void set_r_bot_type(enum RIBBON_BOT_TYPE type)
{
    mydata->r_bot_type = type;
}

uint8_t get_ribbon_ID_by_ID(uint16_t bot_ID) //return the ribbon ID of the kilobot_uid
{
    
    for(int i = 0; i < mydata->N_Neighbors; i++)
    {
        if(mydata->neighbors[i].ID == bot_ID)
        {
            return mydata->neighbors[i].ribbon_ID;
        }
    }

    return -1;

}