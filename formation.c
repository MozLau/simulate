/* Kilobot Edge following demo
 *
 * Ivica Slavkov, Fredrik Jansson  2015
 */


/*
 1.Need to let kilobot with id 0 excluded
 */

#include <math.h>

//#include <kilombo.h>

#include <stdbool.h>
#include "shape.c"
#include "formation.h"
#include "shape.h"

//Include all common functions
#include "common.c"

//Include all the states function
#include "IDLE.c"
#include "MOVE.c"
// #include "REASSEMBLY.c"

//radius is 17cm
//communication range is 250cm (edit in kilombo.json)
//edge following distance 108mm (edit in kilombo.json)


#ifdef SIMULATOR
#include <stdio.h>    // for printf
#else
#define DEBUG         // for printf to serial port
#include "debug.h"
#endif

#if 1
#include "liu_path.h"
#define WITH_MOTION
extern uint8_t is_occupied[200][200];
extern bool should_move_to_shape;

#endif


REGISTER_USERDATA(MyUserdata);



///////////////////////////////////Communication Between Robots///////////////////////////////////////

// message rx callback function. Pushes message to ring buffer.
void rxbuffer_push(message_t *msg, distance_measurement_t *dist) {
    received_message_t *rmsg = &RB_back();
    rmsg->msg = *msg;
    rmsg->dist = *dist;
    RB_pushback();
}

message_t *message_tx()
{
    if (mydata->message_lock)
        return 0;
    
    #if 0
    if (mydata->gradient_value == UINT8_MAX)
        return 0;
    #endif

    return &mydata->transmit_msg;
}


void reset_ribbon_status(void)
{
    //reset ribbon
    mydata->ribbon_ID = 0;
    mydata->on_ribbon_ID = 0;
    mydata->n_bot_not_in_shape = 0; //number of kilobot on the ribbon which are not in the shape
    mydata->n_bot_not_in_shape_ahead = 0; //the number of kilobot on the ribbon before me is not in the shape
    mydata->n_bot_need_retain = UINT8_MAX;
    mydata->ribbon_complete = 0;
}

/* Process a received message at the front of the ring buffer.
 * Go through the list of neighbors. If the message is from a bot
 * already in the list, update the information, otherwise
 * add a new entry in the list
 */

void process_message()
{
    uint8_t *data = RB_front().msg.data;
#if 0
     // 添加诊断打印
    printf("\n=== 消息接收诊断 - Robot %d ===\n", kilo_uid);
    printf("收到消息数据: ");
    for(int i = 0; i < 70; i++) { // 检查前70个字节
        if(i % 16 == 0) printf("\n[%02d-%02d]: ", i, i+15);
        printf("%02X ", data[i]);
        
        // 检查特定标记字节
        if(i == 9 && data[i] == 0xAA) printf("(标记9)");
        if(i == 10 && data[i] == 0xBB) printf("(标记10)");
        if(i == 11 && data[i] == 0xCC) printf("(标记11)");
    }
    printf("\n");
    
    // 检查关键数据是否完整
    uint16_t sender_id = data[0] | (data[1] << 8);
    printf("发送者ID: %d (字节0-1: %02X %02X)\n", sender_id, data[0], data[1]);
    
    // 检查六边形坐标
    int16_t hex_q = data[38] | (data[39] << 8);
    int16_t hex_r = data[40] | (data[41] << 8);
    printf("六边形坐标: q=%d, r=%d (字节38-41: %02X %02X %02X %02X)\n", 
           hex_q, hex_r, data[38], data[39], data[40], data[41]);
    
    // 检查移动意图数据
    printf("移动意图: 字节54=%d, 字节55=%d\n", data[54], data[55]);
    printf("目标位置: q=%d, r=%d (字节56-57: %02X %02X)\n", 
           (int8_t)data[56], (int8_t)data[57], data[56], data[57]);
    
    printf("==============================\n");
#endif


        uint8_t i;
        uint16_t ID;
        
        ID = data[0] | (data[1] << 8);
        uint8_t d = estimate_distance(&RB_front().dist);
        double pd = precise_distance(&RB_front().dist);

        //printf("d: %d pf: %f", d, pd);
        
        // search the neighbor list by ID
        for (i = 0; i < mydata->N_Neighbors; i++)
            if (mydata->neighbors[i].ID == ID)
            {// found it
                mydata->neighbors[i].timestamp = kilo_ticks;
                mydata->neighbors[i].dist = d;
                mydata->neighbors[i].precise_dist = pd;
                mydata->neighbors[i].N_Neighbors = data[2];
                mydata->neighbors[i].n_bot_state = data[3] >> 4;
                mydata->neighbors[i].n_bot_move = data[3] & 0x0f;
                mydata->neighbors[i].gradient = data[4];
                //mydata->neighbors[i].angle_8 = data[5];
                mydata->neighbors[i].local_ID= data[6];

                mydata->neighbors[i].x_16 = data[7] | (data[8] << 8);
                mydata->neighbors[i].y_16 = data[9] | (data[10] << 8);
                mydata->neighbors[i].x = convertX16toDouble(mydata->neighbors[i].x_16);
                mydata->neighbors[i].y = convertY16toDouble(mydata->neighbors[i].y_16);
                mydata->neighbors[i].edge_followee_id = data[11] | (data[12] << 8);
                mydata->neighbors[i].N_passing = data[13];
                mydata->neighbors[i].n_bot_type = data[14];
                mydata->neighbors[i].localized = data[15];
                mydata->neighbors[i].on_ribbon_ID = data[17];
                mydata->neighbors[i].ribbon_ID = data[18];
                mydata->neighbors[i].n_bot_not_in_shape = data[19];
                mydata->neighbors[i].n_bot_not_in_shape_ahead = data[20];
                mydata->neighbors[i].n_bot_need_retain = data[21];
                mydata->neighbors[i].ribbon_complete = data[22];
                mydata->neighbors[i].r_bot_type = data[23];
                mydata->neighbors[i].ribbon_my_turn = data[24];

                // mydata->neighbors[i].hex_q = data[26];
                // mydata->neighbors[i].hex_r = data[27];

                mydata->neighbors[i].is_moving = data[28];
                mydata->neighbors[i].has_movement_intent = data[29];
                mydata->neighbors[i].intended_target_q = data[30] | (data[31] << 8);
                mydata->neighbors[i].intended_target_r = data[32] | (data[33] << 8);

                mydata->neighbors[i].idle_ribbon_my_turn = data[36];
                mydata->neighbors[i].stage1complete = data[37];

                mydata->neighbors[i].hex_q = data[38] | (data[39] << 8);
                mydata->neighbors[i].hex_r = data[40] | (data[41] << 8);

                mydata->neighbors[i].sum_of_bots_in_hole = data[42] | (data[43] << 8);
               
                mydata->neighbors[i].shape_position_occupied = data[44];

                mydata->neighbors[i].movement_priority = data[45];


                return;
            }
        
        if (i == mydata->N_Neighbors){          // if nieghbor is not in the record
            if (mydata->N_Neighbors < MAXN-1)   
                mydata->N_Neighbors++;          
        }
        
        // if we have too many neighbors,we overwrite the last entry
        // i now points to where this message should be stored
        mydata->neighbors[i].ID = ID;

        mydata->neighbors[i].timestamp = kilo_ticks;
        mydata->neighbors[i].dist = d;
        mydata->neighbors[i].precise_dist = pd;
        mydata->neighbors[i].N_Neighbors = data[2];
        mydata->neighbors[i].n_bot_state = data[3] >> 4;
        mydata->neighbors[i].n_bot_move = data[3] & 0x0f;
        mydata->neighbors[i].gradient = data[4];
        //mydata->neighbors[i].gradient2 = data[16];
        //mydata->neighbors[i].angle_8 = data[5];
        mydata->neighbors[i].local_ID= data[6];

        mydata->neighbors[i].x_16 = data[7] | (data[8] << 8);
        mydata->neighbors[i].y_16 = data[9] | (data[10] << 8);
        mydata->neighbors[i].x = convertX16toDouble(mydata->neighbors[i].x_16);
        mydata->neighbors[i].y = convertY16toDouble(mydata->neighbors[i].y_16);
        mydata->neighbors[i].edge_followee_id = data[11] | (data[12] << 8);
        mydata->neighbors[i].N_passing = data[13];
        mydata->neighbors[i].n_bot_type = data[14];
        mydata->neighbors[i].localized = data[15];
        //mydata->neighbors[i].gradient2 = data[16];
        mydata->neighbors[i].on_ribbon_ID = data[17];
        mydata->neighbors[i].ribbon_ID = data[18];
        mydata->neighbors[i].n_bot_not_in_shape = data[19];
        mydata->neighbors[i].n_bot_not_in_shape_ahead = data[20];
        mydata->neighbors[i].n_bot_need_retain = data[21];
        mydata->neighbors[i].ribbon_complete = data[22];
        mydata->neighbors[i].r_bot_type = data[23];
        mydata->neighbors[i].ribbon_my_turn = data[24];

        mydata->neighbors[i].has_movement_intent = data[29];
        mydata->neighbors[i].intended_target_q = data[30] | (data[31] << 8);
        mydata->neighbors[i].intended_target_r = data[32] | (data[33] << 8);


        mydata->neighbors[i].idle_ribbon_my_turn = data[36];
        mydata->neighbors[i].stage1complete = data[37];

        mydata->neighbors[i].hex_q = data[38] | (data[39] << 8);
        mydata->neighbors[i].hex_r = data[40] | (data[41] << 8);

        
        mydata->neighbors[i].shape_position_occupied = data[44];


}

/* Go through the list of neighbors, remove entries older than a threshold,
 * currently 2 seconds.
 */
void purgeNeighbors(void)
{
    int8_t i;
    
    for (i = mydata->N_Neighbors-1; i >= 0; i--)
        if (kilo_ticks - mydata->neighbors[i].timestamp  > 64) //32 ticks = 1 s
        { //this one is too old.
            mydata->neighbors[i] = mydata->neighbors[mydata->N_Neighbors-1];
            //replace it by the last entry
            mydata->N_Neighbors--;
        }
}

void setup_message(void)
{
    mydata->message_lock = 1;  //don't transmit while we are forming the message
    // 1 byte for message type

        mydata->transmit_msg.type = NORMAL;
        
        //9 bytes can be used for actual data (message payload)
        mydata->transmit_msg.data[0] = kilo_uid & 0xff;     // 0 low  ID
        mydata->transmit_msg.data[1] = kilo_uid >> 8;       // 1 high ID
        
        mydata->transmit_msg.data[2] = mydata->N_Neighbors; //  number of neighbors
        mydata->transmit_msg.data[3] = (get_bot_state()<<4) | (get_move_type() & 0x0f);     //  bot state, bot type

        mydata->transmit_msg.data[4] = mydata->gradient_value;// gradient value
        //mydata->transmit_msg.data[5] = mydata->angle_8;
        mydata->transmit_msg.data[6] = mydata->local_ID;

        //mydata->transmit_msg.data[2] = mydata->angle; // angle


        mydata->transmit_msg.data[7] = mydata->x_16 & 0xff;     // low x
        mydata->transmit_msg.data[8] = mydata->x_16 >> 8;       // high x
        mydata->transmit_msg.data[9] = mydata->y_16 & 0xff;     // low  y
        mydata->transmit_msg.data[10] = mydata->y_16 >> 8;       // high y

        mydata->transmit_msg.data[11] = find_nearest_N_ID() & 0xff;     // 0 low  ID
        mydata->transmit_msg.data[12] = find_nearest_N_ID() >> 8;  

        mydata->transmit_msg.data[13] = mydata->N_passing;
        mydata->transmit_msg.data[14] = get_bot_type();
        mydata->transmit_msg.data[15] = mydata->localized;

        //Data for dismiss
        //mydata->transmit_msg.data[16] = mydata->gradient_value_2;// gradient value
        mydata->transmit_msg.data[17] = mydata->on_ribbon_ID;
        mydata->transmit_msg.data[18] = mydata->ribbon_ID;
        mydata->transmit_msg.data[19] = mydata->n_bot_not_in_shape;
        mydata->transmit_msg.data[20] = mydata->n_bot_not_in_shape_ahead;
        mydata->transmit_msg.data[21] = mydata->n_bot_need_retain;
        mydata->transmit_msg.data[22] = mydata->ribbon_complete;
        mydata->transmit_msg.data[23] = mydata->r_bot_type;
        mydata->transmit_msg.data[24] = mydata->ribbon_my_turn;
    
        //edge-following info
        mydata->transmit_msg.data[25] = mydata->N_followed;

        mydata->transmit_msg.data[29] = mydata->has_movement_intent;
        mydata->transmit_msg.data[30] = (int16_t)mydata->intended_target_q  & 0xff;
        mydata->transmit_msg.data[31] = (int16_t)mydata->intended_target_q >> 8;
        mydata->transmit_msg.data[32] = (int16_t)mydata->intended_target_r  & 0xff;
        mydata->transmit_msg.data[33] = (int16_t)mydata->intended_target_r >> 8;


        mydata->transmit_msg.data[38] = (int16_t) mydata->hex_q  & 0xff;     // low 
        mydata->transmit_msg.data[39] = (int16_t) mydata->hex_q >> 8;       // high 
        mydata->transmit_msg.data[40] = (int16_t) mydata->hex_r & 0xff;     // low 
        mydata->transmit_msg.data[41] = (int16_t) mydata->hex_r >> 8;       // high 

        mydata->transmit_msg.data[44] = mydata->shape_position_occupied;

     
        //2 bytes for message crc
        mydata->transmit_msg.crc = message_crc(&mydata->transmit_msg);

    mydata->message_lock = 0;
}

void receive_inputs()
{
    while (!RB_empty())
    {
        process_message();
        RB_popfront();
    }
    
}


////////////////////////////////////robot initialization/////////////////////////////////////////////

void setup()
{
    rand_seed(kilo_uid + 1); //seed the random number generator
    
    mydata->message_lock = 0;
    
    mydata->local_ID_generated = 0;
#if 1
    struct Hex cur_hex = cart_to_hex((struct Cartesian){kilo_x, kilo_y});
    mydata->hex_q = cur_hex.q;
    mydata->hex_r = cur_hex.r;
    mydata->original_position = (struct Hex){mydata->hex_q,mydata->hex_r};
    mydata->shape_position_occupied = 0;
#endif
    mydata->gradient_value = UINT8_MAX-1;
    //mydata->gradient_value_2 = UINT8_MAX-1;

    mydata->edge_followee_id = 0;

    mydata->N_Neighbors = 0;
    mydata->N_passing = 0;
    mydata->message_id = 0;
    set_move_type(STOP);
    set_bot_state(IDLE);
    set_bot_type(NOR);
    mydata->N_followed = 0;

   //To determine if the kilobot is stucked
    mydata->previous_x = 0;
    mydata->previous_y = 0;
    mydata->local_tick_stuck = kilo_ticks;
    
    //Edge follow setting
    mydata->edge_follow_radius_near = kilo_lattice_size;
    mydata->edge_follow_radius_far = kilo_lattice_size;

    
    //initialize the gradient
    if (kilo_uid == 0)
        mydata->gradient_value = 0;

    //initialize localization
    #if (LOCALIZATION)

        mydata->x = kilo_x;
        mydata->y = kilo_y;
        mydata->localized = 1;
        mydata->localize_cycle = 0;
        mydata->localizable = 0;
    #else
        mydata->localizable = 1;
    #endif 

    //non-localizable
    mydata->relocalizable_ticks = 0;

    //setup ribbon
    mydata->ribbon_ID = 0;
    mydata->on_ribbon_ID = 0;
    mydata->n_bot_not_in_shape = 0; //number of kilobot on the ribbon which are not in the shape
    mydata->n_bot_not_in_shape_ahead = 0; //the number of kilobot on the ribbon before me is not in the shape
    mydata->n_bot_need_retain = UINT8_MAX;
    mydata->ribbon_complete = 0;
    mydata->ribbon_my_turn = 0;

    //setup idle ribbons
    mydata->idle_ribbon_my_turn = 0;

    //Broadcast of moving robot
    mydata->exists_moving_robot = 0;
    mydata->exists_moving_robot_tick = 0;

    //Move in state
    mydata->stop_out_flag = false;


    //Stage 1 completion check
    mydata->stage1complete = 0;
    mydata->initialPos.q = 0;
    mydata->initialPos.r = 0;

    //move to next lattice point
    mydata->currentPos.q = 0;
    mydata->currentPos.r = 0;
    mydata->move_to_next_lattice_same_ribbon_flag = 0;
    mydata->ribbon_com_tick = 0;
    mydata->moved_within_step = 0;


    //Stage 2
    mydata->re_stop_out_flag = false;
    mydata->sum_of_bots_in_hole = 0;
    mydata->sum_of_bots_in_hole_above = 0;
    for(int i = 0; i < 2; i++)
    {
        mydata->sum_of_bots_ID[i] = 0;
    }

    mydata->reassemble_bot_ID = 0;

    mydata->system_step = 0;

    mydata->faulty_type_tick = 0;
    mydata->faulty_type = NIL;


    ////////////////Read hex boundary from hex_data.txt////////////////////////
    mydata->lattice_s = NULL;
    mydata->lattice_t = NULL; 
    mydata->lattice_shape = NULL;
    mydata->lattice_s_size = 0;
    mydata->lattice_t_size = 0;
    mydata->lattice_shape_size = 0;

    // Call the function to read the groups from the file
    read_boundary_from_file("shape_data.txt",
         &mydata->lattice_shape, &mydata->lattice_shape_size,
          &mydata->lattice_s, &mydata->lattice_s_size,
           &mydata->lattice_t, &mydata->lattice_t_size,
            &mydata->recycleFlagSeq);

    //printf("\nSequence size %d:\n", mydata->lattice_s_size);
    int a = (int)mydata->lattice_s_size * 0.5;
    #if 0
    for (int i = 0; i < mydata->lattice_s_size; i++) { 
        printf("Hex %d: (q: %d, r: %d) (q: %d, r: %d) %d\n", i, mydata->lattice_s[i].q, mydata->lattice_s[i].r, mydata->lattice_t[i].q, mydata->lattice_t[i].r, mydata->recycleFlagSeq[i]);
    }
#endif
#if 1
 // 初始化当前机器人的六边形坐标
            mydata->hex_q = cur_hex.q;
            mydata->hex_r = cur_hex.r;
    mydata->shape_occupancy = (uint8_t*)malloc(mydata->lattice_shape_size * sizeof(uint8_t));
    for (int i = 0; i < mydata->lattice_shape_size; i++) {
        // 1. 初始化 shape_occupancy 数组
        mydata->shape_occupancy[i] = 0;

        // 2. 初始化 is_occupied 全局数组（如果存在）
        int q = mydata->lattice_shape[i].q;
        int r = mydata->lattice_shape[i].r;
        
    }
#endif

    // 初始化移动协调字段
    mydata->claim_chance = 0;
    mydata->has_movement_intent = 0;
    mydata->intended_target_q =  99;
    mydata->intended_target_r =  99;

    init_move_history();
    
    setup_message();
}


//////////////////////////////////////////UI display/////////////////////////////////////////

uint8_t colorNum[] = {
    RGB(0,0,0),  //0 - off
    RGB(1,0,0),  //1 - red
    RGB(0,1,0),  //2 - green
    RGB(0,0,1),  //3 - blue
    RGB(1,1,0),  //4 - yellow
    RGB(0,1,1),  //5 - cyan
    RGB(1,0,1),  //6 - purple
    RGB(2,1,0),  //7  - orange
    RGB(1,1,1),  //8  - white
    RGB(3,3,3),   //9  - bright white
    RGB(3,0,0),  //10 - error red
};

extern char* (*callback_botinfo) (void);
char *botinfo(void);

//json_t *json_state();


#ifdef SIMULATOR
// provide a text string for the status bar, about this bot
static char botinfo_buffer[10000];
char *botinfo(void)
{
    int i;
    char *p = botinfo_buffer;
    

    p += sprintf (p, "ID: %d \n", kilo_uid);
    p += sprintf (p, "Local ID: %d \n", mydata->local_ID);
    p += sprintf (p, "Stage 1 Complete: %d \n", mydata->stage1complete);
    p += sprintf (p, "Have Moving robot? : %d \n", mydata->exists_moving_robot); 
    p += sprintf (p, "tick : %d \n", mydata->exists_moving_robot_tick);
    p += sprintf (p, "Have child ribbon? : %d \n", mydata->exists_child_idle_ribbon);
    p += sprintf (p, "Systemstep: %d %d\n", mydata->system_step,mydata->lattice_s_size);
    p += sprintf (p, "Inshape: %d \n", am_in_shape());
    p += sprintf (p, "Distance to boundary : %f \n", dist_to_boundary(kilo_x, kilo_y)); 


    
    p += sprintf (p,"------End Detect------\n");

    p += sprintf (p,"Robots in hole: %d\n", mydata->num_bots_in_hole);

    //move type
    switch (get_move_type())
    {
        case STOP: p += sprintf (p, "Move type: Stop \n"); break;
        case EDGE: p += sprintf (p, "Move type: Edge \n"); break;
        case RIBBON: p += sprintf (p, "Move type: Ribbon \n"); break;
        case SHUFFLE_MOVE: p += sprintf (p, "Move type: Shuffle move \n"); break;
        case DETACH: p += sprintf (p, "Move type: Detached \n"); break;
        case STUCK: p += sprintf (p, "Move type: Stuck \n"); break;
        case PAUSE: p += sprintf (p, "Move type: Pause \n"); break;
        default: p += sprintf (p, "Move type: error \n"); break;
    }

    //bot state
    switch (get_bot_state())
    {
        
        case IDLE: p += sprintf (p, "Bot state: Idle \n"); break;
        case MOVE_OUT: p += sprintf (p, "Bot state: Move Out \n"); break;
        case MOVE_IN: p += sprintf (p, "Bot state: Move In \n"); break;
        case STOP_OUT: p += sprintf (p, "Bot state: Stop out \n"); break;
        case STOP_IN: p += sprintf (p, "Bot state: Stop in \n"); break;
        case STOP_OUT_AGAIN: p += sprintf (p, "Bot state: Stop out again \n"); break;
        case STOP_IN_AGAIN: p += sprintf (p, "Bot state: Stop in again \n"); break;
        case JOIN_BACK_IDLE: p += sprintf (p, "Bot state: Join back idle \n"); break;
        case SHUFFLE: p += sprintf (p, "Bot state: Shuffle \n"); break;
        case LOC_ERROR: p += sprintf (p, "Bot state: localization error \n"); break;
        //case RIBBON: p += sprintf (p, "Bot state: Ribbon \n"); break;
        case RIBBON_ACT: p += sprintf (p, "Bot state: Ribbon_active \n"); break;
        case REASSEMBLY_OUT: p += sprintf (p, "Bot state: Reassembly out \n"); break;
        case REASSEMBLY_IN: p += sprintf (p, "Bot state: Reassembly in \n"); break;
        case FAULTY: p += sprintf (p, "Bot state: Faulty in \n"); break;
        default: p += sprintf (p, "Bot state: error code %d \n", get_bot_state()); break;
    }

    //bot type
    switch (get_bot_type())
    {
        case BASE: p += sprintf (p, "Bot type: Base \n"); break;
        case NOR: p += sprintf (p, "Bot type: Normal \n"); break;
        case RIBBON_LEADER: p += sprintf (p, "Bot type: Ribbon leader \n"); break;
        case RIBBON_SHUFFLE_ENDER: p += sprintf (p, "Bot type: Ribbon shuffle ender \n"); break;
        default: p += sprintf (p, "Bot state: error \n"); break;
    }

    
    switch (mydata->faulty_type)
    {
        case NIL: p += sprintf (p, "Fault type: NIL \n"); break;
        case FOUT: p += sprintf (p, "Fault type: F_OUT \n"); break;
        case FIN: p += sprintf (p, "Fault type: F_IN \n"); break;
        case FSUB: p += sprintf (p, "Fault type: F_SUB \n"); break;
        default: p += sprintf (p, "faulty_type: error \n"); break;
    }
    
    p += sprintf (p, "Gradient: %d ", mydata->gradient_value);
    //p += sprintf (p, "Gradient_2: %d\n", mydata->gradient_value_2);

    p += sprintf (p, "\n");

    p += sprintf (p, "Localizable: %d \n", mydata->localizable);
    p += sprintf (p, "Localized: %d \n", mydata->localized);

    p += sprintf (p, "x: %.2f , y: %.2f\n", kilo_x, kilo_y);
     p += sprintf (p, "Dir: %.1f, Target dir: %.1f\n", kilo_direction, mydata->target_direction);
    p += sprintf (p, "Ex: %.2f , Ey: %.2f\n", mydata->x, mydata->y);
    p += sprintf (p, "Nearest Hex: q=%d , r=%d\n", mydata->hex_q, mydata->hex_r);

    p += sprintf (p, "\n");
    p += sprintf (p, "Prex: %.2f , Prey: %.2f \n", mydata->previous_x, mydata->previous_y);
    p += sprintf (p, "Stuck_counter: %d\n", kilo_ticks -mydata->local_tick_stuck);

    p += sprintf (p, "\n--------------------------\n");
    p += sprintf (p, "Idle Ribbon %d, ON_ribbon_ID: %d \n", mydata->idle_ribbon_ID, mydata->idle_on_ribbon_ID);
    p += sprintf (p, "Is tail: %d\n", am_at_tail());
    p += sprintf (p, "My turn: %d\n", mydata->idle_ribbon_my_turn);
#if 0
    p += sprintf (p, "\n--------------------------\n");
    p += sprintf (p, "Ribbon %d, ON_ribbon_ID: %d \n", mydata->ribbon_ID, mydata->on_ribbon_ID);
    p += sprintf (p, "ribbon_complete %d\n", mydata->ribbon_complete);
    p += sprintf (p, "N_not_in %d, N_not_in_ahead: %d \n", mydata->n_bot_not_in_shape, mydata->n_bot_not_in_shape_ahead);
#endif   
    //ribbon bot type
    switch (mydata->r_bot_type)
    {
    case RECYCLE: p += sprintf (p, "Ribbon bot type: Recycle \n"); break;
    case RETAIN: p += sprintf (p, "Ribbon bot type: Retain \n"); break;
    case INACTIVE: p += sprintf (p, "Ribbon bot type: Inactive \n"); break;
    default: p += sprintf (p, "Bot state: error \n"); break;
    }   

    p += sprintf(p, "ribbon_my_turn: %d\n", mydata->ribbon_my_turn);


    p += sprintf (p, "\n-----------Move Out-----------\n");
    p += sprintf(p, "sum_of_bots_in_hole: %d\n", mydata->sum_of_bots_in_hole);


    p += sprintf (p, "\n-----------Ribbon-----------\n");
    p += sprintf(p, "sum_of_bots_in_hole_above: %d\n", mydata->sum_of_bots_in_hole_above);

    //p += sprintf (p, "Sumed_ID: %d %d",  mydata->sum_of_bots_ID[0], mydata->sum_of_bots_ID[1]);
    p += sprintf (p, "Moved: %d \n",  mydata->moved_within_step);
    p += sprintf (p, "currentPos: %d %d\n", mydata->currentPos.q, mydata->currentPos.r);
    p += sprintf (p, "\n--------------------------\n");
    p += sprintf (p, "Followee: %d ", mydata->edge_followee_id);
    p += sprintf (p, "dist: %.2f \n", get_dist_by_ID(mydata->edge_followee_id));
    p += sprintf (p, "Edge Following Radius: (%d, %d) \n", mydata->edge_follow_radius_near, mydata->edge_follow_radius_far);
    
    p += sprintf (p, "\n");
    p += sprintf (p, "Neighbor: %d, nearest dist: %d, \n", mydata->N_Neighbors, find_nearest_N_dist(0));
    p += sprintf(p, "Neighbor: \n");
    
    for (i = 0; i < mydata->N_Neighbors; i++)
    {
        p += sprintf (p, "        ID:%d ", mydata->neighbors[i].ID);
        
        //p += sprintf (p, "LOC %d ", mydata->neighbors[i].localized);
        //p += sprintf (p, "(%.2f ,%.2f)", mydata->neighbors[i].x, mydata->neighbors[i].y);
        //p += sprintf (p, "dist %d ", mydata->neighbors[i].dist);
        p += sprintf (p, "prec_dist:%.2f ", mydata->neighbors[i].precise_dist);
        p += sprintf (p, "(%d %d)\n", mydata->neighbors[i].hex_q,mydata->neighbors[i].hex_r);
        //p += sprintf (p, "N_passing: %d", mydata->neighbors[i].N_passing);
        //p += sprintf (p, "State: %d", mydata->neighbors[i].n_bot_type);
        //p += sprintf (p, "followee:%d ;", mydata->neighbors[i].edge_followee_id);

        
    }
    
    p += sprintf (p, "\n");
#if 0
    p += sprintf(p, "Passing: ");
    p += sprintf (p, "N_passing: %d \n", mydata->N_passing);
    p += sprintf (p, "N_passing_distinct: %d \n", countDistinct(mydata->passing_by_ID, mydata->N_passing));
    p += sprintf (p, "N_followed: %d \n", mydata->N_followed);
#endif
    for (i = 0; i < mydata->N_passing; i++)
    {
        p += sprintf (p, "ID: %d ", mydata->passing_by_ID[i]);
        
        p += sprintf (p, "timestamp: %d \n", mydata->timestamp[i]);
        //p += sprintf (p, "In: %d \n", mydata->is_move_in[i]);
        
    }
    
    p += sprintf (p, "passingby_gradient: %d\n", mydata->N_passing_gradient);
    for (i = 0; i < 2; i++)
    {
        p += sprintf (p, "ID: %d \n", mydata->passingby_ID_gradient[i]);
        
    }

    p += sprintf (p, "movement_intent: %d\n", mydata->has_movement_intent);
    p += sprintf (p, "intended_target_q: %d\n", mydata->intended_target_q);
    p += sprintf (p, "intended_target_r: %d\n", mydata->intended_target_r);
    // struct Hex point1 = {mydata->hex_q, mydata->hex_r};
    // struct Hex point2 = {5,5};
    


    return botinfo_buffer;
}
#endif



/////////////////////////////////////////The main loop////////////////////////////////////
void loop()
{
    //mydata->localized = 0;
    // remove neighbors in the memory that is older than 2s
    purgeNeighbors();
    
    //receive messages
    receive_inputs();
/*
    mydata->x = kilo_x;
    mydata->y = kilo_y;
*/
    //Update nearest distance
    mydata->dist = get_dist_by_ID(mydata->edge_followee_id); 
    
    // Initialize reusable counter
    uint8_t i;
    
   //generate temperary local ID
    if ((mydata->local_ID_generated) == 0)
    {
        mydata->local_ID = rand()%256;
        mydata->local_ID_generated = 1;
    }
    else
    {
        for (int i = 0; i < mydata->N_Neighbors; i++)
        {
            if (mydata->local_ID == mydata->neighbors[i].local_ID)
                mydata->local_ID_generated =0;
        }
    }
/*这个不能注释！更新位置*/  
   //Update x, y use localization
    #if (LOCALIZATION)
    #if 0
        if(!is_position_in_shape((struct Hex){mydata->hex_q,mydata->hex_r})){
            mydata->x = kilo_x;
            mydata->y = kilo_y;
            struct Hex cur_hex = cart_to_hex((struct Cartesian){mydata->x, mydata->y});
                    mydata->hex_q = cur_hex.q;
                    mydata->hex_r = cur_hex.r;
        }else{
            mydata->localizable =  check_localizablitiy();

        // run localization only if there are at least three noncolinear neighbors
            if(mydata->localizable == 1) 
            {

                //non-stop localization
                if (mydata->localized == 0)
                {
                    global_localization();
                    #if 0
                    struct Hex cur_hex = cart_to_hex((struct Cartesian){mydata->x, mydata->y});
                    mydata->hex_q = cur_hex.q;
                    mydata->hex_r = cur_hex.r;
                    #endif
                }


            }
        }

    #else
         mydata->localizable =  check_localizablitiy();

        // run localization only if there are at least three noncolinear neighbors
            if(mydata->localizable == 1) 
            {

                //non-stop localization
                if (mydata->localized == 0)
                {
                    global_localization();
                    #if 0
                    struct Hex cur_hex = cart_to_hex((struct Cartesian){mydata->x, mydata->y});
                    mydata->hex_q = cur_hex.q;
                    mydata->hex_r = cur_hex.r;
                    #endif
                }


            }
    #endif
    #else
        mydata->x = kilo_x;
        mydata->y = kilo_y;

    #endif


    mydata->x_16 = (uint16_t)((mydata->x + x_range) / (2 * x_range) * 65535);
    mydata->y_16 = (uint16_t)((mydata->y + y_range) / (2 * y_range) * 65535);


    //set color for each robot

    set_color(colorNum[kilo_uid % 9 + 1]);

        
    
    switch(get_bot_state()) {
        case IDLE:    
            set_bot_state(FIND_SHAPE_POSITION);
            break;
        case FIND_SHAPE_POSITION:
            findShapePosition();
            if(mydata->claim_chance < 10){
                mydata->claim_chance += 1;
            }else{
                mydata->claim_chance = 0;
                if(mydata->intended_target_q !=99 && mydata->intended_target_r != 99){
                    set_bot_state(CLAIM_TARGET);
                }else{
                    set_bot_state(IDLE);
                }
            }
            break;
        case CLAIM_TARGET:
            claimTarget();
            if(mydata->claim_chance < 10){
                mydata->claim_chance += 1;
            }else{
                mydata->claim_chance = 0;
                set_bot_state(SOLVE_CONFLICT);
            }
            break;
        case SOLVE_CONFLICT:
            solveconflict();
            if(mydata->claim_chance < 10){
                mydata->claim_chance += 1;
            }else{
                mydata->claim_chance = 0;
                set_bot_state(MOVE_TO_SHAPE);
            }
            break;
        case MOVE_TO_SHAPE:
#if 0
            if(mydata->message_lock == 1){
                break;
            }
#endif
            moveToShape();
            break;
    }

  
    //send message
    setup_message();

}

//////////////////main/////////////////////

int main(void)
{

    // initialize hardware
    kilo_init();
    
    // initialize ring buffer
    RB_init();
    
    // register message callbacks
    kilo_message_rx = rxbuffer_push;
    kilo_message_tx = message_tx;
    
    // register your program
    kilo_start(setup, loop);
    
    SET_CALLBACK(botinfo, botinfo);
    SET_CALLBACK(reset, setup);
    //SET_CALLBACK(json_state, json_state);


    
    
#ifdef DEBUG
    // setup debugging, i.e. printf to serial port, in real Kilobot
    debug_init();
#endif
    
    // SET_CALLBACK(botinfo, botinfo);
    // SET_CALLBACK(reset, setup);
    
    // RB_init();                       // initialize ring buffer
    // kilo_message_rx = rxbuffer_push;
    // kilo_message_tx = message_tx;    // register our transmission function
    
    // kilo_start(setup, loop);
    
    return 0;
}
