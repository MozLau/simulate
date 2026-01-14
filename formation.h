#ifndef GRN_H
#define GRN_H


#include <math.h>
#include <stdint.h>
#include <kilombo.h>



#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884197169399375105820974944
#endif


#define GENES 6

#define DIFFGENES 6  // Maximal number of diffusing genes, limited by the message length.

#define MAXN 30
#define RB_SIZE 16  // Ring buffer size. Choose a power of two for faster code
// memory usage: 16*RB_SIZE
// 8 works too, but complains in the simulator
// when the bots are very dense
#define x_range 5000 //x in [-x_range, x_range]
#define X_ZERO 32767
#define y_range 5000
#define Y_ZERO 32767


/////////////////////////////////
#define OMNI_SPEED 7    //speed

#define PRECISE_LOC 1 //1 for using precise distance

#define NUM_OF_LOC_ROUNDS 10
#define LOCALIZATIONTOL 10  //start from 10mm to the nearest lattice point, trigger move_to_nearest_lattice_point
#define MOVE_TO_NEAREST_LATTICE_ERROR 0.5

#define GRADIEANT_SETTLED 200
#define IDLE_RIBBON_SETTLED 400
#define WARMUP 500 //time for warmup

////////////////////////////////////////////////////////////////
//#define LATTICE_SIZE 108 //adjust in kilombo.json

extern uint8_t NGenes;

enum RIBBON_BOT_TYPE {RECYCLE, RETAIN, INACTIVE};

enum BOTTYPE {BASE, NOR, ANCHOR,BOUNDARY_IN,BOUNDARY_OUT,RIBBON_LEADER,RIBBON_SHUFFLE_ENDER};
//enum BOTSTATES {WAIT, LISTEN, MOVE};

enum MOVE_TYPE{STOP,EDGE, RIBBON, STABILIZE, SHUFFLE_MOVE , DETACH, STUCK, AVOID, LEFT,RIGHT,STRAIGHT,PAUSE};
// enum BOTSTATE {IDLE,MOVE_OUT,MOVE_IN,STOP_IN, STOP_OUT, MOVE_DELETED_IN, STOP_DELETED_IN, STOP_DELETED_OUT, STOP_OUT_INNER,LOC_ERROR,DISMISS};
#if 0
enum BOTSTATE {IDLE,MOVE_OUT,MOVE_IN,STOP_IN, STOP_OUT, STOP_IN_AGAIN, STOP_OUT_AGAIN, JOIN_BACK_IDLE, SHUFFLE,LOC_ERROR, RIBBON_ACT, REASSEMBLY_OUT,REASSEMBLY_IN, FAULTY};
#else
enum BOTSTATE {
    IDLE, MOVE_OUT, MOVE_IN, STOP_IN, STOP_OUT, STOP_IN_AGAIN, STOP_OUT_AGAIN, 
    JOIN_BACK_IDLE, SHUFFLE, LOC_ERROR, RIBBON_ACT, REASSEMBLY_OUT, REASSEMBLY_IN, 
    FAULTY, MOVE_TO_TARGET,
    // 新增状态
    FIND_SHAPE_POSITION,    // 寻找形状位置
    WAITTING,
    MOVE_TO_SHAPE,          // 向形状移动
    PLAN_MOVEMENT,  
};
#endif
enum FAULTTYPE {NIL, FOUT, FIN, FSUB};

#define LOCALIZATION 1
#define SHOW_LOCALIZED 0
#define USE_DISMISS 1
#define SHOW_INNER_BOUNDARY 0
#define EPOCH 1
#define N_PASSING_MAX 16


//#define EDGE_FOLLOW_RADIUS 54

//#define EDGE_FOLLOW_RADIUS 54
//#define EDGE_FOLLOW_RADIUS_INNER 55

#define WAKEUP_TIME_DELAY 1000





// struct Hex {
//     int q; // Axial coordinate q
//     int r; // Axial coordinate r
// };

struct Cartesian {
    float x; // Cartesian x-coordinate
    float y; // Cartesian y-coordinate
};

struct RibbonHead {
    int q; // Axial coordinate q
    int r; // Axial coordinate r
    int t; // Total number of lattice points of the ribbon
    int h; // Number of lattice points not in the ribbon
};

typedef struct {

    uint16_t ID;
    uint8_t dist;
    double precise_dist;
    //  int delta_dist;
    uint8_t gradient;
    //uint8_t gradient2; // gradient used for inner boundary
    uint8_t n_bot_state;
    uint8_t n_bot_move;
    uint8_t n_bot_type;
    uint8_t r_bot_type; //ribbon bot type
    uint8_t localized; //0 for localzating, 1 for localizated, 2 for stopped but unlocalized

    uint16_t edge_followee_id; //the id of the kilobot being followed about

    uint8_t N_Neighbors;
    uint32_t timestamp; //last seen timestamp
    uint8_t local_ID;
    
    uint16_t x_16;
    uint16_t y_16;
    double x;
    double y;

    uint8_t angle_8;
    uint8_t gradient_value;
    
    uint8_t receive_lock; //0 if receive completed
    uint8_t received_message_id; //the id of the current message

    //ribbon
    uint8_t ribbon_ID;
    uint8_t on_ribbon_ID;
    uint8_t ribbon_complete;
    uint8_t n_bot_not_in_shape; //number of kilobot on the ribbon which are not in the shape
    uint8_t not_in_shape_checked; //the kilobot has been counted for n_not_in_shape
    uint8_t n_bot_not_in_shape_ahead; //the number of kilobot on the ribbon before me is not in the shape + 1
    uint8_t n_bot_need_retain;
    uint8_t ribbon_my_turn;  // 1 if the ribbon this the one to shuffle now

    //idle ribbon
    uint8_t idle_ribbon_ID;
    uint8_t idle_on_ribbon_ID; 
    uint8_t idle_ribbon_my_turn; // 1 if the idle ribbon this the one to move

    //hex
    int16_t hex_q;
    int16_t hex_r;

    //determine whether starts to move
    uint8_t exists_moving_robot;
    uint32_t exists_moving_robot_tick; //the kilo tick for the lastest update
    uint8_t exists_child_idle_ribbon;
    
    //For STAGE 1
    uint8_t stage1complete;  //1 if true
    uint16_t sum_of_bots_in_hole;
    uint16_t sum_of_bots_ID; // the ribbon leader has just been recorded by the move out robot

    int N_passing;
    
    uint8_t N_followed;

    uint16_t system_step;

    //Faulty message broadcast
    uint8_t faulty_type;
    uint32_t faulty_type_tick; //the kilo tick for the lastest update

    uint8_t shape_position_occupied;
    
    uint8_t is_moving;
    uint8_t has_movement_intent;
    int16_t intended_target_q;
    int16_t intended_target_r;
    uint32_t target_intent_time;        // 目标意图时间
    uint8_t movement_priority;

} Neighbor_t;


typedef struct {
    message_t msg;
    distance_measurement_t dist;
} received_message_t;

typedef struct {
    struct Hex move_history[5];
    int history_index;
    bool history_initialized;
} MoveHistoryTracker;

typedef struct
{
    Neighbor_t neighbors[MAXN];
    
    int N_Neighbors;
    uint8_t dist; //nearest distance to the stationary kilobot
    uint8_t bot_type;
    uint8_t r_bot_type; //ribbon bot type
    uint8_t bot_state;
    uint8_t move_type;
    uint8_t edge_follow_radius_near;
    uint8_t edge_follow_radius_far;


    uint16_t x_16; //in the range [-x_range, x_range]
    double x;
    uint16_t y_16;
    double y;
    uint8_t angle_8;
    
    uint8_t gradient_value;
    uint16_t edge_followee_id; //the id of the kilobot being followed about

    //For non-localizable edge-following
    double target_direction;
    uint32_t relocalizable_ticks; //if the robot is localizable again, stop the bot to do localization for 100 ticks
    
    //Local id
    char local_ID_generated; //0 for false, 1 for true
    uint8_t local_ID;
    
    //Message
    message_t transmit_msg;
    uint8_t message_id;
    char message_lock;
    
    received_message_t RXBuffer[RB_SIZE];
    uint8_t RXHead, RXTail;
    
    //Time delay
    uint32_t local_tick; // Delay move type changing
    uint32_t stop_delay_local_tick; // Delay after enter the shape before stop
    uint32_t move_in_delay_local_tick; // Delay before change to MOVE_IN

    uint32_t outside_delay_local_tick;
    uint32_t inside_delay_local_tick;
    uint32_t dismiss_start_delay_local_tick; //delay to transit to shuffle state

    uint32_t pause_local_tick; //kilobot in PAUSE move type move again
    
    //Stuck check
    uint32_t local_tick_stuck; //Timestep for stuck check
    double previous_x;
    double previous_y;
    
    
    //Front identification
    uint16_t passing_by_ID[N_PASSING_MAX]; //To decide which kilobot is in the front
    uint32_t timestamp[N_PASSING_MAX];
    int N_passing;

    //Gradient (MOVE-IN to STOP-IN condition)
    uint16_t passingby_ID_gradient[2];
    int N_passing_gradient;

    uint8_t N_followed;
    
    //localization
    uint8_t localized; //1 for localized
    uint16_t localize_cycle; //count the iteration for greedy search 
    uint8_t localizable; //1 if there are three localized robots in communication range

    //Ribbon
    uint8_t ribbon_ID;
    uint8_t on_ribbon_ID;
    uint8_t ribbon_complete;
    uint8_t n_bot_not_in_shape; //number of kilobot on the ribbon which are not in the shape
    uint8_t not_in_shape_checked; //the kilobot has been counted for n_not_in_shape
    uint8_t n_bot_not_in_shape_ahead; //the number of kilobot on the ribbon before me is not in the shape + 1
    uint8_t n_bot_need_retain;
    uint8_t ribbon_my_turn; // 1 if the ribbon this the one to shuffle now

    //Hex<--->Cartesian
    int hex_q;
    int hex_r;
    double cart_x;
    double cart_y;

    //read from txt
    struct Hex *lattice_s;
    int lattice_s_size;
    struct Hex *lattice_t;
    int lattice_t_size;
    int *recycleFlagSeq; //1 for stop in H1; 2 for stop in H2; 0 for non-applicable
    struct Hex *lattice_shape;
    int lattice_shape_size;

    //SuEA from txt
    struct Hex *path;
    int *path_length;
    int total_number_of_path;
    int path_point_index;
    
    
    //Ribbon from txt
    struct RibbonHead *ribbon_heads;
    int num_of_ribbon_heads;

    //idle ribbon
    uint8_t idle_ribbon_ID;
    uint8_t idle_on_ribbon_ID; 
    uint8_t idle_ribbon_my_turn; // 1 if the idle ribbon this the one to move

    //determine whether starts to move from idle state
    uint8_t exists_moving_robot;
    uint32_t exists_moving_robot_tick; //the kilo tick for the lastest update
    uint8_t exists_child_idle_ribbon;
    uint32_t next_system_step_tick;

    //For MOVE IN state
    bool stop_out_flag;  //true if the robot moves out of the shape after entering the shape 

    //For STAGE 1
    struct Hex initialPos;
    uint8_t stage1complete;  //1 if true

    //For ribbon shuffle
    struct Hex currentPos;
    int move_to_next_lattice_same_ribbon_flag;
    uint32_t ribbon_com_tick;
    int moved_within_step;

    //For stage 2
    bool re_stop_out_flag;
    uint16_t sum_of_bots_in_hole;
    uint16_t sum_of_bots_ID[2];  //for the move out robot, record the ribbon leader ID
    uint16_t sum_of_bots_in_hole_above;

    uint16_t reassemble_bot_ID; //for the ribbon leader, record the reassemble robot ID

    //End detection
    int num_bots_in_hole;
    int passed_bot_ID;

    //System step
    uint16_t system_step;

    //Faulty message broadcast
    uint8_t faulty_type;
    uint32_t faulty_type_tick; //the kilo tick for the lastest update

    //SuEA
    int target_q;
    int target_r;

#if 1 
    uint8_t *shape_occupancy;           // 形状位置占用状态数组
    uint8_t shape_position_occupied;    // 是否占据形状
    uint8_t relocation_occupied;        // 是否已经补位
    struct Hex o_original_position;
    struct Hex original_position;
    uint8_t target_shape_index;
    MoveHistoryTracker move_tracker;
    uint32_t relocation_start_time;  // 开始补位的时间戳
    uint32_t total_relocation_time;  // 累计补位时间

#endif

#if 1
    uint8_t is_moving;                    // 是否正在移动
    uint8_t movement_priority;           // 移动优先级（可用于解决冲突）
    uint32_t movement_start_time;        // 移动开始时间
    uint8_t has_movement_intent;         // 是否有移动意图
    int16_t intended_target_q;
    int16_t intended_target_r;          // 意图移动的目标位置
    uint32_t target_intent_time;        // 目标意图时间


    uint8_t should_follow_leader;       // 是否应该跟随领导者
    uint32_t last_leader_update;        // 上次收到领导者位置更新的时间
    int16_t leader_target_position_q;  // 领导者的目标位置
    int16_t leader_target_position_r;  // 领导者的目标位置
    int16_t leader_current_position_q; // 领导者当前位置
    int16_t leader_current_position_r; // 领导者当前位置
    uint8_t leader_is_moving;           // 领导者是否正在移动


#endif

} MyUserdata;

#if 0
REGISTER_USERDATA(MyUserdata);

#else
extern MyUserdata *mydata;
extern int UserdataSize;
#endif



// Ring buffer operations. Taken from kilolib's ringbuffer.h
// but adapted for use with mydata->

// Ring buffer operations indexed with head, tail
// These waste one entry in the buffer, but are interrupt safe:
//   * head is changed only in popfront
//   * tail is changed only in pushback
//   * RB_popfront() is to be called AFTER the data in RB_front() has been used
//   * head and tail indices are uint8_t, which can be updated atomically
//     - still, the updates need to be atomic, especially in RB_popfront()

#define RB_init() {	\
mydata->RXHead = 0; \
mydata->RXTail = 0;\
}

#define RB_empty() (mydata->RXHead == mydata->RXTail)

#define RB_full()  ((mydata->RXHead+1)%RB_SIZE == mydata->RXTail)

#define RB_front() mydata->RXBuffer[mydata->RXHead]

#define RB_back() mydata->RXBuffer[mydata->RXTail]

#define RB_popfront() mydata->RXHead = (mydata->RXHead+1)%RB_SIZE;

#define RB_pushback() {\
mydata->RXTail = (mydata->RXTail+1)%RB_SIZE;\
if (RB_empty())\
{ mydata->RXHead = (mydata->RXHead+1)%RB_SIZE;	\
printf("Full.\n"); }				\
}


/*
 // Ring buffer operations indexed with head, count
 // These save one entry in the buffer, but are less interrupt safe due to the shared count
 
 #define RB_init() {	\
 mydata->RXHead = 0; \
 mydata->RXCount = 0;\
 }
 #define RB_empty() (mydata->RXCount == 0)
 #define RB_full()  (mydata->RXCount == RB_SIZE)
 #define RB_front() (mydata->RXBuffer[mydata->RXHead])
 #define RB_back()  (mydata->RXBuffer[(mydata->RXHead+mydata->RXCount)%RB_SIZE])
 #define RB_popfront() {\
 mydata->RXHead = (mydata->RXHead+1)%RB_SIZE;\
 mydata->RXCount--;\
 }
 
 #define RB_pushback() {\
 if (RB_full())\
 mydata->RXHead = (mydata->RXHead+1)%RB_SIZE;\
 else\
 mydata->RXCount++;\
 }
 */


#endif

//declaration of functions in formation.c

double convertX16toDouble(uint16_t a);
double convertY16toDouble(uint16_t a);
double crossProductZ(double x1, double y1, double x2, double y2); //vector [x1,y1] cross [x2,y2]//int vect_A[], int vect_B[], int cross_P[])
void set_bot_state(enum BOTSTATE state);
enum BOTSTATE get_bot_state(void);
void set_move_type(enum MOVE_TYPE type);
enum MOVE_TYPE get_move_type(void);
void set_bot_type(enum BOTTYPE type);
enum BOTTYPE get_bot_type(void);

enum RIBBON_BOT_TYPE get_r_bot_type(void);
void set_r_bot_type(enum RIBBON_BOT_TYPE type);
uint8_t get_ribbon_ID_by_ID(uint16_t bot_ID);
void reset_ribbon_status(void);

void update_passing_by_ID (uint16_t ID);
void update_passingby_ID_gradient (uint16_t ID);
// uint8_t is_stuck() ;
// void follow_edge(uint8_t near, uint8_t far);
void shuffle_move(uint8_t near, uint8_t far);

double target_direction(double bot_x, double bot_y, double target_x, double target_y);


/////////////////functions in common.c///////////////
int check_localizablitiy();
void global_localization();
void single_localization();
void trilateration();

void broadcast_exists_moving_bot();
void broadcast_moving_bot_stopped();

uint8_t get_N_neighbors_in_dist(uint8_t distance);

double find_dist_to_next_bot(); //return the distance to the robot behind me and on the same ribbon
uint16_t find_nearest_N_ID();
uint16_t find_nearest_N_ID_lattice(); //considering the lattice point the neighbor occupied
uint16_t find_nearest_N_ID_lattice_ribbon(); //only consider the lattice point on the parent ribbon
uint16_t find_nearest_ribbon_N_ID_lattice(); //condiser only ribbon, considering the lattice point the neighbor occupied

double get_dist_by_ID(uint16_t ID);
uint8_t get_gradient_by_ID(uint16_t ID);
double get_neighbourX_by_ID(uint16_t ID);
double get_neighbourY_by_ID(uint16_t ID);
int16_t get_neighbour_q_by_ID(uint16_t ID);
int16_t get_neighbour_r_by_ID(uint16_t ID);
//uint8_t find_neighbour_gradient_by_ID(uint16_t ID);

uint8_t find_nearest_N_dist(int output_selection);


/////////////////////////Stage 1////////////////////////////////////
int countDistinct(uint16_t arr[], int size);
bool checkExist(uint16_t arr[], int size, uint16_t num);
void update_sum_ID (uint16_t ID);
void checkStage1Completion();
void boardcastStage1Complete();

void edge_following_omni_clockwise_wrt_lattice(); //following perimeter
void edge_following_omni_clockwise_wrt_robot();
void edge_following_omni_counter_clockwise_wrt_robot();
void edge_following_omni_clockwise_nonlocalizable(); //without localization information
void edge_following_omni_clockwise_wrt_lattice_ribbon(); //edge-following wrt the lower ribbon
void omni_stop();
void omni_move_to_nearest_lattice();

void ribbon_following_omni_clockwise_wrt_lattice(); //following ribbon

void record_current_position_for_next_lattice();
void move_to_next_lattice_point_same_ribbon(); //move to the next lattice point in the current system step

struct Hex nearest_lattice(struct Cartesian position);
struct Hex my_nearest_lattice();
double distance_to_lattice(struct Hex position);
double distance_to_nearest_lattice();
struct Cartesian hex_to_Cart(struct Hex hex);
bool is_lattice_point_adjacent(struct Hex point1, struct Hex point2);

bool is_in_shape(struct Hex hexPoint);
bool am_in_shape();
// bool is_in_hole(struct Hex hexPoint);
// bool am_in_hole();
bool is_at_ribbon_head(struct Hex hexPoint);
bool am_at_ribbon_head();

void step_detection(); //detect the systemstep to determine if should stop 



void shift_forward(struct Hex *lattice_s, int size);
void insert_at(struct Hex *lattice_s, int size, int z, struct Hex newElement);
void insert_at_recycle(struct Hex *lattice_r,  int size, int z, struct Hex newElement, int *recycleFlagSeq);


void update_passingby_ID_gradient (uint16_t ID);
void update_passing_by_ID (uint16_t ID);
double convertX16toDouble(uint16_t a);
double convertY16toDouble(uint16_t a);
double crossProductZ(double x1, double y1, double x2, double y2);
float normalize_to_pi(float angle);
double target_direction(double bot_x, double bot_y, double target_x, double target_y);

//Bot state
void set_bot_state(enum BOTSTATE state);
enum BOTSTATE get_bot_state(void);
void set_move_type(enum MOVE_TYPE type);
enum MOVE_TYPE get_move_type(void);
void set_bot_type(enum BOTTYPE type);
enum BOTTYPE get_bot_type(void);
enum RIBBON_BOT_TYPE get_r_bot_type(void);
void set_r_bot_type(enum RIBBON_BOT_TYPE type);
uint8_t get_ribbon_ID_by_ID(uint16_t bot_ID) ;

//SuEA
int omni_move_to_lattice(struct Hex *point);
void read_path_from_file(const char *filename, struct Hex **path, int **path_ending_index, int* total_number_of_path);