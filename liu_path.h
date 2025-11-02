#ifndef LIU_PATH_H
#define LIU_PATH_H

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <kilombo.h>
#include "formation.h"

uint8_t is_occupied[200][200] = {0};
uint8_t occupied = 0;
uint8_t have_oucciped_count = 0;
struct Hex global_vacancy = {99,99};
bool global_vacancy_in_shape = false;
uint8_t vacancy_exists = 0;

bool should_move_to_shape = true;
uint8_t current_formation_phase = 0; // 0:形状进入阶段, 1:补位阶段
uint8_t shape_entry_in_progress = 0; // 当前是否有机器人正在进入形状
uint16_t current_shape_entry_robot = 0; // 当前正在进入形状的机器人ID
uint32_t last_shape_entry_tick = 0; // 上次有机器人开始进入形状的时间

// 全局让位请求
struct Hex global_relocation_request = {99, 99};
struct Hex global_relocation_requester = {99, 99};
uint8_t relocation_request_active = 0;


// 补位完成检测
// 全局变量
struct Hex last_relocation_from = {99, 99};
struct Hex last_relocation_to = {99, 99};
struct Hex last_find_from = {99, 99};
struct Hex last_find_to = {99, 99};
uint8_t any_robot_relocating = 0;  // 是否有机器人正在补位
uint16_t relocating_robot_id = 99;  // 正在补位的机器人ID

struct Hex first_relocation = {99, 99};
uint8_t relocation_chance = 0;
uint8_t relocation_chain_complete = 0;
uint32_t last_relocation_activity = 0;

/* ---------辅助函数-------- */
// 检查六边形位置是否在形状定义内
bool is_position_in_shape(struct Hex pos) {
    for (int i = 0; i < mydata->lattice_shape_size; i++) {
        if (mydata->lattice_shape[i].q == pos.q && 
            mydata->lattice_shape[i].r == pos.r) {
            return true;
        }
    }
    return false;
}

// 精确判断两个六边形坐标是否相邻（距离为1）
bool is_hex_adjacent(struct Hex a, struct Hex b) {
    int dq = a.q - b.q;
    int dr = a.r - b.r;
    
    // 六边形网格中相邻的位置有6种可能
    return (dq == 1 && dr == 0) ||   // 右
           (dq == -1 && dr == 0) ||  // 左
           (dq == 0 && dr == 1) ||   // 右上
           (dq == 0 && dr == -1) ||  // 左下
           (dq == 1 && dr == -1) ||  // 右下  
           (dq == -1 && dr == 1);    // 左上
}


struct Hex cart_to_hex(struct Cartesian cart) {
    float lattice_size = kilo_lattice_size;
    float dx = lattice_size;   
    float dy = sqrt(3.0f)/2.0f * lattice_size; 

    // Convert Cartesian coordinates to axial coordinates
    float r = cart.y / dy;
    float q = cart.x / dx - 0.5f * r;

    // Round to nearest integer coordinates
    int q_round = round(q);
    int r_round = round(r);

    struct Hex hex = {q_round, r_round};
    
    return hex;
}




/*  分布式 */

/* -------补位---------- */
// 当机器人移动到形状位置时，发布空缺信息
void publish_vacancy_after_movement(struct Hex old_position) {
    mydata->known_vacancy = old_position;
    mydata->vacancy_timestamp = kilo_ticks;
    printf("Robot %d: 发布空缺位置 (%d,%d)\n", kilo_uid, old_position.q, old_position.r);
}

// 在 can_safely_start_movement 中考虑消息延迟
bool can_safely_start_movement() {
    // 检查邻居中是否有正在移动的
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        if (mydata->neighbors[i].is_moving) {
            printf("Robot %d: 邻居 %d 正在移动，等待...\n", 
                   kilo_uid, mydata->neighbors[i].ID);
            return false;
        }
        
        // 🔧 考虑消息延迟：即使邻居没有设置 is_moving，
        // 但如果他们最近有移动意图，也要小心
        if (mydata->neighbors[i].has_movement_intent &&
            kilo_ticks - mydata->neighbors[i].timestamp < 30) {
            printf("Robot %d: 邻居 %d 最近有移动意图，等待...\n",
                   kilo_uid, mydata->neighbors[i].ID);
            return false;
        }
    }
    return true;
}

// 检查是否有可补位的空缺
bool has_relocation_opportunity() {
    // 如果已经在形状内，不参与补位
    if (mydata->shape_position_occupied) {
        return false;
    }
    
    // 检查已知的空缺是否有效
    if (mydata->known_vacancy.q == 99 && mydata->known_vacancy.r == 99) {
        return false;
    }
    
    // 检查空缺信息是否过时
    if (kilo_ticks - mydata->vacancy_timestamp > 200) {
        return false;
    }
    
    // 检查是否与空缺相邻
    if (!is_hex_adjacent((struct Hex){mydata->hex_q, mydata->hex_r}, mydata->known_vacancy)) {
        return false;
    }
    
    // 检查是否可以安全移动
    if (!can_safely_start_movement()) {
        return false;
    }
    
    return true;
}

// 检查是否可以开始补位
bool can_proceed_with_relocation() {
    // 检查邻居中是否有冲突的补位意图
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        if (mydata->neighbors[i].has_relocation_intent &&
            mydata->neighbors[i].relocation_target.q == mydata->relocation_target.q &&
            mydata->neighbors[i].relocation_target.r == mydata->relocation_target.r) {
            
            // 基于ID的冲突解决：ID小的获胜
            if (kilo_uid > mydata->neighbors[i].ID) {
                printf("Robot %d: 补位冲突，邻居 %d 有更高优先级\n", 
                       kilo_uid, mydata->neighbors[i].ID);
                return false;
            }
        }
    }
    return true;
}

// 检查补位机会（分布式版本）
void checkChainRelocationOpportunity_distributed() {
    if (get_bot_state() == CHAIN_RELOCATION || 
        get_bot_state() == FIND_SHAPE_POSITION ||
        get_bot_state() == MOVE_TO_SHAPE) {
        return;
    }
    
    if (!has_relocation_opportunity()) {
        return;
    }
    
    // 设置补位意图
    mydata->has_relocation_intent = 1;
    mydata->relocation_target = mydata->known_vacancy;
    mydata->relocation_source = (struct Hex){mydata->hex_q, mydata->hex_r};
    
    printf("Robot %d: 声明补位意图 从(%d,%d)到(%d,%d)\n", 
           kilo_uid, mydata->hex_q, mydata->hex_r, 
           mydata->known_vacancy.q, mydata->known_vacancy.r);
    
    // 等待一段时间让意图传播
    if (kilo_ticks - mydata->last_movement_check > 50) {
        if (can_proceed_with_relocation()) {
            start_relocation();
        }
        mydata->last_movement_check = kilo_ticks;
    }
}



void chainRelocationState_distributed() {
    struct Hex target_hex = mydata->relocation_target;
    
    if (omni_move_to_lattice(&target_hex) == 1) {
        // 补位完成
        finish_movement();
        set_bot_state(IDLE);
        
        // 更新位置信息
        global_localization();
        struct Hex cur_hex = cart_to_hex((struct Cartesian){mydata->x, mydata->y});
        mydata->hex_q = cur_hex.q;
        mydata->hex_r = cur_hex.r;
        
        struct Hex old_position = mydata->original_position;
        mydata->original_position = (struct Hex){mydata->hex_q, mydata->hex_r};
        
        // 如果从形状外补位到形状内，发布新的空缺
        if (!is_position_in_shape(old_position) && is_position_in_shape((struct Hex){mydata->hex_q, mydata->hex_r})) {
            publish_vacancy_after_movement(old_position);
        }
        
        printf("Robot %d: 补位完成\n", kilo_uid);
    }
}

// 开始补位
void start_relocation() {
    mydata->has_relocation_intent = 0;
    mydata->is_moving = 1;
    mydata->movement_start_time = kilo_ticks;
    
    set_bot_state(CHAIN_RELOCATION);
    printf("Robot %d: 开始补位移动\n", kilo_uid);
}



/* --------移动阶段------------ */

// 解决移动冲突（基于ID的优先级）
bool resolve_movement_conflict(uint16_t neighbor_id) {
    // 简单的基于ID的冲突解决：ID小的优先
    if (kilo_uid < neighbor_id) {
        return true;  // 我有更高优先级
    } else {
        return false; // 对方有更高优先级
    }
}

// 检查是否可以安全移动
bool can_safely_move() {
    // 检查通信范围内的邻居是否有正在移动的
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        if (mydata->neighbors[i].is_moving) {
            printf("Robot %d: 邻居 %d 正在移动，等待...\n", 
                   kilo_uid, mydata->neighbors[i].ID);
            return false;
        }
        
        // 还可以检查邻居是否有移动意图到相同区域
        if (mydata->neighbors[i].has_movement_intent &&
            mydata->neighbors[i].intended_target.q == mydata->target_q &&
            mydata->neighbors[i].intended_target.r == mydata->target_r) {
            printf("Robot %d: 邻居 %d 有相同移动意图，协调中...\n",
                   kilo_uid, mydata->neighbors[i].ID);
            return resolve_movement_conflict(mydata->neighbors[i].ID);
        }
    }
    return true;
}

// 开始移动前的准备
void prepare_for_movement(struct Hex target) {
    mydata->has_movement_intent = 1;
    mydata->intended_target = target;
    mydata->movement_start_time = kilo_ticks;
    
    // 等待一小段时间让意图传播
    if (kilo_ticks - mydata->last_movement_check > 50) {
        if (can_safely_move()) {
            mydata->is_moving = 1;
            mydata->has_movement_intent = 0;
            printf("Robot %d: 开始移动到 (%d,%d)\n", 
                   kilo_uid, target.q, target.r);
        }
        mydata->last_movement_check = kilo_ticks;
    }
}

// 完成移动后清理状态
void finish_movement() {
    mydata->is_moving = 0;
    mydata->has_movement_intent = 0;
    mydata->intended_target = (struct Hex){99, 99};
}

void init_move_history() {
    for (int i = 0; i < 5; i++) {
        mydata->move_tracker.move_history[i] = (struct Hex){0, 0};
    }
    mydata->move_tracker.history_index = 0;
    mydata->move_tracker.history_initialized = false;
}

void record_move(struct Hex new_pos) {
    mydata->move_tracker.move_history[mydata->move_tracker.history_index] = new_pos;
    mydata->move_tracker.history_index = (mydata->move_tracker.history_index + 1) % 5;
    mydata->move_tracker.history_initialized = true;
}

bool is_position_in_history(struct Hex target_pos) {
    if (!mydata->move_tracker.history_initialized) {
        return false;
    }
    
    for (int i = 0; i < 5; i++) {
        // 跳过未记录的位置
        if (mydata->move_tracker.move_history[i].q == 0 && 
            mydata->move_tracker.move_history[i].r == 0) {
            continue;
        }
        
        if (mydata->move_tracker.move_history[i].q == target_pos.q && 
            mydata->move_tracker.move_history[i].r == target_pos.r) {
            return true;
        }
    }
    return false;
}

double hex_distance(int q1, int r1, int q2, int r2) {
    return (abs(q1 - q2) + abs(q1 + r1 - q2 - r2) + abs(r1 - r2)) / 2.0;
}


int check_target_localizability(struct Hex start, struct Hex target)
{
    int localized_bot_count = 0;
    int valid_neighbor_indices[20] = {};  // 存储有效的邻居索引
    int cnt = 0;

    for (int i = 0; i < mydata->N_Neighbors; i++)
    {
        if (mydata->neighbors[i].localized == 1)
        {
            #if 0
            // ✅ 修复：只检查目标点与邻居的距离
            double dist_to_target = hex_distance(target.q, target.r, 
                                               mydata->neighbors[i].hex_q, 
                                               mydata->neighbors[i].hex_r);
            
            // 如果目标点在邻居的通信范围内
            if(dist_to_target <= 2.0)  // 使用你的通信范围，比如2格
            {
                localized_bot_count++;
                valid_neighbor_indices[cnt] = i;
                cnt++;
            }
            #else
            struct Cartesian start_cart = hex_to_Cart(start);
            struct Cartesian target_cart = hex_to_Cart(target);
            // 在 check_target_localizability 中改为物理距离：
            double dist_to_target = sqrt(
                pow(target_cart.x - mydata->neighbors[i].x, 2) + 
                pow(target_cart.y - mydata->neighbors[i].y, 2)
            );
            if (dist_to_target <= 120.0)  // 使用物理距离，120单位
            {
                localized_bot_count++;
                valid_neighbor_indices[cnt] = i;
                cnt++;
            }
            #endif
        }
    }

    if (localized_bot_count >= 3) {
        // 检查是否存在任意三个不共线的点
        for (int i = 0; i < localized_bot_count; i++) {
            for (int j = i + 1; j < localized_bot_count; j++) {
                for (int k = j + 1; k < localized_bot_count; k++) {
                    int idx_i = valid_neighbor_indices[i];
                    int idx_j = valid_neighbor_indices[j]; 
                    int idx_k = valid_neighbor_indices[k];
                    
                    double dx1 = mydata->neighbors[idx_j].x - mydata->neighbors[idx_i].x;
                    double dy1 = mydata->neighbors[idx_j].y - mydata->neighbors[idx_i].y;
                    double dx2 = mydata->neighbors[idx_k].x - mydata->neighbors[idx_i].x;
                    double dy2 = mydata->neighbors[idx_k].y - mydata->neighbors[idx_i].y;

                    double cross = fabs(dx1 * dy2 - dx2 * dy1);
                    double tol = 0.01 * kilo_lattice_size * kilo_lattice_size * 0.25;

                    if (cross > tol) {
                        return 1;  // 找到一组不共线的三点
                    }
                }
            }
        }
        return 0;  // 所有三元组都共线
    } else {
        return 0;
    }
}

int count_occupied_positions() {
    int occupied_count = 0;
    for (int q = -100; q <= 100; q++) {
        for (int r = -100; r <= 100; r++) {
            if (is_occupied[q + 100][r + 100] == 1) {
                occupied_count++;
            }
        }
    }
    return occupied_count;
}

void update_occupancy(int shape_index, uint8_t occupied) {
    if (shape_index < 0 || shape_index >= mydata->lattice_shape_size) {
        printf("Robot %d: ERROR - 更新形状占用数组失败\n", kilo_uid, shape_index);
        return;
    }
    
    // 1. 更新 shape_occupancy
    mydata->shape_occupancy[shape_index] = occupied;
    
    // 2. 更新 is_occupied 全局数组
    int q = mydata->lattice_shape[shape_index].q;
    int r = mydata->lattice_shape[shape_index].r;
    
    if (q >= -100 && q <= 100 && r >= -100 && r <= 100) {
        is_occupied[q + 100][r + 100] = occupied;
        printf("Robot %d: updated is_occupied[%d][%d] = %d for position (%d,%d)\n", 
               kilo_uid, q + 100, r + 100, occupied, q, r);
    } else {
        printf("Robot %d: WARNING - position (%d,%d) out of is_occupied bounds\n", 
               kilo_uid, q, r);
    }
}

void update_occupancy_for_hex(struct Hex hex, uint8_t occupied) {
    // 找到这个六边形位置对应的形状索引
    for (int i = 0; i < mydata->lattice_shape_size; i++) {
        if (mydata->lattice_shape[i].q == hex.q && 
            mydata->lattice_shape[i].r == hex.r) {
            // 使用现有的 update_occupancy 函数
            update_occupancy(i, occupied);
            printf("更新位置 (%d,%d) 占用状态为 %d\n", hex.q, hex.r, occupied);
            return;
        }
    }
    printf("错误：未找到位置 (%d,%d) 的形状索引\n", hex.q, hex.r);
}

// 快速从笛卡尔坐标获取六边形坐标
struct Hex get_hex_from_cartesian(float x, float y) {
    float lattice_size = kilo_lattice_size;
    float dx = lattice_size;   
    float dy = sqrt(3.0f)/2.0f * lattice_size; 

    float r = y / dy;
    float q = x / dx - 0.5f * r;

    struct Hex hex = {round(q), round(r)};
    return hex;
}

/* ------ 避障 --------- */

bool is_hex_occupied(struct Hex hex) {
    // 检查邻居中是否有机器人占据这个位置
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        if (mydata->neighbors[i].hex_q == hex.q && 
            mydata->neighbors[i].hex_r == hex.r &&
            mydata->neighbors[i].shape_position_occupied) {
            return true;
        }
    }
    return false;
}

// 检查移动路径上是否有已进入形状的机器人
bool is_path_blocked_by_shaped_robots(struct Hex start, struct Hex target) {
    // 获取从起点到终点的所有六边形位置
    
    struct Hex path_hexes[10];  // 假设路径最多10个六边形
    int path_length = get_hex_path(start, target, path_hexes);
    
    for (int i = 0; i < path_length; i++) {
        // 检查这个六边形位置是否有已进入形状的机器人
        for (int j = 0; j < mydata->N_Neighbors; j++) {
            if (mydata->neighbors[j].shape_position_occupied &&
                mydata->neighbors[j].hex_q == path_hexes[i].q &&
                mydata->neighbors[j].hex_r == path_hexes[i].r) {
                printf("Robot %d: 路径被已进入形状的 Robot %d 阻挡 at (%d,%d)\n", 
                       kilo_uid, mydata->neighbors[j].ID, path_hexes[i].q, path_hexes[i].r);
                return true;
            }
        }
    }
    return false;
}

// 获取两个六边形之间的路径
int get_hex_path(struct Hex start, struct Hex end, struct Hex *path) {
    int dq = end.q - start.q;
    int dr = end.r - start.r;
    int path_index = 0;
    
    struct Hex current = start;
    
    while (current.q != end.q || current.r != end.r) {
        path[path_index++] = current;
        
        if (current.q < end.q) current.q++;
        else if (current.q > end.q) current.q--;
        
        if (current.r < end.r) current.r++;
        else if (current.r > end.r) current.r--;
    }
    
    path[path_index++] = end;  // 包含终点
    return path_index;
}


bool is_collision_imminent(struct Hex current_pos, struct Hex target_pos) {
    struct Cartesian current_cart = hex_to_Cart(current_pos);
    struct Cartesian target_cart = hex_to_Cart(target_pos);
    
    float move_dx = target_cart.x - current_cart.x;
    float move_dy = target_cart.y - current_cart.y;
    float move_distance = sqrt(move_dx * move_dx + move_dy * move_dy);
    
    if (move_distance < 0.001) return false;
    
    // 归一化
    float norm_dx = move_dx / move_distance;
    float norm_dy = move_dy / move_distance;
    
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        struct Hex neighbor_hex = {mydata->neighbors[i].hex_q, mydata->neighbors[i].hex_r};
        struct Cartesian neighbor_cart = hex_to_Cart(neighbor_hex);
        
        float nx = neighbor_cart.x - current_cart.x;
        float ny = neighbor_cart.y - current_cart.y;
        float neighbor_distance = sqrt(nx * nx + ny * ny);
        
        if (neighbor_distance < 25.0) {
            return true;
        }
        
        float projection = nx * norm_dx + ny * norm_dy;
        float perpendicular_distance = fabs(nx * norm_dy - ny * norm_dx);
        
        if (projection > -10 && 
            perpendicular_distance < 34 && 
            projection < move_distance + 20.0) {
            #if 0
            #if 0
            printf("当前六角(%d,%d)->物理(%.1f,%.1f)\n", current_pos.q, current_pos.r, current_cart.x, current_cart.y);
            printf("目标六角(%d,%d)->物理(%.1f,%.1f)\n", target_pos.q, target_pos.r, target_cart.x, target_cart.y);
            printf("邻居六角(%d,%d)->物理(%.1f,%.1f)\n", neighbor_hex.q, neighbor_hex.r, neighbor_cart.x, neighbor_cart.y);
            #endif
            printf("Robot %d: 检测到碰撞风险！邻居 %d,坐标(%d,%d), 垂直距离=%.1f, 投影=%.1f\n", 
                   kilo_uid, mydata->neighbors[i].ID, mydata->neighbors[i].hex_q, mydata->neighbors[i].hex_r,
                   perpendicular_distance, projection);
            #endif
            return true;
        }
    }
    
    return false;
}

struct Hex find_blocking_robot(struct Hex start, struct Hex target) {
    int distance = (int)hex_distance(start.q, start.r, target.q, target.r);
    struct Hex path_hexes[distance+3];
    int path_length = get_hex_path(start, target, path_hexes);
    
    for (int i = 0; i < path_length; i++) {

        // 找到第一个阻挡路径的形状内机器人
        for (int j = 0; j < mydata->N_Neighbors; j++) {
            if (mydata->neighbors[j ].shape_position_occupied &&
                mydata->neighbors[j].hex_q == path_hexes[i].q &&
                mydata->neighbors[j].hex_r == path_hexes[i].r) {
                return (struct Hex){mydata->neighbors[j].hex_q, mydata->neighbors[j].hex_r};
            }
        }
    }
    return (struct Hex){99, 99};
}

bool is_robot_in_shape(int q, int r) {
    // 方法1：检查是否在形状的六边形坐标范围内
    // 假设形状数据包含了所有属于形状的六边形位置
    
    // 检查是否在lattice_shape中
    if ((q == -3 && r == 4) ||
        (q == -2 && r == 4) || 
        (q == -2 && r == 3) ||
        (q == -1 && r == 3)) {
        return false;
    }

    for (int i = 0; i < mydata->lattice_shape_size; i++) {
        if (mydata->lattice_shape[i].q == q && mydata->lattice_shape[i].r == r) {
            return true;
        }
    }
    
    return false;
}


bool is_position_empty(int q, int r) {
    if (q >= -100 && q <= 100 && r >= -100 && r <= 100) {
        return !is_occupied[q + 100][r + 100];
    }
    return false;
}

bool is_robot_at_position(int q, int r) {
    if (q >= -100 && q <= 100 && r >= -100 && r <= 100) {
        return is_occupied[q + 100][r + 100];
    }
    return false;
}


/*
void request_shape_robots_to_relocate(struct Hex requester_pos, struct Hex target_pos) {
    // 找到阻挡路径的形状内机器人
    struct Hex blocking_robot = find_blocking_robot(requester_pos, target_pos);
    
    if (blocking_robot.q != 99 && blocking_robot.r != 99) {
        printf("请求机器人 (%d,%d) 让出位置\n", blocking_robot.q, blocking_robot.r);
        
        // 通过消息广播让位请求
        // 或者设置全局变量
        set_global_relocation_request(blocking_robot, requester_pos);
    }
}


struct Hex find_alternative_path(struct Hex start, struct Hex target) {
    // 六边形的6个可能方向
    struct Hex directions[6] = {
        {1, 0}, {1, -1}, {0, -1}, 
        {-1, 0}, {-1, 1}, {0, 1}
    };
    printf("机器人 %d 当前点为：（%d，%d）\n", kilo_uid,start.q, start.r);
    
    // 首先检查是否可定位
    int localizable = check_localizablitiy();
    
    // 尝试每个方向作为中间点
    for (int i = 0; i < 6; i++) {
        struct Hex intermediate = {start.q + directions[i].q, start.r + directions[i].r};

        // 🔧 关键修改：检查这个中间点是否在形状内
        //if (!is_position_in_shape(intermediate)) continue;  // 不在形状内，跳过

        printf("当前检查点（%d，%d）\n",intermediate.q,intermediate.r);
        
        // ✅ 新增条件：检查中间点是否在邻居通信范围内
        if (localizable == 1) {
            // 检查中间点是否在至少三个邻居的通信范围内
            int in_range_count = 0;
            for (int j = 0; j < mydata->N_Neighbors; j++) {
                if (mydata->neighbors[j].localized == 1) {
                    // 计算中间点与邻居的物理距离
                    #if 0
                    double intermediate_x = intermediate.q * kilo_lattice_size;
                    double intermediate_y = intermediate.r * kilo_lattice_size * sqrt(3)/2;
                    #else
// 正确的应该是：
double intermediate_x = intermediate.q * kilo_lattice_size + intermediate.r * 0.5 * kilo_lattice_size;
double intermediate_y = intermediate.r * kilo_lattice_size * sqrt(3)/2;
                    #endif
                    double dx = intermediate_x - mydata->neighbors[j].x;
                    double dy = intermediate_y - mydata->neighbors[j].y;
                    double dist_to_neighbor = sqrt(dx*dx + dy*dy);
                    
                    // 使用配置文件中的通信半径 120
                    if (dist_to_neighbor <= 120.0) {
                        in_range_count++;
                    }
                }
            }
            
            // 只有中间点在至少三个已定位邻居的通信范围内才考虑
            if (in_range_count < 3) {
                printf("中间点(%d,%d)不在通信范围内，跳过\n", intermediate.q, intermediate.r);
                continue;
            }
        }

        // 检查是否被占据且路径畅通
        if (!is_hex_occupied(intermediate) && !is_collision_imminent(intermediate,target) &&
            !is_path_blocked_by_shaped_robots(start, intermediate) &&
            !is_path_blocked_by_shaped_robots(intermediate, target)) {
            printf("找到形状内替代路径 via (%d,%d)\n", intermediate.q, intermediate.r);
            return intermediate;
        }
    }
    
    // 没有找到形状内的替代路径，返回原目标
    printf("❌ 形状内路径全被堵死，触发让位机制\n");
    request_shape_robots_to_relocate(start, target);
    return (struct Hex){99,99};
}
*/

/* -------功能：保持阶段---------*/


/* -------功能：寻找阶段---------*/
struct Hex find_farthest_unoccupied_target() {
    struct Hex best = {99, 99};
    double best_dist = 0;  // 改为0，找最大距离

    const int COMM_RANGE = 2;
    int self_q = mydata->hex_q;
    int self_r = mydata->hex_r;

    for (int dq = -COMM_RANGE; dq <= COMM_RANGE; dq++) {
        for (int dr = -COMM_RANGE; dr <= COMM_RANGE; dr++) {
            int q = self_q + dq;
            int r = self_r + dr;

            // 跳过超出通信范围的点
            if (hex_distance(self_q, self_r, q, r) > COMM_RANGE) continue;

            // ===== 开始过滤 =====
            if(is_collision_imminent((struct Hex){self_q,self_r},(struct Hex){q,r})) continue;
            if (!is_position_in_shape((struct Hex){q,r})) continue;
            if (is_occupied[q + 100][r + 100]) continue;
            if (q == self_q && r == self_r) continue;
            if (r < self_r) continue;

            if ((q == -3 && r == 4) || (q == -2 && r == 4) || 
                (q == -2 && r == 3) || (q == -1 && r == 3)) continue;

            if (last_find_from.q == q && last_find_from.r == r) {
                int have_oucciped_count = count_occupied_positions();
                printf("已有 %d 个机器人\n",have_oucciped_count);
                if(have_oucciped_count == 28){

                }else{
                    continue;
                }
            }

            if (!check_target_localizability((struct Hex){self_q, self_r}, (struct Hex){q, r})) {
                continue;
            }

            double dist = hex_distance(self_q, self_r, q, r);
            if (dist > best_dist) {  // 改为找最大距离
                best_dist = dist;
                best.q = q;
                best.r = r;
            }
        }
    }

    printf("Robot %d: 找到最远目标点 (%d, %d), 距离=%.1f\n", kilo_uid, best.q, best.r, best_dist);
    return best;
}

// 检查目标是否已被邻居声明
bool is_target_claimed_by_neighbor(int q, int r) {
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        if (mydata->neighbors[i].has_movement_intent &&
            mydata->neighbors[i].intended_target.q == q &&
            mydata->neighbors[i].intended_target.r == r) {
            
            // 如果邻居的声明时间更早，则尊重其声明
            if (mydata->neighbors[i].target_intent_time < mydata->target_intent_time) {
                return true;
            }
            // 如果同时声明，ID小的获胜
            else if (mydata->neighbors[i].target_intent_time == mydata->target_intent_time &&
                     mydata->neighbors[i].ID < kilo_uid) {
                return true;
            }
        }
    }
    return false;
}

struct Hex find_nearest_unoccupied_target_distributed() {
    struct Hex best = {99, 99};
    double best_dist = INFINITY;

    const int COMM_RANGE = 3;
    int self_q = mydata->hex_q;
    int self_r = mydata->hex_r;

    for (int dq = -COMM_RANGE; dq <= COMM_RANGE; dq++) {
        for (int dr = -COMM_RANGE; dr <= COMM_RANGE; dr++) {
            int q = self_q + dq;
            int r = self_r + dr;

            // 跳过超出通信范围的点
            if (hex_distance(self_q, self_r, q, r) > COMM_RANGE) continue;

            // 碰撞检测
            if(is_collision_imminent((struct Hex){self_q,self_r},(struct Hex){q,r})) continue;

            // 不在形状内
            if (!is_position_in_shape((struct Hex){q,r})) continue;

            // 被占据，这个不能要，应该改为
            if (is_occupied[q + 100][r + 100]) continue;
            if (q == self_q && r == self_r) continue;
            if (r < self_r) continue;
            if (mydata->original_position.q == q && mydata->original_position.r == r) continue;
            if ((q == -3 && r == 4) || (q == -2 && r == 4) || 
                (q == -2 && r == 3) || (q == -1 && r == 3)) continue;
            if (is_position_in_history((struct Hex){q,r})) continue;
            if (!check_target_localizability((struct Hex){self_q, self_r}, (struct Hex){q, r})) continue;

            // 🔧 新增：检查邻居是否已经声明了这个目标
            if (is_target_claimed_by_neighbor(q, r)) {
                continue;  // 跳过已被声明的目标
            }

            double dist = hex_distance(self_q, self_r, q, r);
            if (dist < best_dist) {
                best_dist = dist;
                best.q = q;
                best.r = r;
            }
        }
    }

    printf("Robot %d: 找到目标点 (%d, %d), 距离=%.1f\n", kilo_uid, best.q, best.r, best_dist);
    return best;
}
#if 1
struct Hex find_nearest_unoccupied_target() {
    struct Hex best = {99, 99};
    double best_dist = INFINITY;

    const int COMM_RANGE = 3;
    int self_q = mydata->hex_q;
    int self_r = mydata->hex_r;

    for (int dq = -COMM_RANGE; dq <= COMM_RANGE; dq++) {
        for (int dr = -COMM_RANGE; dr <= COMM_RANGE; dr++) {
            int q = self_q + dq;
            int r = self_r + dr;

            // 跳过超出通信范围的点
            if (hex_distance(self_q, self_r, q, r) > COMM_RANGE) continue;

            // 🔽🔽🔽 在这里插入调试打印 🔽🔽🔽
            #if 0
            printf("Checking (%d, %d): occupied=%d, r_check=%d, last_from=%d, localizable=%d\n",
                   q, r,
                   is_occupied[q + 100][r + 100],           // 是否被占据
                   r < self_r,                              // 是否 r < self_r（会被跳过）
                   (last_find_from.q == q && last_find_from.r == r), // 是否是上一次起点
                   check_target_localizability((struct Hex){self_q, self_r}, (struct Hex){q, r})
            );
            #endif
            // 🔼🔼🔼 调试打印结束 🔼🔼🔼

            // ===== 开始过滤 =====
            if(is_collision_imminent((struct Hex){self_q,self_r},(struct Hex){q,r})) continue;
            if (!is_position_in_shape((struct Hex){q,r})) continue;
            if (is_occupied[q + 100][r + 100]) continue;
            if (q == self_q && r == self_r) continue;
            if (r < self_r) continue;
            if (mydata->original_position.q == q && mydata->original_position.r == r) continue;

            if ((q == -3 && r == 4) || (q == -2 && r == 4) || 
                (q == -2 && r == 3) || (q == -1 && r == 3)) continue;

            if (is_position_in_history((struct Hex){q,r})){
                #if 0
                if(have_oucciped_count>27){
                    printf("累计数量：%d\n",have_oucciped_count);
                }else{
                    continue;
                }
                #endif 
                continue;
            } 
            //if (last_find_from.q == q && last_find_from.r == r) continue;

            if (!check_target_localizability((struct Hex){self_q, self_r}, (struct Hex){q, r})) {
                continue;
            }

            double dist = hex_distance(self_q, self_r, q, r);
            if (dist < best_dist) {
                best_dist = dist;
                best.q = q;
                best.r = r;
            }
        }
    }



    printf("Robot %d: 找到目标点 (%d, %d), 距离=%.1f\n", kilo_uid, best.q, best.r, best_dist);
    return best;
}
#else
struct Hex find_nearest_unoccupied_target() {
    struct Hex best = {99, 99};
    double best_dist = INFINITY;

    
    for (int i = 0; i < mydata->lattice_shape_size; i++) {
        int q = mydata->lattice_shape[i].q;
        int r = mydata->lattice_shape[i].r;
        
        // 原有的过滤条件
        if (is_occupied[q + 100][r + 100]) continue;
        if(r < mydata->hex_r) continue;
        if ((q == -3 && r == 4) || (q == -2 && r == 4) || 
            (q == -2 && r == 3) || (q == -1 && r == 3)) continue;
        if(last_find_from.q == q && last_find_from.r == r) continue;

        // 只考虑相邻的格子
        if (!is_hex_adjacent((struct Hex){mydata->hex_q, mydata->hex_r}, (struct Hex){q, r})) {
            continue;
        }

        if(!check_target_localizability((struct Hex){mydata->hex_q,mydata->hex_r},(struct Hex){q,r})) continue;

        double dist = hex_distance(mydata->hex_q, mydata->hex_r, q, r);
        if (dist < best_dist) {
            best_dist = dist;
            best.q = q;
            best.r = r;
        }
    }

    printf("Robot %d: 找到目标点 (%d, %d), 距离=%.1f\n", kilo_uid, best.q, best.r, best_dist);
    return best;
}
#endif

int find_optimal_shape_position_index() {
    #if 0
    struct Hex nearest = find_farthest_unoccupied_target();
    #else
    struct Hex nearest = find_nearest_unoccupied_target();
    #endif
    // 找到对应的索引
    for (int i = 0; i < mydata->lattice_shape_size; i++) {
        if (mydata->lattice_shape[i].q == nearest.q && 
            mydata->lattice_shape[i].r == nearest.r) {
            printf("Robot %d: found target index %d for position (%d,%d)\n", 
                   kilo_uid, i, nearest.q, nearest.r);
            return i;
        }
    }
    
    //printf("Robot %d: ERROR - 无法找到索引 (%d,%d)\n", kilo_uid, nearest.q, nearest.r);
    return -1;
}

// 检查是否可以声明目标
bool can_claim_target(struct Hex target) {
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        if (mydata->neighbors[i].has_movement_intent &&
            mydata->neighbors[i].intended_target.q == target.q &&
            mydata->neighbors[i].intended_target.r == target.r) {
            
            // 冲突解决：时间优先，ID次优
            if (mydata->neighbors[i].target_intent_time < mydata->target_intent_time) {
                return false; // 邻居声明更早
            } else if (mydata->neighbors[i].target_intent_time == mydata->target_intent_time &&
                       mydata->neighbors[i].ID < kilo_uid) {
                return false; // 同时声明，但邻居ID更小
            }
        }
    }
    return true;
}

// 找到形状索引
int find_shape_index(struct Hex position) {
    for (int i = 0; i < mydata->lattice_shape_size; i++) {
        if (mydata->lattice_shape[i].q == position.q && 
            mydata->lattice_shape[i].r == position.r) {
            return i;
        }
    }
    return -1;
}

// 检查目标是否仍然可用
bool is_target_still_available(struct Hex target) {
    // 检查是否被实际占据
    if (is_occupied[target.q + 100][target.r + 100]) {
        return false;
    }
    
    // 检查是否在形状内
    if (!is_position_in_shape(target)) {
        return false;
    }
    
    return true;
}

void findShapePositionState_distributed() {
    static uint32_t target_selection_start = 0;
    static struct Hex selected_target = {99, 99};
    
    // 第一阶段：选择目标并声明意图
    // 还没有选择过
    if (!mydata->has_movement_intent) {
        selected_target = find_nearest_unoccupied_target_distributed();
        
        if (selected_target.q != 99 && selected_target.r != 99) {
            // 声明目标意图
            mydata->has_movement_intent = 1;
            mydata->intended_target = selected_target;
            mydata->target_intent_time = kilo_ticks;
            target_selection_start = kilo_ticks;
            
            printf("Robot %d: 声明目标意图 (%d,%d)\n", kilo_uid, selected_target.q, selected_target.r);
        } else {
            // 没有找到合适目标
            printf("Robot %d: 没有找到合适目标\n", kilo_uid);
            set_bot_state(IDLE);
            return;
        }
    }
    
    // 第二阶段：等待目标确认
    if (mydata->has_movement_intent) {
        // 检查目标是否仍然可用
        if (!is_target_still_available(selected_target)) {
            printf("Robot %d: 目标 (%d,%d) 已被占用，重新选择\n", 
                   kilo_uid, selected_target.q, selected_target.r);
            mydata->has_movement_intent = 0;
            return;
        }
        
        // 等待一段时间让意图传播并解决冲突
        if (kilo_ticks - target_selection_start > 100) { // 等待100个tick
            if (can_claim_target(selected_target)) {
                // 成功获得目标，开始移动准备
                mydata->target_shape_index = find_shape_index(selected_target);
                mydata->target_q = selected_target.q;
                mydata->target_r = selected_target.r;
                
                printf("Robot %d: 成功获得目标 (%d,%d)，准备移动\n", 
                       kilo_uid, selected_target.q, selected_target.r);
                
                mydata->has_movement_intent = 0; // 清除目标意图
                set_bot_state(MOVE_TO_SHAPE);
            } else {
                // 目标冲突，重新选择
                printf("Robot %d: 目标冲突，重新选择\n", kilo_uid);
                mydata->has_movement_intent = 0;
            }
        }
    }
}

// 寻找形状位置状态
void findShapePositionState() {

    if(!is_position_in_shape((struct Hex){mydata->hex_q,mydata->hex_r})){
        // 设置补位
    }
    
    int best_index = find_optimal_shape_position_index();
    //printf("Robot %d: found best shape index = %d\n", kilo_uid, best_index);
    
    if (best_index != -1) {

        if(is_position_in_shape((struct Hex){mydata->hex_q,mydata->hex_r})){
            have_oucciped_count--;
        }

        mydata->target_shape_index = best_index;
        mydata->target_q = mydata->lattice_shape[best_index].q;
        mydata->target_r = mydata->lattice_shape[best_index].r;
        last_find_from.q = mydata->original_position.q; 
        last_find_from.r = mydata->original_position.r; 
        last_find_to.q = mydata->target_q; 
        last_find_to.r = mydata->target_r; 
        
 
        shape_entry_in_progress = 1;
        current_shape_entry_robot = kilo_uid;
        last_shape_entry_tick = kilo_ticks;
        current_formation_phase = 0;
        relocation_chain_complete = 0;
        printf("=== 新一轮形状进入开始 ===\n"); 
        printf("%d\n",kilo_uid);
        
        set_bot_state(MOVE_TO_SHAPE);
    } else {

            set_bot_state(IDLE);
            printf("Robot %d: 没找到合适的，下一个找位置\n", kilo_uid);
            should_move_to_shape = true;
            set_move_type(STOP);
            omni_stop();
        
    }
}

/*-------功能：移动阶段 --------*/


// 向形状移动状态

void moveToShapeState_distributed() {
    static int move_attempts = 0;
    static struct Hex current_target = {0, 0};

    // 如果还没有正式开始移动，先准备
    if (!mydata->is_moving) {
        current_target.q = mydata->target_q;
        current_target.r = mydata->target_r;
        prepare_for_movement(current_target);
        return;
    }

    move_attempts++;
    
    //printf("Robot %d: === MOVE_TO_SHAPE (attempt %d) ===\n", kilo_uid, move_attempts);


    current_target.q = mydata->target_q;
    current_target.r = mydata->target_r;
    
    int result = omni_move_to_lattice(&current_target);
    if(result){
        if (current_target.q != mydata->target_q || current_target.r != mydata->target_r) {
            current_target.q = mydata->target_q;
            current_target.r = mydata->target_r;
            /*
            struct Hex cur_hex = cart_to_hex((struct Cartesian){kilo_x, kilo_y});
            mydata->hex_q = cur_hex.q;
            mydata->hex_r = cur_hex.r;
            */
            global_localization();
            struct Hex cur_hex = cart_to_hex((struct Cartesian){mydata->x, mydata->y});
            mydata->hex_q = cur_hex.q;
            mydata->hex_r = cur_hex.r;
            printf("Robot %d: 到达中间点，继续向最终目标移动\n", kilo_uid);
        }else{
            finish_movement();
            struct Hex old_position = mydata->original_position;
            have_oucciped_count++;
            record_move(current_target);

            if(mydata->original_position.q >= -100 && mydata->original_position.q<= 100
            && mydata->original_position.r >= -100 && mydata->original_position.r<= 100){
                is_occupied[mydata->original_position.q + 100][mydata->original_position.r + 100] = 0;
            }
            // 去掉更新位置
            /*
            struct Hex cur_hex = cart_to_hex((struct Cartesian){kilo_x, kilo_y});
            mydata->hex_q = cur_hex.q;
            mydata->hex_r = cur_hex.r;
            */
            global_localization();
            struct Hex cur_hex = cart_to_hex((struct Cartesian){mydata->x, mydata->y});
            mydata->hex_q = cur_hex.q;
            mydata->hex_r = cur_hex.r;
            mydata->original_position.q = mydata->hex_q;
            mydata->original_position.r = mydata->hex_r;
            printf("Robot %d: 已经到达位置 (%d.%d), 下一步补位\n",kilo_uid,mydata->hex_q,mydata->hex_r);
            // 发布空缺信息
            publish_vacancy_after_movement(old_position);
            update_occupancy(mydata->target_shape_index, 1);
            mydata->shape_position_occupied = 1;

            shape_entry_in_progress = 0;
            occupied++;


            set_bot_state(IDLE);
        }
    }


    global_localization();

    // 移动超时检查
    if (kilo_ticks - mydata->movement_start_time > 5000) { // 5秒超时
        printf("Robot %d: 移动超时，放弃移动\n", kilo_uid);
        finish_movement();
        set_bot_state(IDLE);
    }
    }


void moveToShapeState() {
    static int move_attempts = 0;
    static struct Hex current_target = {0, 0};
    move_attempts++;
    
    //printf("Robot %d: === MOVE_TO_SHAPE (attempt %d) ===\n", kilo_uid, move_attempts);


    current_target.q = mydata->target_q;
    current_target.r = mydata->target_r;
    
    int result = omni_move_to_lattice(&current_target);
    if(result){
        if (current_target.q != mydata->target_q || current_target.r != mydata->target_r) {
            current_target.q = mydata->target_q;
            current_target.r = mydata->target_r;
            /*
            struct Hex cur_hex = cart_to_hex((struct Cartesian){kilo_x, kilo_y});
            mydata->hex_q = cur_hex.q;
            mydata->hex_r = cur_hex.r;
            */
            global_localization();
            struct Hex cur_hex = cart_to_hex((struct Cartesian){mydata->x, mydata->y});
            mydata->hex_q = cur_hex.q;
            mydata->hex_r = cur_hex.r;
            printf("Robot %d: 到达中间点，继续向最终目标移动\n", kilo_uid);
        }else{
            have_oucciped_count++;
            record_move(current_target);

            if(mydata->original_position.q >= -100 && mydata->original_position.q<= 100
            && mydata->original_position.r >= -100 && mydata->original_position.r<= 100){
                is_occupied[mydata->original_position.q + 100][mydata->original_position.r + 100] = 0;
            }
            // 去掉更新位置
            /*
            struct Hex cur_hex = cart_to_hex((struct Cartesian){kilo_x, kilo_y});
            mydata->hex_q = cur_hex.q;
            mydata->hex_r = cur_hex.r;
            */
            global_localization();
            struct Hex cur_hex = cart_to_hex((struct Cartesian){mydata->x, mydata->y});
            mydata->hex_q = cur_hex.q;
            mydata->hex_r = cur_hex.r;
            mydata->original_position.q = mydata->hex_q;
            mydata->original_position.r = mydata->hex_r;
            printf("Robot %d: 已经到达位置 (%d.%d), 下一步补位\n",kilo_uid,mydata->hex_q,mydata->hex_r);
            update_occupancy(mydata->target_shape_index, 1);
            mydata->shape_position_occupied = 1;

            shape_entry_in_progress = 0;
            occupied++;


            set_bot_state(IDLE);
        }
    }
    if(check_localizablitiy()){
            //printf("可以定位\n");
        }else{
            printf("机器人 %d移动 不可以定位\n", kilo_uid);
        }
    global_localization();
}

/* ----------功能：补位-----------*/

#if 0
// 检查链式补位机会，实际上形状内的也需要补位。
void checkChainRelocationOpportunity(void) {
    //if (!mydata->formation_initialized) return;
    // 只有形状外的机器人才参与链式补位
    #if 1
    if (mydata->shape_position_occupied) 
    {
        return; // 已经在形状内，不参与补位
    }
    #endif

    if(get_bot_state() == CHAIN_RELOCATION || get_bot_state() == FIND_SHAPE_POSITION
    || get_bot_state() == MOVE_TO_SHAPE || get_bot_state() == MAINTAIN_POSITION) 
    {
        return;
    }
    
    // 确保不会回头

    if(mydata->original_position.q == 
        && mydata->original_position.r == )
    {
        return;
    }

#if 1
    // 不会往下走
    if(mydata->hex_r > ){
        return;
    }
#endif

    // 确保不会振荡
    if(last_relocation_from.q ==  
        && last_relocation_from.r ==  && mydata->original_position.q == last_relocation_to.q && mydata->original_position.r == last_relocation_to.r) 
    {

        return;
    }

    // 空位不存在
    if (!vacancy_exists)
    {
        return;
    } 

    // 补位完成
    if (relocation_chain_complete)
    {
        return;
    }

    struct Hex my_hex = {mydata->hex_q, mydata->hex_r};
/*
    if(is_collision_imminent(my_hex,global_vacancy)){
        return;
    }
*/
    if(is_position_in_shape(global_vacancy)){
        if(!check_target_localizability((struct Hex){my_hex.q,my_hex.r},(struct Hex){global_vacancy.q,global_vacancy.r})) 
            return;
    }else{

    }

    // 不相邻就不补位
    #if 1
    if (!is_hex_adjacent(my_hex, global_vacancy)) {   
        return; // 不相邻，不参与补位
    }
#endif

    

     /* 
    // 与形状距离为1 不补位
    struct Hex target = find_nearest_unoccupied_target();
    if (is_hex_adjacent(my_hex, target)) {
        printf("该点与形状距离为1不补位\n");
        return; // 相邻，不参与补位
    }
    */


#if 0
#if 0
    if( my_hex.q == global_vacancy.q){
#else
    if(my_hex.r == global_vacancy.r){
#endif

    return;
    }

#endif


    relocation_chain_complete = 0;
    // 符合条件，该点需要去补位
    last_relocation_from.q = mydata->hex_q;
    last_relocation_from.r = mydata->hex_r;
    last_relocation_to.q = global_vacancy.q;
    last_relocation_to.r = global_vacancy.r;
    printf("在 检查补位中，补位机器人 %d： （%d,%d）全局空位（%d，%d）\n", kilo_uid, mydata->hex_q,mydata->hex_r,global_vacancy.q,global_vacancy.r);
    mydata->original_position.q = mydata->hex_q;
    mydata->original_position.r = mydata->hex_r;

    any_robot_relocating = 1;
    relocating_robot_id = kilo_uid;

    set_bot_state(CHAIN_RELOCATION);
    mydata->relocation_start_time = kilo_ticks;
    should_move_to_shape = false;
 
}
#endif

#if 0
// 链式补位状态
void chainRelocationState() {
    //printf("机器人 %d 还在补位 原位置（%d，%d）\n", kilo_uid,mydata->original_position.q,mydata->original_position.r);
    struct Hex target_hex = (struct Hex){global_vacancy.q,global_vacancy.r};
    if (!mydata->is_moving) {
        struct Hex target_hex = (struct Hex){global_vacancy.q, global_vacancy.r};
        prepare_for_movement(target_hex);
        return;
    }

    if(target_hex.q == first_relocation.q && target_hex.r == first_relocation.r){
        first_relocation.q = 99;
        first_relocation.r = 99;
        relocation_chance = 0;
        // 重置状态，开始新一轮
        relocation_chain_complete = 1;
        shape_entry_in_progress = 0;
        current_shape_entry_robot = 0;
        clear_global_vacancy(); // 清除当前空缺
        should_move_to_shape = true;
        printf("重置状态，开始新一轮\n");
        // 触发新的形状进入
        current_formation_phase = 0;
        return;
    }


    if (omni_move_to_lattice(&target_hex) == 1) {
        relocating_robot_id == 99;
        // 到达补位位置
        set_bot_state(IDLE);
        set_move_type(STOP);
        omni_stop();
        // 清空原来的占据
        if(is_robot_in_shape(mydata->original_position.q,mydata->original_position.r)){
            is_occupied[mydata->original_position.q+100][mydata->original_position.r+100] = 0;
        }

        finish_movement();

        struct Hex cur_hex = {99,99};
        if(is_position_in_shape(global_vacancy)){
            global_localization();
            cur_hex = cart_to_hex((struct Cartesian){mydata->x, mydata->y});
            mydata->hex_q = cur_hex.q;
            mydata->hex_r = cur_hex.r;
        }else{
            cur_hex = cart_to_hex((struct Cartesian){kilo_x, kilo_y});
            mydata->x = kilo_x;
            mydata->y = kilo_y;
            mydata->hex_q = cur_hex.q;
            mydata->hex_r = cur_hex.r;
        }

        if(relocation_chance == 0){
            first_relocation.q = mydata->hex_q;
            first_relocation.r = mydata->hex_r;
        }
        relocation_chance++;

        
        any_robot_relocating = 0;

        mydata->relocation_occupied = 1;

        // 补位完成，设置新空位
        set_global_vacancy(mydata->original_position.q,mydata->original_position.r);

        // 补位完成，更新新的原始坐标
   
        mydata->original_position.q = cur_hex.q;
        mydata->original_position.r = cur_hex.r;
        if(is_robot_in_shape(cur_hex.q,cur_hex.r)){
            is_occupied[cur_hex.q+100][cur_hex.r+100] = 1;
        }
        vacancy_exists = 1;
        // 触发下一轮链式补位检查
        //printf("3. 从这里进入补位\n");
        //checkChainRelocationOpportunity();
        // 🔧 任何补位完成的机器人都更新活动时间
        last_relocation_activity = kilo_ticks;
        //printf("Robot %d 补位完成，更新补位活动时间 %d\n", kilo_uid,last_relocation_activity);
    }else{
        //printf("没有移动到\n");
        struct Hex cur_hex = {99,99};
        if(is_position_in_shape(global_vacancy)){
            global_localization();

        }else{
            //printf("更新位置\n");
            cur_hex = cart_to_hex((struct Cartesian){kilo_x, kilo_y});
            mydata->x = kilo_x;
            mydata->y = kilo_y;
            
            
        }
        mydata->total_relocation_time = kilo_ticks - mydata->relocation_start_time;
        last_relocation_activity = kilo_ticks;
    }
}

/* ------- 补位检测 ------  */
void check_relocation_chain_completion(void) {

    if (get_bot_state() == CHAIN_RELOCATION 
        || get_bot_state() == FIND_SHAPE_POSITION
        || get_bot_state() == MOVE_TO_SHAPE
    ) {
        //printf("因为机器人 %d 的状态是 %d\n",kilo_uid,get_bot_state() );
        return;
    }

    
    static uint32_t last_check_tick = 0;
    //printf("当前 kilo_ticks %d\n", kilo_ticks);
    // 每50个tick检查一次，避免太频繁
    if (kilo_ticks - last_check_tick < 50) 
    {
        return;
    }
    last_check_tick = kilo_ticks;
    
    // 条件1: 当前有形状入口在进行中
    if (shape_entry_in_progress) {
        //printf("正在入形\n");
        relocation_chain_complete = 0;
        return;
    }
    
    // 条件2: 一段时间内没有补位活动
    if (kilo_ticks - last_relocation_activity > 1000 || mydata->total_relocation_time > 1000) {
        //printf("=== 补位链已完成，切换到下一轮形状进入 ===\n");
        first_relocation.q = 99;
        first_relocation.r = 99;
        // 重置状态，开始新一轮
        mydata->total_relocation_time = 0;
        relocation_chain_complete = 1;
        shape_entry_in_progress = 0;
        current_shape_entry_robot = 0;
        clear_global_vacancy(); // 清除当前空缺
        should_move_to_shape = true;

        // 触发新的形状进入
        current_formation_phase = 0;
    }
}
#endif
#endif