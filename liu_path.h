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

void set_global_vacancy(int q, int r) {
    global_vacancy.q = q;
    global_vacancy.r = r;
    //printf("=== 设置全局空缺坐标: (%d,%d) ===\n", q, r);
}

void confirm_global_vacancy(){
    vacancy_exists = 1;
    printf("=== 确认全局空缺坐标: (%d,%d) ===\n", global_vacancy.q, global_vacancy.r);
}

// 清除空缺坐标
void clear_global_vacancy() {
    global_vacancy.q = 99;
    global_vacancy.r = 99;
    vacancy_exists = 0;
    //printf("=== 清除全局空缺坐标 ===\n");
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
#if 1

int calculate_move_score(int from_q, int from_r, int to_q, int to_r, int fail_count) {
    int score = 100;
    double dist = hex_distance(from_q, from_r, to_q, to_r);
    
    // 基础：距离越近分数越高
    score -= (int)(dist * 10);
    
    // ✅ 方向偏好（不允许向下移动）
    if (to_r > from_r) {
        // 向上移动：最高优先级
        score += 30;
        
        if (to_q == from_q) {
            // 纯向上：额外奖励
            score += 15;
        } else if (abs(to_q - from_q) == 1) {
            // 斜向移动：较好
            score += 10;
        }
    } else if (to_r == from_r) {
        // 水平移动：失败时才考虑
        if (fail_count >= 58) {
            score += 5;  // 失败时轻微奖励水平移动
        } else {
            score -= 10; // 正常时惩罚水平移动
        }
    }
    // 向下移动：完全禁止，不会进入这里
    
    // ✅ 随机扰动打破对称性
    score += (rand() % 6);
    
    return score;
}

struct Hex find_nearest_unoccupied_target() {
    struct Hex best = {99, 99};
    double best_dist = INFINITY;
    int best_score = -1000;

    const int COMM_RANGE = 3;
    int self_q = mydata->hex_q;
    int self_r = mydata->hex_r;

    // ✅ 卡住检测：基于是否找到目标
    static int consecutive_failures = 0;

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
            
            // ✅ 绝对禁止向下移动
            if (r < self_r) continue;
            
            // ✅ 动态水平移动限制：基于连续失败次数
            if (r == self_r) {  // 水平移动
                if (consecutive_failures < 58) {  // 短暂失败时禁止水平移动
                    continue;
                }
                // 多次失败时，允许水平移动
            }
            
            if (mydata->original_position.q == q && mydata->original_position.r == r) continue;
            #if 0
            if ((q == -6 && r == 8) || (q == -5 && r == 8) || 
                (q == -6 && r == 7) || (q == -5 && r == 7)|| 
                (q == -5 && r == 6) || (q == -4 && r == 6)|| 
                (q == -5 && r == 5) || (q == -4 && r == 5)|| 
                (q == -4 && r == 4) || (q == -3 && r == 4)|| 
                (q == -4 && r == 3) || (q == -3 && r == 3)|| 
                (q == -3 && r == 2) || (q == -2 && r == 2)|| 
                (q == -1 && r == 4) || (q == 0 && r == 4)|| 
                (q == 0 && r == 3) || (q == 1 && r == 3)|| 
                (q == 0 && r == 2) || (q == 1 && r == 2)) continue;
            #else
            if ((q == -3 && r == 4) || (q == -2 && r == 4) || 
                (q == -2 && r == 3) || (q == -1 && r == 3)) continue;
            #endif
            if (is_position_in_history((struct Hex){q,r})) continue;
            if (!check_target_localizability((struct Hex){self_q, self_r}, (struct Hex){q, r})) continue;

            // ✅ 评分系统
            int score = calculate_move_score(self_q, self_r, q, r, consecutive_failures);
            
            if (score > best_score) {
                best_score = score;
                best.q = q;
                best.r = r;
                best_dist = hex_distance(self_q, self_r, q, r);
            }
        }
    }

    // ✅ 更新失败计数
    if (best.q == 99) {
        consecutive_failures++;
        printf("Robot %d: 未找到目标点，连续失败=%d\n", kilo_uid, consecutive_failures);
    } else {
        consecutive_failures = 0;
        printf("Robot %d: 找到目标点 (%d, %d), 评分=%d\n", 
               kilo_uid, best.q, best.r, best_score);
    }
    
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

// 寻找形状位置状态
void findShapePositionState() {

    if(!is_position_in_shape((struct Hex){mydata->hex_q,mydata->hex_r})){
        set_global_vacancy(mydata->hex_q,mydata->hex_r);
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
        update_occupancy(mydata->target_shape_index, 1);
 
        shape_entry_in_progress = 1;
        current_shape_entry_robot = kilo_uid;
        last_shape_entry_tick = kilo_ticks;
        current_formation_phase = 0;
        relocation_chain_complete = 0;
        printf("=== 新一轮形状进入开始 ===\n"); 
        printf("%d\n",kilo_uid);
        
        set_bot_state(MOVE_TO_SHAPE);
    } else {

            clear_global_vacancy();
            set_bot_state(IDLE);
            printf("Robot %d: 没找到合适的，下一个找位置\n", kilo_uid);
            should_move_to_shape = true;
            printf("这里设+\n");
            set_move_type(STOP);
            omni_stop();
        
    }
}

/*-------功能：移动阶段 --------*/


// 向形状移动状态
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
            //update_occupancy(mydata->target_shape_index, 1);
            mydata->shape_position_occupied = 1;

            shape_entry_in_progress = 0;
            occupied++;

            if(global_vacancy.q != 99 && global_vacancy.r != 99){
                confirm_global_vacancy();
            }
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

    if(mydata->original_position.q == global_vacancy.q 
        && mydata->original_position.r == global_vacancy.r)
    {
        return;
    }

#if 1
    // 不会往下走
    if(mydata->hex_r > global_vacancy.r){
        return;
    }
#endif

    // 确保不会振荡
    if(last_relocation_from.q == global_vacancy.q 
        && last_relocation_from.r == global_vacancy.r && mydata->original_position.q == last_relocation_to.q && mydata->original_position.r == last_relocation_to.r) 
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
#if 0
    if(is_collision_imminent(my_hex,global_vacancy)){
        return;
    }
#endif
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
    printf("这里设-\n");
 
}


// 链式补位状态
void chainRelocationState() {
    //printf("机器人 %d 还在补位 原位置（%d，%d）\n", kilo_uid,mydata->original_position.q,mydata->original_position.r);
    struct Hex target_hex = (struct Hex){global_vacancy.q,global_vacancy.r};

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
        printf("这里设+\n");
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
        printf("这里设+\n");

        // 触发新的形状进入
        current_formation_phase = 0;
    }
}

#endif