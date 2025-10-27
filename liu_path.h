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
uint8_t num_rangwei = 0;

// 防止振荡
// 全局变量记录最近的让位历史
struct Hex last_relocation_robots[2];
int relocation_history_count = 0;

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

// 统一的占用状态更新函数
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

double hex_distance(int q1, int r1, int q2, int r2) {
    return (abs(q1 - q2) + abs(q1 + r1 - q2 - r2) + abs(r1 - r2)) / 2.0;
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


bool is_collision_imminent(struct Hex target_pos) {
    struct Cartesian target_cart = hex_to_Cart(target_pos);
    struct Cartesian my_cart = {kilo_x, kilo_y};
    
    //printf("STEP1: 我的位置 kilo_x=%.1f, kilo_y=%.1f\n", kilo_x, kilo_y);
    //printf("STEP1: 目标六角坐标(%d,%d) -> 笛卡尔(%.1f,%.1f)\n", target_pos.q, target_pos.r, target_cart.x, target_cart.y);
    
    float move_dx = target_cart.x - my_cart.x;
    float move_dy = target_cart.y - my_cart.y;
    //printf("STEP2: 移动向量 dx=%.1f, dy=%.1f\n", move_dx, move_dy);
    
    float move_distance = sqrt(move_dx * move_dx + move_dy * move_dy);
    //printf("STEP3: 移动距离=%.1f\n", move_distance);
    
    if (move_distance < 0.001) return false;
    
    // 归一化
    float norm_dx = move_dx / move_distance;
    float norm_dy = move_dy / move_distance;
    //printf("STEP4: 归一化方向 (%.3f, %.3f)\n", norm_dx, norm_dy);
    
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        // 使用六边形坐标重新计算邻居位置，避免存储误差
        struct Hex neighbor_hex = {mydata->neighbors[i].hex_q, mydata->neighbors[i].hex_r};
        struct Cartesian neighbor_cart = hex_to_Cart(neighbor_hex);
        
        float nx = neighbor_cart.x - my_cart.x;
        float ny = neighbor_cart.y - my_cart.y;
        float neighbor_distance = sqrt(nx * nx + ny * ny);
        
        if (neighbor_distance < 25.0) {
            return true;
        }
        
        // 投影距离
        float projection = nx * move_dx + ny * move_dy;
        
        // 正确的垂直距离计算
        //float perpendicular_distance = sqrt(neighbor_distance * neighbor_distance - projection * projection);
        float perpendicular_distance = fabs(nx * move_dy - ny * move_dx) / move_distance;
        
        
        if (projection > -10 && 
            perpendicular_distance < 34 && projection < move_distance + 20.0
          ) {
                printf("Robot %d: 检测到碰撞风险！邻居 %d,坐标(%d,%d), 垂直距离=%.1f, 投影=%.1f\n", kilo_uid, mydata->neighbors[i].ID, mydata->neighbors[i].hex_q,mydata->neighbors[i].hex_r,perpendicular_distance, projection);
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
            if (mydata->neighbors[j].shape_position_occupied &&
                mydata->neighbors[j].hex_q == path_hexes[i].q &&
                mydata->neighbors[j].hex_r == path_hexes[i].r) {
                return (struct Hex){mydata->neighbors[j].hex_q, mydata->neighbors[j].hex_r};
            }
        }
    }
    return (struct Hex){99, 99};
}


float calculate_distance_to_shape_edge(int q, int r) {
    float min_distance = 9999.0;
    
    // 检查是否在S边界上
    for (int i = 0; i < mydata->lattice_s_size; i++) {
        if (mydata->lattice_s[i].q == q && mydata->lattice_s[i].r == r) {
            return 0.0; // 就在边界上
        }
    }
    
    // 检查是否在T边界上  
    for (int i = 0; i < mydata->lattice_t_size; i++) {
        if (mydata->lattice_t[i].q == q && mydata->lattice_t[i].r == r) {
            return 0.0; // 就在边界上
        }
    }
    
    // 计算到最近边界点的距离
    for (int i = 0; i < mydata->lattice_s_size; i++) {
        struct Cartesian boundary_cart = hex_to_Cart(mydata->lattice_s[i]);
        struct Cartesian robot_cart = hex_to_Cart((struct Hex){q, r});
        
        float dx = boundary_cart.x - robot_cart.x;
        float dy = boundary_cart.y - robot_cart.y;
        float distance = sqrt(dx * dx + dy * dy);
        
        if (distance < min_distance) {
            min_distance = distance;
        }
    }
    
    for (int i = 0; i < mydata->lattice_t_size; i++) {
        struct Cartesian boundary_cart = hex_to_Cart(mydata->lattice_t[i]);
        struct Cartesian robot_cart = hex_to_Cart((struct Hex){q, r});
        
        float dx = boundary_cart.x - robot_cart.x;
        float dy = boundary_cart.y - robot_cart.y;
        float distance = sqrt(dx * dx + dy * dy);
        
        if (distance < min_distance) {
            min_distance = distance;
        }
    }
    
    return min_distance;
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

struct Hex find_robot_near_boundary(void) {
    struct Hex best_robot = {99, 99};
    int ID = 0;
    float min_edge_distance = 9999.0;
    
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        if (is_robot_in_shape(mydata->neighbors[i].hex_q, mydata->neighbors[i].hex_r)) {
            float edge_distance = calculate_distance_to_shape_edge(
                mydata->neighbors[i].hex_q, mydata->neighbors[i].hex_r);
            
            // 找距离边界最近的机器人
            if (edge_distance < min_edge_distance) {
                min_edge_distance = edge_distance;
                best_robot.q = mydata->neighbors[i].hex_q;
                best_robot.r = mydata->neighbors[i].hex_r;
                ID = mydata->neighbors[i].ID;
            }
        }
    }

    printf("找到应该移动的机器人 %d 为 (%d,%d)\n", ID,best_robot.q,best_robot.r);
    
    return best_robot;
}

// 返回机器人在哪个边界：'T'=上边界, 'L'=左边界, 'B'=下边界, 'R'=右边界, 'N'=不在边界
char get_robot_boundary_type(int q, int r) {
    // 找到形状的边界范围
    int min_q = 999, max_q = -999, min_r = 999, max_r = -999;
    
    for (int i = 0; i < mydata->lattice_shape_size; i++) {
        if (mydata->lattice_shape[i].q < min_q) min_q = mydata->lattice_shape[i].q;
        if (mydata->lattice_shape[i].q > max_q) max_q = mydata->lattice_shape[i].q;
        if (mydata->lattice_shape[i].r < min_r) min_r = mydata->lattice_shape[i].r;
        if (mydata->lattice_shape[i].r > max_r) max_r = mydata->lattice_shape[i].r;
    }
    
    // 检查边界
    if (r == max_r) return 'T'; // 上边界：不能改变r
    if (q == min_q) return 'L'; // 左边界：不能改变q
    if (r == min_r) return 'B'; // 下边界：不能改变r  
    if (q == max_q) return 'R'; // 右边界：不能改变q
    
    return 'N'; // 不在边界
}

bool is_position_empty(int q, int r) {
    if (q >= -100 && q <= 100 && r >= -100 && r <= 100) {
        return !is_occupied[q + 100][r + 100];
    }
    return false;
}

struct Hex find_restricted_move_position(struct Hex current_pos) {
    struct Hex directions[6] = {{1,0}, {1,-1}, {0,-1}, {-1,0}, {-1,1}, {0,1}};
    
    char boundary_type = get_robot_boundary_type(current_pos.q, current_pos.r);
    
    if (boundary_type != 'N') {
        printf("机器人(%d,%d)在%c边界，限制移动方向\n", 
               current_pos.q, current_pos.r, boundary_type);
        
        for (int i = 0; i < 6; i++) {
            struct Hex candidate = {
                current_pos.q + directions[i].q,
                current_pos.r + directions[i].r
            };
            
            // 根据边界类型限制移动
            bool allowed = true;
            switch (boundary_type) {
                case 'T': // 上边界：不能改变r
                    allowed = (candidate.r == current_pos.r);
                    break;
                case 'L': // 左边界：不能改变q  
                    allowed = (candidate.q == current_pos.q);
                    break;
                    /*
                case 'B': // 下边界：不能改变r
                    allowed = (candidate.r == current_pos.r);
                    break;
                case 'R': // 右边界：不能改变q
                    allowed = (candidate.q == current_pos.q);
                    break;
                    */
            }
            
            if (allowed && 
                is_robot_in_shape(candidate.q, candidate.r) && 
                is_position_empty(candidate.q, candidate.r)) {
                printf("边界机器人移动到 (%d,%d)\n", candidate.q, candidate.r);
                return candidate;
            }
        }
    } else {
        // 非边界机器人：可以任意方向移动
        for (int i = 0; i < 6; i++) {
            struct Hex candidate = {
                current_pos.q + directions[i].q,
                current_pos.r + directions[i].r
            };
            
            if (is_robot_in_shape(candidate.q, candidate.r) && 
                is_position_empty(candidate.q, candidate.r)) {
                return candidate;
            }
        }
    }
    
    return (struct Hex){99, 99};
}

// 按r坐标从大到小排序（最上方的机器人排前面）
void sort_robots_by_r(struct Hex *robots, int count) {
    for (int i = 0; i < count - 1; i++) {
        for (int j = 0; j < count - i - 1; j++) {
            if (robots[j].r < robots[j + 1].r) {
                // 交换位置
                struct Hex temp = robots[j];
                robots[j] = robots[j + 1];
                robots[j + 1] = temp;
            }
        }
    }
}


bool can_robot_move_directly(struct Hex robot) {
    struct Hex directions[6] = {{1,0}, {1,-1}, {0,-1}, {-1,0}, {-1,1}, {0,1}};
    char boundary_type = get_robot_boundary_type(robot.q, robot.r);
    
    for (int i = 0; i < 6; i++) {
        struct Hex target = {robot.q + directions[i].q, robot.r + directions[i].r};
        
        // 根据边界类型限制移动方向
        bool allowed = true;
        if (boundary_type != 'N') {
            switch (boundary_type) {
                case 'T':  // 上下边界：不能改变r
                    allowed = (target.r == robot.r);
                    break;
                case 'L':  // 左右边界：不能改变q
                    allowed = (target.q == robot.q);
                    break;
            }
        }
        
        if (allowed && 
            is_robot_in_shape(target.q, target.r) && 
            is_position_empty(target.q, target.r)) {
            printf("   找到空位(%d,%d)可以移动\n", target.q, target.r);
            return true;
        }
    }
    return false;
}

bool find_move_path(struct Hex current, struct Hex *visited, int *visited_count) {
    // 检查是否已经访问过（避免循环）
    for (int i = 0; i < *visited_count; i++) {
        if (visited[i].q == current.q && visited[i].r == current.r) {
            return false;
        }
    }
    
    // 标记为已访问
    visited[*visited_count] = current;
    (*visited_count)++;
    
    // 检查当前机器人是否能直接移动
    if (can_robot_move_directly(current)) {
        return true;
    }
    
    // 如果不能直接移动，递归检查邻居机器人
    struct Hex neighbors[6];
    int neighbor_count = get_shape_neighbors(current, neighbors);
    
    for (int i = 0; i < neighbor_count; i++) {
        if (find_move_path(neighbors[i], visited, visited_count)) {
            return true;
        }
    }
    
    return false;
}

bool is_robot_at_position(int q, int r) {
    if (q >= -100 && q <= 100 && r >= -100 && r <= 100) {
        return is_occupied[q + 100][r + 100];
    }
    return false;
}

int get_robot_id_at(int q, int r) {
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        if (mydata->neighbors[i].hex_q == q && mydata->neighbors[i].hex_r == r) {
            return mydata->neighbors[i].ID;
        }
    }
    return -1; // 没有找到机器人
}

int get_shape_neighbors(struct Hex pos, struct Hex *neighbors) {
    struct Hex directions[6] = {{1,0}, {1,-1}, {0,-1}, {-1,0}, {-1,1}, {0,1}};
    int count = 0;
    
    for (int i = 0; i < 6; i++) {
        struct Hex neighbor = {pos.q + directions[i].q, pos.r + directions[i].r};
        if (is_robot_in_shape(neighbor.q, neighbor.r) && 
            is_robot_at_position(neighbor.q, neighbor.r) && // 该位置有机器人
            get_robot_id_at(neighbor.q, neighbor.r) != 0) { // 且不是机器人0
            neighbors[count] = neighbor;
            count++;
        }
    }
    return count;
}

bool has_move_chain(struct Hex start_robot) {
    struct Hex visited[50];
    int visited_count = 0;
    
    return find_move_path(start_robot, visited, &visited_count);
}

struct Hex find_nearest_robot_to(struct Hex target) {
    struct Hex nearest = {99, 99};
    int min_distance = 9999;
    
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        if (is_robot_in_shape(mydata->neighbors[i].hex_q, mydata->neighbors[i].hex_r) &&
            mydata->neighbors[i].ID != 0) {
            
            int dist = hex_distance(target.q,target.r, mydata->neighbors[i].hex_q, mydata->neighbors[i].hex_r);
            if (dist < min_distance) {
                min_distance = dist;
                nearest.q = mydata->neighbors[i].hex_q;
                nearest.r = mydata->neighbors[i].hex_r;
            }
        }
    }
    return nearest;
}


struct Hex find_movable_robot_chain(void) {
    // 从所有形状内机器人开始搜索
    struct Hex all_shape_robots[50];
    int count = 0;
    
    // 收集所有形状内机器人（排除机器人0）
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        if (is_robot_in_shape(mydata->neighbors[i].hex_q, mydata->neighbors[i].hex_r) &&
            mydata->neighbors[i].ID != 0) {
            all_shape_robots[count].q = mydata->neighbors[i].hex_q;
            all_shape_robots[count].r = mydata->neighbors[i].hex_r;
            count++;
        }
    }
    
    // 按r坐标排序（最上方优先）
    sort_robots_by_r(all_shape_robots, count);
    
    // 依次检查每个机器人是否能移动
    for (int i = 0; i < count; i++) {
        struct Hex candidate = all_shape_robots[i];
        if (has_move_chain(candidate)) {
            printf("✅ 找到可移动链的机器人 (%d,%d)\n", candidate.q, candidate.r);
            return candidate;
        }
    }
    
    return (struct Hex){99, 99};
}

void set_global_relocation_request(struct Hex blocking_robot, struct Hex requester_pos) {
    global_relocation_request = blocking_robot;
    global_relocation_requester = requester_pos;
    relocation_request_active = 1;
    printf("=== 全局让位请求：机器人 (%d,%d) 为 (%d,%d) 让位 ===\n",
           blocking_robot.q, blocking_robot.r, requester_pos.q, requester_pos.r);
}

struct Hex find_top_robot_in_shape(void) {
    struct Hex top_robot = {99, 99};
    int max_r = -999;
    
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        if (is_robot_in_shape(mydata->neighbors[i].hex_q, mydata->neighbors[i].hex_r) &&
            mydata->neighbors[i].ID != 0) { // 排除机器人0
            
            if (mydata->neighbors[i].hex_r > max_r) {
                max_r = mydata->neighbors[i].hex_r;
                top_robot.q = mydata->neighbors[i].hex_q;
                top_robot.r = mydata->neighbors[i].hex_r;
            }
        }
    }
    
    if (top_robot.q != 99) {
        printf("找到最上方机器人 (%d,%d)\n", top_robot.q, top_robot.r);
    }
    
    return top_robot;
}

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
    printf("当前点为：（%d，%d）\n", start.q, start.r);
    
    // 尝试每个方向作为中间点
    for (int i = 0; i < 6; i++) {

        struct Hex intermediate = {start.q + directions[i].q, start.r + directions[i].r};

        // 🔧 关键修改：检查这个中间点是否在形状内
        //if (!is_position_in_shape(intermediate)) continue;  // 不在形状内，跳过

        printf("当前检查点（%d，%d）\n",intermediate.q,intermediate.r);
        // 检查是否被占据且路径畅通
        if (!is_hex_occupied(intermediate) && !is_collision_imminent(target) &&
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

// 检测附近是否有机器人会碰撞

// 计算点C到直线AB的垂直距离
float calculate_perpendicular_distance(struct Cartesian A, struct Cartesian B, struct Cartesian C) {
    // 直线AB的向量
    float AB_x = B.x - A.x;
    float AB_y = B.y - A.y;
    
    // 向量AC
    float AC_x = C.x - A.x;
    float AC_y = C.y - A.y;
    
    // 计算叉积的绝对值 |AB × AC|
    float cross_product = fabs(AB_x * AC_y - AB_y * AC_x);
    
    // 直线AB的长度
    float AB_length = sqrt(AB_x * AB_x + AB_y * AB_y);
    
    if (AB_length < 0.001) return INFINITY; // 避免除零
    
    // 垂直距离 = |AB × AC| / |AB|
    return cross_product / AB_length;
}
// 计算点C在直线AB上的投影比例
// 返回值：0=在A点，1=在B点，<0=在A之前，>1=在B之后
float calculate_projection_ratio(struct Cartesian A, struct Cartesian B, struct Cartesian C) {
    // 直线AB的向量
    float AB_x = B.x - A.x;
    float AB_y = B.y - A.y;
    
    // 向量AC
    float AC_x = C.x - A.x;
    float AC_y = C.y - A.y;
    
    // 直线AB长度的平方
    float AB_length_sq = AB_x * AB_x + AB_y * AB_y;
    
    if (AB_length_sq < 0.001) return 0; // 避免除零
    
    // 投影比例 = (AC · AB) / |AB|²
    float dot_product = AC_x * AB_x + AC_y * AB_y;
    return dot_product / AB_length_sq;
}

bool is_any_robot_too_close(void) {
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        struct Cartesian neighbor_cart = {mydata->neighbors[i].x, mydata->neighbors[i].y};
        
        float dx = neighbor_cart.x - kilo_x;
        float dy = neighbor_cart.y - kilo_y;
        float distance = sqrt(dx * dx + dy * dy);
        
        if (distance < 40.0) { // 30mm 内认为太近
            printf("Robot %d: 邻居 %d 太近: %.1fmm\n", 
                   kilo_uid, mydata->neighbors[i].ID, distance);
            return true;
        }
    }
    return false;
}

void debug_collision_detection(struct Hex target_pos) {
    struct Cartesian target_cart = hex_to_Cart(target_pos);
    
    printf("=== 碰撞检测调试 ===\n");
    printf("我的位置: (%.1f, %.1f)\n", kilo_x, kilo_y);
    printf("目标位置: (%.1f, %.1f)\n", target_cart.x, target_cart.y);
    printf("邻居数量: %d\n", mydata->N_Neighbors);
    
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        struct Cartesian neighbor_cart = {mydata->neighbors[i].x, mydata->neighbors[i].y};
        float dx = neighbor_cart.x - kilo_x;
        float dy = neighbor_cart.y - kilo_y;
        float distance = sqrt(dx * dx + dy * dy);
        
        printf("邻居 %d: 位置(%.1f,%.1f), 距离=%.1f\n", 
               mydata->neighbors[i].ID, neighbor_cart.x, neighbor_cart.y, distance);
    }
    printf("==================\n");
}
/* -------功能：保持阶段---------*/
void maintainPositionState() {
    set_move_type(STOP);
}

/* -------功能：寻找阶段---------*/
struct Hex find_nearest_unoccupied_target() {
    struct Hex best = {99, 99};
    double best_dist = INFINITY;

    for (int i = 0; i < mydata->lattice_shape_size; i++) {
        int q = mydata->lattice_shape[i].q;
        int r = mydata->lattice_shape[i].r;
        
        // 检查是否已被占用
        if (is_occupied[q + 100][r + 100]) {
            if(q == -4 && r== 3){
                printf("正在检查-4，3 被占用了！\n");
            }
            continue;
        }

        if(r < mydata->hex_r){
            if(q == -4 && r== 3){
                printf("正在检查-4，3 r减小了！\n");
            }
            continue;
        }

        if ((q == -3 && r == 4) ||
        (q == -2 && r == 4) || 
        (q == -2 && r == 3) ||
        (q == -1 && r == 3)) {
            if(q == -4 && r== 3){
                printf("正在检查-4，3 不在形状内！\n");
            }
        continue;
    }

        if(last_find_from.q == q && last_find_from.r == r){
            if(q == -4 && r== 3){
                printf("正在检查-4，3 是上一次的位置！\n");
            }
            continue;
        }

        double dist = hex_distance(mydata->hex_q, mydata->hex_r, q, r);
        #if 0
        if(dist == hex_distance(-3,-1,-3,0)){
            if(kilo_uid ==0){
                if (dist < best_dist) {
                    best_dist = dist;
                    best.q = q;
                    best.r = r;
                }
            }
        }else{
            if (dist < best_dist) {
                    best_dist = dist;
                    best.q = q;
                    best.r = r;
                }
        }
        #else
        if (dist < best_dist && dist == hex_distance(-3,-1,-3,0)) {
            best_dist = dist;
            best.q = q;
            best.r = r;
        }
        #endif
    }

    // ✅ 调试输出
    //printf("Robot %d: nearest unoccupied target = (%d, %d), dist=%.1f\n",kilo_uid, best.q, best.r, best_dist);

    return best;
}

int find_optimal_shape_position_index() {
    struct Hex nearest = find_nearest_unoccupied_target();
    
    // 找到对应的索引
    for (int i = 0; i < mydata->lattice_shape_size; i++) {
        if (mydata->lattice_shape[i].q == nearest.q && 
            mydata->lattice_shape[i].r == nearest.r) {
            printf("Robot %d: found target index %d for position (%d,%d)\n", 
                   kilo_uid, i, nearest.q, nearest.r);
            return i;
        }
    }
    
    printf("Robot %d: ERROR - 无法找到索引 (%d,%d)\n", 
           kilo_uid, nearest.q, nearest.r);
    return -1;
}

// 检查机器人是否能移动
bool can_robot_move(struct Hex robot_pos) {
    struct Hex directions[6] = {{1,0}, {1,-1}, {0,-1}, {-1,0}, {-1,1}, {0,1}};

    
    for (int i = 0; i < 6; i++) {
        struct Hex target = {
            robot_pos.q + directions[i].q,
            robot_pos.r + directions[i].r
        };

        if(target.q == last_find_from.q && target.r == last_find_from.r){
            continue;
        }

        if(target.r < robot_pos.r){
            continue;
        }
        // 只要在形状内且为空就可以移动
        if (is_robot_in_shape(target.q, target.r) && 
            is_position_empty(target.q, target.r)) {
            return true;
        }
    }
    
    return false;
}

bool was_recently_relocated(struct Hex pos) {
    for (int i = 0; i < relocation_history_count; i++) {
        if (last_relocation_robots[i].q == pos.q && 
            last_relocation_robots[i].r == pos.r) {
            return true;
        }
    }
    return false;
}

void add_to_relocation_history(struct Hex pos) {
    // 滚动记录
    if (relocation_history_count >= 2) {
        for (int i = 0; i < 1; i++) {
            last_relocation_robots[i] = last_relocation_robots[i+1];
        }
        relocation_history_count = 1;
    }
    last_relocation_robots[relocation_history_count++] = pos;
}
#if 0
void initiate_simple_relocation(void) {
    printf("=== 启动简单让位 ===\n");
    
    struct Hex candidates[50];
    int candidate_count = 0;
    
    // 收集所有候选机器人
    for (int i = 0; i < mydata->lattice_shape_size; i++) {
        struct Hex pos = mydata->lattice_shape[i];
        
        if (is_occupied[pos.q + 100][pos.r + 100] &&
            can_robot_move(pos)) {
            
            candidates[candidate_count++] = pos;
        }
    }
    
    if (candidate_count == 0) {
        printf("❌ 没有找到可以移动的机器人\n");
        return;
    }
    
    // 随机选择一个，打破振荡模式
    int random_index = rand() % candidate_count;
    struct Hex selected = candidates[random_index];
    
    printf("随机选择机器人 %d at (%d,%d) 进行让位\n", 
           get_robot_id_at(selected.q, selected.r), selected.q, selected.r);
    set_global_relocation_request(selected, (struct Hex){0,0});
}
#else 
void initiate_simple_relocation(void) {
    printf("=== 启动简单让位 ===\n");
    
    // 遍历所有形状位置，找到第一个可以移动的机器人
    for (int i = 0; i < mydata->lattice_shape_size; i++) {
        struct Hex pos = mydata->lattice_shape[i];
        printf("正在遍历机器人 位置（%d，%d）\n",  pos.q, pos.r);
        
        // 如果这个位置被占用且不是机器人0
#if 0
        if (is_occupied[pos.q + 100][pos.r + 100] && !(pos.q == -3 && pos.r == 1)) {
#else
        if (is_occupied[pos.q + 100][pos.r + 100]) { 
#endif   
            // 检查这个机器人是否能移动
            if (can_robot_move(pos)) {
                if (!was_recently_relocated(pos)){
                printf("选择机器人 %d at (%d,%d) 进行让位\n", get_robot_id_at(pos.q, pos.r), pos.q, pos.r);
                add_to_relocation_history(pos);
                set_global_relocation_request(pos,(struct Hex){0,0});
                return;}
            }
        }

        if(pos.q == 0 || pos.r == 3){
            printf("%d 占有\n",is_occupied[pos.q + 100][pos.r + 100]);
        }
    }
    
    printf("❌ 没有找到可以移动的机器人\n");
}
#endif

// 寻找形状位置状态
void findShapePositionState() {

    set_global_vacancy(mydata->hex_q,mydata->hex_r);
    
    int best_index = find_optimal_shape_position_index();
    printf("Robot %d: found best shape index = %d\n", kilo_uid, best_index);
    
    if (best_index != -1) {
        mydata->target_shape_index = best_index;
        mydata->target_q = mydata->lattice_shape[best_index].q;
        mydata->target_r = mydata->lattice_shape[best_index].r;
        last_find_from.q = mydata->original_position.q; 
        last_find_from.r = mydata->original_position.r; 
        last_find_to.q = mydata->target_q; 
        last_find_to.r = mydata->target_r; 
        
        /*
        struct Cartesian target_cart = hex_to_Cart(mydata->lattice_shape[best_index]);
        mydata->target_cart_x = target_cart.x;
        mydata->target_cart_y = target_cart.y;
        
        printf("Robot %d: moving to shape position (%d, %d) -> cartesian (%.1f, %.1f)\n", 
               kilo_uid, mydata->target_hex_q, mydata->target_hex_r, 
               mydata->target_cart_x, mydata->target_cart_y);
        */ 
        shape_entry_in_progress = 1;
        current_shape_entry_robot = kilo_uid;
        last_shape_entry_tick = kilo_ticks;
        current_formation_phase = 0;
        relocation_chain_complete = 0;
        printf("=== 新一轮形状进入开始 ===\n"); 
        num_rangwei =  0;   
        
        set_bot_state(MOVE_TO_SHAPE);
    } else {
        // 找不到开始让位
        if(num_rangwei >500){
            should_move_to_shape = true;
            clear_global_vacancy();
            set_bot_state(IDLE);
            printf("Robot %d: 超过次数，新的进入\n", kilo_uid);
            set_move_type(STOP);
            omni_stop();
        }else{
            num_rangwei++;
            initiate_simple_relocation();
            clear_global_vacancy();
            set_bot_state(IDLE);
            printf("Robot %d: 没找到合适的，开始让位\n", kilo_uid);
            set_move_type(STOP);
            omni_stop();
        }
    }
}

/*-------功能：移动阶段 --------*/

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

// 向形状移动状态
#if 0
void moveToShapeState() {
    static int move_attempts = 0;
    static struct Hex current_target;
    move_attempts++;
    
    //printf("Robot %d: === MOVE_TO_SHAPE (attempt %d) ===\n", kilo_uid, move_attempts);
    
    if (move_attempts == 1) {
        current_target.q = mydata->target_q;
        current_target.r = mydata->target_r;
    }
    struct Hex current_hex = cart_to_hex((struct Cartesian){kilo_x,kilo_y});
    
    //printf("Robot %d: 从当前的 (%d,%d)(%.1f,%.1f) 移动到 (%d,%d)\n", kilo_uid, current_hex.q, current_hex.r,kilo_x, kilo_y, mydata->target_q, mydata->target_r);

    int result = omni_move_to_lattice(&current_target);
    if(result){
        if (current_target.q != mydata->target_q || current_target.r != mydata->target_r) {
            current_target.q = mydata->target_q;
            current_target.r = mydata->target_r;

            printf("Robot %d: 到达中间点，继续向最终目标移动\n", kilo_uid);
        }else{
            /*
            struct Hex final_alogrithm_hex = my_nearest_lattice();

            printf("Robot %d: FINAL POSITION: (%d,%d)\n", kilo_uid, final_alogrithm_hex.q, final_alogrithm_hex.r);
            
            if (final_alogrithm_hex.q == target_hex.q && final_alogrithm_hex.r == target_hex.r && final_alogrithm_hex.q == current_hex.q && final_alogrithm_hex.r == current_hex.r) {
                printf("最终确定 %d: ✅到达\n", kilo_uid);
            } else {
                printf("Robot %d: ❌ FAILED - Wrong final position!\n", kilo_uid);
            }
            */
            global_relocation_request = (struct Hex){99,99};
            update_occupancy(mydata->target_shape_index, 1);
            mydata->shape_position_occupied = 1;
            shape_entry_in_progress = 0;
            occupied++;

            confirm_global_vacancy();
            set_bot_state(IDLE);
        }
    }

    // 更新坐标
    struct Hex cur_hex = cart_to_hex((struct Cartesian){kilo_x, kilo_y});
    mydata->hex_q = cur_hex.q;
    mydata->hex_r = cur_hex.r;


}

#else
void moveToShapeState() {
    static int move_attempts = 0;
    static struct Hex current_target = {0, 0};
    move_attempts++;
    
    //printf("Robot %d: === MOVE_TO_SHAPE (attempt %d) ===\n", kilo_uid, move_attempts);
    
    // debug_collision_detection(current_target);

    current_target.q = mydata->target_q;
    current_target.r = mydata->target_r;
    
    struct Hex current_hex = cart_to_hex((struct Cartesian){kilo_x,kilo_y});
    
    //printf("Robot %d: 从当前的 (%d,%d)(%.1f,%.1f) 移动到 (%d,%d)\n", kilo_uid, current_hex.q, current_hex.r,kilo_x, kilo_y, mydata->target_q, mydata->target_r);
#if 1

    if (is_collision_imminent(current_target)) {
        //printf("Robot %d: 路径被阻挡，寻找替代路径\n", kilo_uid);
        current_target = find_alternative_path(current_hex, current_target);
    }

    if (!is_position_in_shape(current_target)){
        // 不在形状内
        printf("不在形状内\n");
        request_shape_robots_to_relocate((struct Hex){mydata->hex_q,mydata->hex_r}, (struct Hex){mydata->target_q,mydata->target_r});
        return;
    }
    //printf("当前点机器人 %d 当前点为(%d,%d) 当前目标点 (%d,%d), 最终目标点(%d,%d)\n", kilo_uid,mydata->hex_q,mydata->hex_r,current_target.q, current_target.r,mydata->target_q,mydata->target_r);

    if(global_relocation_request.q != 99 && global_relocation_request.r != 99){// || (current_target.q == 99 && current_target.r == 99)){
        // 这里进行下一轮找位置
        set_bot_state(IDLE);
        printf("返回\n");
        return;
    }


#endif
    int result = omni_move_to_lattice(&current_target);
    if(result){
        if (current_target.q != mydata->target_q || current_target.r != mydata->target_r) {
            current_target.q = mydata->target_q;
            current_target.r = mydata->target_r;
            printf("Robot %d: 到达中间点，继续向最终目标移动\n", kilo_uid);
        }else{
            /*
            struct Hex final_alogrithm_hex = my_nearest_lattice();

            printf("Robot %d: FINAL POSITION: (%d,%d)\n", kilo_uid, final_alogrithm_hex.q, final_alogrithm_hex.r);
            
            if (final_alogrithm_hex.q == target_hex.q && final_alogrithm_hex.r == target_hex.r && final_alogrithm_hex.q == current_hex.q && final_alogrithm_hex.r == current_hex.r) {
                printf("最终确定 %d: ✅到达\n", kilo_uid);
            } else {
                printf("Robot %d: ❌ FAILED - Wrong final position!\n", kilo_uid);
            }
            */
            if(mydata->original_position.q >= -100 && mydata->original_position.q<= 100
            && mydata->original_position.r >= -100 && mydata->original_position.r<= 100){
                is_occupied[mydata->original_position.q + 100][mydata->original_position.r + 100] = 0;
            }
            struct Hex update_hex = my_nearest_lattice();
            mydata->hex_q = update_hex.q;
            mydata->hex_r = update_hex.r;
            mydata->original_position.q = mydata->hex_q;
            mydata->original_position.r = mydata->hex_r;
            printf("Robot %d: 已经到达位置 (%d.%d), 下一步补位\n",kilo_uid,mydata->hex_q,mydata->hex_r);
            update_occupancy(mydata->target_shape_index, 1);
            mydata->shape_position_occupied = 1;
            
            shape_entry_in_progress = 0;
            occupied++;


            confirm_global_vacancy();
            set_bot_state(MAINTAIN_POSITION);
        }
    }
}
#endif 
/* ----------功能：补位-----------*/

// 检查链式补位机会，实际上形状内的也需要补位。
void checkChainRelocationOpportunity(void) {
    //if (!mydata->formation_initialized) return;

    // 只有形状外的机器人才参与链式补位
    if (mydata->shape_position_occupied) 
    {
        return; // 已经在形状内，不参与补位
    }

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
    
    //printf("Robot %d: 🔍 Checking chain relocation at (%d,%d) [OUTSIDE SHAPE]...\n", \
           kilo_uid, mydata->hex_q, mydata->hex_r);
    
    // 寻找可以移动到的位置（邻近形状或邻近其他即将移动的机器人）
#if 0
    struct Hex target_pos = find_chain_relocation_target();
    
    if (target_pos.q != 99 && target_pos.r != 99) {
        printf("Robot %d: 🚀 Found chain relocation target (%d,%d)\n", 
               kilo_uid, target_pos.q, target_pos.r);
        initiate_chain_relocation_to_position(target_pos);
    }
#else
    struct Hex my_hex = {mydata->hex_q, mydata->hex_r};

    if(is_collision_imminent(global_vacancy)){
        return;
    }


    // 不相邻就不补位
    if (!is_hex_adjacent(my_hex, global_vacancy)) {   
        return; // 不相邻，不参与补位
    }
        
    // 与形状距离为1 不补位
#if 1
    struct Hex target = find_nearest_unoccupied_target();
    if (is_hex_adjacent(my_hex, target)) {
        printf("该点与形状距离为1不补位\n");
        return; // 相邻，不参与补位
    }
#endif

/*
#if 0
    if( my_hex.q == global_vacancy.q){
#else
    if(my_hex.r == global_vacancy.r){
#endif

      return;
    }
*/



    relocation_chain_complete = 0;
    // 符合条件，该点需要去补位
    last_relocation_from.q = mydata->hex_q;
    last_relocation_from.r = mydata->hex_r;
    last_relocation_to.q = global_vacancy.q;
    last_relocation_to.r = global_vacancy.r;
    printf("补位机器人 %d： （%d,%d）全局空位（%d，%d）\n", kilo_uid, mydata->hex_q,mydata->hex_r,global_vacancy.q,global_vacancy.r);
    mydata->original_position.q = mydata->hex_q;
    mydata->original_position.r = mydata->hex_r;

    any_robot_relocating = 1;
    relocating_robot_id = kilo_uid;

    set_bot_state(CHAIN_RELOCATION);
    should_move_to_shape = false;


    #endif
}

// 链式补位状态
void chainRelocationState() {
    //printf("机器人 %d 还在补位 原位置（%d，%d）\n", kilo_uid,mydata->o_original_position.q,mydata->o_original_position.r);
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

        // 触发新的形状进入
        current_formation_phase = 0;
        return;
    }
    // 🔧 更新补位活动时间戳
    /*
    if (kilo_uid == 0) {
        last_relocation_activity = kilo_ticks;
        relocation_chain_complete = 0;
    }
    */

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

        /*
        struct Hex point_in_Hex = my_nearest_lattice();
        mydata->hex_q = point_in_Hex.q;
        mydata->hex_r = point_in_Hex.r;
        */

        // 更新坐标
        struct Hex cur_hex = cart_to_hex((struct Cartesian){kilo_x, kilo_y});
        mydata->hex_q = cur_hex.q;
        mydata->hex_r = cur_hex.r;

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
        printf("Robot %d 补位完成，更新补位活动时间 %d\n", kilo_uid,last_relocation_activity);
    }else{
        last_relocation_activity = kilo_ticks;
    }
}


bool is_relocation_requested_for_me(void) {
    if (!relocation_request_active) return false;
    
    struct Hex my_hex = {mydata->hex_q, mydata->hex_r};
    return (my_hex.q == global_relocation_request.q && 
            my_hex.r == global_relocation_request.r);
}

void check_relocation_request(void) {
    // 检查是否有让位请求针对自己
    if (is_relocation_requested_for_me()) {
        printf("Robot %d: 收到让位请求，寻找新位置\n", kilo_uid);
        
        // 寻找可移动的空闲形状位置
        struct Hex new_position = find_nearest_unoccupied_target();
        
        if (new_position.q != 99 && new_position.r != 99) {
            printf("Robot %d: 让位到新位置 (%d,%d)\n", kilo_uid, new_position.q, new_position.r);
            
            // 释放当前位置
            update_occupancy_for_hex((struct Hex){mydata->hex_q, mydata->hex_r}, 0);
            
            // 移动到新位置
            mydata->target_q = new_position.q;
            mydata->target_r = new_position.r;
            
            // 找到对应的形状索引
            for (int i = 0; i < mydata->lattice_shape_size; i++) {
                if (mydata->lattice_shape[i].q == new_position.q && 
                    mydata->lattice_shape[i].r == new_position.r) {
                    mydata->target_shape_index = i;
                    break;
                }
            }
            
            set_bot_state(MOVE_TO_SHAPE);
        } else {
            printf("Robot %d: 无法让位，没有可用位置\n", kilo_uid);
        }
    }
}

/* ------- 补位检测 ------  */
void check_relocation_chain_completion(void) {
    if(kilo_ticks / 100 == 0){
        //printf("进行补位检测\n");
    }
    // 只有Robot 0负责检测全局进度
    //if (kilo_uid != 0) return;

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
    if (kilo_ticks - last_relocation_activity > 1000) {
        //printf("=== 补位链已完成，切换到下一轮形状进入 ===\n");
        
        // 重置状态，开始新一轮
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