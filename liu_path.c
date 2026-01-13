
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <kilombo.h>
#include "formation.h"
#include "liu_path.h"

/* ---------- swarm ----------*/
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

/* ---------辅助函数-------- */


uint16_t calculate_follower_id(uint16_t robot_id, uint16_t total_robots) {
    uint16_t group_count = 6;
    uint16_t follower_id = robot_id + group_count;
    return (follower_id < total_robots) ? follower_id : 0xFFFF;
}

uint16_t calculate_leader_id(uint16_t robot_id, uint16_t total_robots) {
    uint16_t group_count = 6;
    return (robot_id >= group_count) ? (robot_id - group_count) : 0xFFFF;
}

void initialize_chain_system(uint16_t total_robots) {
    mydata->leader_id = calculate_leader_id(kilo_uid, total_robots);
    mydata->follower_id = calculate_follower_id(kilo_uid, total_robots);
    mydata->is_chain_leader = (kilo_uid < 6);
    mydata->chain_position = kilo_uid / 6;
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

bool should_start_finding_now() {
    // 基于ID和时间的随机退避
    uint32_t base_delay = (kilo_uid % 10) * 50;  // 基于ID的延迟
    return (kilo_ticks % 200) > base_delay;      // 每200个tick重新评估
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

//  检查位置是否被邻居占据
bool is_position_occupied_by_neighbor(struct Hex target) {
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        if (mydata->neighbors[i].shape_position_occupied &&
            mydata->neighbors[i].hex_q == target.q &&
            mydata->neighbors[i].hex_r == target.r) {
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

// 判断输入点是否与给定点集中的任意一点相邻（距离为1）
bool is_adjacent_to_boundary(struct Hex input) {
    // 定义目标点集
    struct Hex target_points[] = {
        {-3, 1}, {-2, 1}, {-1, 1}, 
        {0, 1}, {1, 1}, {2, 1}
    };
    int num_points = sizeof(target_points) / sizeof(target_points[0]);
    
    // 检查输入点是否与任意目标点相邻
    for (int i = 0; i < num_points; i++) {
        if (is_hex_adjacent(input, target_points[i])) {
            return true;
        }
    }
    
    return false;
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


double hex_distance(int q1, int r1, int q2, int r2) {
    return (abs(q1 - q2) + abs(q1 + r1 - q2 - r2) + abs(r1 - r2)) / 2.0;
}


/* ---------------------------- 补位 ---------------------------- */
// 当机器人移动到形状位置时，发布空缺信息
void publish_vacancy_after_movement(struct Hex old_position) {
    mydata->known_vacancy = old_position;
    mydata->vacancy_timestamp = kilo_ticks;
    printf("Robot %d: 发布空缺位置 (%d,%d)\n", kilo_uid, old_position.q, old_position.r);
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

// 考虑消息延迟
bool can_safely_start_movement() {
    // 如果还没有目标，则不能移动
    if (mydata->intended_target_q == 99 && mydata->intended_target_r == 99) {
        return false;
    }

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
            mydata->neighbors[i].intended_target_q == mydata->target_q &&
            mydata->neighbors[i].intended_target_r == mydata->target_r) {
            printf("Robot %d: 邻居 %d 有相同移动意图，协调中...\n",
                   kilo_uid, mydata->neighbors[i].ID);
            return resolve_movement_conflict(mydata->neighbors[i].ID);
        }
    }
    return true;
}

// 增加随机退避机制
bool can_safely_move_enhanced() {
    if (!can_safely_move()) {
        return false;
    }
    
    // 随机退避，避免多个机器人同时开始
    uint32_t random_backoff = (kilo_uid % 5) * 10; // 基于ID的随机延迟
    if (kilo_ticks % 100 < random_backoff) {
        return false;
    }
    
    return true;
}

// 开始移动前的准备
void prepare_for_movement(struct Hex target) {
    mydata->has_movement_intent = 1;
    mydata->intended_target_q = target.q;
    mydata->intended_target_r = target.r;
    mydata->target_intent_time = kilo_ticks;
    mydata->movement_start_time = kilo_ticks;
    
    // 等待一小段时间让意图传播
    if (kilo_ticks - mydata->last_movement_check > 50) {
        if (can_safely_move_enhanced()) {
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
    mydata->intended_target_q = 99;
    mydata->intended_target_r = 99;
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

/* -------功能：准备阶段---------*/


// 检查位置是否为空且可用
bool is_position_empty_and_available(int q, int r) {
    // 检查是否在形状内
    if (!is_position_in_shape((struct Hex){q, r})) {
        return false;
    }
    

    // 检查是否被占据,不能用全局，
    if (is_position_occupied_by_neighbor((struct Hex){q, r})) {
        return false;
    }
    
    // 检查是否在排除列表
    if ((q == -3 && r == 4) || (q == -2 && r == 4) || 
        (q == -2 && r == 3) || (q == -1 && r == 3)) {
        return false;
    }
    
    // 检查是否可以定位
    if (!check_target_localizability((struct Hex){mydata->hex_q, mydata->hex_r}, 
                                    (struct Hex){q, r})) {
        return false;
    }
    
    // 检查是否在移动历史中（避免振荡）
    if (is_position_in_history((struct Hex){q, r})) {
        return false;
    }
    
    return true;
}


// 检查是否有可用的空位

// 检查是否有可用的空位（分布式版本）
bool has_available_vacancies() {
    // 方法1：检查已知的空缺信息
    if (mydata->known_vacancy.q != 99 && mydata->known_vacancy.r != 99) {
        // 检查空缺信息是否过时
        if (kilo_ticks - mydata->vacancy_timestamp < 200) {
            // 🔧 修改：检查这个空缺是否仍然可用
            if (is_position_empty_and_available(mydata->known_vacancy.q, mydata->known_vacancy.r)) {
                printf("第 %lu 次 --- 检查阶段 Robot %d: 已知空缺位置 (%d,%d) 仍然可用\n", 
                       kilo_ticks, kilo_uid, mydata->known_vacancy.q, mydata->known_vacancy.r);
                return true;
            } else {
                printf("Robot %d: 已知空缺位置 (%d,%d) 已不可用\n", 
                       kilo_uid, mydata->known_vacancy.q, mydata->known_vacancy.r);
                // 清除过时的空缺信息
                mydata->known_vacancy = (struct Hex){99, 99};
            }
        }
    }
    
    // 方法2：扫描通信范围内的空位
    const int SCAN_RANGE = 1;
    int self_q = mydata->hex_q;
    int self_r = mydata->hex_r;
    
    int available_count = 0;
    
    for (int dq = -SCAN_RANGE; dq <= SCAN_RANGE; dq++) {
        for (int dr = -SCAN_RANGE; dr <= SCAN_RANGE; dr++) {
            int q = self_q + dq;
            int r = self_r + dr;
            
            // 跳过超出通信范围的点
            if (hex_distance(self_q, self_r, q, r) > SCAN_RANGE) continue;
            
            // 检查是否是空位
            if (is_position_empty_and_available(q, r)) {
                printf("第 %lu 次 --- 检查阶段:Robot %d: 发现空位 (%d,%d)\n", kilo_ticks, kilo_uid, q, r);
                available_count++;
                // 记录这个空缺信息
                mydata->known_vacancy.q = q;
                mydata->known_vacancy.r = r;
                mydata->vacancy_timestamp = kilo_ticks;
                return true; // 找到一个就返回
            }
        }
    }
    
    // 方法3：检查邻居的空缺信息
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        if (mydata->neighbors[i].known_vacancy.q != 99 && 
            mydata->neighbors[i].known_vacancy.r != 99) {
            
            // 检查邻居的空缺信息是否过时
            if (kilo_ticks - mydata->neighbors[i].vacancy_timestamp < 200) {
                int q = mydata->neighbors[i].known_vacancy.q;
                int r = mydata->neighbors[i].known_vacancy.r;
                
                // 检查这个空缺是否可用
                if (is_position_empty_and_available(q, r)) {
                    printf("Robot %d: 邻居 %d 报告空缺位置 (%d,%d) 可用\n", 
                           kilo_uid, mydata->neighbors[i].ID, q, r);
                    // 更新自己的空缺信息
                    mydata->known_vacancy.q = q;
                    mydata->known_vacancy.r = r;
                    mydata->vacancy_timestamp = kilo_ticks;
                    return true;
                }
            }
        }
    }
    
    //printf("Robot %d: 未发现可用空位 (扫描了 %d 个位置)\n", kilo_uid, available_count);
    return false;
}


// 检查是否可以开始查找（更宽松的条件）
bool can_start_finding_enhanced() {
    // 如果已经在移动过程中，不重复触发
    if (get_bot_state() != IDLE) {
        return false;
    }
    
    // 检查是否有可用空位
    if (!has_available_vacancies()) {
        return false;
    }
#if 0
    // 基于ID和时间的随机退避，避免所有机器人同时开始
    uint32_t random_delay = (kilo_uid % 8) * 30;
    return (kilo_ticks % 400) > (200 + random_delay);+
#else
    return true;
#endif
}

// 第二阶段检查：是否可以开始移动（严格条件）
bool can_start_moving() {
    // 检查邻居中是否有正在移动的
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        if (mydata->neighbors[i].is_moving) {
            printf("Robot %d: 邻居 %d 正在移动，等待...\n", 
                   kilo_uid, mydata->neighbors[i].ID);
            return false;
        }
        
        // 考虑消息延迟
        if (mydata->neighbors[i].has_movement_intent &&
            kilo_ticks - mydata->neighbors[i].timestamp < 60) {
            printf("Robot %d: 邻居 %d 最近有移动意图，等待...\n",
                   kilo_uid, mydata->neighbors[i].ID);
            return false;
        }
    }
    return true;
}

/* -------功能：寻找阶段---------*/

// 检查目标是否已被邻居声明
bool is_target_claimed_by_neighbor(int q, int r) {
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        if (mydata->neighbors[i].has_movement_intent &&
            mydata->neighbors[i].intended_target_q == q &&
            mydata->neighbors[i].intended_target_r == r) {
            
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

// 在移动前进行最终验证
bool verify_target_availability(struct Hex target) {
    // 方法1：检查物理距离上是否有机器人已经在这个位置
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        struct Cartesian target_cart = hex_to_Cart(target);
        double dist = sqrt(pow(target_cart.x - mydata->neighbors[i].x, 2) + 
                          pow(target_cart.y - mydata->neighbors[i].y, 2));
        
        // 如果邻居物理上很接近目标位置，说明可能已经占据
        if (dist < kilo_lattice_size * 0.7) { // 70% 的格子大小
            printf("Robot %d: 目标位置 (%d,%d) 附近有邻居 %d，距离=%.1f\n",
                   kilo_uid, target.q, target.r, mydata->neighbors[i].ID, dist);
            return false;
        }
    }
    
    return true;
}

struct Hex find_nearest_unoccupied_target_distributed() {
    struct Hex best = {99, 99};
    double best_dist = INFINITY;

    const int COMM_RANGE = 1;
    int self_q = mydata->hex_q;
    int self_r = mydata->hex_r;

    for (int dq = -COMM_RANGE; dq <= COMM_RANGE; dq++) {
        for (int dr = -COMM_RANGE; dr <= COMM_RANGE; dr++) {
            int q = self_q + dq;
            int r = self_r + dr;

            // 选中自己了
            if (q == self_q && r == self_r) continue;

            // 跳过超出通信范围的点
            if (hex_distance(self_q, self_r, q, r) > COMM_RANGE) continue;

            // 往深处走
            if (r < self_r) continue;

            // 碰撞检测
            if(is_collision_imminent((struct Hex){self_q,self_r},(struct Hex){q,r})) continue;

            // 不在形状内
            if (!is_position_in_shape((struct Hex){q,r})) continue;

            // 被占据，这个不能要，应该改为邻居是否选中
            if (!is_position_empty_and_available(q, r)) continue;

            // 上一次的位置
            if (mydata->original_position.q == q && mydata->original_position.r == r) continue;
            if (is_position_in_history((struct Hex){q,r})) continue;

            // 中间空心点
            if ((q == -3 && r == 4) || (q == -2 && r == 4) || 
                (q == -2 && r == 3) || (q == -1 && r == 3)) continue;

            // 定位            
            if (!check_target_localizability((struct Hex){self_q, self_r}, (struct Hex){q, r})) continue;

            // 🔧 新增：检查邻居是否已经声明了这个目标
            if (is_target_claimed_by_neighbor(q, r)) {
                printf("邻居已经声明\n");
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

    printf("第 %lu 次 ---Robot %d: 找到目标点 (%d, %d), 距离=%.1f\n", kilo_ticks, kilo_uid, best.q, best.r, best_dist);
    return best;
}
#if 0
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
#endif

// 检查是否可以声明目标
bool can_claim_target(struct Hex target) {
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        printf("声明阶段,检查邻居\n");
        printf("邻居 id: %d\n",mydata->neighbors[i].ID);
        printf("邻居 位置: (%d,%d)\n",mydata->neighbors[i].hex_q,mydata->neighbors[i].hex_r);
        printf("邻居 意图: %d\n",mydata->neighbors[i].has_movement_intent);
        printf("邻居 目标: (%d,%d)\n",mydata->neighbors[i].intended_target_q,mydata->neighbors[i].intended_target_r);
        if (mydata->neighbors[i].has_movement_intent &&
            mydata->neighbors[i].intended_target_q == target.q &&
            mydata->neighbors[i].intended_target_r == target.r) {
            printf("邻居的意图 %d\n",mydata->neighbors[i].has_movement_intent);
            // 冲突解决：时间优先,ID次优
            if (mydata->neighbors[i].target_intent_time < mydata->target_intent_time) {
                set_bot_state(WAITTING);
                printf("邻居的时间更早\n");
                return false; // 邻居声明更早
            } else if (mydata->neighbors[i].target_intent_time == mydata->target_intent_time &&
                       mydata->neighbors[i].ID < kilo_uid) {
                set_bot_state(WAITTING);
                printf("邻居的 id 更小\n");
                return false; // 同时声明，但邻居ID更小
            }else{
                printf("我的意图更优\n");
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

// 在移动过程中处理冲突
void handle_movement_conflict(struct Hex conflict_position) {
    printf("Robot %d: 检测到移动冲突 at (%d,%d)，重新规划\n", 
           kilo_uid, conflict_position.q, conflict_position.r);
    
    // 停止当前移动
    finish_movement();
    
    // 标记该位置为被占据

    
    // 重新选择目标
    set_bot_state(FIND_SHAPE_POSITION);
}


// 检查本地维护的占据信息
bool is_occupied_locally(struct Hex target) {
    // 自己就占着这个格
    if (mydata->hex_q == target.q && mydata->hex_r == target.r)
        return true;

    for (int i = 0; i < mydata->N_Neighbors; i++) {

        // 邻居当前格子等于目标
        if (mydata->neighbors[i].hex_q == target.q && mydata->neighbors[i].hex_r == target.r)
            return true;

        // 邻居正移动到目标
        if (mydata->neighbors[i].is_moving && mydata->neighbors[i].intended_target_q == target.q && mydata->neighbors[i].intended_target_r == target.r)
            return true;
    }

    return false;
}


// 检查目标是否仍然可用（分布式版本）
bool is_target_still_available(struct Hex target) {
    // 方法1：检查本地维护的占据信息（通过邻居消息更新）
    if (is_occupied_locally(target)) {
        return false;
    }
    
    // 方法2：检查邻居是否正在移动到这个目标
    if (is_target_claimed_by_neighbor(target.q, target.r)) {
        return false;
    }
    
    // 方法3：检查邻居是否已经占据这个位置
    if (is_position_occupied_by_neighbor(target)) {
        return false;
    }
    
    // 检查是否在形状内
    if (!is_position_in_shape(target)) {
        return false;
    }
    
    return true;
}

/* ---------- 追踪补位--------------*/
void track_leader(){
    if (mydata->is_chain_leader) {
        return; // 领导者不需要跟踪自己
    }
    
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        if (mydata->neighbors[i].ID == mydata->leader_id) {
            // 更新领导者信息
            mydata->leader_current_position_q = mydata->neighbors[i].hex_q;
            mydata->leader_current_position_r = mydata->neighbors[i].hex_r;
            mydata->leader_is_moving = mydata->neighbors[i].is_moving;
            
            // 如果领导者有移动意图，记录其目标位置
            if (mydata->neighbors[i].has_movement_intent) {
                mydata->leader_target_position_q = mydata->neighbors[i].intended_target_q;
                mydata->leader_target_position_r = mydata->neighbors[i].intended_target_r;
            }
            
            break;
        }
    }
}

// 检查是否应该开始跟随领导者移动
bool should_follow_leader_movement() {
    if (mydata->is_chain_leader || mydata->shape_position_occupied) {
        return false;
    }
    
    // 检查领导者是否正在移动或有移动意图
    if (mydata->leader_is_moving || 
        (mydata->neighbors[i].has_movement_intent && 
         mydata->neighbors[i].ID == mydata->leader_id)) {
        
        // 检查领导者是否已经离开了原位置
        struct Hex leader_old_position = {99,99};
        leader_old_position.q = mydata->leader_current_position_q;
        leader_old_position.r = mydata->leader_current_position_r;
        if (mydata->neighbors[i].hex_q != leader_old_position.q || 
            mydata->neighbors[i].hex_r != leader_old_position.r) {
            
            printf("Robot %d: 领导者 %d 已移动，开始跟随\n", kilo_uid, mydata->leader_id);
            return true;
        }
    }
    
    return false;
}

// 链式跟随状态
void chainFollowingState() {
    static uint32_t last_follow_check = 0;
    
    // 限制检查频率
    if (kilo_ticks - last_follow_check < 20) return;
    last_follow_check = kilo_ticks;
    
    printf("Robot %d: 在链式跟随状态，跟踪领导者 %d\n", kilo_uid, mydata->leader_id);
    
    // 持续跟踪领导者状态
    track_leader_status();
    
    // 检查领导者是否还在移动
    if (!mydata->leader_is_moving && 
        !has_leader_movement_intent()) {
        printf("Robot %d: 领导者已停止移动，结束跟随\n", kilo_uid);
        set_bot_state(IDLE);
        return;
    }
    
    struct Hex target = (struct Hex){mydata->leader_current_position_q, mydata->leader_current_position_r};
    // 移动到领导者的原位置
    if (omni_move_to_lattice(&target) == 1) {
        printf("Robot %d: 已移动到领导者的原位置 (%d,%d)\n", 
               kilo_uid, mydata->leader_current_position.q, mydata->leader_current_position.r);
        
        // 更新自己的位置
        global_localization();
        struct Hex cur_hex = cart_to_hex((struct Cartesian){mydata->x, mydata->y});
        mydata->hex_q = cur_hex.q;
        mydata->hex_r = cur_hex.r;
        
        // 发布自己的原位置作为新的空缺，供下一级跟随者使用
        struct Hex my_old_position = mydata->original_position;
        mydata->original_position = (struct Hex){mydata->hex_q, mydata->hex_r};
        
        if (!is_position_in_shape(my_old_position)) {
            publish_vacancy_after_movement(my_old_position);
            printf("Robot %d: 发布原位置空缺 (%d,%d)\n", 
                   kilo_uid, my_old_position.q, my_old_position.r);
        }
        
        set_bot_state(IDLE);
    }
}

#if 1
void findShapePositionState_distributed() {
    
    // 第一阶段：选择目标并声明意图
    // 还没有选择过
    if (!mydata->has_movement_intent) {
        struct Hex selected_target = find_nearest_unoccupied_target_distributed();
        
        if (selected_target.q != 99 && selected_target.r != 99) {
            // 声明目标意图
            mydata->has_movement_intent = 1;
            mydata->intended_target_q = selected_target.q;
            mydata->intended_target_r = selected_target.r;
            mydata->target_intent_time = kilo_ticks;
            
            printf("第 %lu 次 --- Robot %d: 声明目标意图 (%d,%d)\n", kilo_ticks, kilo_uid, selected_target.q, selected_target.r);
        } else {
            // 没有找到合适目标
            //printf("Robot %d: 没有找到合适目标\n", kilo_uid);
            set_bot_state(IDLE);
            return;
        }
    }
    
    // 第二阶段：等待目标确认
    if (mydata->has_movement_intent) {
        struct Hex current_target = (struct Hex){99,99};
        current_target.q = mydata->intended_target_q;
        current_target.r = mydata->intended_target_r;
        // 检查目标是否仍然可用
        if (!is_target_still_available(current_target) || 
                !verify_target_availability(current_target )) {
            printf("Robot %d: 目标 (%d,%d) 已被占用，重新选择\n", 
                   kilo_uid, current_target.q, current_target.r);
            mydata->has_movement_intent = 0;
            mydata->intended_target_q = 99;
            mydata->intended_target_r = 99;
            return;
        }
        
        // 等待一段时间让意图传播并解决冲突
        if (kilo_ticks - mydata->target_intent_time > 100) { // 等待100个tick
            if (can_claim_target(current_target)) {
                // 成功获得目标，开始移动准备
                mydata->target_shape_index = find_shape_index(current_target);
                mydata->target_q = current_target.q;
                mydata->target_r = current_target.r;
                
                printf("Robot %d: 成功获得目标 (%d,%d)，准备移动\n", 
                       kilo_uid, current_target.q, current_target.r);
                
                mydata->has_movement_intent = 0; // 清除目标意图
                set_bot_state(PLAN_MOVEMENT);
            } else {
                // 目标冲突，重新选择
                printf("Robot %d: 目标冲突，重新选择\n", kilo_uid);
                mydata->has_movement_intent = 0;
                mydata->intended_target_q = 99;
                mydata->intended_target_r = 99;
            }
        }
    }
}
#else
void findShapePositionState_distributed() {
    
    printf("Robot %d: 在 FIND_SHAPE_POSITION 状态\n", kilo_uid);
    
    // 第一阶段：选择目标并声明意图
    if (!mydata->has_movement_intent) {
        struct Hex selected_target = find_nearest_unoccupied_target_distributed();
        
        if (selected_target.q != 99 && selected_target.r != 99) {
            // 声明目标意图
            mydata->has_movement_intent = 1;
            mydata->intended_target_q = selected_target.q;
            mydata->intended_target_r = selected_target.r;
            mydata->target_intent_time = kilo_ticks;
            
            printf("Robot %d: ✅ 声明目标意图 (%d,%d)\n", kilo_uid, selected_target.q, selected_target.r);
        } else {
            // 没有找到合适目标，回到IDLE状态
            printf("Robot %d: ❌ 没有找到合适目标，返回IDLE\n", kilo_uid);
            set_bot_state(IDLE);
            return;
        }
    }
    
    // 第二阶段：等待目标确认
    if (mydata->has_movement_intent) {
        struct Hex current_target = (struct Hex){mydata->intended_target_q,mydata->intended_target_r};
        
        printf("Robot %d: 检查目标 (%d,%d) 可用性\n", kilo_uid, current_target.q, current_target.r);
        
        // 检查目标是否仍然可用
        if (!is_target_still_available(current_target)) {
            printf("Robot %d: ❌ 目标 (%d,%d) 已被占用，重新选择\n", 
                   kilo_uid, current_target.q, current_target.r);
            mydata->has_movement_intent = 0;
            mydata->intended_target_q = 99;
            mydata->intended_target_r = 99;
            return;
        }
        
        if (!verify_target_availability(current_target)) {
            printf("Robot %d: ❌ 目标 (%d,%d) 验证失败，重新选择\n", 
                   kilo_uid, current_target.q, current_target.r);
            mydata->has_movement_intent = 0;
            mydata->intended_target_q = 99;
            mydata->intended_target_r = 99;
            return;
        }
        
        // 等待一段时间让意图传播并解决冲突
        if (kilo_ticks - mydata->target_intent_time > 100) {
            if (can_claim_target(current_target)) {
                // 成功获得目标，开始移动准备
                mydata->target_shape_index = find_shape_index(current_target);
                mydata->target_q = current_target.q;
                mydata->target_r = current_target.r;
                
                printf("Robot %d: 🚀 成功获得目标 (%d,%d)，准备移动\n", 
                       kilo_uid, current_target.q, current_target.r);
                
                mydata->has_movement_intent = 0; // 清除目标意图
                set_bot_state(MOVE_TO_SHAPE);
            } else {
                // 目标冲突，重新选择
                printf("Robot %d: ⚠️ 目标冲突，重新选择\n", kilo_uid);
                mydata->has_movement_intent = 0;
                mydata->intended_target_q = 99;
                mydata->intended_target_r = 99;
            }
        }
    }
}
#endif
// 寻找形状位置状态
#if 0
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
#endif

// 简化的路径冲突检测
bool is_path_conflict_simple(struct Cartesian my_pos, struct Cartesian my_target,
                            struct Cartesian neighbor_pos, struct Cartesian neighbor_target) {
    // 将位置转换为六边形坐标
    struct Hex my_hex = cart_to_hex(my_pos);
    struct Hex my_target_hex = cart_to_hex(my_target);
    struct Hex neighbor_hex = cart_to_hex(neighbor_pos);
    struct Hex neighbor_target_hex = cart_to_hex(neighbor_target);
    
    // 情况1：目标位置相同
    if (my_target_hex.q == neighbor_target_hex.q && 
        my_target_hex.r == neighbor_target_hex.r) {
        return true;
    }
    
    // 情况2：目标位置相邻且当前位置也相邻
    if (is_hex_adjacent(my_target_hex, neighbor_target_hex) &&
        is_hex_adjacent(my_hex, neighbor_hex)) {
        return true;
    }
    
    // 情况3：交换位置（我的目标是邻居的位置，邻居的目标是我的位置）
    if (my_target_hex.q == neighbor_hex.q && my_target_hex.r == neighbor_hex.r &&
        neighbor_target_hex.q == my_hex.q && neighbor_target_hex.r == my_hex.r) {
        return true;
    }
    
    // 情况4：物理距离很近且移动方向相向
    double current_distance = sqrt(pow(my_pos.x - neighbor_pos.x, 2) + 
                                  pow(my_pos.y - neighbor_pos.y, 2));
    if (current_distance < kilo_lattice_size * 1.5) {
        // 简单的方向判断：如果目标点在对方另一侧，可能相向
        double my_to_neighbor_dist = sqrt(pow(my_target.x - neighbor_pos.x, 2) + 
                                         pow(my_target.y - neighbor_pos.y, 2));
        double neighbor_to_my_dist = sqrt(pow(neighbor_target.x - my_pos.x, 2) + 
                                         pow(neighbor_target.y - my_pos.y, 2));
        
        if (my_to_neighbor_dist < current_distance * 0.8 && 
            neighbor_to_my_dist < current_distance * 0.8) {
            return true;
        }
    }
    
    return false;
}

/*-------功能：移动阶段 --------*/
// 更精细的冲突检测
bool detect_immediate_conflict_enhanced(int target_q, int target_r) {
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        // 检查1：目标声明冲突
        if (mydata->neighbors[i].has_movement_intent &&
            mydata->neighbors[i].intended_target_q == target_q &&
            mydata->neighbors[i].intended_target_r == target_r) {
            
            // 基于时间戳和ID的优先级
            if (mydata->neighbors[i].target_intent_time < mydata->target_intent_time) {
                return true; // 邻居声明更早
            } else if (mydata->neighbors[i].target_intent_time == mydata->target_intent_time &&
                       mydata->neighbors[i].ID < kilo_uid) {
                return true; // 同时声明，邻居ID更小
            }
        }
        
        // 检查2：路径冲突（邻居也在向同一区域移动）
        struct Cartesian my_pos = {mydata->x, mydata->y};
        struct Cartesian target_pos = hex_to_Cart((struct Hex){target_q, target_r});
        struct Cartesian neighbor_target = hex_to_Cart((struct Hex){mydata->neighbors[i].intended_target_q,mydata->neighbors[i].intended_target_r});
        
        if (is_path_conflict_simple(my_pos, target_pos, 
                            (struct Cartesian){mydata->neighbors[i].x, mydata->neighbors[i].y}, 
                            neighbor_target)) {
            printf("Robot %d: 检测到路径冲突 with neighbor %d\n", kilo_uid, mydata->neighbors[i].ID);
            return true;
        }
        
        // 检查3：物理接近冲突
        struct Cartesian target_cart = hex_to_Cart((struct Hex){target_q, target_r});
        double dist = sqrt(pow(target_cart.x - mydata->neighbors[i].x, 2) + 
                          pow(target_cart.y - mydata->neighbors[i].y, 2));
        
        if (dist < kilo_lattice_size * 0.5) {
            printf("Robot %d: 检测到物理冲突，邻居 %d 距离目标 %.1f mm\n", 
                   kilo_uid, mydata->neighbors[i].ID, dist);
            return true;
        }
    }
    return false;
}

bool detect_immediate_conflict(int target_q, int target_r) {
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        // 如果邻居也声明要移动到同一个目标
        if (mydata->neighbors[i].has_movement_intent &&
            mydata->neighbors[i].intended_target_q == target_q &&
            mydata->neighbors[i].intended_target_r == target_r) {
            
            // 基于ID解决冲突
            if (mydata->neighbors[i].ID < kilo_uid) {
                return true; // 冲突，对方优先级更高
            }
        }
        
        // 如果邻居物理上已经很接近目标
        struct Cartesian target_cart = hex_to_Cart((struct Hex){target_q, target_r});
        double dist = sqrt(pow(target_cart.x - mydata->neighbors[i].x, 2) + 
                          pow(target_cart.y - mydata->neighbors[i].y, 2));
        
        if (dist < kilo_lattice_size * 0.5) { // 50% 的格子大小
            printf("Robot %d: 检测到物理冲突，邻居 %d 距离目标 %.1f mm\n", 
                   kilo_uid, mydata->neighbors[i].ID, dist);
            return true;
        }
    }
    return false;
}

// 向形状移动状态

void planMovementState_distributed() {
    static struct Hex target;
    
    // === 0️⃣ 初始化目标 ===
    target.q = mydata->target_q;
    target.r = mydata->target_r;

    // === 1️⃣ 邻居意图抑制 ===
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        if (mydata->neighbors[i].has_movement_intent &&
            kilo_ticks - mydata->neighbors[i].target_intent_time < 250) {
            return; // 邻居刚宣告意图，自己暂缓
        }
    }

    // === 2️⃣ 随机 backoff 初始化 ===
    if (!mydata->intent_backoff_until) {
        mydata->intent_backoff_until = kilo_ticks + (rand() % 400 + 100); // 100~500 tick
        return;
    }

    // 未到 backoff 结束，不行动
    if (kilo_ticks < mydata->intent_backoff_until) return;

    // === 3️⃣ 宣告移动意图 ===
    if (!mydata->has_movement_intent) {
        mydata->has_movement_intent = 1;
        mydata->intended_target_q = target.q;
        mydata->intended_target_r = target.r;
        mydata->target_intent_time = kilo_ticks;
        printf("Robot %d: 📡 宣告移动意图 (%d,%d)\n", kilo_uid, target.q, target.r);
        return;
    }

    // === 4️⃣ 等待意图传播 ===
    if (kilo_ticks - mydata->target_intent_time < 250) {
        return; // INTENT_WAIT
    }

    // === 5️⃣ 局部并发上限 ===
    int moving_count = 0;
    for (int i = 0; i < mydata->N_Neighbors; i++) {
        if (mydata->neighbors[i].is_moving) moving_count++;
    }
    if (moving_count >= 2) {
        // 太多人动，重新 backoff
        mydata->intent_backoff_until = kilo_ticks + (rand() % 400 + 200);
        mydata->has_movement_intent = 0;
        return;
    }

    // === 6️⃣ 最终检查是否安全启动 ===
    if (can_claim_target(target) && can_safely_move_enhanced()) {
        mydata->is_moving = 1;
        mydata->has_movement_intent = 0;
        mydata->movement_start_time = kilo_ticks;
        mydata->intent_backoff_until = 0;
        printf("Robot %d: ✅ 确认安全，进入移动阶段 (%d,%d)\n",
               kilo_uid, target.q, target.r);
        set_bot_state(MOVE_TO_SHAPE);
    } else {
        // 无法启动则重试
        printf("Robot %d: ❌ 条件不符，重新规划\n", kilo_uid);
        mydata->has_movement_intent = 0;
        mydata->intent_backoff_until = kilo_ticks + (rand() % 400 + 100);
        set_bot_state(FIND_SHAPE_POSITION);
    }
}

/* --------- 等待 ----------------*/
void waitting_distributed(){
    // 检查邻居移动状态

    for (int i = 0; i < mydata->N_Neighbors; i++) {
        // 检查是否有邻居正在移动
        if (mydata->neighbors[i].is_moving) {
            //printf("Robot %d: 邻居 %d 正在移动，继续等待\n", kilo_uid, mydata->neighbors[i].ID);
            return;
        }
    }

    set_bot_state(FIND_SHAPE_POSITION);
}

/* ---------- 移动 --------------*/
void moveToShapeState_distributed() {
    static struct Hex current_target = {0, 0};
    current_target.q = mydata->target_q;
    current_target.r = mydata->target_r;

    // === 1️⃣ 定期冲突检测 ===
    if ((kilo_ticks % 20 == 0) && detect_immediate_conflict(mydata->target_q, mydata->target_r)) {
        printf("Robot %d ⚠️ 冲突检测触发，停止移动\n", kilo_uid);
        handle_movement_conflict((struct Hex){mydata->target_q, mydata->target_r});
        finish_movement();
        set_bot_state(FIND_SHAPE_POSITION);
        return;
    }
    printf("机器人 %d 开始进入位置(%d,%d)\n", kilo_uid,mydata->target_q,mydata->target_r);
    // === 2️⃣ 执行移动 ===
    int result = omni_move_to_lattice(&current_target);

    if (result) {
        if (current_target.q != mydata->target_q || current_target.r != mydata->target_r) {
            global_localization();
            struct Hex cur_hex = cart_to_hex((struct Cartesian){mydata->x, mydata->y});
            mydata->hex_q = cur_hex.q;
            mydata->hex_r = cur_hex.r;
            printf("Robot %d: 到达中间点 (%d,%d)\n", kilo_uid, cur_hex.q, cur_hex.r);
        } else {
            // === 3️⃣ 到达最终目标 ===
            finish_movement();
            struct Hex old_pos = mydata->original_position;
            record_move(current_target);

            global_localization();
            struct Hex cur_hex = cart_to_hex((struct Cartesian){mydata->x, mydata->y});
            mydata->hex_q = cur_hex.q;
            mydata->hex_r = cur_hex.r;
            mydata->original_position = cur_hex;
            mydata->shape_position_occupied = 1;

            publish_vacancy_after_movement(old_pos);
            printf("Robot %d: 🎯 到达目标 (%d,%d)，发布空位\n", kilo_uid, cur_hex.q, cur_hex.r);

            mydata->intent_backoff_until = 0;
            set_bot_state(IDLE);
        }
    }

    // === 4️⃣ 超时保护 ===
    if (kilo_ticks - mydata->movement_start_time > 5000) {
        printf("Robot %d: ⏰ 移动超时，终止\n", kilo_uid);
        finish_movement();
        mydata->intent_backoff_until = 0;
        set_bot_state(IDLE);
    }
}
