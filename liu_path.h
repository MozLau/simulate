#ifndef LIU_PATH_H
#define LIU_PATH_H

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
//#include <kilombo.h>
#include "formation.h"

/* ---------- swarm ----------*/
// 检查六边形位置是否在形状定义内
bool is_position_in_shape(struct Hex pos);

/* ---------辅助函数-------- */


uint16_t calculate_follower_id(uint16_t robot_id, uint16_t total_robots);

uint16_t calculate_leader_id(uint16_t robot_id, uint16_t total_robots);

// 快速从笛卡尔坐标获取六边形坐标
struct Hex get_hex_from_cartesian(float x, float y);

bool should_start_finding_now();


//  检查位置是否被邻居占据
bool is_position_occupied_by_neighbor(struct Hex target);

// 精确判断两个六边形坐标是否相邻（距离为1）
bool is_hex_adjacent(struct Hex a, struct Hex b);

// 判断输入点是否与给定点集中的任意一点相邻（距离为1）
bool is_adjacent_to_boundary(struct Hex input);


struct Hex cart_to_hex(struct Cartesian cart);


double hex_distance(int q1, int r1, int q2, int r2);


/* ---------------------------- 补位 ---------------------------- */

// 考虑消息延迟
bool can_safely_start_movement();


/* --------移动阶段------------ */

// 解决移动冲突（基于ID的优先级）
bool resolve_movement_conflict(uint16_t neighbor_id);

// 检查是否可以安全移动
bool can_safely_move();

// 增加随机退避机制
bool can_safely_move_enhanced();

// 开始移动前的准备
void prepare_for_movement(struct Hex target);

// 完成移动后清理状态
void finish_movement();

void init_move_history();

void record_move(struct Hex new_pos);

bool is_position_in_history(struct Hex target_pos);


int check_target_localizability(struct Hex start, struct Hex target);

/* ------ 避障 --------- */

bool is_hex_occupied(struct Hex hex);

// 检查移动路径上是否有已进入形状的机器人
bool is_path_blocked_by_shaped_robots(struct Hex start, struct Hex target);

// 获取两个六边形之间的路径
int get_hex_path(struct Hex start, struct Hex end, struct Hex *path);


bool is_collision_imminent(struct Hex current_pos, struct Hex target_pos);

struct Hex find_blocking_robot(struct Hex start, struct Hex target);

bool is_robot_in_shape(int q, int r);

/* -------功能：准备阶段---------*/


// 检查位置是否为空且可用
bool is_position_empty_and_available(int q, int r);


// 检查是否可以开始查找（更宽松的条件）
bool can_start_finding_enhanced();

// 第二阶段检查：是否可以开始移动（严格条件）
bool can_start_moving();

/* -------功能：寻找阶段---------*/

// 检查目标是否已被邻居声明
bool is_target_claimed_by_neighbor(int q, int r);

// 在移动前进行最终验证
bool verify_target_availability(struct Hex target);

struct Hex find_nearest_unoccupied_target_distributed();
#if 0
int find_optimal_shape_position_index();
#endif

// 检查是否可以声明目标
bool can_claim_target(struct Hex target);

// 找到形状索引
int find_shape_index(struct Hex position);

// 在移动过程中处理冲突
void handle_movement_conflict(struct Hex conflict_position);


// 检查本地维护的占据信息
bool is_occupied_locally(struct Hex target);


// 检查目标是否仍然可用（分布式版本）
bool is_target_still_available(struct Hex target);

/* ---------- 追踪补位--------------*/
void track_leader();

// 检查是否应该开始跟随领导者移动
bool should_follow_leader_movement();

#if 1
void findShapePositionState_distributed();
#else
void findShapePositionState_distributed();
#endif
// 寻找形状位置状态
#if 0
void findShapePositionState();
#endif

// 简化的路径冲突检测
bool is_path_conflict_simple(struct Cartesian my_pos, struct Cartesian my_target,
                            struct Cartesian neighbor_pos, struct Cartesian neighbor_target);

/*-------功能：移动阶段 --------*/
// 更精细的冲突检测
bool detect_immediate_conflict_enhanced(int target_q, int target_r);

bool detect_immediate_conflict(int target_q, int target_r);

// 向形状移动状态

void planMovementState_distributed();

/* --------- 等待 ----------------*/
void waitting_distributed();

/* ---------- 移动 --------------*/
void moveToShapeState_distributed();




#endif