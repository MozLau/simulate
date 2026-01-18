import matplotlib.pyplot as plt
import numpy as np

ROBOT_RADIUS = 16.5  # mm
SAFETY_MARGIN = 2.0
COLLISION_DIST = 2 * ROBOT_RADIUS + SAFETY_MARGIN

def hex_to_cart(q, r, size=10.0):
    x = size * (np.sqrt(3) * q + np.sqrt(3)/2 * r)
    y = size * (3/2 * r)
    return x, y

def point_to_segment_dist(a, b, p):
    a = np.array(a); b = np.array(b); p = np.array(p)
    ab = b - a
    ap = p - a
    ab2 = np.dot(ab, ab)
    if ab2 == 0:
        return np.linalg.norm(ap)
    t = np.dot(ap, ab) / ab2
    t = np.clip(t, 0, 1)
    closest = a + t * ab
    return np.linalg.norm(p - closest), closest

def visualize_with_radius(current_hex, target_hex, neighbor_hexes):
    curr = hex_to_cart(*current_hex)
    targ = hex_to_cart(*target_hex)
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # 画机器人本体（圆形）
    robot_curr = plt.Circle(curr, ROBOT_RADIUS, color='green', alpha=0.3, label='Current Robot')
    robot_targ = plt.Circle(targ, ROBOT_RADIUS, color='red', alpha=0.3, label='Target Robot')
    ax.add_patch(robot_curr)
    ax.add_patch(robot_targ)
    
    # 标出当前点和目标点的 HEX 坐标（不是 Cartesian！）
    ax.text(curr[0] + 2, curr[1] + 2, f'Current\n({current_hex[0]}, {current_hex[1]})', 
            fontsize=9, color='green', ha='left', va='bottom')
    ax.text(targ[0] + 2, targ[1] + 2, f'Target\n({target_hex[0]}, {target_hex[1]})', 
            fontsize=9, color='red', ha='left', va='bottom')
    
    # 画路径
    ax.plot([curr[0], targ[0]], [curr[1], targ[1]], 'k--', alpha=0.6, label='Path')
    
    collision = False
    for i, nh in enumerate(neighbor_hexes):
        pos = hex_to_cart(*nh)
        d_min, closest = point_to_segment_dist(curr, targ, pos)
        
        # 画邻居本体
        neighbor_circle = plt.Circle(pos, ROBOT_RADIUS, color='gray', alpha=0.4)
        ax.add_patch(neighbor_circle)
        ax.plot(pos[0], pos[1], 'x', color='black')
        ax.text(pos[0] + 2, pos[1] + 2, f'N{i}\n({nh[0]}, {nh[1]})', 
                fontsize=9, ha='left', va='bottom')
        
        # 画最近点连线
        ax.plot([pos[0], closest[0]], [pos[1], closest[1]], 
                'r:' if d_min < COLLISION_DIST else 'b:', linewidth=1)
        
        if d_min < COLLISION_DIST:
            collision = True
            print(f"⚠️  Collision risk with N{i} ({nh}): min distance = {d_min:.1f} < {COLLISION_DIST:.1f}")
        else:
            print(f"✅ Safe from N{i} ({nh}): min distance = {d_min:.1f}")
    
    # 设置图形
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.5)
    ax.legend()
    title = "Collision Detection with Robot Radius" + (" → ⚠️ COLLISION IMMINENT" if collision else " → ✅ SAFE")
    ax.set_title(title)
    plt.xlabel('X (mm)')
    plt.ylabel('Y (mm)')
    plt.show()

# 示例
if __name__ == '__main__':
    current = (0, 0)
    target  = (4, 1)
    neighbors = [
        (2, 0),    # 可能擦边
        (1, 1),    # 很近
        (1, 3),

    ]
    visualize_with_radius(current, target, neighbors)