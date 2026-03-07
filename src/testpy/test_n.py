import numpy as np
import math

def find_rx_ry_from_z_vector(z_target):
    """
    根据目标Z轴方向向量计算Rx和Ry角
    
    参数:
        z_target: 目标Z轴方向向量 [x, y, z] (不需要是单位向量)
    
    返回:
        rx, ry: 弧度制的旋转角度
    """
    # 1. 归一化目标向量
    z_target = [-0.08852038747774034, 0.1230344691696256, 0.9884465895519794]
    norm = math.sqrt(z_target[0]**2 + z_target[1]**2 + z_target[2]**2)
    if norm < 1e-9:
        raise ValueError("输入向量长度为零")
    
    nx, ny, nz = z_target[0]/norm, z_target[1]/norm, z_target[2]/norm
    
    # 2. 处理万向锁特殊情况
    # 当ny接近±1时，cos(rx)接近0，出现万向锁
    if abs(abs(ny) - 1.0) < 1e-6:
        # n_y = ±1, 则rx = -π/2 或 π/2
        rx = -math.pi/2 if ny > 0 else math.pi/2
        # 此时Ry可以是任意值，但通常设为0
        ry = 0.0
    else:
        # 3. 正常情况计算
        # 由 n_y = -sin(rx) 得
        rx = math.asin(-ny)  # rx在[-π/2, π/2]之间
        
        # 计算cos(rx)
        cos_rx = math.cos(rx)
        
        # 由 n_x = cos(rx)*sin(ry), n_z = cos(rx)*cos(ry) 得
        # 使用atan2确保正确的象限
        ry = math.atan2(nx, nz)
    
    return rx, ry

def rotation_matrix_from_rx_ry(rx, ry):
    """
    根据Rx和Ry角生成旋转矩阵
    
    参数:
        rx: 绕X轴旋转角度(弧度)
        ry: 绕Y轴旋转角度(弧度)
    
    返回:
        3x3旋转矩阵
    """
    # 计算三角函数值
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    
    # 旋转矩阵 Ry * Rx
    R = np.array([
        [cy,     sx*sy,      cx*sy],
        [0,      cx,         -sx],
        [-sy,    sx*cy,      cx*cy]
    ])
    
    return R

def verify_solution(z_target, rx, ry):
    """
    验证计算结果：旋转后的Z轴是否与目标方向一致
    
    参数:
        z_target: 目标Z轴方向
        rx, ry: 计算得到的旋转角
    """
    # 生成旋转矩阵
    R = rotation_matrix_from_rx_ry(rx, ry)
    
    # 旋转后的Z轴是旋转矩阵的第三列
    z_result = R[:, 2]
    
    # 归一化目标向量用于比较
    norm = math.sqrt(z_target[0]**2 + z_target[1]**2 + z_target[2]**2)
    z_target_norm = np.array(z_target) / norm
    
    # 计算误差
    error = np.linalg.norm(z_result - z_target_norm)
    
    return z_result, error

def print_rotation_info(z_target, rx, ry, degrees=True):
    """
    打印旋转信息
    
    参数:
        z_target: 目标Z轴方向
        rx, ry: 旋转角度(弧度)
        degrees: 是否以度为单位输出
    """
    # 归一化目标向量
    norm = math.sqrt(z_target[0]**2 + z_target[1]**2 + z_target[2]**2)
    nx, ny, nz = z_target[0]/norm, z_target[1]/norm, z_target[2]/norm
    
    print("=" * 60)
    print("目标Z轴方向 (单位向量):")
    print(f"  n = [{nx:.4f}, {ny:.4f}, {nz:.4f}]")
    print()
    
    # 角度转换
    if degrees:
        rx_deg = math.degrees(rx)
        ry_deg = math.degrees(ry)
        print("计算得到的旋转角度:")
        print(f"  Rx = {rx:.4f} rad = {rx_deg:.2f}°")
        print(f"  Ry = {ry:.4f} rad = {ry_deg:.2f}°")
    else:
        print("计算得到的旋转角度:")
        print(f"  Rx = {rx:.4f} rad")
        print(f"  Ry = {ry:.4f} rad")
    print()
    
    # 验证结果
    z_result, error = verify_solution(z_target, rx, ry)
    print("旋转后的Z轴方向:")
    print(f"  z_result = [{z_result[0]:.6f}, {z_result[1]:.6f}, {z_result[2]:.6f}]")
    print(f"验证误差 = {error:.10f}")
    print("=" * 60)

def find_all_solutions(z_target):
    """
    找出所有可能的Rx, Ry解（考虑正负两种情况）
    
    参数:
        z_target: 目标Z轴方向
    
    返回:
        solutions: 所有可能的(rx, ry)解列表
    """
    # 主解
    rx1, ry1 = find_rx_ry_from_z_vector(z_target)
    solutions = [(rx1, ry1)]
    
    # 归一化目标向量
    norm = math.sqrt(z_target[0]**2 + z_target[1]**2 + z_target[2]**2)
    nx, ny, nz = z_target[0]/norm, z_target[1]/norm, z_target[2]/norm
    
    # 寻找第二个解（如果有）
    # 由 sin(rx) = -ny，可以得到另一个rx = π - arcsin(-ny)
    if abs(ny) <= 1.0:  # 确保在arcsin定义域内
        # 计算另一个可能的rx
        rx2 = math.pi - math.asin(-ny)
        
        # 确保rx2在合理范围内
        while rx2 > math.pi:
            rx2 -= 2*math.pi
        while rx2 < -math.pi:
            rx2 += 2*math.pi
        
        # 对应的ry需要调整π弧度
        ry2 = ry1 + math.pi
        # 规范化到[-π, π]范围
        while ry2 > math.pi:
            ry2 -= 2*math.pi
        while ry2 < -math.pi:
            ry2 += 2*math.pi
        
        solutions.append((rx2, ry2))
    
    return solutions

def main():
    """主函数：测试几个示例"""
    
    print("基于法向量求解RPY角（Z轴对齐）")
    print("=" * 60)
    
    # 示例1：Z轴指向基座标系Z轴正方向
    print("\n示例1: Z轴指向基座标系Z轴正方向")
    z_target1 = [0, 0, 1]
    rx1, ry1 = find_rx_ry_from_z_vector(z_target1)
    print_rotation_info(z_target1, rx1, ry1)
    
    # 示例2：Z轴指向基座标系Y轴正方向
    print("\n示例2: Z轴指向基座标系Y轴正方向")
    z_target2 = [0, 1, 0]
    rx2, ry2 = find_rx_ry_from_z_vector(z_target2)
    print_rotation_info(z_target2, rx2, ry2)
    
    # 示例3：Z轴指向基座标系X轴正方向
    print("\n示例3: Z轴指向基座标系X轴正方向")
    z_target3 = [1, 0, 0]
    rx3, ry3 = find_rx_ry_from_z_vector(z_target3)
    print_rotation_info(z_target3, rx3, ry3)
    
    # 示例4：Z轴指向一般方向
    print("\n示例4: Z轴指向一般方向 [1, 1, 1]")
    z_target4 = [1, 1, 1]
    rx4, ry4 = find_rx_ry_from_z_vector(z_target4)
    print_rotation_info(z_target4, rx4, ry4)
    
    # 示例5：Z轴指向另一方向
    print("\n示例5: Z轴指向 [0.6, -0.3, 0.741]")
    z_target5 = [0.6, -0.3, 0.741]
    rx5, ry5 = find_rx_ry_from_z_vector(z_target5)
    print_rotation_info(z_target5, rx5, ry5)
    
    # 显示所有可能解
    print("\n\n查找所有可能解:")
    print("=" * 60)
    for i, z_target in enumerate([z_target4, z_target5], 1):
        print(f"\n目标方向 {i}: {z_target}")
        solutions = find_all_solutions(z_target)
        for j, (rx, ry) in enumerate(solutions, 1):
            rx_deg, ry_deg = math.degrees(rx), math.degrees(ry)
            print(f"  解{j}: Rx = {rx_deg:.2f}°, Ry = {ry_deg:.2f}°")
            z_result, error = verify_solution(z_target, rx, ry)
            print(f"      验证误差 = {error:.10f}")

if __name__ == "__main__":
    main()