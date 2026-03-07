#!/usr/bin/env python3
"""
手眼标定反向验证测试程序
功能：验证正向转换的逆过程，确保数学一致性
"""
import numpy as np
import math
import os
from ament_index_python.packages import get_package_share_directory


class HandEyeInverseVerification:
    def __init__(self):
        # 加载手眼矩阵
        self.EH_matrix = self.load_handeye_matrix()
        
        # 测试用例
        self.test_cases = self.create_test_cases()
        
        print("=" * 60)
        print("手眼标定反向验证测试程序")
        print("=" * 60)
    
    def load_handeye_matrix(self):
        """从文件加载手眼矩阵"""
        try:
            package_path = get_package_share_directory('handeye_coord_transformer')
            config_path = os.path.join(package_path, 'config', 'EHMatrix.txt')
            
            # 读取16个浮点数
            EHArray = np.zeros(16, dtype=np.float32)
            
            with open(config_path, 'r') as file:
                data = file.read().strip().split()
                
                if len(data) < 16:
                    raise ValueError(f"文件数据不足16个: {len(data)}")
                
                for i in range(16):
                    EHArray[i] = float(data[i])
            
            return np.array(EHArray).reshape(4, 4)
            
        except Exception as e:
            print(f"加载标定文件失败: {e}")
            # 使用单位矩阵作为默认值
            return np.eye(4, dtype=np.float32)
    
    def create_test_cases(self):
        """创建测试用例"""
        test_cases = []
        
        # 测试用例1：基础位置
        test_cases.append({
            'name': '基础位置',
            'tcp_pose': [500.0, 0.0, 500.0, 0.0, 0.0, 0.0],  # [x, y, z, rx, ry, rz]
            'camera_point': [0.0, 0.0, 1000.0]  # 相机坐标系下的点
        })
        
        # 测试用例2：有旋转的位置
        test_cases.append({
            'name': '旋转位置',
            'tcp_pose': [600.0, 100.0, 400.0, 0.0, 0.0, math.radians(45)],  # 绕Z轴旋转45度
            'camera_point': [50.0, -30.0, 800.0]
        })
        
        # 测试用例3：多个旋转轴
        test_cases.append({
            'name': '多轴旋转',
            'tcp_pose': [300.0, -200.0, 600.0, 
                        math.radians(10), math.radians(20), math.radians(30)],
            'camera_point': [-20.0, 40.0, 1200.0]
        })
        
        # 测试用例4：随机测试点
        test_cases.append({
            'name': '随机测试',
            'tcp_pose': [450.0 + np.random.uniform(-100, 100),
                        150.0 + np.random.uniform(-100, 100),
                        350.0 + np.random.uniform(-100, 100),
                        math.radians(np.random.uniform(-30, 30)),
                        math.radians(np.random.uniform(-30, 30)),
                        math.radians(np.random.uniform(-30, 30))],
            'camera_point': [np.random.uniform(-50, 50),
                            np.random.uniform(-50, 50),
                            np.random.uniform(500, 1500)]
        })
        
        return test_cases
    
    def euler_to_rotation_matrix(self, rx, ry, rz):
        """将欧拉角转换为旋转矩阵 (ZYX顺序)"""
        # 计算三角函数
        rxs = math.sin(rx)
        rxc = math.cos(rx)
        rys = math.sin(ry)
        ryc = math.cos(ry)
        rzs = math.sin(rz)
        rzc = math.cos(rz)
        
        # 构建绕X、Y、Z轴的旋转矩阵
        RotX = np.array([[1, 0, 0],
                         [0, rxc, -rxs],
                         [0, rxs, rxc]])
        
        RotY = np.array([[ryc, 0, rys],
                         [0, 1, 0],
                         [-rys, 0, ryc]])
        
        RotZ = np.array([[rzc, -rzs, 0],
                         [rzs, rzc, 0],
                         [0, 0, 1]])
        
        # 计算旋转矩阵：R = Rz * Ry * Rx (ZYX欧拉角顺序)
        Temp = RotY @ RotX
        R = RotZ @ Temp
        
        return R
    
    def pose_to_transform_matrix(self, tcp_pose):
        """将TCP位姿转换为4x4变换矩阵"""
        # 提取位置和欧拉角
        x, y, z, rx, ry, rz = tcp_pose
        
        # 获取旋转矩阵
        R = self.euler_to_rotation_matrix(rx, ry, rz)
        
        # 构建4x4变换矩阵
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        
        return T
    
    def transform_camera_to_base(self, camera_point, tcp_pose):
        """正向转换：相机坐标系 → 基座坐标系"""
        # 将相机点转换为齐次坐标
        p_c = np.array([camera_point[0], camera_point[1], camera_point[2], 1.0])
        
        # 计算正向转换：p_b = T_b_t * T_c_t * p_c
        T_b_t = self.pose_to_transform_matrix(tcp_pose)
        
        # 正向变换
        p_b_homo = T_b_t @ self.EH_matrix @ p_c
        p_b = p_b_homo[:3]
        
        return p_b
    
    def transform_base_to_camera(self, base_point, tcp_pose):
        """反向转换：基座坐标系 → 相机坐标系"""
        # 将基座点转换为齐次坐标
        p_b = np.array([base_point[0], base_point[1], base_point[2], 1.0])
        
        # 计算反向转换：p_c = inv(T_c_t) * inv(T_b_t) * p_b
        T_b_t = self.pose_to_transform_matrix(tcp_pose)
        
        # 计算逆矩阵
        T_b_t_inv = np.linalg.inv(T_b_t)
        T_c_t_inv = np.linalg.inv(self.EH_matrix)
        
        # 反向变换
        p_c_homo = T_c_t_inv @ T_b_t_inv @ p_b
        p_c = p_c_homo[:3]
        
        return p_c
    
    def calculate_pose_inverse(self, tcp_pose):
        """计算TCP位姿的逆变换"""
        # 获取变换矩阵
        T_b_t = self.pose_to_transform_matrix(tcp_pose)
        
        # 计算逆矩阵
        T_t_b = np.linalg.inv(T_b_t)
        
        # 从变换矩阵提取位置和欧拉角
        # 注意：这里简化为只提取位置，不提取欧拉角
        position = T_t_b[:3, 3]
        
        return position
    
    def run_single_test(self, test_case):
        """运行单个测试用例"""
        name = test_case['name']
        tcp_pose = test_case['tcp_pose']
        camera_point = test_case['camera_point']
        
        print(f"\n{'='*60}")
        print(f"测试用例: {name}")
        print('='*60)
        
        # 打印输入参数
        print(f"输入TCP位姿:")
        print(f"  位置: [{tcp_pose[0]:.1f}, {tcp_pose[1]:.1f}, {tcp_pose[2]:.1f}] mm")
        print(f"  姿态: [{math.degrees(tcp_pose[3]):.1f}, "
              f"{math.degrees(tcp_pose[4]):.1f}, "
              f"{math.degrees(tcp_pose[5]):.1f}] °")
        print(f"输入相机点: [{camera_point[0]:.1f}, "
              f"{camera_point[1]:.1f}, {camera_point[2]:.1f}] mm")
        
        # 正向转换：相机点 → 基座点
        base_point = self.transform_camera_to_base(camera_point, tcp_pose)
        print(f"\n正向转换结果:")
        print(f"  基座点: [{base_point[0]:.3f}, {base_point[1]:.3f}, {base_point[2]:.3f}] mm")
        
        # 反向转换：基座点 → 相机点
        recovered_camera_point = self.transform_base_to_camera(base_point, tcp_pose)
        print(f"\n反向转换结果:")
        print(f"  恢复的相机点: [{recovered_camera_point[0]:.3f}, "
              f"{recovered_camera_point[1]:.3f}, {recovered_camera_point[2]:.3f}] mm")
        
        # 计算误差
        error = np.linalg.norm(np.array(camera_point) - recovered_camera_point)
        print(f"\n验证误差:")
        print(f"  原始相机点与恢复相机点的距离误差: {error:.6f} mm")
        
        if error < 1e-6:
            print("  ✓ 正向和反向转换一致")
        else:
            print(f"  ⚠ 存在微小误差: {error:.6f} mm")
        
        # 计算闭环误差：正向→反向→正向
        recovered_base_point = self.transform_camera_to_base(recovered_camera_point, tcp_pose)
        loop_error = np.linalg.norm(base_point - recovered_base_point)
        print(f"  闭环误差: {loop_error:.6f} mm")
        
        return {
            'name': name,
            'original_camera_point': camera_point,
            'base_point': base_point,
            'recovered_camera_point': recovered_camera_point,
            'error': error,
            'loop_error': loop_error
        }
    
    def run_known_point_test(self):
        """已知基座点求相机点的测试"""
        print(f"\n{'='*60}")
        print("已知基座点反向求相机点测试")
        print('='*60)
        
        # 创建已知的基座点（例如：已知的机器人工作空间点）
        known_base_points = [
            [500.0, 0.0, 500.0],      # 机器人基座附近
            [600.0, 200.0, 400.0],    # 工作空间点1
            [400.0, -150.0, 600.0],   # 工作空间点2
        ]
        
        # 使用固定的TCP位姿
        tcp_pose = [550.0, 100.0, 450.0, 0.0, 0.0, math.radians(30)]
        
        print(f"使用TCP位姿: [{tcp_pose[0]:.1f}, {tcp_pose[1]:.1f}, {tcp_pose[2]:.1f}] mm")
        
        for i, base_point in enumerate(known_base_points):
            print(f"\n--- 测试点 {i+1} ---")
            print(f"已知基座点: [{base_point[0]:.1f}, {base_point[1]:.1f}, {base_point[2]:.1f}] mm")
            
            # 反向计算相机点
            camera_point = self.transform_base_to_camera(base_point, tcp_pose)
            print(f"计算得到的相机点: [{camera_point[0]:.3f}, "
                  f"{camera_point[1]:.3f}, {camera_point[2]:.3f}] mm")
            
            # 验证：再正向计算基座点
            recovered_base_point = self.transform_camera_to_base(camera_point, tcp_pose)
            error = np.linalg.norm(np.array(base_point) - recovered_base_point)
            print(f"验证误差: {error:.6f} mm")
            
            if error < 1e-6:
                print("  ✓ 验证通过")
            else:
                print(f"  ⚠ 存在误差")
    
    def run_tcp_pose_inverse_test(self):
        """测试TCP位姿的逆变换"""
        print(f"\n{'='*60}")
        print("TCP位姿逆变换测试")
        print('='*60)
        
        # 测试几个TCP位姿
        test_poses = [
            [500.0, 0.0, 500.0, 0.0, 0.0, 0.0],
            [600.0, 100.0, 400.0, math.radians(30), 0.0, 0.0],
            [400.0, -200.0, 600.0, 0.0, math.radians(20), 0.0],
        ]
        
        for i, tcp_pose in enumerate(test_poses):
            print(f"\n测试位姿 {i+1}:")
            print(f"  原始TCP位姿: [{tcp_pose[0]:.1f}, {tcp_pose[1]:.1f}, {tcp_pose[2]:.1f}] mm")
            
            # 创建变换矩阵
            T_b_t = self.pose_to_transform_matrix(tcp_pose)
            
            # 计算逆矩阵
            T_t_b = np.linalg.inv(T_b_t)
            
            # 验证：T_b_t * T_t_b = I
            identity_check = T_b_t @ T_t_b
            identity_error = np.max(np.abs(identity_check - np.eye(4)))
            
            print(f"  单位矩阵验证误差: {identity_error:.10f}")
            
            if identity_error < 1e-10:
                print("  ✓ 逆矩阵验证通过")
            else:
                print("  ✗ 逆矩阵验证失败")
    
    def analyze_handeye_matrix(self):
        """分析手眼矩阵的性质"""
        print(f"\n{'='*60}")
        print("手眼矩阵分析")
        print('='*60)
        
        print(f"手眼矩阵 (T_c_t):")
        for i in range(4):
            print(f"  [{self.EH_matrix[i,0]:.6f}, {self.EH_matrix[i,1]:.6f}, "
                  f"{self.EH_matrix[i,2]:.6f}, {self.EH_matrix[i,3]:.6f}]")
        
        # 分离旋转和平移
        R = self.EH_matrix[:3, :3]
        t = self.EH_matrix[:3, 3]
        
        # 检查旋转矩阵性质
        RRT = R @ R.T
        identity_error = np.max(np.abs(RRT - np.eye(3)))
        print(f"\n旋转矩阵正交性误差: {identity_error:.10f}")
        
        det = np.linalg.det(R)
        print(f"旋转矩阵行列式: {det:.6f} (应为+1)")
        
        print(f"\n平移向量: [{t[0]:.1f}, {t[1]:.1f}, {t[2]:.1f}] mm")
        print(f"平移向量长度: {np.linalg.norm(t):.1f} mm")
        
        # 计算逆矩阵
        EH_inv = np.linalg.inv(self.EH_matrix)
        print(f"\n手眼矩阵的逆 (T_t_c):")
        for i in range(4):
            print(f"  [{EH_inv[i,0]:.6f}, {EH_inv[i,1]:.6f}, "
                  f"{EH_inv[i,2]:.6f}, {EH_inv[i,3]:.6f}]")
        
        # 验证逆矩阵
        identity_check = self.EH_matrix @ EH_inv
        identity_error = np.max(np.abs(identity_check - np.eye(4)))
        print(f"逆矩阵验证误差: {identity_error:.10f}")
    
    def run_all_tests(self):
        """运行所有测试"""
        self.analyze_handeye_matrix()
        
        # 运行标准测试用例
        results = []
        for test_case in self.test_cases:
            result = self.run_single_test(test_case)
            results.append(result)
        
        # 运行已知点测试
        self.run_known_point_test()
        
        # 运行TCP逆变换测试
        self.run_tcp_pose_inverse_test()
        
        # 统计结果
        self.summarize_results(results)
    
    def summarize_results(self, results):
        """汇总测试结果"""
        print(f"\n{'='*60}")
        print("测试结果汇总")
        print('='*60)
        
        total_tests = len(results)
        passed_tests = sum(1 for r in results if r['error'] < 1e-6)
        
        print(f"总测试用例: {total_tests}")
        print(f"通过测试: {passed_tests}")
        print(f"失败测试: {total_tests - passed_tests}")
        
        if total_tests == passed_tests:
            print("\n✅ 所有测试通过！正向和反向转换一致。")
        else:
            print("\n⚠ 部分测试未通过，请检查手眼标定参数。")
        
        # 显示详细误差统计
        print(f"\n误差统计:")
        max_error = max(r['error'] for r in results)
        min_error = min(r['error'] for r in results)
        avg_error = sum(r['error'] for r in results) / total_tests
        
        print(f"  最大误差: {max_error:.10f} mm")
        print(f"  最小误差: {min_error:.10f} mm")
        print(f"  平均误差: {avg_error:.10f} mm")
        
        print(f"\n闭环误差统计:")
        max_loop_error = max(r['loop_error'] for r in results)
        avg_loop_error = sum(r['loop_error'] for r in results) / total_tests
        print(f"  最大闭环误差: {max_loop_error:.10f} mm")
        print(f"  平均闭环误差: {avg_loop_error:.10f} mm")


def main():
    """主函数"""
    # 创建测试实例
    verifier = HandEyeInverseVerification()
    
    # 运行所有测试
    verifier.run_all_tests()
    
    print(f"\n{'='*60}")
    print("测试完成")
    print('='*60)


if __name__ == '__main__':
    main()