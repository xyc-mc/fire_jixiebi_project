import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import String
import struct
import time
import sys
import os
from base_msgs.msg import ControlMsg
import shutil


try:
    from .VSensorSDK_linux import *
except ImportError as e:
    print(f"错误: 无法导入VSensorSDK_linux模块: {e}")
    print("请确保VSensorSDK_linux.py文件在正确的目录中")
    sys.exit(1)

class AreaScanCameraNode(Node):
    def __init__(self):
        super().__init__('camera_control_node')
        
        # 声明保存路径参数

        self.declare_parameter('workspace_dir', '/home/ubuntu/fire_jixiebi_ws')
        self.workspace_dir = self.get_parameter('workspace_dir').value
        self.save_dir = os.path.join(self.workspace_dir, 'src/point_rgb_address/camera_data')
        # 创建保存目录
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.control_sub = self.create_subscription(ControlMsg, 'camera_control', self.camera_callback, 10)
        self.control_pub = self.create_publisher(ControlMsg, '/address_control', 10)

        self.process_pub = self.create_publisher(String, '/process_data', 10)

        self.error_pub = self.create_publisher(String, '/error_data', 10)
        
        

    def camera_callback(self, data):
        try:
            process = String()
            if data.frequency == 1:   
                process.data = 'camera_first'
                self.process_pub.publish(process)
            elif data.frequency == 2:
                process.data = 'camera_second'
                self.process_pub.publish(process)

            control_msg = ControlMsg()
            shutil.rmtree(self.save_dir)
            os.makedirs(self.save_dir, exist_ok=True)
            if data.switch_control:
                # 相机控制器
                self.camera_controller = AreaScanCameraController(self)
                
                # 初始化相机
                if not self.camera_controller.initialize_camera():
                    self.get_logger().error('相机初始化失败！')
                    return
                
                # 执行单次采集、保存和发布
                self.perform_single_capture()
                
                self.get_logger().info('采集完成')

                control_msg.switch_control = True
                control_msg.frequency = data.frequency
                self.control_pub.publish(control_msg)

                self.camera_controller.cleanup()
            else:
                return
            
        except Exception as e:
            self.error_pub.publish(process)
            self.get_logger().error('相机控制出现未知错误，取消本次任务')
            return

    def perform_single_capture(self):
        """执行单次采集、保存数据"""
        result = self.camera_controller.single_capture()
        if not result:
            self.get_logger().error('点云采集失败！')
            return
        
        
        # 保存数据到文件
        self.camera_controller.save_rgb_and_pointcloud(
            result, 
            os.path.join(self.save_dir, "capture")
        )

class AreaScanCameraController:
    def __init__(self, node):
        self.node = node
        self.connected = False
        self.initialized = False
    
    def initialize_camera(self):
        """初始化相机设备"""
        # 获取SDK版本
        version = GetSdkVersionString()
        self.node.get_logger().info(f"SDK版本: {version}")
            
        # 获取设备列表
        self.node.get_logger().info("扫描设备...")
        dev_list = GetDeviceList()
        err_code = GetLastError()
            
        if err_code != 0:
            self.node.get_logger().error(f"获取设备列表失败，错误码: {err_code}")
            return False
            
        if len(dev_list) == 0:
            self.node.get_logger().error("未找到任何设备")
            return False
                
        self.node.get_logger().info(f"找到 {len(dev_list)} 个设备:")
        for i, dev_info in enumerate(dev_list):
            self.node.get_logger().info(f"  设备{i}: {dev_info.GetCameraName()} - {dev_info.GetAddress()}")
            
        # 连接第一个设备
        self.node.get_logger().info("连接设备...")
        err_code = DeviceConnect(0)
        if err_code != 0:
            self.node.get_logger().error(f"设备连接失败，错误码: {err_code}")
            return False
            
        self.connected = True
        self.node.get_logger().info("设备连接成功")
            
        # 设置为面扫模式
        self.node.get_logger().info("设置为面扫模式...")
        err_code = SetRestructionMode(MODE_AREA)
        if err_code != 0:
            self.node.get_logger().warning(f"设置面扫模式失败，错误码: {err_code}")
            # 继续执行，可能某些设备不支持此设置
            
        # 参数初始化
        self.node.get_logger().info("等待参数初始化...")
        err_code = DeviceParameterInit()
        if err_code != 0:
            self.node.get_logger().error(f"参数初始化失败，错误码: {err_code}")
            return False
                
        self.initialized = True
        self.node.get_logger().info("参数初始化成功")
            
        # 设置采集参数
        return self.set_capture_parameters()
            
        
    
    def set_capture_parameters(self):
        """设置采集参数"""
        self.node.get_logger().info("=== 设置采集参数 ===")
        
        try:
            # 设置Z轴范围 350-700
            self.node.get_logger().info("设置Z轴范围: 350-700mm")
            err_code = SetZaxisRange(350, 700)
            if err_code != 0:
                self.node.get_logger().error(f"设置Z轴范围失败，错误码: {err_code}")
                return False
            self.node.get_logger().info("Z轴范围设置成功")
            
            # 设置扫描时间间隔
            self.node.get_logger().info("设置扫描时间间隔: 3000ms")
            err_code = SetScanTimeInterval(5)  # 5对应3000ms
            if err_code != 0:
                self.node.get_logger().warning(f"设置扫描时间间隔失败，错误码: {err_code}")
                self.node.get_logger().info("注意: 此功能可能只支持多线扫描3D设备")
                # 继续执行，不中断
            else:
                self.node.get_logger().info("扫描时间间隔设置成功")
            
            # 设置曝光时间
            self.node.get_logger().info("设置彩色曝光时间")
            err_code = SetExposureTime(4, TYPE_RGB)
            if err_code != 0:
                self.node.get_logger().error(f"设置彩色曝光时间失败，错误码: {err_code}")
                return False
            
            self.node.get_logger().info("设置灰色曝光时间: 0.01ms")
            err_code = SetExposureTime(0.01, TYPE_GRAY)
            if err_code != 0:
                self.node.get_logger().error(f"设置灰色曝光时间失败，错误码: {err_code}")
                return False
            
            # 设置为采集模式
            self.node.get_logger().info("设置为采集模式")
            err_code = SetCaptureMode(MODE_CAPTURE)
            if err_code != 0:
                self.node.get_logger().error(f"设置采集模式失败，错误码: {err_code}")
                return False
            
            self.node.get_logger().info("采集参数设置完成")
            return True
            
        except Exception as e:
            self.node.get_logger().error(f"参数设置异常: {str(e)}")
            return False
    
    def single_capture(self):
        """单次采集RGB和点云"""
        try:
            self.node.get_logger().info("开始单次采集...")
            start_time = time.time()
            
            result = VSensorResult()
            result = SingleRestruction(result, OUTPUT_MODE_POINT_AND_Gray)
            err_code = GetLastError()
            
            capture_time = time.time() - start_time
            
            if err_code != 0:
                self.node.get_logger().error(f"单次采集失败，错误码: {err_code}")
                return None
            
            self.node.get_logger().info(f"单次采集成功，耗时: {capture_time:.3f}秒，点数: {result.nPointNum}")
            return result
            
        except Exception as e:
            self.node.get_logger().error(f"采集异常: {str(e)}")
            return None
    
    def save_rgb_and_pointcloud(self, result, base_filename="area_scan_capture"):
        """保存RGB图像和点云数据"""
        try:
            self.node.get_logger().info(f"=== 保存数据: {base_filename} ===")
            
            # 保存点云
            pcd_filename = f"{base_filename}.pcd"
            err_code = Save3DCloud(pcd_filename, result, POINT_TYPR_DISORDER)
            if err_code == 0:
                self.node.get_logger().info(f"点云保存成功: {pcd_filename}")
            else:
                self.node.get_logger().warning(f"点云保存失败，错误码: {err_code}")
            
            # 保存RGB图像（如果支持）
            rgb_filename = f"{base_filename}_rgb.jpg"
            err_code = SaveRGBMap(rgb_filename, result)
            if err_code == 0:
                self.node.get_logger().info(f"RGB图像保存成功: {rgb_filename}")
            else:
                self.node.get_logger().warning(f"RGB图像保存失败，错误码: {err_code} (可能设备不支持RGB采集)")
                
        except Exception as e:
            self.node.get_logger().warning(f"保存数据异常: {str(e)}")
    

    def cleanup(self):
        """清理资源"""
        try:
            if self.connected:
                DeviceUnInit()
                self.node.get_logger().info("相机资源已释放")
        except Exception as e:
            self.node.get_logger().error(f"资源清理异常: {str(e)}")

def main():
    rclpy.init()
    node = AreaScanCameraNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()