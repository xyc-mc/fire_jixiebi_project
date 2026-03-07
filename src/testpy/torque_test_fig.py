import rclpy
from rclpy.node import Node
from elite_msgs.msg import RobotState
import numpy as np
import time
from collections import deque
import threading
import sys
import select
import csv
import os


class TorqueRecorderNode(Node):
    def __init__(self):
        super().__init__('torque_recorder_node')
        
        # 数据存储
        self.time_data = deque()  # 时间数据
        self.j2_data = deque()    # J2关节力矩数据
        self.j3_data = deque()    # J3关节力矩数据
        
        # 记录控制标志
        self.is_recording = False
        self.start_time = None
        self.record_count = 0
        
        # CSV文件相关
        self.csv_file = None
        self.csv_writer = None
        self.csv_filename = None
        
        # ROS2订阅
        self.subscription = self.create_subscription(
            RobotState,
            '/robot_state',
            self.listener_callback,
            10
        )
        
        # 初始化键盘监听线程
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()
        
        self.get_logger().info('Torque recorder node initialized')
        self.get_logger().info('Press Enter to start recording...')
    
    def create_csv_file(self):
        """创建CSV文件用于保存数据"""
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f'torque_data_{timestamp}.csv'
        
        # 确保目录存在
        os.makedirs('torque_data', exist_ok=True)
        self.csv_filename = os.path.join('torque_data', self.csv_filename)
        
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # 写入表头
        self.csv_writer.writerow(['Time(0.1s)', 'J2_Torque(Nm)', 'J3_Torque(Nm)'])
        
        self.get_logger().info(f'Data will be saved to: {self.csv_filename}')
    
    def close_csv_file(self):
        """关闭CSV文件"""
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
    
    def keyboard_listener(self):
        """监听键盘输入的线程"""
        while rclpy.ok():
            # 检测回车键
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if line.strip() == '':
                    self.toggle_recording()
    
    def toggle_recording(self):
        """切换记录状态"""
        if not self.is_recording:
            # 开始记录
            self.is_recording = True
            self.start_time = time.time()
            self.record_count = 0
            
            # 清空数据
            self.time_data.clear()
            self.j2_data.clear()
            self.j3_data.clear()
            
            # 创建CSV文件
            self.create_csv_file()
            
            self.get_logger().info('Started recording torque data...')
            self.get_logger().info('Press Enter again to stop recording')
        else:
            # 停止记录
            self.is_recording = False
            self.get_logger().info('Stopped recording')
            
            # 关闭CSV文件
            self.close_csv_file()
            
            # 计算并显示统计数据
            self.display_statistics()
            
            # 提供绘图选项
            self.offer_plotting_options()
    
    def listener_callback(self, msg):
        """ROS2消息回调函数"""
        if self.is_recording:
            try:
                # 检查torque字段是否存在且为列表
                if hasattr(msg, 'torque'):
                    # 确保至少包含3个关节的力矩数据
                    if len(msg.torque) >= 3:
                        # 记录J2（索引1）和J3（索引2）关节力矩
                        j2_torque = msg.torque[1]  # J2关节
                        j3_torque = msg.torque[2]  # J3关节
                        
                        # 计算相对时间（单位：0.1秒）
                        elapsed_time = (time.time() - self.start_time) * 10  # 转换为0.1秒单位
                        
                        # 存储数据
                        self.time_data.append(elapsed_time)
                        self.j2_data.append(j2_torque)
                        self.j3_data.append(j3_torque)
                        
                        # 写入CSV文件
                        if self.csv_writer:
                            self.csv_writer.writerow([f"{elapsed_time:.1f}", 
                                                     f"{j2_torque:.4f}", 
                                                     f"{j3_torque:.4f}"])
                        
                        self.record_count += 1
                        
                        # 每记录100个点输出一次状态
                        if self.record_count % 100 == 0:
                            self.get_logger().info(f'Recorded {self.record_count} data points')
                    else:
                        self.get_logger().warning(f'Torque array too short: {len(msg.torque)} < 3')
                else:
                    self.get_logger().warning('Message does not have torque field or torque is not a list')
                    
            except Exception as e:
                self.get_logger().error(f'Error processing message: {str(e)}')
    
    def display_statistics(self):
        """显示统计信息"""
        if len(self.time_data) == 0:
            self.get_logger().warning('No data recorded!')
            return
        
        # 转换为numpy数组
        time_array = np.array(self.time_data)
        j2_array = np.array(self.j2_data)
        j3_array = np.array(self.j3_data)
        
        print("\n" + "="*60)
        print("DATA STATISTICS")
        print("="*60)
        print(f"Total data points: {len(time_array)}")
        print(f"Time range: {time_array[0]:.1f} to {time_array[-1]:.1f} (0.1 sec units)")
        print(f"Duration: {(time_array[-1] - time_array[0])/10:.2f} seconds")
        print("-"*60)
        
        print("J2 Joint Torque:")
        print(f"  Min: {j2_array.min():.4f} Nm")
        print(f"  Max: {j2_array.max():.4f} Nm")
        print(f"  Avg: {j2_array.mean():.4f} Nm")
        print(f"  Std: {j2_array.std():.4f} Nm")
        
        print("\nJ3 Joint Torque:")
        print(f"  Min: {j3_array.min():.4f} Nm")
        print(f"  Max: {j3_array.max():.4f} Nm")
        print(f"  Avg: {j3_array.mean():.4f} Nm")
        print(f"  Std: {j3_array.std():.4f} Nm")
        
        print("="*60)
        
        # 保存统计数据到文件
        stats_filename = self.csv_filename.replace('.csv', '_stats.txt')
        with open(stats_filename, 'w') as f:
            f.write("TORQUE DATA STATISTICS\n")
            f.write("="*60 + "\n")
            f.write(f"Data file: {self.csv_filename}\n")
            f.write(f"Total data points: {len(time_array)}\n")
            f.write(f"Time range: {time_array[0]:.1f} to {time_array[-1]:.1f} (0.1 sec units)\n")
            f.write(f"Duration: {(time_array[-1] - time_array[0])/10:.2f} seconds\n")
            f.write("-"*60 + "\n")
            f.write("J2 Joint Torque:\n")
            f.write(f"  Min: {j2_array.min():.4f} Nm\n")
            f.write(f"  Max: {j2_array.max():.4f} Nm\n")
            f.write(f"  Avg: {j2_array.mean():.4f} Nm\n")
            f.write(f"  Std: {j2_array.std():.4f} Nm\n")
            f.write("\nJ3 Joint Torque:\n")
            f.write(f"  Min: {j3_array.min():.4f} Nm\n")
            f.write(f"  Max: {j3_array.max():.4f} Nm\n")
            f.write(f"  Avg: {j3_array.mean():.4f} Nm\n")
            f.write(f"  Std: {j3_array.std():.4f} Nm\n")
        
        self.get_logger().info(f'Statistics saved to: {stats_filename}')
    
    def offer_plotting_options(self):
        """提供绘图选项"""
        print("\nWould you like to plot the data?")
        print("1. Install required packages and plot now")
        print("2. Save data only (use external tools like Excel, MATLAB, or Python offline)")
        print("3. Continue recording (press Enter again)")
        
        try:
            choice = input("Enter choice (1-3): ").strip()
            
            if choice == '1':
                self.install_and_plot()
            elif choice == '2':
                print(f"Data saved to: {self.csv_filename}")
                print("You can use Excel, MATLAB, or Python (with numpy<2) to plot the data.")
            elif choice == '3':
                print("Ready to record again. Press Enter when ready...")
            else:
                print("Invalid choice. Data has been saved.")
        except Exception as e:
            print(f"Error in user input: {e}")
    
    def install_and_plot(self):
        """尝试安装所需的库并绘图"""
        print("\nAttempting to install required packages...")
        
        try:
            import subprocess
            import importlib
            
            # 尝试导入numpy，检查版本
            try:
                import numpy as np_current
                numpy_version = np_current.__version__
                print(f"Current NumPy version: {numpy_version}")
                
                # 如果numpy版本>=2，尝试降级
                if int(numpy_version.split('.')[0]) >= 2:
                    print("NumPy version >= 2 detected. Attempting to downgrade...")
                    subprocess.check_call([sys.executable, "-m", "pip", "install", "numpy<2", "--upgrade"])
                    # 重新加载numpy
                    importlib.reload(np_current)
                    print("NumPy downgraded successfully")
            except Exception as e:
                print(f"Error checking NumPy: {e}")
            
            # 尝试导入matplotlib
            try:
                import matplotlib.pyplot as plt
                print("Matplotlib is already installed")
            except ImportError:
                print("Installing matplotlib...")
                subprocess.check_call([sys.executable, "-m", "pip", "install", "matplotlib"])
                import matplotlib.pyplot as plt
                print("Matplotlib installed successfully")
            
            # 如果到这里没有出错，开始绘图
            self.plot_data_with_matplotlib()
            
        except Exception as e:
            print(f"Failed to install/plot: {e}")
            print("\nAlternative: Install packages manually:")
            print("  pip install 'numpy<2' matplotlib")
            print(f"Then plot the data from: {self.csv_filename}")
    
    def plot_data_with_matplotlib(self):
        """使用matplotlib绘图"""
        try:
            import matplotlib.pyplot as plt
            
            # 读取CSV数据
            data = np.genfromtxt(self.csv_filename, delimiter=',', skip_header=1)
            
            if data.size == 0:
                print("No data to plot")
                return
            
            time_data = data[:, 0]
            j2_data = data[:, 1]
            j3_data = data[:, 2]
            
            # 创建图表
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
            
            # 绘制J2关节力矩
            ax1.plot(time_data, j2_data, 'b-', linewidth=1.5)
            ax1.set_xlabel('Time (0.1 sec)')
            ax1.set_ylabel('Torque (Nm)')
            ax1.set_title('J2 Joint Torque')
            ax1.grid(True, alpha=0.3)
            
            # 绘制J3关节力矩
            ax2.plot(time_data, j3_data, 'r-', linewidth=1.5)
            ax2.set_xlabel('Time (0.1 sec)')
            ax2.set_ylabel('Torque (Nm)')
            ax2.set_title('J3 Joint Torque')
            ax2.grid(True, alpha=0.3)
            
            plt.tight_layout()
            
            # 保存图表
            plot_filename = self.csv_filename.replace('.csv', '.png')
            plt.savefig(plot_filename, dpi=300)
            print(f"Plot saved as: {plot_filename}")
            
            plt.show()
            
        except Exception as e:
            print(f"Error plotting data: {e}")


def main():
    rclpy.init()
    
    try:
        node = TorqueRecorderNode()
        
        # 在主线程中运行ROS2节点
        print("\n" + "="*60)
        print("TORQUE RECORDER NODE")
        print("="*60)
        print("Instructions:")
        print("1. Press Enter to START recording")
        print("2. Move the robot to generate torque data")
        print("3. Press Enter again to STOP recording")
        print("4. Data will be saved automatically as CSV")
        print("5. View statistics and choose plotting option")
        print("="*60)
        print("Waiting for Enter to start recording...")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n\nProgram interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()
        print("Program terminated")


if __name__ == "__main__":
    main()