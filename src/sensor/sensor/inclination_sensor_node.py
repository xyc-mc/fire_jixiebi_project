import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import socket
import struct
import time

class InclinationSensor:
    """基于原始 socket 的倾角传感器驱动类（Modbus TCP，功能码4）"""
    def __init__(self, host, port, slave, timeout=3):
        self.host = host
        self.port = port
        self.slave = slave
        self.timeout = timeout
        self.sock = None
        self.connected = False

    def connect(self):
        """建立 TCP 连接"""
        try:
            if self.sock is None:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(self.timeout)
                self.sock.connect((self.host, self.port))
                self.connected = True
                print(f"成功连接到 {self.host}:{self.port} (从站 {self.slave})")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            self.connected = False
            self.sock = None
            return False

    def _send_receive(self, unit_id, function_code, start_address, quantity):
        """发送 Modbus 请求并解析响应（功能码 4）"""
        if not self.connected:
            raise Exception("未连接")

        transaction_id = 1
        protocol_id = 0
        length = 6  # 后续字节数（unit_id + function_code + start_address + quantity 共6字节）
        # MBAP 头 + PDU
        request = struct.pack('>HHHBBHH',
                              transaction_id,
                              protocol_id,
                              length,
                              unit_id,
                              function_code,
                              start_address,
                              quantity)
        self.sock.send(request)
        response = self.sock.recv(256)
        if len(response) < 9:
            raise Exception("响应太短")

        recv_func = response[7]
        if recv_func & 0x80:
            error_code = response[8]
            raise Exception(f"Modbus 异常，错误码: {error_code}")

        # 解析寄存器值（大端序）
        registers = []
        for i in range(quantity):
            val = struct.unpack('>H', response[9 + i*2:11 + i*2])[0]
            registers.append(val)
        return registers

    def read_euler_angles(self):
        """读取欧拉角 (Roll, Pitch, Yaw) 功能码 4，地址 0x0000，数量 3"""
        if not self.connected:
            if not self.connect():
                return None
        try:
            data = self._send_receive(self.slave, 4, 0x0000, 3)
            # 将16位无符号转换为有符号并除以100
            def to_signed(x):
                return x if x <= 0x7FFF else x - 0x10000
            return {
                "Roll": to_signed(data[0]) / 100.0,
                "Pitch": to_signed(data[1]) / 100.0,
                "Yaw": to_signed(data[2]) / 100.0,
            }
        except Exception as e:
            print(f"读取欧拉角失败: {e}")
            self.connected = False  # 标记连接可能已断开
            self.sock = None
            return None

    def close(self):
        """关闭连接"""
        if self.sock:
            self.sock.close()
            self.sock = None
            self.connected = False
            print(f"已断开 {self.host}:{self.port}")


class InclinationSensorNode(Node):
    def __init__(self):
        super().__init__('inclination_sensor_node')

        # 声明参数
        self.declare_parameter('host', '192.168.1.203')
        self.declare_parameter('port', 4199)
        self.declare_parameter('slave', 5)
        self.declare_parameter('timeout', 3)
        self.declare_parameter('publish_rate', 10.0)

        # 获取参数
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        slave = self.get_parameter('slave').value
        timeout = self.get_parameter('timeout').value
        rate = self.get_parameter('publish_rate').value

        # 创建传感器实例
        self.sensor = InclinationSensor(host=host, port=port, slave=slave, timeout=timeout)
        self.sensor.connect()

        # 发布者
        self.pub = self.create_publisher(Float32MultiArray, 'inclination/euler', 10)

        # 定时器
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info('倾角传感器节点已启动 (基于 socket)')

    def timer_callback(self):
        euler = self.sensor.read_euler_angles()
        if euler is not None:
            msg = Float32MultiArray()
            msg.data = [euler['Roll'], euler['Pitch'], euler['Yaw']]
            self.pub.publish(msg)
            self.get_logger().debug(f'Roll: {euler["Roll"]:.2f}, Pitch: {euler["Pitch"]:.2f}, Yaw: {euler["Yaw"]:.2f}')
        else:
            self.get_logger().warn('无法读取欧拉角')

    def destroy_node(self):
        self.sensor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InclinationSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()