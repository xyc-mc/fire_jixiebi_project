import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ConnectionException, ModbusException

class InfraredDistance:
    """单个红外距离传感器的 Modbus 驱动类"""
    def __init__(self, host, port, timeout=5):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.client = ModbusTcpClient(host=self.host, port=self.port, timeout=self.timeout)
        self.connected = False

    def connect(self):
        """建立与 Modbus 服务器的连接"""
        try:
            self.connected = self.client.connect()
            if self.connected:
                print(f"成功连接到 {self.host}:{self.port}")
            else:
                print(f"无法连接到 {self.host}:{self.port}")
            return self.connected
        except Exception as e:
            print(f"连接异常: {e}")
            self.connected = False
            return False

    def read_once(self):
        """读取一次距离值，成功返回整数，失败返回 None"""
        if not self.connected:
            print(f"传感器 {self.host}:{self.port} 未连接，尝试重新连接...")
            if not self.connect():
                return None

        try:
            # 读取保持寄存器 0x0201，从站地址默认为 1
            response = self.client.read_holding_registers(0x0201)

            if not response.isError():
                register_value = response.registers[0]  # 寄存器值直接为整数
                return register_value
            else:
                print(f"读取失败: {self.host}:{self.port} - {response}")
                return None
        except (ConnectionException, ModbusException) as e:
            print(f"读取异常: {e}")
            self.connected = False  # 连接可能已断开，标记为未连接
            return None

    def close(self):
        """关闭 Modbus 连接"""
        if self.client:
            self.client.close()
            self.connected = False
            print(f"已断开 {self.host}:{self.port}")


class InfraredDistanceNode(Node):
    """ROS2 节点，管理两个红外传感器并发布距离数据"""
    def __init__(self):
        super().__init__('infrared_distance_node')

        # 声明参数
        self.declare_parameter('sensor1_host', '192.168.1.202')
        self.declare_parameter('sensor1_port', 4198)
        self.declare_parameter('sensor2_host', '192.168.1.201')
        self.declare_parameter('sensor2_port', 4197)
        self.declare_parameter('timeout', 5)
        self.declare_parameter('publish_rate', 10.0)  # Hz

        # 获取参数
        sensor1_host = self.get_parameter('sensor1_host').value
        sensor1_port = self.get_parameter('sensor1_port').value
        sensor2_host = self.get_parameter('sensor2_host').value
        sensor2_port = self.get_parameter('sensor2_port').value
        timeout = self.get_parameter('timeout').value
        rate = self.get_parameter('publish_rate').value

        # 创建两个传感器实例
        self.sensor1 = InfraredDistance(host=sensor1_host, port=sensor1_port, timeout=timeout)
        self.sensor2 = InfraredDistance(host=sensor2_host, port=sensor2_port, timeout=timeout)

        # 尝试连接
        self.sensor1.connect()
        self.sensor2.connect()

        # 创建发布者
        self.pub1 = self.create_publisher(Int32, 'sensor1/distance', 10)
        self.pub2 = self.create_publisher(Int32, 'sensor2/distance', 10)

        # 创建定时器，周期读取并发布
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info('红外距离节点已启动')

    def timer_callback(self):
        # 读取传感器1
        dist1 = self.sensor1.read_once()
        if dist1 is not None:
            msg = Int32()
            msg.data = dist1
            self.pub1.publish(msg)
            self.get_logger().debug(f'sensor1 distance: {dist1}')
        else:
            self.get_logger().warn('无法读取传感器1')

        # 读取传感器2
        dist2 = self.sensor2.read_once()
        if dist2 is not None:
            msg = Int32()
            msg.data = dist2
            self.pub2.publish(msg)
            self.get_logger().debug(f'sensor2 distance: {dist2}')
        else:
            self.get_logger().warn('无法读取传感器2')
        dist = (dist1 + dist2) / 2 
        self.get_logger().debug(f'average distance: {dist}')


def main(args=None):
    rclpy.init(args=args)
    node = InfraredDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()