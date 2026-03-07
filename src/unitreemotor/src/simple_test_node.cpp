#include <unistd.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

using namespace std::chrono_literals;

class SimpleMotorNode : public rclcpp::Node
{
public:
    SimpleMotorNode() : Node("simple_motor_node")
    {
        // 初始化串口
        serial_ = new SerialPort("/dev/ttyUSB0");

        // 设置电机参数
        motor_type_ = MotorType::GO_M8010_6;
        cmd_.motorType = motor_type_;
        data_.motorType = motor_type_;
        cmd_.mode = queryMotorMode(motor_type_, MotorMode::FOC);
        cmd_.id = 0;
        cmd_.kp = 0.0;
        cmd_.kd = 0.01;
        cmd_.q = 0.0;
        gear_ratio_ = queryGearRatio(motor_type_, MotorMode);
        cmd_.dq = -6.28 * gear_ratio_; // 默认速度
        cmd_.tau = 0.0;

        // 创建定时器
        timer_ = this->create_wall_timer(
            10ms, // 100Hz
            std::bind(&SimpleMotorNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "简单电机节点启动");
    }

    ~SimpleMotorNode()
    {
        if (serial_)
        {
            delete serial_;
            serial_ = nullptr;
        }
    }

private:
    void timer_callback()
    {
        if (serial_->sendRecv(&cmd_, &data_))
        {
            RCLCPP_INFO(this->get_logger(),
                        "位置: %.3f, 速度: %.3f, 温度: %.1f, 错误: %d",
                        data_.q, data_.dq / gear_ratio_, static_cast<double>(data_.temp), data_.merror); // 这里加分号
        }
    }

    // 成员变量应该在私有部分声明
    SerialPort *serial_;
    MotorCmd cmd_;
    MotorData data_;
    MotorType motor_type_;
    double gear_ratio_;
    rclcpp::TimerBase::SharedPtr timer_;
}; // 类定义结束必须加分号！

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleMotorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}