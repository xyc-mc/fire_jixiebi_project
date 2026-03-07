#include <memory>
#include <mutex>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "unitreemotor_msgs/msg/motor_command.hpp"
#include "unitreemotor_msgs/msg/motor_status.hpp"
#include "unitreemotor_msgs/srv/motor_control.hpp"
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

class UnitreeMotorNode : public rclcpp::Node
{
public:
    // 控制模式枚举，与MotorCommand.msg保持一致
    enum class ControlMode : uint8_t
    {
        STOP = 0,
        VELOCITY_CONTROL = 2,
        POSITION_CONTROL = 1,
        TORQUE_CONTROL = 4,
        DAMPING = 3,
        ZERO_TORQUE = 5,
        POS_TORQUE = 6
    };

    UnitreeMotorNode() : Node("unitree_motor_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("motor_id", 0);
        this->declare_parameter<double>("default_kp", 0.01);
        this->declare_parameter<double>("default_kd", 0.01);
        this->declare_parameter<double>("gear_ratio", queryGearRatio(MotorType::GO_M8010_6));
        this->declare_parameter<double>("control_frequency", 200.0);
        this->declare_parameter<double>("max_velocity", 10.0);
        this->declare_parameter<double>("max_torque", 5.0);

        // 获取参数
        std::string port = this->get_parameter("port").as_string();
        motor_id_ = this->get_parameter("motor_id").as_int();
        default_kp_ = this->get_parameter("default_kp").as_double();
        default_kd_ = this->get_parameter("default_kd").as_double();
        gear_ratio_ = this->get_parameter("gear_ratio").as_double();
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        max_torque_ = this->get_parameter("max_torque").as_double();
        double control_frequency = this->get_parameter("control_frequency").as_double();

        // 初始化状态变量
        current_angle_ = 0.0;
        current_torque_ = 0.0;
        current_velocity_ = 0.0;
        target_angle_ = 0.0;
        target_velocity_ = 0.0;
        target_torque_ = 0.0;
        current_mode_ = ControlMode::STOP; // 初始为停止模式

        // 初始化串口
        RCLCPP_INFO(this->get_logger(), "初始化串口: %s", port.c_str());
        try
        {
            serial_port_ = std::make_unique<SerialPort>(port);
            RCLCPP_INFO(this->get_logger(), "串口初始化成功");
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(this->get_logger(), "串口初始化失败: %s", e.what());
            throw;
        }

        // 初始化电机命令
        init_motor_cmd();

        // 创建发布者 - 发布位置和力矩
        position_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "motor/position", 10);
        torque_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "motor/torque", 10);
        status_pub_ = this->create_publisher<unitreemotor_msgs::msg::MotorStatus>(
            "motor/status", 10);

        // 创建订阅者 - 接收控制命令（使用MotorCommand消息）
        command_sub_ = this->create_subscription<unitreemotor_msgs::msg::MotorCommand>(
            "motor/command", 10,
            std::bind(&UnitreeMotorNode::command_callback, this, std::placeholders::_1));

        // 创建服务 - 控制电机
        control_service_ = this->create_service<unitreemotor_msgs::srv::MotorControl>(
            "motor/control",
            std::bind(&UnitreeMotorNode::control_service_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // 创建定时器 - 控制循环
        control_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / control_frequency),
            std::bind(&UnitreeMotorNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Unitree电机节点启动成功");
        RCLCPP_INFO(this->get_logger(), "电机ID: %d, 减速比: %.2f", motor_id_, gear_ratio_);
    }

    ~UnitreeMotorNode()
    {
        stop_motor();
    }

private:
    void init_motor_cmd()
    {
        cmd_.motorType = MotorType::GO_M8010_6;
        data_.motorType = MotorType::GO_M8010_6;
        cmd_.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
        cmd_.id = motor_id_;
        cmd_.kp = default_kp_;
        cmd_.kd = default_kd_;
        cmd_.q = 0.0;
        cmd_.dq = 0.0;
        cmd_.tau = 0.0;
    }

    void command_callback(const unitreemotor_msgs::msg::MotorCommand::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(motor_mutex_);

        switch (static_cast<ControlMode>(msg->command_type))
        {
        case ControlMode::STOP:
            current_mode_ = ControlMode::STOP;
            RCLCPP_INFO(this->get_logger(), "收到停止命令");
            break;

        case ControlMode::VELOCITY_CONTROL:
            current_mode_ = ControlMode::VELOCITY_CONTROL;
            target_velocity_ = msg->target_value;
            cmd_.kp = 0.0; // 速度模式下kp必须为0
            cmd_.kd = (msg->kd > 0) ? msg->kd : default_kd_;
            RCLCPP_INFO(this->get_logger(), "速度控制模式, 目标速度: %.3f rad/s, kd: %.3f",
                        target_velocity_, cmd_.kd);
            break;

        case ControlMode::POSITION_CONTROL:
            current_mode_ = ControlMode::POSITION_CONTROL;
            target_angle_ = msg->target_value;
            cmd_.kp = (msg->kp > 0) ? msg->kp : default_kp_;
            cmd_.kd = (msg->kd > 0) ? msg->kd : default_kd_;
            RCLCPP_INFO(this->get_logger(), "位置控制模式, 目标位置: %.3f rad, kp: %.3f, kd: %.3f",
                        target_angle_, cmd_.kp, cmd_.kd);
            break;

        case ControlMode::TORQUE_CONTROL:
            current_mode_ = ControlMode::TORQUE_CONTROL;
            target_torque_ = msg->target_value;
            cmd_.kp = 0.0;
            cmd_.kd = 0.0;
            RCLCPP_INFO(this->get_logger(), "力矩控制模式, 目标力矩: %.3f Nm", target_torque_);
            break;

        case ControlMode::POS_TORQUE:
            current_mode_ = ControlMode::POS_TORQUE;
            target_angle_ = msg->target_value;
            target_velocity_ = msg->target_velocity; // 使用新字段
            target_torque_ = msg->tau;
            cmd_.kp = (msg->kp > 0) ? msg->kp : default_kp_;
            cmd_.kd = (msg->kd > 0) ? msg->kd : default_kd_;
            RCLCPP_INFO(this->get_logger(), "位置力矩控制模式, 目标位置: %.3f rad, 目标速度: %.3f rad/s, 目标力矩: %.3f Nm, kp: %.3f, kd: %.3f",
                        target_angle_, target_velocity_, target_torque_, cmd_.kp, cmd_.kd);
            break;

        case ControlMode::DAMPING:
            current_mode_ = ControlMode::DAMPING;
            cmd_.kd = (msg->kd > 0) ? msg->kd : default_kd_;
            RCLCPP_INFO(this->get_logger(), "阻尼模式, kd: %.3f", cmd_.kd);
            break;

        case ControlMode::ZERO_TORQUE:
            current_mode_ = ControlMode::ZERO_TORQUE;
            RCLCPP_INFO(this->get_logger(), "零力矩模式");
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "未知的控制模式: %d", msg->command_type);
            break;
        }

        // 更新限制参数
        if (msg->max_velocity > 0)
            max_velocity_ = msg->max_velocity;
        if (msg->max_torque > 0)
            max_torque_ = msg->max_torque;
    }

    void control_service_callback(
        const std::shared_ptr<unitreemotor_msgs::srv::MotorControl::Request> request,
        std::shared_ptr<unitreemotor_msgs::srv::MotorControl::Response> response)
    {
        std::lock_guard<std::mutex> lock(motor_mutex_);

        response->success = true;

        // 根据命令类型设置控制模式（与服务定义完全一致）
        switch (request->command)
        {
        case 0: // 停止模式
            current_mode_ = ControlMode::STOP;
            response->message = "停止模式已设置";
            RCLCPP_INFO(this->get_logger(), "停止模式");
            break;

        case 1: // 位置模式
            current_mode_ = ControlMode::POSITION_CONTROL;
            target_angle_ = request->value;
            cmd_.kp = (request->kp > 0) ? request->kp : default_kp_;
            cmd_.kd = (request->kd > 0) ? request->kd : default_kd_;
            response->message = "位置模式已设置，目标角度: " + std::to_string(request->value) + " rad";
            RCLCPP_INFO(this->get_logger(), "位置模式，目标角度: %.3f rad, kp: %.3f, kd: %.3f",
                        target_angle_, cmd_.kp, cmd_.kd);
            break;

        case 2: // 速度模式
            current_mode_ = ControlMode::VELOCITY_CONTROL;
            target_velocity_ = request->value;
            cmd_.kp = 0.0; // 速度模式下kp必须为0
            cmd_.kd = (request->kd > 0) ? request->kd : default_kd_;
            response->message = "速度模式已设置，目标速度: " + std::to_string(request->value) + " rad/s";
            RCLCPP_INFO(this->get_logger(), "速度模式，目标速度: %.3f rad/s, kd: %.3f",
                        target_velocity_, cmd_.kd);
            break;

        case 3: // 阻尼模式
            current_mode_ = ControlMode::DAMPING;
            cmd_.kd = (request->kd > 0) ? request->kd : default_kd_;
            response->message = "阻尼模式已设置，阻尼系数: " + std::to_string(cmd_.kd);
            RCLCPP_INFO(this->get_logger(), "阻尼模式，kd: %.3f", cmd_.kd);
            break;

        case 4: // 力矩模式
            current_mode_ = ControlMode::TORQUE_CONTROL;
            target_torque_ = request->value;
            cmd_.kp = 0.0;
            cmd_.kd = 0.0;
            response->message = "力矩模式已设置，目标力矩: " + std::to_string(request->value) + " Nm";
            RCLCPP_INFO(this->get_logger(), "力矩模式，目标力矩: %.3f Nm", target_torque_);
            break;

        case 5: // 零力矩模式
            current_mode_ = ControlMode::ZERO_TORQUE;
            cmd_.kp = 0.0;
            cmd_.kd = 0.0;
            response->message = "零力矩模式已设置";
            RCLCPP_INFO(this->get_logger(), "零力矩模式");
            break;

        case 6: // 力位混合模式
            current_mode_ = ControlMode::POS_TORQUE;
            target_angle_ = request->value;
            target_velocity_ = 0.0;        // 服务中没有目标速度字段，设为0
            target_torque_ = request->tau; // 使用tau字段作为力矩值
            cmd_.kp = (request->kp > 0) ? request->kp : default_kp_;
            cmd_.kd = (request->kd > 0) ? request->kd : default_kd_;
            response->message = "力位混合模式已设置，目标位置: " + std::to_string(request->value) +
                                " rad, 目标力矩: " + std::to_string(request->tau) + " Nm";
            RCLCPP_INFO(this->get_logger(), "力位混合模式，目标位置: %.3f rad, 目标力矩: %.3f Nm, kp: %.3f, kd: %.3f",
                        target_angle_, target_torque_, cmd_.kp, cmd_.kd);
            break;

        default:
            response->success = false;
            response->message = "未知的控制命令: " + std::to_string(request->command);
            RCLCPP_WARN(this->get_logger(), "未知的控制命令: %d", request->command);
            break;
        }

        // 更新限制参数
        if (request->max_velocity > 0)
            max_velocity_ = request->max_velocity;
        if (request->max_torque > 0)
            max_torque_ = request->max_torque;

        // 更新响应中的状态
        update_motor_status(response->status);
    }

    void control_loop()
    {
        if (!serial_port_)
            return;

        std::lock_guard<std::mutex> lock(motor_mutex_);

        try
        {
            // 根据当前控制模式设置电机命令
            double limited_velocity = 0.0; // 在switch外部定义，避免跳转问题

            switch (current_mode_)
            {
            case ControlMode::STOP:
                // 停止模式：保持当前位置，速度为0
                cmd_.q = current_angle_ * gear_ratio_;
                cmd_.dq = 0.0;
                cmd_.tau = 0.0;
                break;

            case ControlMode::POSITION_CONTROL:
                // 位置模式: q=目标位置, dq=0, tau=0
                cmd_.q = target_angle_ * gear_ratio_;
                cmd_.dq = 0.0;
                cmd_.tau = 0.0;
                break;

            case ControlMode::VELOCITY_CONTROL:
                // 速度模式: q=0, dq=目标速度, kp=0, tau=0
                cmd_.q = 0.0;
                // 限制速度不超过最大值
                limited_velocity = target_velocity_;
                if (max_velocity_ > 0 && std::abs(target_velocity_) > max_velocity_)
                {
                    limited_velocity = (target_velocity_ > 0) ? max_velocity_ : -max_velocity_;
                }
                cmd_.dq = limited_velocity * gear_ratio_;
                cmd_.kp = 0.0; // 速度模式下kp必须为0
                cmd_.tau = 0.0;
                break;

            case ControlMode::TORQUE_CONTROL:
                // 力矩模式: q=0, dq=0, kp=0, kd=0, tau=目标力矩
                cmd_.q = 0.0;
                cmd_.dq = 0.0;
                cmd_.kp = 0.0;
                cmd_.kd = 0.0;
                cmd_.tau = target_torque_;
                // 限制力矩不超过最大值
                if (max_torque_ > 0 && std::abs(cmd_.tau) > max_torque_)
                {
                    cmd_.tau = (cmd_.tau > 0) ? max_torque_ : -max_torque_;
                }
                break;

            case ControlMode::DAMPING:
                // 阻尼模式: q=0, dq=0, kp=0, kd=阻尼系数, tau=0
                cmd_.q = 0.0;
                cmd_.dq = 0.0;
                cmd_.kp = 0.0;
                cmd_.tau = 0.0;
                // kd已经在命令回调中设置
                break;

            case ControlMode::ZERO_TORQUE:
                // 零力矩模式: 所有参数为0
                cmd_.q = 0.0;
                cmd_.dq = 0.0;
                cmd_.kp = 0.0;
                cmd_.kd = 0.0;
                cmd_.tau = 0.0;
                break;

            case ControlMode::POS_TORQUE:
                // 位置力矩混合模式
                cmd_.q = target_angle_ * gear_ratio_;
                cmd_.dq = target_velocity_ * gear_ratio_; // 使用目标角速度
                cmd_.tau = target_torque_;                // 使用前馈力矩

                // 限制力矩不超过最大值
                if (max_torque_ > 0 && std::abs(cmd_.tau) > max_torque_)
                {
                    cmd_.tau = (cmd_.tau > 0) ? max_torque_ : -max_torque_;
                }

                // 限制速度不超过最大值
                if (max_velocity_ > 0 && std::abs(cmd_.dq / gear_ratio_) > max_velocity_)
                {
                    double limited = (cmd_.dq > 0) ? max_velocity_ * gear_ratio_ : -max_velocity_ * gear_ratio_;
                    cmd_.dq = limited;
                }
                break;

            default:
                // 默认处理
                RCLCPP_WARN(this->get_logger(), "未知的控制模式: %d", static_cast<int>(current_mode_));
                break;
            }

            // 发送控制命令并接收数据
            if (serial_port_->sendRecv(&cmd_, &data_))
            {
                // 更新当前状态
                current_angle_ = data_.q / gear_ratio_;
                current_velocity_ = data_.dq / gear_ratio_;
                current_torque_ = data_.tau;

                // 发布状态信息
                publish_motor_status();

                // 发布单独的位置和力矩消息
                auto position_msg = std_msgs::msg::Float64();
                position_msg.data = current_angle_;
                position_pub_->publish(position_msg);

                auto torque_msg = std_msgs::msg::Float64();
                torque_msg.data = current_torque_;
                torque_pub_->publish(torque_msg);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "与电机通信失败");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "控制循环异常: %s", e.what());
        }
    }

    void stop_motor()
    {
        // 切换到停止模式
        current_mode_ = ControlMode::STOP;
        cmd_.tau = 0.0;
        cmd_.dq = 0.0;
        cmd_.q = current_angle_ * gear_ratio_;

        // 发送停止命令
        if (serial_port_)
        {
            try
            {
                serial_port_->sendRecv(&cmd_, &data_);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "停止电机时出错: %s", e.what());
            }
        }
    }

    void publish_motor_status()
    {
        auto status_msg = unitreemotor_msgs::msg::MotorStatus();

        // 设置时间戳
        status_msg.header.stamp = this->now();
        status_msg.header.frame_id = "motor_" + std::to_string(motor_id_);

        // 基础状态
        status_msg.position = current_angle_;
        status_msg.velocity = current_velocity_;
        status_msg.torque = current_torque_;
        status_msg.temperature = static_cast<double>(data_.temp);

        // 注意：电压和电流从库文件中没有，设为0
        status_msg.voltage = 0.0;
        status_msg.current = 0.0;

        // 状态标志
        status_msg.is_enabled = (current_mode_ != ControlMode::STOP);
        status_msg.is_connected = true;
        status_msg.error_code = data_.merror;
        status_msg.error_msg = "data_.merror"; // 根据需要设置
        status_msg.control_mode = static_cast<uint8_t>(current_mode_);

        // 目标值
        switch (current_mode_)
        {
        case ControlMode::POSITION_CONTROL:
            status_msg.target_position = target_angle_;
            status_msg.target_velocity = 0.0;
            status_msg.target_torque = 0.0;
            break;
        case ControlMode::VELOCITY_CONTROL:
            status_msg.target_position = 0.0;
            status_msg.target_velocity = target_velocity_;
            status_msg.target_torque = 0.0;
            break;
        case ControlMode::TORQUE_CONTROL:
            status_msg.target_position = 0.0;
            status_msg.target_velocity = 0.0;
            status_msg.target_torque = target_torque_;
            break;
        case ControlMode::POS_TORQUE:
            status_msg.target_position = target_angle_;
            status_msg.target_velocity = target_velocity_;
            status_msg.target_torque = target_torque_;
            break;
        default:
            status_msg.target_position = 0.0;
            status_msg.target_velocity = 0.0;
            status_msg.target_torque = 0.0;
            break;
        }

        status_pub_->publish(status_msg);
    }

    void update_motor_status(unitreemotor_msgs::msg::MotorStatus &status)
    {
        // 更新时间戳
        status.header.stamp = this->now();
        status.header.frame_id = "motor_" + std::to_string(motor_id_);

        // 基础状态
        status.position = current_angle_;
        status.velocity = current_velocity_;
        status.torque = current_torque_;
        status.temperature = static_cast<double>(data_.temp);
        status.voltage = 0.0;
        status.current = 0.0;

        // 状态标志
        status.is_enabled = (current_mode_ != ControlMode::STOP);
        status.is_connected = true;
        status.error_code = data_.merror;
        status.error_msg = "";
        status.control_mode = static_cast<uint8_t>(current_mode_);

        // 目标值
        switch (current_mode_)
        {
        case ControlMode::POSITION_CONTROL:
            status.target_position = target_angle_;
            status.target_velocity = 0.0;
            status.target_torque = 0.0;
            break;
        case ControlMode::VELOCITY_CONTROL:
            status.target_position = 0.0;
            status.target_velocity = target_velocity_;
            status.target_torque = 0.0;
            break;
        case ControlMode::TORQUE_CONTROL:
            status.target_position = 0.0;
            status.target_velocity = 0.0;
            status.target_torque = target_torque_;
            break;
        case ControlMode::POS_TORQUE:
            status.target_position = target_angle_;
            status.target_velocity = target_velocity_;
            status.target_torque = target_torque_;
            break;
        default:
            status.target_position = 0.0;
            status.target_velocity = 0.0;
            status.target_torque = 0.0;
            break;
        }
    }

private:
    // ROS2接口
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_pub_;
    rclcpp::Publisher<unitreemotor_msgs::msg::MotorStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<unitreemotor_msgs::msg::MotorCommand>::SharedPtr command_sub_;
    rclcpp::Service<unitreemotor_msgs::srv::MotorControl>::SharedPtr control_service_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 串口和电机
    std::unique_ptr<SerialPort> serial_port_;
    MotorCmd cmd_;
    MotorData data_;

    // 参数
    int motor_id_;
    double default_kp_;
    double default_kd_;
    double gear_ratio_;
    double max_velocity_;
    double max_torque_;

    // 状态变量
    ControlMode current_mode_;
    double current_angle_;
    double current_velocity_;
    double current_torque_;
    double target_angle_;
    double target_velocity_;
    double target_torque_;

    // 互斥锁
    std::mutex motor_mutex_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<UnitreeMotorNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    }
    catch (const std::exception &e)
    {
        std::cerr << "程序异常: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}