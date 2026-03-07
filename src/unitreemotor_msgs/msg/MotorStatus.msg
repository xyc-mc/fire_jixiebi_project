# MotorStatus.msg
std_msgs/Header header

# 基础状态
float64 position           # 当前位置/rad
float64 velocity           # 当前速度/rad/s
float64 torque             # 当前力矩/Nm
float64 temperature        # 温度/°C
float64 voltage            # 电压/V
float64 current            # 电流/A

# 状态标志
bool is_enabled            # 电机使能状态
bool is_connected          # 连接状态
uint8 error_code           # 错误码
string error_msg           # 错误信息
uint8 control_mode         # 当前控制模式

# 目标值
float64 target_position    # 目标位置
float64 target_velocity    # 目标速度
float64 target_torque      # 目标力矩