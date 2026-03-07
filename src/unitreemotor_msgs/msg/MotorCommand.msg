# MotorCommand.msg
# 电机控制命令

uint8 STOP = 0
uint8 VELOCITY_CONTROL = 2
uint8 POSITION_CONTROL = 1
uint8 TORQUE_CONTROL = 4
uint8 DAMPING = 3
uint8 ZERO_TORQUE = 5
uint8 POS_TORQUE = 6

uint8 command_type         # 控制模式类型
float64 target_value       # 目标值（位置/rad，速度/rad/s，力矩/Nm）
float64 kp                 # 比例增益
float64 kd                 # 微分增益
float64 tau                # 力矩值
float64 target_velocity    # 目标速度（用于力位混合控制）
float64 max_velocity       # 最大速度限制
float64 max_torque         # 最大力矩限制