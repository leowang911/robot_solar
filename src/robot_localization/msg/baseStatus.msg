# msg/baseStatus.msg
Header header
int16 speed       # 速度 (单位根据设备定义)
int32 distance       #  (单位根据设备定义)
uint8 sensor_state     # 传感器状态
int8 complete_state   # 完成状态
uint8 rc_state         # RC状态
uint8 error
uint8 voltage