#ifndef __COM_MC_MASTER_H__
#define __COM_MC_MASTER_H__

/******************************************************************************/
/** Includes                                                                 **/
/******************************************************************************/
#include <rtdef.h>

/******************************************************************************/
/** Exported Define                                                          **/
/******************************************************************************/
#define  MC_CAN_NAME              "can1"

#define  THREAD_MC_SEND_STACK_SIZE  2048   
#define  THREAD_MC_SEND_PRIORITY       5
#define  THREAD_MC_SEND_TIMESLICE      5

#define  THREAD_MC_RECV_STACK_SIZE  2048
#define  THREAD_MC_RECV_PRIORITY       4
#define  THREAD_MC_RECV_TIMESLICE      5

#define  MQ_MC_CTRL_STACK_SIZE       128

#define  MC_NUM                        3  // 电机数量

#define  MC_CMD_SPEED_CLOSED_LOOP   0xA2  // 速度闭环控制命令
#define  MC_CMD_SET_ACCEL           0x43  // 写入加减速度到 RAM 和 ROM 命令
#define  MC_CMD_CLOSE               0x80  // 电机关闭命令
#define  MC_CMD_STOP                0x81  // 电机停止命令

#define  MC_MODE_SPEED                 1
#define  MC_MODE_CLOSE                 2

#define  WHEEL_RADIUS                   48.74//mm
#define  ENCODER_MM                  0.0158310f //单位1mm
#define  WHEEL_CIRCUMFERENCE         306.2  // 单位 mm
#define  ENCODER_MAX_RANGE           65535  // 单位rad
#define  MC_RUN_SPEED_MAX        10.0f  // Run Speed
#define  MC_BRUSH_SPEED_MAX      28.0f  // Run Speed

#define  MOTOR_MASTER_ID               99


/* Exported types ------------------------------------------------------------*/
struct mc_ctrl_msg {
    float left_speed;   // 左轮速度
    float right_speed;  // 右轮速度
    float front_speed;  // 前毛刷速度
};

struct mc_master {
    rt_int16_t tempr[MC_NUM];   // 电机温度  1℃/LSB
    rt_int16_t iq[MC_NUM];      // 电机转矩电流值  0.01A/LSB
    rt_int16_t speed[MC_NUM];   // 电机输出轴转速  1dps/LSB
    rt_int32_t degree[MC_NUM];  // 电机输出轴角度  1degree/LSB
};

/******************************************************************************/
/** Exported Variables                                                       **/
/******************************************************************************/
extern struct rt_messagequeue mc_ctrl_mq;
extern struct mc_master mc_master;

#endif /* __COM_MC_MASTER_H__ */

/******************************************************************************/
/** (C) COPYRIGHT 2024 WUJIAN                                    END OF FILE **/
/******************************************************************************/
