#ifndef __COM_IO_CTRL_H__
#define __COM_IO_CTRL_H__

/******************************************************************************/
/** Includes                                                                 **/
/******************************************************************************/
#include <rtdef.h>

/******************************************************************************/
/** Exported Define                                                          **/
/******************************************************************************/
#define  IN1              GET_PIN(D,  8)
#define  IN2              GET_PIN(D,  9)
#define  IN3              GET_PIN(D, 10)
#define  IN4              GET_PIN(D, 11)
#define  IN5              GET_PIN(D, 12)
#define  IN6              GET_PIN(D, 13)
#define  IN7              GET_PIN(D, 14)
#define  IN8              GET_PIN(D, 15)

#define  OUT1             GET_PIN(A,  6)

#define  THREAD_INPUT_STACK_SIZE    2048
#define  THREAD_INPUT_PRIORITY         6
#define  THREAD_INPUT_TIMESLICE        5

#define  THREAD_OUTPUT_STACK_SIZE   2048
#define  THREAD_OUTPUT_PRIORITY       15
#define  THREAD_OUTPUT_TIMESLICE       5

#define  MQ_OUT_CTRL_STACK_SIZE       64

#define  IO_CMD_MOTOR_BRAKE_ON         1  // 高电平 - 锁死-不通电
#define  IO_CMD_MOTOR_BRAKE_OFF        2  // 低电平 - 启动-通电

/******************************************************************************/
/** Exported Variables                                                       **/
/******************************************************************************/
extern struct rt_messagequeue out_ctrl_mq;
extern rt_uint8_t io_sensor_msg;

#endif /* __COM_IO_CTRL_H__ */

/******************************************************************************/
/** (C) COPYRIGHT 2024 WUJIAN                                    END OF FILE **/
/******************************************************************************/
