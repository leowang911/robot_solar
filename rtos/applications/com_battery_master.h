/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-03-21     000       the first version
 */
#ifndef APPLICATIONS_COM_BATTERY_H_
#define APPLICATIONS_COM_BATTERY_H_

#include <rtdef.h>

#define BATTERY_UART_NAME "uart6"

#define THREAD_BATTERY_RECV_STACK_SIZE 4096 // 接收线程的堆栈大小
#define THREAD_BATTERY_RECV_PRIORITY 7      // 接收线程的优先级
#define THREAD_BATTERY_RECV_TIMESLICE 2     // 接收线程的时间片

#define MQ_BATTERY_RECV_STACK_SIZE 256 // 接收消息队列的堆栈大小
/******************************************************************************/
/** Exported Types                                                           **/
/******************************************************************************/
struct battery_recv_msg
{
    rt_device_t dev;
    rt_size_t size;
};

extern rt_uint8_t battery_percent;

/*error_report定义
bit0    电机通讯报错         bit1    激光测距超出范围
bit2    吊舱限位无法触发     bit3   电池内部报错
bit4    后探边保护触发         bit5     侧探边保护触发
bit6    前探边保护触发             bit7

*/
extern rt_uint8_t error_report;

#endif /* __COM_IMU_MASTER_H__ */
