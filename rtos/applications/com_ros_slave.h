/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-03-21     000       the first version
 */
#ifndef APPLICATIONS_COM_ROS_SLAVE_H_
#define APPLICATIONS_COM_ROS_SLAVE_H_

#include <rtdef.h>

#define  ROS_UART_NAME              "uart2"
#define  ROS_EN              GET_PIN(D, 4)

#define  LASER_UART_NAME            "uart4"
#define  LASER_EN            GET_PIN(D, 3)

#define  OUTPUT1_CTRL         GET_PIN(E, 10)
#define  OUTPUT2_CTRL         GET_PIN(E, 9)

#define  INPUT1_CTRL         GET_PIN(E, 14)
#define  INPUT2_CTRL         GET_PIN(E, 13)


#define  THREAD_ROS_RECV_STACK_SIZE 4096 // 控制线程的堆栈大小
#define  THREAD_ROS_RECV_PRIORITY      6  // 控制线程的优先级
#define  THREAD_ROS_RECV_TIMESLICE     6  // 控制线程的时间片

#define  MQ_ROS_RECV_STACK_SIZE       64  // 接收消息队列的堆栈大小
#define  MQ_ROS_RECV_TIMEOUT         100  // 接收超时时间 单位毫秒ms
/******************************************************************************/
/** Exported Types                                                           **/
/******************************************************************************/
struct ros_recv_msg {
    rt_device_t dev;
    rt_size_t   size;
};

struct laser_recv_msg {
    rt_device_t dev;
    rt_size_t   size;
};

struct imu_pid_loc {
    float kp;         // 比例系数 Proportional
    float ki;         // 积分系数 Integral
    float kd;         // 微分系数 Derivative

    float error;      // 当前误差
    float last_error; // 上次误差
    float sum;        // 累计积分
};

struct ros_ctrl{
    // float speed[3];       //[0,4096]对应 -20rad/s ~ 20rad/s
    rt_int32_t robot_dis;       //单位m/s
    float robot_yaw;
    rt_int16_t Rolling_speed;
    rt_int32_t sum_distance;    //单位m/s
    
    rt_uint8_t robot_state;
    rt_int8_t complete_state;
    float laser_dis[2];
    float yaw;

};

rt_int32_t updateTotalMileage(rt_int32_t distance_multi, rt_int16_t encoder, rt_int16_t encoder_last);

#endif /* __COM_IMU_MASTER_H__ */

