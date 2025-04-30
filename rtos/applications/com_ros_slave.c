/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-03-21     zyh       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#define LOG_TAG "com.ros"
#define LOG_LVL LOG_LVL_DBG
#include <ulog.h>
#include "com_ros_slave.h"
#include "com_mc_master.h"
#include "com_io_ctrl.h"
#include "com_dev_param.h"
#include <stdlib.h>
#include "modbus.h"
#include "com_battery_master.h"

#define UP_ON rt_pin_write(OUTPUT1_CTRL, PIN_HIGH)
#define UP_OFF rt_pin_write(OUTPUT1_CTRL, PIN_LOW)
#define DOWN_ON rt_pin_write(OUTPUT2_CTRL, PIN_HIGH)
#define DOWN_OFF rt_pin_write(OUTPUT2_CTRL, PIN_LOW)

#define pid_p 0.8f
#define pid_d 0.1f

rt_device_t ros_uart_dev, laser_uart_dev;

rt_align(RT_ALIGN_SIZE) rt_uint8_t ros_recv_mq_stack[MQ_ROS_RECV_STACK_SIZE];
struct rt_messagequeue ros_recv_mq;

rt_align(RT_ALIGN_SIZE) rt_uint8_t laser_recv_mq_stack[MQ_ROS_RECV_STACK_SIZE];
struct rt_messagequeue laser_recv_mq;

rt_align(RT_ALIGN_SIZE) rt_uint8_t ros_recv_thread_stack[THREAD_ROS_RECV_STACK_SIZE];
struct rt_thread ros_recv_thread;

rt_err_t ros_rx_callback(rt_device_t dev, rt_size_t size);
rt_err_t laser_rx_callback(rt_device_t dev, rt_size_t size);
rt_int8_t ros_get_state(void);
rt_int8_t get_laser(void);
void start_ros_recv_thread(void *parameter);
rt_uint8_t auto_find_corner(float yaw, rt_uint8_t sensor);
rt_uint8_t auto_washing(float yaw, rt_uint8_t sensor);
void imu_pid_loc_initialize(struct imu_pid_loc *pid, float kp, float ki, float kd);
float imu_pid_loc_calculate(struct imu_pid_loc *pid, float set_val, float actual_val);
float angle_yaw(float base_yaw, float angle);

struct ros_ctrl ros_auto;
float Motor_L, Motor_R;

int com_ros_slave_init(void)
{
    rt_err_t err = RT_EOK;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    rt_pin_mode(ROS_EN, PIN_MODE_OUTPUT);
    rt_pin_write(ROS_EN, PIN_LOW);

    rt_pin_mode(LASER_EN, PIN_MODE_OUTPUT);
    rt_pin_write(LASER_EN, PIN_LOW);

    rt_pin_mode(OUTPUT1_CTRL, PIN_MODE_OUTPUT);
    rt_pin_write(OUTPUT1_CTRL, PIN_LOW);

    rt_pin_mode(OUTPUT2_CTRL, PIN_MODE_OUTPUT);
    rt_pin_write(OUTPUT2_CTRL, PIN_LOW);

    rt_pin_mode(INPUT1_CTRL, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(INPUT2_CTRL, PIN_MODE_INPUT_PULLUP);

    /** 查找UART设备 **/
    ros_uart_dev = rt_device_find(ROS_UART_NAME);
    RT_ASSERT(ros_uart_dev != RT_NULL);

    /** 配置UART设备参数 **/
    config.baud_rate = 115200;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity = PARITY_NONE;
    config.rx_bufsz = 64;
    err = rt_device_control(ros_uart_dev, RT_DEVICE_CTRL_CONFIG, &config);
    RT_ASSERT(err == RT_EOK);
    err = rt_device_open(ros_uart_dev, RT_DEVICE_FLAG_RX_NON_BLOCKING);
    RT_ASSERT(err == RT_EOK);
    err = rt_device_set_rx_indicate(ros_uart_dev, ros_rx_callback);
    RT_ASSERT(err == RT_EOK);

    laser_uart_dev = rt_device_find(LASER_UART_NAME);
    RT_ASSERT(laser_uart_dev != RT_NULL);
    err = rt_device_control(laser_uart_dev, RT_DEVICE_CTRL_CONFIG, &config);
    RT_ASSERT(err == RT_EOK);
    err = rt_device_open(laser_uart_dev, RT_DEVICE_FLAG_RX_NON_BLOCKING);
    RT_ASSERT(err == RT_EOK);
    err = rt_device_set_rx_indicate(laser_uart_dev, laser_rx_callback);
    RT_ASSERT(err == RT_EOK);

    /**  **/
    err = rt_mq_init(&ros_recv_mq,                // 句柄
                     "ros_recv_mq",               // 名称
                     &ros_recv_mq_stack[0],       // 缓冲区起始地址
                     sizeof(struct ros_recv_msg), // 消息长度
                     sizeof(ros_recv_mq_stack),   // 缓冲区大小
                     RT_IPC_FLAG_PRIO);           // 等待方式
    if (err == RT_EOK)
    {
        LOG_I("ros init recv mq successful.");
    }
    else
    {
        LOG_E("ros init recv mq failed.");
        return -RT_ERROR;
    }

    err = rt_mq_init(&laser_recv_mq,                // 句柄
                     "laser_recv_mq",               // 名称
                     &laser_recv_mq_stack[0],       // 缓冲区起始地址
                     sizeof(struct laser_recv_msg), // 消息长度
                     sizeof(laser_recv_mq_stack),   // 缓冲区大小
                     RT_IPC_FLAG_PRIO);             // 等待方式
    if (err == RT_EOK)
    {
        LOG_I("ros init recv mq successful.");
    }
    else
    {
        LOG_E("ros init recv mq failed.");
        return -RT_ERROR;
    }

    err = rt_thread_init(&ros_recv_thread,              // 句柄
                         "ros_recv_thread",             // 名称
                         start_ros_recv_thread,         // 入口函数
                         RT_NULL,                       // 入口函数的参数
                         &ros_recv_thread_stack[0],     // 堆栈的起始地址
                         sizeof(ros_recv_thread_stack), // 堆栈大小
                         THREAD_ROS_RECV_PRIORITY,      // 优先级
                         THREAD_ROS_RECV_TIMESLICE);    // 时间片
    if (err == RT_EOK)
    {
        rt_thread_startup(&ros_recv_thread);
        LOG_I("ros init recv thread successful.");
    }
    else
    {
        LOG_E("ros init recv thread failed.");
        return -RT_ERROR;
    }
    return RT_EOK;
}
#ifdef FINSH_USING_MSH
INIT_APP_EXPORT(com_ros_slave_init);
#endif

rt_err_t ros_rx_callback(rt_device_t dev, rt_size_t size)
{
    rt_err_t err = RT_EOK;
    struct ros_recv_msg rx_msg = {0};

    rx_msg.dev = dev;
    rx_msg.size = size;

    err = rt_mq_send(&ros_recv_mq, &rx_msg, sizeof(rx_msg));

    return err;
}

rt_err_t laser_rx_callback(rt_device_t dev, rt_size_t size)
{
    rt_err_t err = RT_EOK;
    struct laser_recv_msg rx_msg = {0};

    rx_msg.dev = dev;
    rx_msg.size = size;

    err = rt_mq_send(&laser_recv_mq, &rx_msg, sizeof(rx_msg));

    return err;
}

void start_ros_recv_thread(void *parameter)
{
    rt_base_t level0;
    rt_err_t err = RT_EOK;

    struct imu_pid_loc pid; // PID
    imu_pid_loc_initialize(&pid, pid_p, 0.0, pid_d);

    float yaw, p_yaw, angle_inc, base_yaw; // 角度PID输出值
    rt_uint8_t sensor;
    struct mc_ctrl_msg ctrl_msg;
    rt_uint8_t mode, automatic, stop_align;
    rt_uint8_t ros_mode;
    rt_uint16_t laser_dis[2];
    rt_int16_t diff_dis;
    rt_uint8_t ec_state = 0;
    rt_int32_t p_dis, now_dis, last_dis;
    rt_int32_t sum_dis = 0;

    rt_uint8_t auto_runing_flag = 0;

    for (;;)
    {
        rt_thread_mdelay(10);

        if (ros_get_state() == 0)
        {
            level0 = rt_hw_interrupt_disable();
            sensor = io_sensor_msg;
            mode = dev_param.run.dev_mode;
            automatic = dev_param.run.dev_auto;
            stop_align = dev_param.run.align_enable;
            ros_mode = ros_auto.robot_state;
            ctrl_msg.front_speed = ros_auto.Rolling_speed;
            yaw = ros_auto.yaw;
            rt_hw_interrupt_enable(level0);

            if (mode != DEV_MODE_STOP)
            {
                if (automatic != DEV_AUTO_STOP)
                {
                    ros_mode = AUTOWASHING;
                    // LOG_I("ros_mode: %d", ros_mode);
                    switch (ros_mode)
                    {

                    case AUTORUNING: // 自动运行

                        level0 = rt_hw_interrupt_disable();
                        p_dis = ros_auto.robot_dis;
                        p_yaw = ros_auto.robot_yaw;
                        now_dis = ros_auto.sum_distance;
                        rt_hw_interrupt_enable(level0);
                        if (auto_runing_flag != 2)
                        {
                            ros_auto.complete_state = 0;
                            if ((abs(yaw - p_yaw) > 0.4f) && (auto_runing_flag == 0))
                            {
                                angle_inc = imu_pid_loc_calculate(&pid, p_yaw, yaw);
                                MAX_LIMIT(angle_inc, MC_RUN_SPEED_MAX / 30.0f);
                                MIN_LIMIT(angle_inc, -MC_RUN_SPEED_MAX / 30.0f);
                                MID_LIMIT(angle_inc, MC_RUN_SPEED_MAX / 50.0f);
                                ctrl_msg.left_speed = -angle_inc;
                                ctrl_msg.right_speed = angle_inc;
                            }
                            else
                            {
                                if (auto_runing_flag == 0)
                                {
                                    last_dis = now_dis;
                                    auto_runing_flag = 1;
                                    sum_dis = 0;
                                }
                                sum_dis = now_dis - last_dis;

                                if (p_dis - sum_dis > 3)
                                {
                                    if (p_dis - sum_dis > 500)
                                    {
                                        // angle_inc = imu_pid_loc_calculate(&pid, p_yaw, yaw);
                                        MAX_LIMIT(angle_inc, MC_RUN_SPEED_MAX / 5.0f);
                                        MIN_LIMIT(angle_inc, -MC_RUN_SPEED_MAX / 5.0f);
                                        ctrl_msg.left_speed = MC_RUN_SPEED_MAX / 2.0f;  //- angle_inc;
                                        ctrl_msg.right_speed = MC_RUN_SPEED_MAX / 2.0f; //+ angle_inc;
                                    }
                                    else if (p_dis - sum_dis > 100)
                                    {

                                        // angle_inc = imu_pid_loc_calculate(&pid, p_yaw, yaw);
                                        MAX_LIMIT(angle_inc, MC_RUN_SPEED_MAX / 25.0f);
                                        MIN_LIMIT(angle_inc, -MC_RUN_SPEED_MAX / 25.0f);
                                        ctrl_msg.left_speed = MC_RUN_SPEED_MAX / 10.0f;  //- angle_inc;
                                        ctrl_msg.right_speed = MC_RUN_SPEED_MAX / 10.0f; //+ angle_inc;
                                    }
                                    else
                                    {

                                        // angle_inc = imu_pid_loc_calculate(&pid, p_yaw, yaw);
                                        MAX_LIMIT(angle_inc, MC_RUN_SPEED_MAX / 50.0f);
                                        MIN_LIMIT(angle_inc, -MC_RUN_SPEED_MAX / 50.0f);
                                        ctrl_msg.left_speed = MC_RUN_SPEED_MAX / 20.0f;  //- angle_inc;
                                        ctrl_msg.right_speed = MC_RUN_SPEED_MAX / 20.0f; //+ angle_inc;
                                    }
                                }
                                else if (p_dis - sum_dis < -3)
                                {
                                    //                                    angle_inc = imu_pid_loc_calculate(&pid, p_yaw, yaw);
                                    MAX_LIMIT(angle_inc, MC_RUN_SPEED_MAX / 25.0f);
                                    MIN_LIMIT(angle_inc, -MC_RUN_SPEED_MAX / 25.0f);
                                    ctrl_msg.left_speed = -MC_RUN_SPEED_MAX / 10.0f;  //+ angle_inc;
                                    ctrl_msg.right_speed = -MC_RUN_SPEED_MAX / 10.0f; //- angle_inc;
                                }
                                else
                                {
                                    if ((abs(yaw - p_yaw) > 0.4f))
                                    {
                                        angle_inc = imu_pid_loc_calculate(&pid, p_yaw, yaw);
                                        MAX_LIMIT(angle_inc, MC_RUN_SPEED_MAX / 30.0f);
                                        MIN_LIMIT(angle_inc, -MC_RUN_SPEED_MAX / 30.0f);
                                        MID_LIMIT(angle_inc, MC_RUN_SPEED_MAX / 50.0f);
                                        ctrl_msg.left_speed = -angle_inc;
                                        ctrl_msg.right_speed = angle_inc;
                                    }
                                    else
                                    {
                                        ctrl_msg.left_speed = 0;
                                        ctrl_msg.right_speed = 0;
                                        sum_dis = 0;
                                        auto_runing_flag = 2;
                                        ros_auto.complete_state = 2;
                                    }
                                }
                            }
                        }

                        break;
                    case UNLOADING:
                    {
                        ctrl_msg.front_speed = 0;
                        ros_auto.complete_state = 0;
                        get_laser();
                        level0 = rt_hw_interrupt_disable();
                        laser_dis[0] = ros_auto.laser_dis[0];
                        laser_dis[1] = ros_auto.laser_dis[1];
                        rt_hw_interrupt_enable(level0);
                        if (rt_pin_read(INPUT1_CTRL) != 0)
                        {
                            DOWN_OFF;
                            UP_ON;
                        }
                        else
                        {
                            DOWN_OFF;
                            UP_OFF;

                            if ((laser_dis[0] < 1200) && (laser_dis[1] < 1200))
                            {
                                ctrl_msg.left_speed = -MC_RUN_SPEED_MAX / 10.0f;
                                ctrl_msg.right_speed = -MC_RUN_SPEED_MAX / 10.0f;
                                base_yaw = yaw;
                            }
                            else
                            {
                                ctrl_msg.left_speed = 0;
                                ctrl_msg.right_speed = 0;
                                ros_auto.complete_state = 3;

                                dev_param.run.auto_clean_step = FD_FORWARD_EDGE;
                            }
                        }
                    }
                    break;
                    case LOADING:
                    {
                        ctrl_msg.front_speed = 0;
                        ros_auto.complete_state = 0;
                        get_laser();
                        level0 = rt_hw_interrupt_disable();
                        laser_dis[0] = ros_auto.laser_dis[0];
                        laser_dis[1] = ros_auto.laser_dis[1];
                        rt_hw_interrupt_enable(level0);
                        //                        LOG_I("%d , %d", laser_dis[0], laser_dis[1]);
                        if ((get_laser() == 0) || (laser_dis[0] != 65535) || (laser_dis[1] != 65535))
                        {
                            diff_dis = laser_dis[0] - laser_dis[1];
                            // LOG_I("%d ",rt_pin_read(INPUT1_CTRL));
                            if (rt_pin_read(INPUT1_CTRL) == 1 && ec_state == 0)
                            {
                                DOWN_OFF;
                                UP_ON;
                            }
                            else
                            {
                                // LOG_I("1");
                                if (rt_pin_read(INPUT1_CTRL) == 0 && ec_state == 0)
                                {
                                    DOWN_OFF;
                                    UP_OFF;
                                    ec_state = 1;
                                }

                                if ((laser_dis[0] < 3000) || (laser_dis[1] < 3000))
                                {
                                    if (ec_state == 2)
                                    {
                                        if (rt_pin_read(INPUT2_CTRL))
                                        {

                                            DOWN_ON;
                                            UP_OFF;
                                        }
                                        else
                                        {
                                            UP_OFF;
                                            DOWN_OFF;

                                            ros_auto.complete_state = 4;
                                        }
                                    }
                                    else if ((laser_dis[0] > 270) && (laser_dis[1] > 270))
                                    {
                                        ctrl_msg.left_speed = MC_RUN_SPEED_MAX / 10.0f;
                                        ctrl_msg.right_speed = MC_RUN_SPEED_MAX / 10.0f;
                                    }
                                    else if (abs(diff_dis) > 5)
                                    {
                                        if (diff_dis > 0)
                                        {
                                            ctrl_msg.left_speed = MC_RUN_SPEED_MAX / 50.0f;
                                            ctrl_msg.right_speed = -MC_RUN_SPEED_MAX / 50.0f;
                                        }
                                        else
                                        {
                                            ctrl_msg.left_speed = -MC_RUN_SPEED_MAX / 50.0f;
                                            ctrl_msg.right_speed = MC_RUN_SPEED_MAX / 50.0f;
                                        }
                                    }
                                    else
                                    {

                                        if ((laser_dis[0] < 270) && (laser_dis[1] < 270))
                                        {
                                            if ((laser_dis[0] < 125) || (laser_dis[1] < 125))
                                            {
                                                ctrl_msg.left_speed = -MC_RUN_SPEED_MAX / 40.0f;
                                                ctrl_msg.right_speed = -MC_RUN_SPEED_MAX / 40.0f;
                                            }
                                            else if (((laser_dis[0] < 130) && (laser_dis[0] > 125)) && ((laser_dis[1] < 130) && (laser_dis[1] > 125)))
                                            {
                                                ctrl_msg.left_speed = 0;
                                                ctrl_msg.right_speed = 0;
                                                //                                                LOG_I("%d",rt_pin_read(INPUT2_CTRL));
                                                ec_state = 2;
                                            }
                                            else
                                            {
                                                ctrl_msg.left_speed = MC_RUN_SPEED_MAX / 30.0f;
                                                ctrl_msg.right_speed = MC_RUN_SPEED_MAX / 30.0f;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    break;
                    case ENERGENCUSTOP:
                        ros_auto.complete_state = 0;
                        ctrl_msg.left_speed = 0;
                        ctrl_msg.right_speed = 0;
                        ctrl_msg.front_speed = 0;
                        break;
                    case INIT:
                        level0 = rt_hw_interrupt_disable();
                        HAL_NVIC_SystemReset();
                        rt_hw_interrupt_enable(level0);
                        break;
                    case AUTOCORNER:
                        auto_find_corner(yaw, sensor);
                        ctrl_msg.left_speed = Motor_L;
                        ctrl_msg.right_speed = Motor_R;
                        break;
                    case AUTOWASHING:
                        auto_washing(yaw, sensor);
                        ctrl_msg.left_speed = Motor_L;
                        ctrl_msg.right_speed = Motor_R;
                        break;
                    case IDLE:
                        UP_OFF;
                        DOWN_OFF;
                        ec_state = 0;
                        auto_runing_flag = 0;
                        ros_auto.complete_state = 1;
                        ctrl_msg.left_speed = 0;
                        ctrl_msg.right_speed = 0;
                        // ctrl_msg.front_speed = 0;
                        dev_param.run.auto_clean_step = START_CALIBRATION;
                        dev_param.run.auto_coner_step = FD_FORWARD_EDGE;
                        break;
                    default:
                        ctrl_msg.left_speed = 0;
                        ctrl_msg.right_speed = 0;
                        // ctrl_msg.front_speed = 0;
                    }
                    /** 限制最大速度 **/
                    MAX_LIMIT(ctrl_msg.left_speed, MC_RUN_SPEED_MAX);
                    MIN_LIMIT(ctrl_msg.left_speed, -MC_RUN_SPEED_MAX);
                    MAX_LIMIT(ctrl_msg.right_speed, MC_RUN_SPEED_MAX);
                    MIN_LIMIT(ctrl_msg.right_speed, -MC_RUN_SPEED_MAX);

                    /** 设置毛刷速度 **/
                    MAX_LIMIT(ctrl_msg.front_speed, MC_BRUSH_SPEED_MAX);
                    MIN_LIMIT(ctrl_msg.front_speed, -MC_BRUSH_SPEED_MAX);

                    /**边界保护**/
                    if (stop_align == 2)
                    {
                        if ((ctrl_msg.left_speed < 0) && (ctrl_msg.right_speed < 0)) // 后退
                        {
                            if (((sensor & 0x10) == 0x10) || ((sensor & 0x20) == 0x20))
                            {
                                ctrl_msg.left_speed = 0;
                                ctrl_msg.right_speed = 0;
                            }
                        }
                        else if ((ctrl_msg.left_speed > 0) && (ctrl_msg.right_speed > 0)) // 前进
                        {
                            if (((sensor & 0x05) == 0x05) || ((sensor & 0x0A) == 0x0A))
                            {
                                ctrl_msg.left_speed = 0;
                                ctrl_msg.right_speed = 0;
                            }
                            else if ((sensor & 0x03) == 0x03)
                            {
                                MAX_LIMIT(ctrl_msg.left_speed, MC_RUN_SPEED_MAX / 5);
                                MIN_LIMIT(ctrl_msg.left_speed, -MC_RUN_SPEED_MAX / 5);
                                MAX_LIMIT(ctrl_msg.right_speed, MC_RUN_SPEED_MAX / 5);
                                MIN_LIMIT(ctrl_msg.right_speed, -MC_RUN_SPEED_MAX / 5);
                            }
                        }
                        if (((sensor & 0x15) == 0x15) || ((sensor & 0x2A) == 0x2A))
                        {
                            ctrl_msg.left_speed = 0;
                            ctrl_msg.right_speed = 0;
                        }
                    }

                    err = rt_mq_send(&mc_ctrl_mq, &ctrl_msg, sizeof(ctrl_msg));
                    if (err != RT_EOK)
                    {
                        LOG_E("mc send ctrl mq failed.");
                    }
                }
            }
        }
        else
        {
        }
    }
}

/** 前进到角落程序可能要改 **/

rt_uint8_t auto_find_corner(float yaw, rt_uint8_t sensor)
{
    rt_base_t level0;
    static float base_yaw = 0;
    struct mc_ctrl_msg ctrl_msg; // 电机速度发送消息队列
    struct imu_pid_loc pid;      // PID
    float angle_inc;             // 角度PID输出值

    static rt_uint8_t align_flag = 0;
    static rt_uint8_t align_cnt = 0;

    float speed_set_value;
    speed_set_value = MC_RUN_SPEED_MAX * 3 / 4;

    imu_pid_loc_initialize(&pid, pid_p, 0.0, pid_d);
    ros_auto.complete_state = 0;

    switch (dev_param.run.auto_coner_step)
    {
    case FD_FORWARD_EDGE:

        if ((sensor & 0x30) == 0)
        {
            /** 后退 - 后左右都在界内 **/
            if (align_flag == 0)
            {
                /** 初始高速 **/
                ctrl_msg.left_speed = -speed_set_value / 2.0f;
                ctrl_msg.right_speed = -speed_set_value / 2.0f;
            }
            else
            {
                /** 校准低速 **/
                ctrl_msg.left_speed = -speed_set_value / 10.0f;
                ctrl_msg.right_speed = -speed_set_value / 10.0f;
                align_flag = 1; // 后退标志
            }
        }
        else if ((sensor & 0x30) == 0x30)
        {
            /** 前进 - 后左右都在界外 **/
            ctrl_msg.left_speed = speed_set_value / 10.0f;
            ctrl_msg.right_speed = speed_set_value / 10.0f;
            if (align_flag == 1)
            {
                align_cnt++; // 校准计数
                if (align_cnt >= AUTO_ALIGN_CNT)
                {
                    /** 校准成功 **/
                    align_cnt = 0;
                    align_flag = 0;
                    ctrl_msg.left_speed = 0;
                    ctrl_msg.right_speed = 0;
                    base_yaw = yaw;                                      // 写航向角
                    dev_param.run.auto_coner_step = FD_TO_EDGE_ROTATION; // 切换下一阶段
                }
            }
            align_flag = 2; // 前进标志
        }
        else if ((sensor & 0x30) == 0x10)
        {
            /** 右转 - 后左探边传感器出界 **/
            ctrl_msg.left_speed = speed_set_value / 10.0f;
            ctrl_msg.right_speed = -speed_set_value / 10.0f;
            align_flag = 4; // 右转标志
        }
        else if ((sensor & 0x30) == 0x20)
        {
            /** 左转 - 后右探边传感器出界 **/
            ctrl_msg.left_speed = -speed_set_value / 10.0f;
            ctrl_msg.right_speed = speed_set_value / 10.0f;
            align_flag = 3; // 左转标志
        }
        break;

    case FD_TO_EDGE_ROTATION:

        if (abs(yaw - angle_yaw(base_yaw, 270.0)) > 0.5f)
        {
            angle_inc = imu_pid_loc_calculate(&pid, angle_yaw(base_yaw, 270.0), yaw);
            MAX_LIMIT(angle_inc, speed_set_value / 7.0f);
            MIN_LIMIT(angle_inc, -speed_set_value / 7.0f);
            MID_LIMIT(angle_inc, speed_set_value / 25.0f);
            ctrl_msg.left_speed = -angle_inc;
            ctrl_msg.right_speed = angle_inc;
        }
        else
        {
            align_flag = 0;
            ctrl_msg.left_speed = 0;
            ctrl_msg.right_speed = 0;
            dev_param.run.auto_coner_step = FD_FORWARD_CORNER;
        }
        break;
    case FD_FORWARD_CORNER:
        if ((sensor & 0x30) == 0)
        {
            /** 后退 - 后左右都在界内 **/
            if (align_flag == 0)
            {
                /** 初始高速 **/
                ctrl_msg.left_speed = -speed_set_value / 2.0f;
                ctrl_msg.right_speed = -speed_set_value / 2.0f;
            }
            else
            {
                /** 校准低速 **/
                ctrl_msg.left_speed = -speed_set_value / 10.0f;
                ctrl_msg.right_speed = -speed_set_value / 10.0f;
                align_flag = 1; // 后退标志
            }
        }
        else if ((sensor & 0x30) == 0x30)
        {
            /** 前进 - 后左右都在界外 **/
            ctrl_msg.left_speed = speed_set_value / 10.0f;
            ctrl_msg.right_speed = speed_set_value / 10.0f;
            if (align_flag == 1)
            {
                align_cnt++; // 校准计数
                if (align_cnt >= AUTO_ALIGN_CNT)
                {
                    /** 校准成功 **/
                    align_cnt = 0;
                    align_flag = 0;
                    ctrl_msg.left_speed = 0;
                    ctrl_msg.right_speed = 0;
                    base_yaw = yaw;                               // 写航向角
                    dev_param.run.auto_coner_step = FD_TO_CORNER; // 切换下一阶段
                }
            }
            align_flag = 2; // 前进标志
        }
        else if ((sensor & 0x30) == 0x10)
        {
            /** 右转 - 后左探边传感器出界 **/
            ctrl_msg.left_speed = speed_set_value / 10.0f;
            ctrl_msg.right_speed = -speed_set_value / 10.0f;
            align_flag = 4; // 右转标志
        }
        else if ((sensor & 0x30) == 0x20)
        {
            /** 左转 - 后右探边传感器出界 **/
            ctrl_msg.left_speed = -speed_set_value / 10.0f;
            ctrl_msg.right_speed = speed_set_value / 10.0f;
            align_flag = 3; // 左转标志
        }
        break;

    case FD_TO_CORNER:
        ctrl_msg.left_speed = 0;
        ctrl_msg.right_speed = 0;
        ros_auto.complete_state = 7;

        break;
    default:
        ctrl_msg.left_speed = 0;
        ctrl_msg.right_speed = 0;
        break;
    }

    level0 = rt_hw_interrupt_disable();
    Motor_L = ctrl_msg.left_speed;
    Motor_R = ctrl_msg.right_speed;
    rt_hw_interrupt_enable(level0);

    return 0;
}

rt_uint8_t auto_washing(float yaw, rt_uint8_t sensor)
{
    rt_base_t level0;

    static float base_yaw = 0;
    struct mc_ctrl_msg ctrl_msg; // 电机速度发送消息队列
    struct imu_pid_loc pid;      // PID
    float angle_inc;             // 角度PID输出值

    static rt_uint8_t align_flag = 0;
    static rt_uint8_t align_cnt = 0;

    static rt_int32_t position;
    rt_int32_t degree;
    static rt_int32_t multi;
    float speed_set_value;
    speed_set_value = MC_RUN_SPEED_MAX * 3 / 5;
    imu_pid_loc_initialize(&pid, pid_p, 0.0, pid_d);

    level0 = rt_hw_interrupt_disable();
    degree = ros_auto.sum_distance;
    ros_auto.complete_state = 0;
    rt_hw_interrupt_enable(level0);
    // LOG_I("robot_yaw  =%d ", dev_param.run.auto_clean_step);

    switch (dev_param.run.auto_clean_step)
    {
    case START_CALIBRATION: /** 校准 == X轴正向 **/
        if ((sensor & 0x30) == 0)
        {
            /** 后退 - 后左右都在界内 **/
            if (align_flag == 0)
            {
                /** 初始高速 **/
                ctrl_msg.left_speed = -speed_set_value / 2.0f;
                ctrl_msg.right_speed = -speed_set_value / 2.0f;
            }
            else
            {
                /** 校准低速 **/
                ctrl_msg.left_speed = -speed_set_value / 10.0f;
                ctrl_msg.right_speed = -speed_set_value / 10.0f;
                align_flag = 1; // 后退标志
            }
        }
        else if ((sensor & 0x30) == 0x30)
        {
            /** 前进 - 后左右都在界外 **/
            ctrl_msg.left_speed = speed_set_value / 10.0f;
            ctrl_msg.right_speed = speed_set_value / 10.0f;
            if (align_flag == 1)
            {
                align_cnt++; // 校准计数
                if (align_cnt >= AUTO_ALIGN_CNT)
                {
                    /** 校准成功 **/
                    align_cnt = 0;
                    align_flag = 0;
                    ctrl_msg.left_speed = 0;
                    ctrl_msg.right_speed = 0;
                    base_yaw = yaw;                             // 写航向角
                    dev_param.run.auto_clean_step = PX_FORWARD; // 切换下一阶段
                }
            }
            align_flag = 2; // 前进标志
        }
        else if ((sensor & 0x30) == 0x10)
        {
            /** 右转 - 后左探边传感器出界 **/
            ctrl_msg.left_speed = speed_set_value / 10.0f;
            ctrl_msg.right_speed = -speed_set_value / 10.0f;
            align_flag = 4; // 右转标志
        }
        else if ((sensor & 0x30) == 0x20)
        {
            /** 左转 - 后右探边传感器出界 **/
            ctrl_msg.left_speed = -speed_set_value / 10.0f;
            ctrl_msg.right_speed = speed_set_value / 10.0f;
            align_flag = 3; // 左转标志
        }
        break;
    case PX_FORWARD: /** 前进到边 == X轴正向 **/
        if (((sensor & 0x07) != 0x07) && ((sensor & 0x0B) != 0x0B))
        {
            /** 运行中 **/
            angle_inc = imu_pid_loc_calculate(&pid, angle_yaw(base_yaw, 0), yaw);

            if ((sensor & 0x03) != 0x03)
            {
                /** 高速 **/
                MAX_LIMIT(angle_inc, speed_set_value / 6.0f);
                MIN_LIMIT(angle_inc, -speed_set_value / 6.0f);
                ctrl_msg.left_speed = speed_set_value - angle_inc;
                ctrl_msg.right_speed = speed_set_value + angle_inc;
            }
            else
            {
                /** 低速 **/
                MAX_LIMIT(angle_inc, speed_set_value / 8.0f);
                MIN_LIMIT(angle_inc, -speed_set_value / 8.0f);
                ctrl_msg.left_speed = speed_set_value / 4.0f - angle_inc;
                ctrl_msg.right_speed = speed_set_value / 4.0f + angle_inc;
            }
        }
        else
        {
            /** 到达目标点 **/
            ctrl_msg.left_speed = 0;
            ctrl_msg.right_speed = 0;
            dev_param.run.auto_clean_step = PX_BACKWARD; // 切换下一阶段
            dev_param.run.auto_cleaned_line_num++;       // 增加已清洗行数
            /** 记录电机位置 **/
            position = degree;
            multi = 0;
        }
        break;
    case PX_BACKWARD: /** 后退 == X轴正向 **/
    {

        multi = degree - position;

        /** 后退指定距离 **/
        if (abs(multi) < AUTO_PX_BACKWARD_DIS)
        {
            angle_inc = imu_pid_loc_calculate(&pid, angle_yaw(base_yaw, 0), yaw);
            MAX_LIMIT(angle_inc, speed_set_value / 5.0f);
            MIN_LIMIT(angle_inc, -speed_set_value / 5.0f);
            ctrl_msg.left_speed = -speed_set_value / 2.0f - angle_inc;
            ctrl_msg.right_speed = -speed_set_value / 2.0f + angle_inc;
        }
        else
        {
            ctrl_msg.left_speed = 0;
            ctrl_msg.right_speed = 0;
            multi = 0;
            dev_param.run.auto_clean_step = PX_TO_PY_ROTATION;
        }
    }
    break;
    case PX_TO_PY_ROTATION: /** 逆时针旋转90° == X轴正向 -> Y轴正向 **/
        if (abs(yaw - angle_yaw(base_yaw, 90.0)) > 0.5f)

        {
            angle_inc = imu_pid_loc_calculate(&pid, angle_yaw(base_yaw, 90.0), yaw);
            MAX_LIMIT(angle_inc, speed_set_value / 7.0f);
            MIN_LIMIT(angle_inc, -speed_set_value / 7.0f);
            MID_LIMIT(angle_inc, speed_set_value / 25.0f);
            ctrl_msg.left_speed = -angle_inc;
            ctrl_msg.right_speed = angle_inc;
            //                                LOG_I("Left:%d  Right:%d", -angle_inc, angle_inc);
        }
        else
        {
            ctrl_msg.left_speed = 0;
            ctrl_msg.right_speed = 0;
            dev_param.run.auto_clean_step = PY_FORWARD;
            /** 记录电机位置 **/
            position = degree;

            multi = 0;
        }
        break;
    case PY_FORWARD: /** 前进换道 == Y轴正向 **/
    {

        multi = degree - position;

        /** 前进指定距离 **/
        if ((abs(multi) < AUTO_PY_FORWARD_DIS) || ((sensor & 0x03) == 0x03))
        {
            angle_inc = imu_pid_loc_calculate(&pid, angle_yaw(base_yaw, 90.0), yaw);
            if ((sensor & 0x03) != 0x03)
            {
                /** 高速 **/
                MAX_LIMIT(angle_inc, speed_set_value / 5.0f);
                MIN_LIMIT(angle_inc, -speed_set_value / 5.0f);
                ctrl_msg.left_speed = speed_set_value - angle_inc;
                ctrl_msg.right_speed = speed_set_value + angle_inc;
            }
            else
            {
                /** 低速 **/
                MAX_LIMIT(angle_inc, speed_set_value / 8.0f);
                MIN_LIMIT(angle_inc, -speed_set_value / 8.0f);
                ctrl_msg.left_speed = speed_set_value / 4.0f - angle_inc;
                ctrl_msg.right_speed = speed_set_value / 4.0f + angle_inc;
            }
        }
        else
        {
            ctrl_msg.left_speed = 0;
            ctrl_msg.right_speed = 0;
            if ((dev_param.run.auto_cleaned_line_num % 2) == 0)
            {
                dev_param.run.auto_clean_step = PY_TO_PX_ROTATION;
            }
            else
            {
                dev_param.run.auto_clean_step = PY_TO_NX_ROTATION;
            }
        }

        /** 检测是否到边 **/
        if (((sensor & 0x07) == 0x07) || ((sensor & 0x0B) == 0x0B))
        {
            ctrl_msg.left_speed = 0;
            ctrl_msg.right_speed = 0;
            dev_param.run.auto_clean_step = PY_BACKWARD;
            if (abs(multi) > AUTO_PY_FORWARD_DIS)
            {
                dev_param.run.end_flag = 1; // 行驶大于一半的距离 -- 再洗一道
            }
            else
            {
                dev_param.run.end_flag = 2;
            }
            /** 记录电机位置 **/
            position = degree;

            multi = 0;
        }
    }
    break;
    case PY_BACKWARD: /** 后退 == Y轴正向 **/
    {                 /** 左 **/
        multi = degree - position;

        /** 后退指定距离 **/
        if (abs(multi) < AUTO_PY_BACKWARD_DIS)
        {
            angle_inc = imu_pid_loc_calculate(&pid, angle_yaw(base_yaw, 90.0), yaw);
            MAX_LIMIT(angle_inc, speed_set_value / 5.0f);
            MIN_LIMIT(angle_inc, -speed_set_value / 5.0f);
            ctrl_msg.left_speed = -speed_set_value / 2.0f - angle_inc;
            ctrl_msg.right_speed = -speed_set_value / 2.0f + angle_inc;
        }
        else
        {
            //                                LOG_I("PY_BACKWARD:%d", (rt_int32_t)run_distance);
            ctrl_msg.left_speed = 0;
            ctrl_msg.right_speed = 0;
            multi = 0;

            if ((dev_param.run.auto_cleaned_line_num % 2) == 0)
            {
                if (dev_param.run.end_flag == 2)
                {
                    dev_param.run.auto_clean_step = PY_TO_NX_ROTATION; // 立即结束
                }
                else
                {
                    dev_param.run.auto_clean_step = PY_TO_PX_ROTATION; // 洗一道在结束
                }
            }
            else
            {
                dev_param.run.auto_clean_step = PY_TO_NX_ROTATION; // 立即结束
            }
        }
    }
    break;
    case PY_TO_NX_ROTATION: /** 逆时针旋转90° == Y轴正向 -> X轴负向 **/
        if (abs(yaw - angle_yaw(base_yaw, 180.0)) > 0.5f)
        {
            angle_inc = imu_pid_loc_calculate(&pid, angle_yaw(base_yaw, 180.0), yaw);
            MAX_LIMIT(angle_inc, speed_set_value / 7.0f);
            MIN_LIMIT(angle_inc, -speed_set_value / 7.0f);
            MID_LIMIT(angle_inc, speed_set_value / 25.0f);
            ctrl_msg.left_speed = -angle_inc;
            ctrl_msg.right_speed = angle_inc;
        }
        else
        {
            ctrl_msg.left_speed = 0;
            ctrl_msg.right_speed = 0;
            dev_param.run.auto_clean_step = NX_FORWARD;
        }
        break;
    case NX_FORWARD: /** 前进到边 == X轴负向 **/
        if (((sensor & 0x07) != 0x07) && ((sensor & 0x0B) != 0x0B))
        {
            /** 运行中 **/
            angle_inc = imu_pid_loc_calculate(&pid, angle_yaw(base_yaw, 180.0), yaw);
            if ((sensor & 0x03) != 0x03)
            {
                /** 高速 **/
                MAX_LIMIT(angle_inc, speed_set_value / 5.0f);
                MIN_LIMIT(angle_inc, -speed_set_value / 5.0f);
                ctrl_msg.left_speed = speed_set_value - angle_inc;
                ctrl_msg.right_speed = speed_set_value + angle_inc;
            }
            else
            {
                /** 低速 **/
                MAX_LIMIT(angle_inc, speed_set_value / 8.0f);
                MIN_LIMIT(angle_inc, -speed_set_value / 8.0f);
                ctrl_msg.left_speed = speed_set_value / 4.0f - angle_inc;
                ctrl_msg.right_speed = speed_set_value / 4.0f + angle_inc;
            }
        }
        else
        {
            /** 到达目标点 **/
            ctrl_msg.left_speed = 0;
            ctrl_msg.right_speed = 0;
            dev_param.run.auto_clean_step = NX_BACKWARD; // 切换下一阶段
            dev_param.run.auto_cleaned_line_num++;       // 增加已清洗行数
            /** 记录电机位置 **/
            position = degree;

            multi = 0;
        }
        break;
    case NX_BACKWARD: /** 后退 == X轴负向 **/
    {
        multi = degree - position;
        // position = degree;

        /** 后退指定距离 **/
        if (abs(multi) < AUTO_NX_BACKWARD_DIS)
        {
            angle_inc = imu_pid_loc_calculate(&pid, angle_yaw(base_yaw, 180.0), yaw);
            MAX_LIMIT(angle_inc, speed_set_value / 5.0f);
            MIN_LIMIT(angle_inc, -speed_set_value / 5.0f);
            ctrl_msg.left_speed = -speed_set_value / 2.0f - angle_inc;
            ctrl_msg.right_speed = -speed_set_value / 2.0f + angle_inc;
        }
        else
        {
            //                                LOG_I("NX_BACKWARD:%d", (rt_int32_t)run_distance);
            ctrl_msg.left_speed = 0;
            ctrl_msg.right_speed = 0;
            multi = 0;

            if (dev_param.run.end_flag == 0)
            {
                dev_param.run.auto_clean_step = NX_TO_PY_ROTATION;
            }
            else
            {
                dev_param.run.auto_clean_step = NX_TO_NY_ROTATION;
            }
        }
    }
    break;
    case NX_TO_PY_ROTATION: /** 顺时针旋转90° == X轴负向 -> Y轴正向 **/
        if (abs(yaw - angle_yaw(base_yaw, 90.0)) > 0.5f)
        {
            angle_inc = imu_pid_loc_calculate(&pid, angle_yaw(base_yaw, 90.0), yaw);
            MAX_LIMIT(angle_inc, speed_set_value / 7.0f);
            MIN_LIMIT(angle_inc, -speed_set_value / 7.0f);
            MID_LIMIT(angle_inc, speed_set_value / 25.0f);
            ctrl_msg.left_speed = -angle_inc;
            ctrl_msg.right_speed = angle_inc;
        }
        else
        {
            ctrl_msg.left_speed = 0;
            ctrl_msg.right_speed = 0;
            dev_param.run.auto_clean_step = PY_FORWARD;
            /** 记录电机位置 **/
            position = degree;

            multi = 0;
        }
        break;
    case PY_TO_PX_ROTATION: /** 顺时针旋转90° == Y轴正向 -> X轴正向 **/
        if (abs(yaw - angle_yaw(base_yaw, 0.0)) > 0.5f)
        {
            angle_inc = imu_pid_loc_calculate(&pid, angle_yaw(base_yaw, 0.0), yaw);
            MAX_LIMIT(angle_inc, speed_set_value / 7.0f);
            MIN_LIMIT(angle_inc, -speed_set_value / 7.0f);
            MID_LIMIT(angle_inc, speed_set_value / 25.0f);
            ctrl_msg.left_speed = -angle_inc;
            ctrl_msg.right_speed = angle_inc;
        }
        else
        {
            ctrl_msg.left_speed = 0;
            ctrl_msg.right_speed = 0;
            dev_param.run.auto_clean_step = START_CALIBRATION;
        }
        break;
    case NX_TO_NY_ROTATION: /** 逆时针旋转90° == X轴负向 -> Y轴负向 **/
        if (abs(yaw - angle_yaw(base_yaw, 270.0)) > 0.5f)
        {
            angle_inc = imu_pid_loc_calculate(&pid, angle_yaw(base_yaw, 270.0), yaw);
            MAX_LIMIT(angle_inc, speed_set_value / 7.0f);
            MIN_LIMIT(angle_inc, -speed_set_value / 7.0f);
            MID_LIMIT(angle_inc, speed_set_value / 25.0f);
            ctrl_msg.left_speed = -angle_inc;
            ctrl_msg.right_speed = angle_inc;
        }
        else
        {
            ctrl_msg.left_speed = 0;
            ctrl_msg.right_speed = 0;
            dev_param.run.auto_clean_step = NY_FORWARD;
            /** 记录电机位置 **/
            position = degree;

            multi = 0;
        }
        break;
    case NY_FORWARD: /** 前进到边 == Y轴负向 **/
        if (((sensor & 0x07) != 0x07) && ((sensor & 0x0B) != 0x0B))
        {
            /** 运行中 **/
            angle_inc = imu_pid_loc_calculate(&pid, angle_yaw(base_yaw, 270.0), yaw);
            if ((sensor & 0x03) != 0x03)
            {
                /** 高速 **/
                MAX_LIMIT(angle_inc, speed_set_value / 5.0f);
                MIN_LIMIT(angle_inc, -speed_set_value / 5.0f);
                ctrl_msg.left_speed = speed_set_value - angle_inc;
                ctrl_msg.right_speed = speed_set_value + angle_inc;
            }
            else
            {
                /** 低速 **/
                MAX_LIMIT(angle_inc, speed_set_value / 8.0f);
                MIN_LIMIT(angle_inc, -speed_set_value / 8.0f);
                ctrl_msg.left_speed = speed_set_value / 4.0f - angle_inc;
                ctrl_msg.right_speed = speed_set_value / 4.0f + angle_inc;
            }
        }
        else
        {
            /** 到达目标点 **/
            ctrl_msg.left_speed = 0;
            ctrl_msg.right_speed = 0;
            dev_param.run.auto_clean_step = NY_BACKWARD;
            /** 记录电机位置 **/
            position = degree;

            multi = 0;
        }
        break;
    case NY_BACKWARD: /** 后退 == Y轴负向 **/
    {
        multi = degree - position;

        /** 后退指定距离 **/
        if (abs(multi) < AUTO_NY_BACKWARD_DIS)
        {
            angle_inc = imu_pid_loc_calculate(&pid, angle_yaw(base_yaw, 270.0), yaw);
            MAX_LIMIT(angle_inc, speed_set_value / 5.0f);
            MIN_LIMIT(angle_inc, -speed_set_value / 5.0f);
            ctrl_msg.left_speed = -speed_set_value / 2.0f - angle_inc;
            ctrl_msg.right_speed = -speed_set_value / 2.0f + angle_inc;
        }
        else
        {
            //                                LOG_I("NX_BACKWARD:%d", (rt_int32_t)run_distance);
            ctrl_msg.left_speed = 0;
            ctrl_msg.right_speed = 0;
            multi = 0;

            dev_param.run.auto_clean_step = NY_TO_PX_ROTATION;
        }
    }
    break;
    case NY_TO_PX_ROTATION: /** 逆时针旋转90° == Y轴负向 -> X轴正向 **/
        if (abs(yaw - angle_yaw(base_yaw, 0.0)) > 0.5f)
        {
            angle_inc = imu_pid_loc_calculate(&pid, angle_yaw(base_yaw, 0.0), yaw);
            MAX_LIMIT(angle_inc, speed_set_value / 7.0f);
            MIN_LIMIT(angle_inc, -speed_set_value / 7.0f);
            MID_LIMIT(angle_inc, speed_set_value / 25.0f);
            ctrl_msg.left_speed = -angle_inc;
            ctrl_msg.right_speed = angle_inc;
        }
        else
        {
            ctrl_msg.left_speed = 0;
            ctrl_msg.right_speed = 0;
            dev_param.run.auto_clean_step = END;
        }
        break;
    case END: /** 结束 **/
        ctrl_msg.left_speed = 0;
        ctrl_msg.right_speed = 0;
        dev_param.run.end_flag = 0;              // 清除结束标志
        dev_param.run.auto_cleaned_line_num = 0; // 清除已清洗行数
        ros_auto.complete_state = 8;
        dev_param.run.auto_clean_step = START_CALIBRATION;

        break;
    default:
        break;
    }
    level0 = rt_hw_interrupt_disable();
    Motor_L = ctrl_msg.left_speed;
    Motor_R = ctrl_msg.right_speed;
    rt_hw_interrupt_enable(level0);
    return 0;
}

/**
 * @brief  位置式PID初始化。
 * @param  pid PID
 * @param  kp  KP
 * @param  ki  KI
 * @param  kd  KD
 * @retval 无返回值。
 */
void imu_pid_loc_initialize(struct imu_pid_loc *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->error = 0;
    pid->last_error = 0;
    pid->sum = 0;
}

/**
 * @brief  PID位置(Location)计算。
 * @param  pid        PID参数地址
 * @param  set_val    设置值(期望值)
 * @param  actual_val 实际值(反馈值)
 * @retval 返回PID输出值。
 */
float imu_pid_loc_calculate(struct imu_pid_loc *pid, float set_val, float actual_val)
{
    float output;

    // 更新当前误差
    pid->error = set_val - actual_val;
    if (pid->error >= 180)
    {
        pid->error = pid->error - 360;
    }
    else if (pid->error < -180)
    {
        pid->error = pid->error + 360;
    }
    // 更新累计误差
    pid->sum += pid->error;

    // 更新输出值
    output = (pid->kp * pid->error) + (pid->ki * pid->sum) + (pid->kd * (pid->error - pid->last_error));

    // 更新上次误差
    pid->last_error = pid->error;

    return output;
}

rt_int32_t updateTotalMileage(rt_int32_t distance_multi, rt_int16_t encoder, rt_int16_t encoder_last)
{
    rt_int16_t res1, res2;
    if (encoder < encoder_last) // 可能的情况
    {
        res1 = encoder + ENCODER_MAX_RANGE - encoder_last; // 正转，delta=+
        res2 = encoder - encoder_last;                     // 反转    delta=-
    }
    else // angle > last
    {
        res1 = encoder - ENCODER_MAX_RANGE - encoder_last; // 反转    delta -
        res2 = encoder - encoder_last;                     // 正转    delta +
    }
    // 不管正反转，肯定是转的角度小的那个是真的
    if (abs(res1) < abs(res2))
        distance_multi += res1;
    else
        distance_multi += res2;

    return distance_multi;
}

rt_int8_t ros_get_state(void)
{
    rt_base_t level0;
    rt_int32_t ret = 0;
    rt_uint8_t tx_buf[14] = {0};
    rt_uint8_t tx_cnt = 0;
    rt_ssize_t tx_size;
    rt_uint16_t check_sum = 0;

    rt_int16_t mc_speed_sum;
    rt_int32_t mc_dis_sum, robot_dis;
    //    struct ros_ctrl ros1;
    struct ros_recv_msg rx_msg;
    rt_ssize_t ros_mq_size;
    rt_uint8_t rx_buf[13];
    rt_size_t read_size;
    rt_int16_t Rolling_speed;
    rt_uint16_t robot_yaw, angle;
    rt_uint8_t robot_state, ctrl_state;
    rt_uint8_t battery;
    rt_uint8_t err_get;

    level0 = rt_hw_interrupt_disable();
    mc_speed_sum = (rt_int16_t)((mc_master.speed[0] + mc_master.speed[1]) / 2.0f);
    mc_dis_sum = (rt_int32_t)((mc_master.degree[0] + mc_master.degree[1]) / 2.0f);
    ctrl_state = dev_param.run.dev_auto;
    ros_auto.sum_distance = mc_dis_sum;
    err_get = error_report;
    battery = battery_percent;
    rt_hw_interrupt_enable(level0);
    //         LOG_I("%d    ,   %d", mc_speed_sum, mc_dis_sum);
    /** 发送 **/
    tx_buf[tx_cnt++] = 0xAA;

    tx_buf[tx_cnt++] = (mc_speed_sum >> 8) & 0xff; // 速度
    tx_buf[tx_cnt++] = mc_speed_sum & 0xff;

    tx_buf[tx_cnt++] = (mc_dis_sum >> 24) & 0xff; // 距离
    tx_buf[tx_cnt++] = (mc_dis_sum >> 16) & 0xff;
    tx_buf[tx_cnt++] = (mc_dis_sum >> 8) & 0xff; // 距离
    tx_buf[tx_cnt++] = mc_dis_sum & 0xff;

    tx_buf[tx_cnt++] = io_sensor_msg; // 传感器状态
    tx_buf[tx_cnt++] = 0x00;

    tx_buf[tx_cnt++] = ros_auto.complete_state; // 完成状态
    tx_buf[tx_cnt++] = ctrl_state;              // dev_param.run.dev_auto;
    tx_buf[tx_cnt++] = battery;
    tx_buf[tx_cnt++] = err_get;

    for (int i = 0; i < tx_cnt; ++i)
        check_sum += tx_buf[i];
    tx_buf[tx_cnt++] = (rt_uint8_t)(check_sum & 0xff);

    rt_pin_write(ROS_EN, PIN_HIGH);
    tx_size = rt_device_write(ros_uart_dev, 0, tx_buf, tx_cnt);
    rt_pin_write(ROS_EN, PIN_LOW);
    if (tx_size != sizeof(tx_buf))
    {
        ret = -1; // 发送失败
    }
    check_sum = 0;
    /** 等待接收 **/
    ros_mq_size = rt_mq_recv(&ros_recv_mq, &rx_msg, sizeof(rx_msg), 100);
    if (ros_mq_size == sizeof(rx_msg))
    {
        read_size = rt_device_read(rx_msg.dev, 0, rx_buf, rx_msg.size);
        if ((read_size == rx_msg.size))
        {
            if (rx_buf[0] == 0x55)
            {
                for (int i = 0; i < 12; ++i)
                    check_sum += rx_buf[i];
                if ((check_sum & 0xff) == rx_buf[12])
                {
                    robot_dis = (rx_buf[4] << 24) | (rx_buf[3] << 16) | (rx_buf[2] << 8) | rx_buf[1];

                    robot_yaw = (rx_buf[6] << 8) | rx_buf[5];
                    Rolling_speed = (rx_buf[8] << 8) | rx_buf[7];
                    angle = (rx_buf[10] << 8) | rx_buf[9];
                    robot_state = rx_buf[11];
                    // yaw_state = rx_buf[12];
                      LOG_I("robot_yaw = %d", angle);
                    //                    for(int i = 0; i<11;i++)
                    //                    {
                    //                     LOG_I("robot_yaw = %d, =%d",robot_yaw,angle);
                    //                    }
                    level0 = rt_hw_interrupt_disable();
                    ros_auto.robot_dis = robot_dis;
                    ros_auto.robot_yaw = robot_yaw / 100.0f;
                    ros_auto.Rolling_speed = Rolling_speed / 100.0f;
                    ros_auto.robot_state = robot_state;
                    ros_auto.yaw = angle / 100.0f;

                    rt_hw_interrupt_enable(level0);

                    return 0;
                }
                // LOG_I("1");
            }
            // LOG_I("2");
            //                     for(int i = 0; i<11;i++)
            //                     {
            //                         LOG_I("rx_buf[%d] = %x",i,rx_buf[i]);
            //                     }
        }
    }

    return ret;
}

rt_int8_t send_laser(rt_uint8_t addr)
{
    rt_uint8_t tx_buf[8];
    rt_uint8_t tx_cnt = 0;
    rt_ssize_t tx_size;
    rt_uint16_t crc;

    tx_buf[tx_cnt++] = addr;
    tx_buf[tx_cnt++] = 0x03;
    tx_buf[tx_cnt++] = 0x00;
    tx_buf[tx_cnt++] = 0x00;
    tx_buf[tx_cnt++] = 0x00;
    tx_buf[tx_cnt++] = 0x02;
    crc = mb_crc_calculate(tx_buf, tx_cnt);
    tx_buf[tx_cnt++] = (rt_uint8_t)(crc);
    tx_buf[tx_cnt++] = (rt_uint8_t)(crc >> 8);

    rt_pin_write(LASER_EN, PIN_HIGH);
    tx_size = rt_device_write(laser_uart_dev, 0, tx_buf, tx_cnt);
    rt_pin_write(LASER_EN, PIN_LOW);
    if (tx_size != sizeof(tx_buf))
    {
        return -1; // 发送失败
    }
    return 0;
}

rt_int8_t get_laser(void)
{
    rt_base_t level0;
    struct ros_recv_msg rx_msg;
    rt_ssize_t laser_mq_size;
    rt_uint8_t rx_buf[9];
    rt_size_t read_size;
    rt_uint16_t laser_distance[2];

    if (send_laser(0x01))
        return -1;

    /** 等待接收 **/
    laser_mq_size = rt_mq_recv(&laser_recv_mq, &rx_msg, sizeof(rx_msg), 100);
    if (laser_mq_size == sizeof(rx_msg))
    {
        read_size = rt_device_read(rx_msg.dev, 0, rx_buf, rx_msg.size);
        // for(int i =0;i<read_size;i++)
        // LOG_I("log[%d] = %x",i,rx_buf[i]);

        if ((read_size == rx_msg.size))
        {

            if (rx_buf[0] == 0x01)
            {
                if (mb_crc_calculate(rx_buf, read_size) == 0)
                {
                    laser_distance[0] = (rx_buf[5] << 8) | (rx_buf[6]);

                    /** 全局变量 **/
                    level0 = rt_hw_interrupt_disable();
                    ros_auto.laser_dis[0] = laser_distance[0];
                    rt_hw_interrupt_enable(level0);
                    // LOG_I("dis[0] = %d",laser_distance[0]);
                }
            }
        }
    }

    if (send_laser(0x02))
        return -2;
    /** 等待接收 **/
    laser_mq_size = rt_mq_recv(&laser_recv_mq, &rx_msg, sizeof(rx_msg), 100);
    if (laser_mq_size == sizeof(rx_msg))
    {
        read_size = rt_device_read(rx_msg.dev, 0, rx_buf, rx_msg.size);
        if ((read_size == rx_msg.size))
        {
            if (rx_buf[0] == 0x02)
            {
                if (mb_crc_calculate(rx_buf, read_size) == 0)
                {
                    laser_distance[1] = (rx_buf[5] << 8) | rx_buf[6];
                    /** 全局变量 **/
                    level0 = rt_hw_interrupt_disable();
                    ros_auto.laser_dis[1] = laser_distance[1];
                    rt_hw_interrupt_enable(level0);
                    // LOG_I("dis[1] = %d",laser_distance[1]);
                }
            }
        }
    }
    return 0;
}

float angle_yaw(float base_yaw, float angle)
{
    float real_yaw = base_yaw + angle;
    if (real_yaw > 360)
    {
        real_yaw = real_yaw - 360;
    }
    else if (real_yaw < 0)
    {
        real_yaw = real_yaw + 360;
    }
    return real_yaw;
}
