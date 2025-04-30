/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs: 1min更新一次电池电量
 * Date           Author       Notes
 * 2025-03-21     zyh       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#define LOG_TAG "com.battery"
#define LOG_LVL LOG_LVL_DBG
#include <ulog.h>
#include "com_battery_master.h"
#include "com_mc_master.h"
#include "com_io_ctrl.h"
#include "com_dev_param.h"
#include "modbus.h"

rt_device_t battery_uart_dev;

rt_align(RT_ALIGN_SIZE) rt_uint8_t battery_recv_mq_stack[MQ_BATTERY_RECV_STACK_SIZE];
struct rt_messagequeue battery_recv_mq;

rt_align(RT_ALIGN_SIZE) rt_uint8_t battery_recv_thread_stack[THREAD_BATTERY_RECV_STACK_SIZE];
struct rt_thread battery_recv_thread;

rt_err_t battery_rx_callback(rt_device_t dev, rt_size_t size);
void start_battery_recv_thread(void *parameter);
rt_uint8_t battery_percent = 0;
rt_uint8_t error_report = 0;
int com_battery_slave_init(void)
{
    rt_err_t err = RT_EOK;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    /** 查找UART设备 **/
    battery_uart_dev = rt_device_find(BATTERY_UART_NAME);
    RT_ASSERT(battery_uart_dev != RT_NULL);

    /** 配置UART设备参数 **/
    config.baud_rate = 9600;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.parity = PARITY_NONE;
    config.rx_bufsz = 64;
    err = rt_device_control(battery_uart_dev, RT_DEVICE_CTRL_CONFIG, &config);
    RT_ASSERT(err == RT_EOK);
    err = rt_device_open(battery_uart_dev, RT_DEVICE_FLAG_RX_NON_BLOCKING);
    RT_ASSERT(err == RT_EOK);
    err = rt_device_set_rx_indicate(battery_uart_dev, battery_rx_callback);
    RT_ASSERT(err == RT_EOK);

    /**  **/
    err = rt_mq_init(&battery_recv_mq,                // 句柄
                     "battery_recv_mq",               // 名称
                     &battery_recv_mq_stack[0],       // 缓冲区起始地址
                     sizeof(struct battery_recv_msg), // 消息长度
                     sizeof(battery_recv_mq_stack),   // 缓冲区大小
                     RT_IPC_FLAG_PRIO);               // 等待方式
    if (err == RT_EOK)
    {
        LOG_I("battery init recv mq successful.");
    }
    else
    {
        LOG_E("battery init recv mq failed.");
        return -RT_ERROR;
    }
    err = rt_thread_init(&battery_recv_thread,              // 句柄
                         "battery_recv_thread",             // 名称
                         start_battery_recv_thread,         // 入口函数
                         RT_NULL,                           // 入口函数的参数
                         &battery_recv_thread_stack[0],     // 堆栈的起始地址
                         sizeof(battery_recv_thread_stack), // 堆栈大小
                         THREAD_BATTERY_RECV_PRIORITY,      // 优先级
                         THREAD_BATTERY_RECV_TIMESLICE);    // 时间片
    if (err == RT_EOK)
    {
        rt_thread_startup(&battery_recv_thread);
        LOG_I("battery init recv thread successful.");
    }
    else
    {
        LOG_E("battery init recv thread failed.");
        return -RT_ERROR;
    }
    return RT_EOK;
}
#ifdef FINSH_USING_MSH
INIT_APP_EXPORT(com_battery_slave_init);
#endif

rt_err_t battery_rx_callback(rt_device_t dev, rt_size_t size)
{
    rt_err_t err = RT_EOK;
    struct battery_recv_msg rx_msg = {0};

    rx_msg.dev = dev;
    rx_msg.size = size;

    err = rt_mq_send(&battery_recv_mq, &rx_msg, sizeof(rx_msg));

    return err;
}
void start_battery_recv_thread(void *parameter)
{
    rt_base_t level0;
    rt_uint8_t tx_buf[7];
    rt_uint8_t tx_cnt = 0;
    rt_ssize_t tx_size;

    struct battery_recv_msg rx_msg;
    rt_ssize_t battery_mq_size;
    rt_uint8_t rx_buf[50];
    rt_size_t read_size;
    rt_uint8_t battery_get;
    rt_uint16_t battery_error = 0;

    for (;;)
    {
        rt_thread_mdelay(10000);

        tx_cnt = 0;
        tx_buf[tx_cnt++] = 0xdd;
        tx_buf[tx_cnt++] = 0xa5;
        tx_buf[tx_cnt++] = 0x03;
        tx_buf[tx_cnt++] = 0x00;
        tx_buf[tx_cnt++] = 0xff;
        tx_buf[tx_cnt++] = 0xfd;
        tx_buf[tx_cnt++] = 0x77;

        tx_size = rt_device_write(battery_uart_dev, 0, tx_buf, tx_cnt);
        if (tx_size != sizeof(tx_buf))
        {
            LOG_E("battery send failed.");
        }
//        rt_thread_mdelay(1000);
//        tx_size = rt_device_write(battery_uart_dev, 0, tx_buf, tx_cnt);
//        if (tx_size != sizeof(tx_buf))
//        {
//            LOG_E("battery send failed.");
//        }
        /** 等待接收 **/
        battery_mq_size = rt_mq_recv(&battery_recv_mq, &rx_msg, sizeof(rx_msg), 100);
        if (battery_mq_size == sizeof(rx_msg))
        {

            read_size = rt_device_read(rx_msg.dev, 0, rx_buf, rx_msg.size);
            if ((read_size == rx_msg.size))
            {

                if (rx_buf[0] == 0xDD && rx_buf[1] == 0x03)
                {

//                         check_sum += rx_buf[i];
                    // if ((check_sum & 0xff) == rx_buf[12])
                    // {
                    battery_error = (rx_buf[21] << 8) | rx_buf[20];
                    battery_get = rx_buf[23];
                    if(battery_error != 0)
                    {
                        level0 = rt_hw_interrupt_disable();
                        error_report  = error_report | (1<<3);;
                        rt_hw_interrupt_enable(level0);
                    }
//                     LOG_I("%d", battery_get);
                    level0 = rt_hw_interrupt_disable();
                    battery_percent  = battery_get;
                    rt_hw_interrupt_enable(level0);

                    // }
                }
            }
        }

    }
}
