/******************************************************************************/
/** Private Includes                                                         **/
/******************************************************************************/
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#define LOG_TAG "com.rc"
#define LOG_LVL LOG_LVL_DBG
#include <ulog.h>
#include "com_rc_slave.h"
#include "com_mc_master.h"
#include "com_io_ctrl.h"
#include "com_dev_param.h"

/******************************************************************************/
/** Private Variables                                                        **/
/******************************************************************************/
rt_device_t rc_uart_dev;

rt_align(RT_ALIGN_SIZE) rt_uint8_t rc_recv_mq_stack[MQ_RC_RECV_STACK_SIZE];
struct rt_messagequeue rc_recv_mq;

rt_align(RT_ALIGN_SIZE) rt_uint8_t rc_recv_thread_stack[THREAD_RC_RECV_STACK_SIZE];
struct rt_thread rc_recv_thread;

struct rc_slave rc_slave = {0};

/******************************************************************************/
/** Private Function Prototypes                                              **/
/******************************************************************************/
rt_err_t rc_rx_callback(rt_device_t dev, rt_size_t size);
void start_rc_recv_thread(void *parameter);

/******************************************************************************/
/** Private Function                                                         **/
/******************************************************************************/
/**
 * @brief  遥控模块初始化。
 * @param  无
 * @retval 返回初始化结果。
 */
int com_rc_slave_init(void)
{
    rt_err_t err = RT_EOK;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    /** 查找UART设备 **/
    rc_uart_dev = rt_device_find(RC_UART_NAME);
    RT_ASSERT(rc_uart_dev != RT_NULL);

    /** 配置UART设备参数 **/
    config.baud_rate = 100000;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_2;
    config.parity = PARITY_EVEN;
    config.rx_bufsz = 64;
    err = rt_device_control(rc_uart_dev, RT_DEVICE_CTRL_CONFIG, &config);
    RT_ASSERT(err == RT_EOK);
    err = rt_device_open(rc_uart_dev, RT_DEVICE_FLAG_RX_NON_BLOCKING);
    RT_ASSERT(err == RT_EOK);
    err = rt_device_set_rx_indicate(rc_uart_dev, rc_rx_callback);
    RT_ASSERT(err == RT_EOK);

    /** 初始化遥控接收消息队列 **/
    err = rt_mq_init(&rc_recv_mq,                // 句柄
                     "rc_recv_mq",               // 名称
                     &rc_recv_mq_stack[0],       // 缓冲区起始地址
                     sizeof(struct rc_recv_msg), // 消息长度
                     sizeof(rc_recv_mq_stack),   // 缓冲区大小
                     RT_IPC_FLAG_PRIO);          // 等待方式
    if (err == RT_EOK)
    {
        LOG_I("rc init recv mq successful.");
    }
    else
    {
        LOG_E("rc init recv mq failed.");
        return -RT_ERROR;
    }

    /** 初始化遥控接收线程 **/
    err = rt_thread_init(&rc_recv_thread,              // 句柄
                         "rc_recv_thread",             // 名称
                         start_rc_recv_thread,         // 入口函数
                         RT_NULL,                      // 入口函数的参数
                         &rc_recv_thread_stack[0],     // 堆栈的起始地址
                         sizeof(rc_recv_thread_stack), // 堆栈大小
                         THREAD_RC_RECV_PRIORITY,      // 优先级
                         THREAD_RC_RECV_TIMESLICE);    // 时间片
    if (err == RT_EOK)
    {
        rt_thread_startup(&rc_recv_thread);
        LOG_I("rc init recv thread successful.");
    }
    else
    {
        LOG_E("rc init recv thread failed.");
        return -RT_ERROR;
    }

    return RT_EOK;
}
#ifdef FINSH_USING_MSH
INIT_APP_EXPORT(com_rc_slave_init);
#endif

/**
 * @brief  无线通讯接收任务。
 * @param  parameter 输入参数
 * @retval 无返回值。
 */
void start_rc_recv_thread(void *parameter)
{
    rt_base_t level0;
    rt_err_t err = RT_EOK;
    rt_ssize_t rc_mq_size = 0;
    struct rc_recv_msg rx_msg;
    struct mc_ctrl_msg ctrl_msg;
    rt_size_t read_size = 0;
    rt_uint8_t rx_buf[25] = {0};
    rt_uint8_t sensor;

    for (;;)
    {
        //        LOG_D("degree1:%d,degree2:%d\n", mc_master.degree[0],mc_master.degree[0]);
        /** 清除缓存 **/
        rt_memset(&rx_msg, 0, sizeof(rx_msg));

        /** 等待接收控制指令 **/
        rc_mq_size = rt_mq_recv(&rc_recv_mq, &rx_msg, sizeof(rx_msg), RT_WAITING_FOREVER);
        if (rc_mq_size == sizeof(rx_msg))
        {
            read_size = rt_device_read(rx_msg.dev, 0, rx_buf, rx_msg.size);
            if ((read_size == rx_msg.size) && ((read_size == 25)))
            {
                if ((rx_buf[0] == 0x0F) && (rx_buf[24] == 0x00))
                {
                    level0 = rt_hw_interrupt_disable();
                    sensor = io_sensor_msg;
                    rt_hw_interrupt_enable(level0);

                    rc_slave.ch[0] = ((rx_buf[2] << 8) + (rx_buf[1])) & 0x07FF;                              // 右摇杆左右
                    rc_slave.ch[1] = ((rx_buf[3] << 5) + (rx_buf[2] >> 3)) & 0x07FF;                         // 左摇杆前后
                    rc_slave.ch[2] = ((rx_buf[5] << 10) + (rx_buf[4] << 2) + (rx_buf[3] >> 6)) & 0x07FF;     // 右摇杆前后
                    rc_slave.ch[3] = ((rx_buf[6] << 7) + (rx_buf[5] >> 1)) & 0x07FF;                         // 左摇杆左右
                    rc_slave.ch[4] = ((rx_buf[7] << 4) + (rx_buf[6] >> 4)) & 0x07FF;                         // 左三档开关
                    rc_slave.ch[5] = ((rx_buf[9] << 9) + (rx_buf[8] << 1) + (rx_buf[7] >> 7)) & 0x07FF;      // 右三档开关
                    rc_slave.ch[6] = ((rx_buf[10] << 6) + (rx_buf[9] >> 2)) & 0x07FF;                        // 按键A
                    rc_slave.ch[7] = ((rx_buf[11] << 3) + (rx_buf[10] >> 5)) & 0x07FF;                       // 按键B
                    rc_slave.ch[8] = ((rx_buf[13] << 8) + (rx_buf[12])) & 0x07FF;                            // 按键C
                    rc_slave.ch[9] = ((rx_buf[14] << 5) + (rx_buf[13] >> 3)) & 0x07FF;                       // 按键D
                    rc_slave.ch[10] = ((rx_buf[16] << 10) + (rx_buf[15] << 2) + (rx_buf[14] >> 6)) & 0x07FF; // 左滑轮
                    rc_slave.ch[11] = ((rx_buf[17] << 7) + (rx_buf[16] >> 1)) & 0x07FF;                      // 右滑轮
                    rc_slave.ch[12] = ((rx_buf[18] << 4) + (rx_buf[17] >> 4)) & 0x07FF;
                    rc_slave.ch[13] = ((rx_buf[20] << 9) + (rx_buf[19] << 1) + (rx_buf[18] >> 7)) & 0x07FF;
                    rc_slave.ch[14] = ((rx_buf[21] << 6) + (rx_buf[20] >> 2)) & 0x07FF;
                    rc_slave.ch[15] = ((rx_buf[22] << 3) + (rx_buf[21] >> 5)) & 0x07FF;

                    //                     LOG_D("\r\n======================================");
                    //                     LOG_D("  %d, %d, %d, %d\r\n", rc_slave.ch[0],  rc_slave.ch[1],  rc_slave.ch[2],  rc_slave.ch[3]);
                    //                     LOG_D("  %d, %x, %d, %d\r\n", rc_slave.ch[4],  rc_slave.ch[5],  rc_slave.ch[6],  rc_slave.ch[7]);
                    //                     LOG_D("  %d, %d, %d, %d\r\n", rc_slave.ch[8],  rc_slave.ch[9],  rc_slave.ch[10], rc_slave.ch[11]);
                    //                     LOG_D("  %d, %d, %d, %d\r\n", rc_slave.ch[12], rc_slave.ch[13], rc_slave.ch[14], rc_slave.ch[15]);
                    //                     LOG_D("======================================\r\n");

                    /** 自动运行 **/
//                    if (rc_slave.ch[8] == RC_CH_MAX_VALUE)
//                    {
//                        if (dev_param.run.dev_auto != DEV_AUTO_FIND)
//                        {
//                            dev_param.run.dev_auto = DEV_AUTO_FIND;
//                            if (dev_param.run.auto_step == READY)
//                            {
//                                dev_param.run.auto_step = FD_FORWARD_EDGE;
//                            }
//                        }
//
//                    }
//                    else
//                    {
                        switch (rc_slave.ch[5])
                        {
                        case RC_CH_MIN_VALUE: // 关闭自动
                            dev_param.run.dev_auto = DEV_AUTO_STOP;
                            dev_param.run.end_flag = 0;
                            dev_param.run.auto_cleaned_line_num = 0;
                            dev_param.run.auto_step = MANUAL;

                            break;
                        case RC_CH_MID_VALUE: // 自动一次
                            if (dev_param.run.dev_auto != DEV_AUTO_ONCE)
                            {
                                dev_param.run.dev_auto = DEV_AUTO_ONCE;
                                if (dev_param.run.auto_step == MANUAL)
                                {
                                    dev_param.run.auto_step = IDLE;
                                    dev_param.run.auto_clean_step = START_CALIBRATION;
                                    dev_param.run.auto_coner_step = FD_FORWARD_EDGE;
                                }
                            }

                            break;
                        case RC_CH_MAX_VALUE: // 自动循环
                            if (dev_param.run.dev_auto != DEV_AUTO_CYCLE)
                            {
                                dev_param.run.dev_auto = DEV_AUTO_CYCLE;
                                if (dev_param.run.auto_step == MANUAL)
                                {
                                    dev_param.run.auto_step = IDLE;
                                    dev_param.run.auto_clean_step = START_CALIBRATION;
                                    dev_param.run.auto_coner_step = FD_FORWARD_EDGE;
                                }
                            }
                            break;
                        default:
                            break;
                        }
//                    }

                    /** 运行模式 **/

                    switch (rc_slave.ch[4])
                    {
                    case RC_CH_MIN_VALUE: // 停机模式
                        dev_param.run.dev_mode = DEV_MODE_STOP;
                        break;
                    case RC_CH_MID_VALUE: // 正常模式
                        dev_param.run.dev_mode = DEV_MODE_NORMAL;
                        break;
                    case RC_CH_MAX_VALUE: // 探边模式
                        dev_param.run.dev_mode = DEV_MODE_AVOID;
                        break;
                    default:
                        break;
                    }

                    /** 毛刷控制 **/
                    switch (rc_slave.ch[6])
                    {
                    case RC_CH_MIN_VALUE: // 关闭毛刷
                        ctrl_msg.front_speed = 0;
                        dev_param.run.brush_enable = 0;
                        break;
                    case RC_CH_MAX_VALUE: // 开启毛刷
                        ctrl_msg.front_speed = MC_BRUSH_SPEED_MAX;
                        dev_param.run.brush_enable = 1;
                        break;
                    default:
                        break;
                    }

                    /** 停机舱对齐控制 **/
                    switch (rc_slave.ch[7])
                    {
                    case RC_CH_MIN_VALUE: // 关闭对齐
                        if (rc_slave.align_last == 1)
                        {
                            rc_slave.align_last = 0;
                            dev_param.run.align_enable = 0;
                        }
                        break;
                    case RC_CH_MAX_VALUE: // 对齐
                        if (rc_slave.align_last == 0)
                        {
                            rc_slave.align_last = 1;
                            dev_param.run.align_enable = 1;
                        }
                        break;
                    default:
                        break;
                    }

                    /** 关闭自动且关闭对齐 或 停机模式 **/
                    if (((dev_param.run.dev_auto == DEV_AUTO_STOP) && (dev_param.run.align_enable == 0)) ||
                        (dev_param.run.dev_mode == DEV_MODE_STOP))
                    {
                        /** 前进后退 **/
                        if (rc_slave.ch[2] == RC_CH_MID_VALUE)
                        {
                            ctrl_msg.left_speed = 0;
                            ctrl_msg.right_speed = 0;
                        }
                        else if (rc_slave.ch[2] < RC_CH_MID_VALUE)
                        {
                            if ((dev_param.run.dev_mode == DEV_MODE_AVOID) &&
                                (((sensor & 0x10) == 0x10) || ((sensor & 0x20) == 0x20)))
                            {
                                ctrl_msg.left_speed = 0;
                                ctrl_msg.right_speed = 0;
                            }
                            else
                            {
                                ctrl_msg.left_speed = -MC_RUN_SPEED_MAX / 2 * (RC_CH_MID_VALUE - rc_slave.ch[2]) / RC_CH_HALF_RANGE;
                                ctrl_msg.right_speed = -MC_RUN_SPEED_MAX / 2 * (RC_CH_MID_VALUE - rc_slave.ch[2]) / RC_CH_HALF_RANGE;
                            }
                        }
                        else if (rc_slave.ch[2] > RC_CH_MID_VALUE)
                        {
                            if ((dev_param.run.dev_mode == DEV_MODE_AVOID) &&
                                (((sensor & 0x05) == 0x05) || ((sensor & 0x0A) == 0x0A)))
                            {
                                ctrl_msg.left_speed = 0;
                                ctrl_msg.right_speed = 0;
                            }
                            else if ((dev_param.run.dev_mode == DEV_MODE_AVOID) && ((sensor & 0x03) == 0x03))
                            {
                                ctrl_msg.left_speed = MC_RUN_SPEED_MAX / 4 * (rc_slave.ch[2] - RC_CH_MID_VALUE) / RC_CH_HALF_RANGE;
                                ctrl_msg.right_speed = MC_RUN_SPEED_MAX / 4 * (rc_slave.ch[2] - RC_CH_MID_VALUE) / RC_CH_HALF_RANGE;
                            }
                            else
                            {
                                ctrl_msg.left_speed = MC_RUN_SPEED_MAX * (rc_slave.ch[2] - RC_CH_MID_VALUE) / RC_CH_HALF_RANGE;
                                ctrl_msg.right_speed = MC_RUN_SPEED_MAX * (rc_slave.ch[2] - RC_CH_MID_VALUE) / RC_CH_HALF_RANGE;
                            }
                        }

                        /** 左转右转 **/
                        if (rc_slave.ch[0] < RC_CH_MID_VALUE)
                        {
                            if (rc_slave.ch[2] != RC_CH_MID_VALUE)
                            {
                                ctrl_msg.left_speed = ctrl_msg.left_speed - ctrl_msg.left_speed * (RC_CH_MID_VALUE - rc_slave.ch[0]) / RC_CH_HALF_RANGE / 2;
                                ctrl_msg.right_speed = ctrl_msg.right_speed + ctrl_msg.right_speed * (RC_CH_MID_VALUE - rc_slave.ch[0]) / RC_CH_HALF_RANGE / 2;
                            }
                            else
                            {
                                ctrl_msg.left_speed = -MC_RUN_SPEED_MAX / 3 * (RC_CH_MID_VALUE - rc_slave.ch[0]) / RC_CH_HALF_RANGE;
                                ctrl_msg.right_speed = MC_RUN_SPEED_MAX / 3 * (RC_CH_MID_VALUE - rc_slave.ch[0]) / RC_CH_HALF_RANGE;
                            }
                        }
                        else if (rc_slave.ch[0] > RC_CH_MID_VALUE)
                        {
                            if (rc_slave.ch[2] != RC_CH_MID_VALUE)
                            {
                                ctrl_msg.left_speed = ctrl_msg.left_speed + ctrl_msg.left_speed * (rc_slave.ch[0] - RC_CH_MID_VALUE) / RC_CH_HALF_RANGE / 2;
                                ctrl_msg.right_speed = ctrl_msg.right_speed - ctrl_msg.right_speed * (rc_slave.ch[0] - RC_CH_MID_VALUE) / RC_CH_HALF_RANGE / 2;
                            }
                            else
                            {
                                ctrl_msg.left_speed = MC_RUN_SPEED_MAX / 3 * (rc_slave.ch[0] - RC_CH_MID_VALUE) / RC_CH_HALF_RANGE;
                                ctrl_msg.right_speed = -MC_RUN_SPEED_MAX / 3 * (rc_slave.ch[0] - RC_CH_MID_VALUE) / RC_CH_HALF_RANGE;
                            }
                        }

                        /** 最大速度限制 **/
                        if (ctrl_msg.left_speed > MC_RUN_SPEED_MAX)
                        {
                            ctrl_msg.left_speed = MC_RUN_SPEED_MAX;
                        }
                        else if (ctrl_msg.left_speed < -MC_RUN_SPEED_MAX)
                        {
                            ctrl_msg.left_speed = -MC_RUN_SPEED_MAX;
                        }
                        if (ctrl_msg.right_speed > MC_RUN_SPEED_MAX)
                        {
                            ctrl_msg.right_speed = MC_RUN_SPEED_MAX;
                        }
                        else if (ctrl_msg.right_speed < -MC_RUN_SPEED_MAX)
                        {
                            ctrl_msg.right_speed = -MC_RUN_SPEED_MAX;
                        }

                        /** 停机模式下速度清除 **/
                        if (dev_param.run.dev_mode == DEV_MODE_STOP)
                        {
                            ctrl_msg.left_speed = 0;
                            ctrl_msg.right_speed = 0;
                            dev_param.run.align_enable = 0;
                        }

                        if ((dev_param.run.dev_mode == DEV_MODE_AVOID) &&
                            (((sensor & 0x15) == 0x15) || ((sensor & 0x2A) == 0x2A)))
                        {
                            ctrl_msg.left_speed = 0;
                            ctrl_msg.right_speed = 0;
                        }

                        /** 发送运动控制消息队列 **/
                        err = rt_mq_send(&mc_ctrl_mq, &ctrl_msg, sizeof(ctrl_msg));
                        if (err != RT_EOK)
                        {
                            LOG_E("mc send ctrl mq failed.");
                        }
                    }
                }
                else
                {
                    LOG_E("rc read rx data failed.");
                }
            }
        }
        else
        {
            LOG_W("rc receive ctrl mq failed, error code = %d.", err);
        }
    }
}

/**
 * @brief  接收完成回调函数。
 * @param  dev  设备地址
 * @param  size 字节长度
 * @retval 返回执行结果。
 */
rt_err_t rc_rx_callback(rt_device_t dev, rt_size_t size)
{
    rt_err_t err = RT_EOK;
    struct rc_recv_msg rx_msg = {0};

    rx_msg.dev = dev;
    rx_msg.size = size;

    err = rt_mq_send(&rc_recv_mq, &rx_msg, sizeof(rx_msg));

    return err;
}

/******************************************************************************/
/** (C) COPYRIGHT 2024 WUJIAN                                    END OF FILE **/
/******************************************************************************/
