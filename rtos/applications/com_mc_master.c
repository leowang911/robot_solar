/******************************************************************************/
/** Private Includes                                                         **/
/******************************************************************************/
#include <rtthread.h>
#include <rtdevice.h>
#define LOG_TAG "com.mc"
#define LOG_LVL LOG_LVL_DBG
#include <ulog.h>
#include "com_mc_master.h"
#include <stdlib.h>
#include "com_ros_slave.h"
/******************************************************************************/
/** Private Variables                                                        **/
/******************************************************************************/
rt_device_t mc_can_dev;

struct rt_semaphore mc_recv_sem;

rt_align(RT_ALIGN_SIZE) rt_uint8_t mc_ctrl_mq_stack[MQ_MC_CTRL_STACK_SIZE];
struct rt_messagequeue mc_ctrl_mq;

rt_align(RT_ALIGN_SIZE) rt_uint8_t mc_send_thread_stack[THREAD_MC_RECV_STACK_SIZE];
struct rt_thread mc_send_thread;

rt_align(RT_ALIGN_SIZE) rt_uint8_t mc_recv_thread_stack[THREAD_MC_SEND_STACK_SIZE];
struct rt_thread mc_recv_thread;

struct mc_master mc_master;

/******************************************************************************/
/** Private Function Prototypes                                              **/
/******************************************************************************/
rt_err_t mc_rx_callback(rt_device_t dev, rt_size_t size);
void start_mc_recv_thread(void *parameter);
void start_mc_send_thread(void *parameter);
rt_int32_t mc_set_speed(rt_uint8_t id, rt_int32_t speed);
rt_int32_t mc_close(rt_uint8_t id);

rt_int32_t motor_set_mode(uint8_t id, uint8_t master_id, uint8_t mode);
rt_int32_t motor_motor_enable(uint8_t id, uint8_t master_id);
rt_int32_t motor_motor_reset(uint8_t id, uint8_t master_id);
rt_int32_t motor_speed_set(uint8_t id, uint8_t master_id, float speed);
rt_int32_t motor_param_set(uint8_t id, uint8_t master_id, rt_uint16_t index, float param);
float rad_to_linear(float rad);

/******************************************************************************/
/** Private Function                                                         **/
/******************************************************************************/
/**
 * @brief  电机驱动模块初始化。
 * @param  无
 * @retval 返回初始化结果。
 */
int com_mc_master_init(void)
{
    rt_err_t err = RT_EOK;

    /** 查找CAN设备 **/
    mc_can_dev = rt_device_find(MC_CAN_NAME);
    RT_ASSERT(mc_can_dev != RT_NULL);

    /** 配置CAN设备参数 **/
    err = rt_device_open(mc_can_dev, (RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX));
    RT_ASSERT(err == RT_EOK);
    err = rt_device_control(mc_can_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);
    RT_ASSERT(err == RT_EOK);
    err = rt_device_control(mc_can_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);
    RT_ASSERT(err == RT_EOK);
    err = rt_device_set_rx_indicate(mc_can_dev, mc_rx_callback);
    RT_ASSERT(err == RT_EOK);

    /** 初始化电机控制接收信号量 **/
    err = rt_sem_init(&mc_recv_sem,      // 句柄
                      "mc_recv_sem",     // 名称
                      0,                 // 初始值
                      RT_IPC_FLAG_PRIO); // 等待方式
    if (err == RT_EOK)
    {
        LOG_I("mc init recv sem successful.");
    }
    else
    {
        LOG_E("mc init recv sem failed.");
        return -RT_ERROR;
    }

    /** 初始化电机控制消息队列 **/
    err = rt_mq_init(&mc_ctrl_mq,                // 句柄
                     "mc_ctrl_mq",               // 名称
                     &mc_ctrl_mq_stack[0],       // 缓冲区起始地址
                     sizeof(struct mc_ctrl_msg), // 消息长度
                     sizeof(mc_ctrl_mq_stack),   // 缓冲区大小
                     RT_IPC_FLAG_PRIO);          // 等待方式
    if (err == RT_EOK)
    {
        LOG_I("mc init ctrl mq successful.");
    }
    else
    {
        LOG_E("mc init ctrl mq failed.");
        return -RT_ERROR;
    }

    /** 初始化电机控制发送线程 **/
    err = rt_thread_init(&mc_send_thread,              // 句柄
                         "mc_send_thread",             // 名称
                         start_mc_send_thread,         // 入口函数
                         RT_NULL,                      // 入口函数的参数
                         &mc_send_thread_stack[0],     // 堆栈的起始地址
                         sizeof(mc_send_thread_stack), // 堆栈大小
                         THREAD_MC_SEND_PRIORITY,      // 优先级
                         THREAD_MC_SEND_TIMESLICE);    // 时间片
    if (err == RT_EOK)
    {
        rt_thread_startup(&mc_send_thread);
        LOG_I("mc init send thread successful.");
    }
    else
    {
        LOG_E("mc init send thread failed.");
        return -RT_ERROR;
    }

    /** 初始化电机控制接收线程 **/
    err = rt_thread_init(&mc_recv_thread,              // 句柄
                         "mc_recv_thread",             // 名称
                         start_mc_recv_thread,         // 入口函数
                         RT_NULL,                      // 入口函数的参数
                         &mc_recv_thread_stack[0],     // 堆栈的起始地址
                         sizeof(mc_recv_thread_stack), // 堆栈大小
                         THREAD_MC_RECV_PRIORITY,      // 优先级
                         THREAD_MC_RECV_TIMESLICE);    // 时间片
    if (err == RT_EOK)
    {
        rt_thread_startup(&mc_recv_thread);
        LOG_I("mc init recv thread successful.");
    }
    else
    {
        LOG_E("mc init recv thread failed.");
        return -RT_ERROR;
    }

    return RT_EOK;
}
#ifdef FINSH_USING_MSH
INIT_APP_EXPORT(com_mc_master_init);
#endif

/**
 * @brief  电机控制接收线程
 * @param  parameter 输入参数
 * @retval 无返回值
 */
void start_mc_recv_thread(void *parameter)
{
    rt_err_t err = RT_EOK;
    struct rt_can_msg msg = {0};
    rt_size_t read_size;
    rt_base_t level0;
    rt_int16_t speed ;
    float rad_s,dis_mm;
    rt_int32_t distance_multi[2] = {0};
    rt_int16_t encoder ;
    rt_int16_t  encoder_last[2] = {0};
    rt_uint8_t first = 0;
    // rt_uint8_t id;

    for (;;)
    {
        /** 等待接收信号量 **/
        err = rt_sem_take(&mc_recv_sem, RT_WAITING_FOREVER);
        if (err == RT_EOK)
        {
            read_size = rt_device_read(mc_can_dev, 0, &msg, sizeof(msg));
            if (read_size == sizeof(msg))
            {
                /** 电机数组编号 **/
                if ((msg.id & 0xFF) == MOTOR_MASTER_ID)
                {
                    if (((msg.id >> 8) & 0xFF) == 1)
                    {
                        speed = (msg.data[2] << 8 | msg.data[3]);
                        if(speed > 0x7fff)
                        {
                            speed = -(0xffff - speed);
                        }
                        else
                        {
                            speed = 0x7fff - speed;      
                        }
                        rad_s = rad_to_linear(speed * 20 / 32768.0f);
                        encoder = (rt_int16_t)(-(msg.data[0] << 8 | msg.data[1]));
                        // LOG_I("speed[0]= %d",speed);
                        if(first == 0)
                        {
                            encoder_last[0] = encoder;
                            first = 1;
                        }
                        distance_multi[0] = updateTotalMileage(distance_multi[0], encoder, encoder_last[0]);
                        encoder_last[0] = encoder;
                        dis_mm = rad_to_linear((distance_multi[0] /32768.0f) * 12.57f);

                        level0 = rt_hw_interrupt_disable();
                        mc_master.degree[0] = (rt_int32_t)dis_mm;
                        mc_master.speed[0] = (rt_int16_t)rad_s;
                        rt_hw_interrupt_enable(level0);
                    }
                    else if (((msg.id >> 8) & 0xFF) == 2)
                    {
                        speed = (msg.data[2] << 8 | msg.data[3]);
                        if(speed > 0x7fff)
                        {
                            speed = 0xffff - speed;
                        }
                        else
                        {
                            speed = -(0x7fff - speed);      
                        }
                        rad_s = rad_to_linear(speed * 20 / 32768.0f);

                        encoder = (rt_int16_t)((msg.data[0] << 8 | msg.data[1]));
                        if(first == 1)
                        {
                            encoder_last[1] = encoder;
                            first = 2;
                        }
                        distance_multi[1] = updateTotalMileage(distance_multi[1], encoder, encoder_last[1]);
                        encoder_last[1] = encoder;
                        dis_mm = rad_to_linear((distance_multi[1] /32768.0f) * 12.57f);

                        level0 = rt_hw_interrupt_disable();
                        mc_master.degree[1] = (rt_int32_t)dis_mm;
                        mc_master.speed[1] = (rt_int16_t)rad_s;
                        rt_hw_interrupt_enable(level0);
                    }
                    // LOG_I("%d    ,   %d",speed,mc_master.speed[1]);
                    //  LOG_I("speed[0]= %d ,speed[1] = %d",mc_master.speed[0],mc_master.speed[1]);
                }
                //                id = msg.id - 0x140 - 1;
                /** 解析接收数据 **/
                //                switch (msg.data[0]) {
                //                    case MC_CMD_SPEED_CLOSED_LOOP:
                //                        level0 = rt_hw_interrupt_disable();
                //                        mc_master.tempr[id]  = (rt_int8_t)(msg.data[1]);
                //                        mc_master.iq[id]     = (rt_int16_t)(msg.data[3] << 8 | msg.data[2]);
                //                        mc_master.speed[id]  = (rt_int16_t)(msg.data[5] << 8 | msg.data[4]);
                //                        mc_master.degree[id] = (rt_int16_t)(msg.data[7] << 8 | msg.data[6]);
                //                        rt_hw_interrupt_enable(level0);
                //                        break;
                //                    default:
                //                        break;
                //                }
            }
            else
            {
                LOG_W("mc read data failed.");
            }
        }
        else
        {
            LOG_W("mc take recv sem failed, error code = %d.", err);
        }
    }
}

/**
 * @brief  电机控制发送线程
 * @param  parameter 输入参数
 * @retval 无返回值
 */
void start_mc_send_thread(void *parameter)
{
    rt_ssize_t recv_mq_size;
    struct mc_ctrl_msg mc_msg;

    /** 延时等待 **/
    rt_thread_mdelay(3000);

    motor_set_mode(0x1, MOTOR_MASTER_ID, 2);  // 设置速度模式     2-从机  1-主机   2-速度模式
    motor_motor_enable(0x1, MOTOR_MASTER_ID); // 使能
    motor_param_set(0x1, MOTOR_MASTER_ID, 0X7018, 20);

    motor_set_mode(0x2, MOTOR_MASTER_ID, 2);  // 设置速度模式     2-从机  1-主机   2-速度模式
    motor_motor_enable(0x2, MOTOR_MASTER_ID); // 使能
    motor_param_set(0x2, MOTOR_MASTER_ID, 0X7018, 20);

    motor_set_mode(0x3, MOTOR_MASTER_ID, 2); // 设置速度模式     2-从机  1-主机   2-速度模式
    motor_param_set(0x3, MOTOR_MASTER_ID, 0X7022, 15);
    motor_motor_enable(0x3, MOTOR_MASTER_ID); // 使能
    motor_param_set(0x3, MOTOR_MASTER_ID, 0X7018, 20);

    //    mc_set_speed(3, -5000 );

    for (;;)
    {
        /** 清除缓存 **/
        rt_memset(&mc_msg, 0, sizeof(mc_msg));

        /** 等待接收控制指令 **/
        recv_mq_size = rt_mq_recv(&mc_ctrl_mq, &mc_msg, sizeof(mc_msg), RT_WAITING_FOREVER);
        if (recv_mq_size == sizeof(mc_msg))
        {
            motor_speed_set(1, MOTOR_MASTER_ID, -mc_msg.left_speed);
            rt_thread_mdelay(1);
            motor_speed_set(2, MOTOR_MASTER_ID, mc_msg.right_speed);
            rt_thread_mdelay(1);
            //            mc_set_speed(3, -mc_msg.front_speed);
            motor_speed_set(3, MOTOR_MASTER_ID, -mc_msg.front_speed);
        }
        else
        {
            LOG_E("mc receive ctrl msg failed.");
        }
    }
}

/**
 * @brief  电机控制接收回调函数
 * @param  dev  设备地址
 * @param  size 接收字节数大小
 * @retval 无返回值
 */
rt_err_t mc_rx_callback(rt_device_t dev, rt_size_t size)
{
    rt_err_t err;

    err = rt_sem_release(&mc_recv_sem);

    return err;
}

/**
 * @brief  设置电机速度
 * @param  id    电机编号
 * @param  speed 速度值
 * @retval 返回设置结果
 */
rt_int32_t motor_set_mode(uint8_t id, uint8_t master_id, uint8_t mode)
{
    struct rt_can_msg msg = {0};
    rt_ssize_t size = 0;
    rt_uint16_t index = 0x7005;

    msg.id = (uint32_t)((18 << 24) | (master_id << 8) | id);
    msg.ide = RT_CAN_EXTID;
    msg.rtr = RT_CAN_DTR;
    msg.len = 8;

    for (uint8_t i = 0; i < 8; i++)
    {
        msg.data[i] = 0;
    }

    memcpy(&msg.data[0], &index, 2);
    memcpy(&msg.data[4], &mode, 1);

    size = rt_device_write(mc_can_dev, 0, &msg, sizeof(msg));
    if (size != sizeof(msg))
    {
        LOG_E("mc set speed failed_1.");
        return -RT_ERROR;
    }

    return RT_EOK;
}

/**
 * @brief  设置电机速度
 * @param  id    电机编号
 * @param  speed 速度值
 * @retval 返回设置结果
 */
rt_int32_t motor_motor_enable(uint8_t id, uint8_t master_id)
{
    struct rt_can_msg msg = {0};
    rt_ssize_t size = 0;

    msg.id = ((uint32_t)(3 << 24) | (master_id << 8) | id);
    msg.ide = RT_CAN_EXTID;
    msg.rtr = RT_CAN_DTR;
    msg.len = 8;

    for (uint8_t i = 0; i < 8; i++)
    {
        msg.data[i] = 0;
    }

    size = rt_device_write(mc_can_dev, 0, &msg, sizeof(msg));
    if (size != sizeof(msg))
    {
        LOG_E("mc set speed failed_2.");
        return -RT_ERROR;
    }

    return RT_EOK;
}

rt_int32_t motor_motor_reset(uint8_t id, uint8_t master_id)
{
    struct rt_can_msg msg = {0};
    rt_ssize_t size = 0;

    msg.id = ((uint32_t)(4 << 24) | (master_id << 8) | id);
    msg.ide = RT_CAN_EXTID;
    msg.rtr = RT_CAN_DTR;
    msg.len = 8;

    for (uint8_t i = 0; i < 8; i++)
    {
        msg.data[i] = 0;
    }

    size = rt_device_write(mc_can_dev, 0, &msg, sizeof(msg));
    if (size != sizeof(msg))
    {
        LOG_E("mc set speed failed_3.");
        return -RT_ERROR;
    }

    return RT_EOK;
}

/**
 * @brief  设置电机速度
 * @param  id    电机编号
 * @param  speed 速度值
 * @retval 返回设置结果
 */
rt_int32_t motor_speed_set(uint8_t id, uint8_t master_id, float speed)
{
    struct rt_can_msg msg = {0};
    rt_ssize_t size = 0;
    rt_uint16_t index = 0x700A;

    msg.id = (uint32_t)((18 << 24) | (master_id << 8) | id);
    msg.ide = RT_CAN_EXTID;
    msg.rtr = RT_CAN_DTR;
    msg.len = 8;

    for (uint8_t i = 0; i < 8; i++)
    {
        msg.data[i] = 0;
    }

    memcpy(&msg.data[0], &index, 2);
    memcpy(&msg.data[4], &speed, 4); // +-  rad/s  -20~20rad/s

    size = rt_device_write(mc_can_dev, 0, &msg, sizeof(msg));
    if (size != sizeof(msg))
    {
        LOG_E("mc set speed failed_4.%d", id);
        return -RT_ERROR;
    }

    return RT_EOK;
}

rt_int32_t motor_param_set(uint8_t id, uint8_t master_id, rt_uint16_t index, float param)
{
    struct rt_can_msg msg = {0};
    rt_ssize_t size = 0;

    msg.id = (uint32_t)((18 << 24) | (master_id << 8) | id);
    msg.ide = RT_CAN_EXTID;
    msg.rtr = RT_CAN_DTR;
    msg.len = 8;

    for (uint8_t i = 0; i < 8; i++)
    {
        msg.data[i] = 0;
    }

    memcpy(&msg.data[0], &index, 2);
    memcpy(&msg.data[4], &param, 4);

    size = rt_device_write(mc_can_dev, 0, &msg, sizeof(msg));
    if (size != sizeof(msg))
    {
        LOG_E("mc set speed failed_5.");
        return -RT_ERROR;
    }

    return RT_EOK;
}

/**
 * @brief  设置电机速度
 * @param  id    电机编号
 * @param  speed 速度值
 * @retval 返回设置结果
 */
rt_int32_t mc_set_speed(rt_uint8_t id, rt_int32_t speed)
{
    struct rt_can_msg msg = {0};
    rt_ssize_t size = 0;

    msg.id = 0x140 + id;
    msg.ide = RT_CAN_STDID;
    msg.rtr = RT_CAN_DTR;
    msg.len = 8;

    msg.data[0] = MC_CMD_SPEED_CLOSED_LOOP;
    msg.data[1] = 0;
    msg.data[2] = 0;
    msg.data[3] = 0;
    msg.data[4] = (rt_uint8_t)(speed >> 0);
    msg.data[5] = (rt_uint8_t)(speed >> 8);
    msg.data[6] = (rt_uint8_t)(speed >> 16);
    msg.data[7] = (rt_uint8_t)(speed >> 24);

    size = rt_device_write(mc_can_dev, 0, &msg, sizeof(msg));
    if (size != sizeof(msg))
    {
        LOG_E("mc set speed failed_6.");
        return -RT_ERROR;
    }

    return RT_EOK;
}

/**
 * @brief  关闭电机
 * @param  id 电机编号
 * @retval 返回关闭结果
 */
rt_int32_t mc_close(rt_uint8_t id)
{
    struct rt_can_msg msg = {0};
    rt_ssize_t size = 0;

    msg.id = 0x140 + id;
    msg.ide = RT_CAN_STDID;
    msg.rtr = RT_CAN_DTR;
    msg.len = 8;

    msg.data[0] = MC_CMD_CLOSE;
    msg.data[1] = 0;
    msg.data[2] = 0;
    msg.data[3] = 0;
    msg.data[4] = 0;
    msg.data[5] = 0;
    msg.data[6] = 0;
    msg.data[7] = 0;

    size = rt_device_write(mc_can_dev, 0, &msg, sizeof(msg));
    if (size != sizeof(msg))
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

float rad_to_linear(float rad)
{
    float linear;
    linear = rad * WHEEL_RADIUS;
   
    return linear;
}

/******************************************************************************/
/** (C) COPYRIGHT 2024 WUJIAN                                    END OF FILE **/
/******************************************************************************/
