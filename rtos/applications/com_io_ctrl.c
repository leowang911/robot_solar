/******************************************************************************/
/** Private Includes                                                         **/
/******************************************************************************/
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#define LOG_TAG "com.io"
#define LOG_LVL LOG_LVL_DBG
#include <ulog.h>
#include "com_io_ctrl.h"

/******************************************************************************/
/** Private Variables                                                        **/
/******************************************************************************/
rt_align(RT_ALIGN_SIZE) rt_uint8_t out_ctrl_mq_stack[MQ_OUT_CTRL_STACK_SIZE];
struct rt_messagequeue out_ctrl_mq;

rt_align(RT_ALIGN_SIZE) rt_uint8_t output_thread_stack[THREAD_OUTPUT_STACK_SIZE];
struct rt_thread output_thread;

rt_align(RT_ALIGN_SIZE) rt_uint8_t input_thread_stack[THREAD_INPUT_STACK_SIZE];
struct rt_thread input_thread;

rt_uint8_t io_sensor_msg;

/******************************************************************************/
/** Private Function Prototypes                                              **/
/******************************************************************************/
static void start_input_thread(void *parameter);
static void start_output_thread(void *parameter);

/******************************************************************************/
/** Private Function                                                         **/
/******************************************************************************/
/**
 * @brief  输入输出模块初始化。
 * @param  无
 * @retval 返回初始化结果。
 */
int com_io_ctrl_init(void) {
    rt_err_t err = RT_EOK;

    /** 初始化输出控制消息队列 **/
    err = rt_mq_init(&out_ctrl_mq,              // 句柄
                     "out_ctrl_mq",             // 名称
                     &out_ctrl_mq_stack[0],     // 缓冲区起始地址
                     sizeof(rt_uint8_t),        // 消息长度
                     sizeof(out_ctrl_mq_stack), // 缓冲区大小
                     RT_IPC_FLAG_PRIO);         // 等待方式
    if (err == RT_EOK) {
        LOG_I("io init ctrl mq successful.");
    } else {
        LOG_E("io init ctrl mq failed.");
        return -RT_ERROR;
    }

    /** 初始化输出线程 **/
    err = rt_thread_init(&output_thread,              // 句柄
                         "output_thread",             // 名称
                         start_output_thread,         // 入口函数
                         RT_NULL,                     // 入口函数的参数
                         &output_thread_stack[0],     // 堆栈的起始地址
                         sizeof(output_thread_stack), // 堆栈大小
                         THREAD_OUTPUT_PRIORITY,      // 优先级
                         THREAD_OUTPUT_TIMESLICE);    // 时间片
    if (err == RT_EOK) {
        rt_thread_startup(&output_thread);
        LOG_I("io init output thread successful.");
    } else {
        LOG_E("io init output thread failed.");
        return -RT_ERROR;
    }

    /** 初始化输入线程 **/
    err = rt_thread_init(&input_thread,              // 句柄
                         "input_thread",             // 名称
                         start_input_thread,         // 入口函数
                         RT_NULL,                    // 入口函数的参数
                         &input_thread_stack[0],     // 堆栈的起始地址
                         sizeof(input_thread_stack), // 堆栈大小
                         THREAD_INPUT_PRIORITY,      // 优先级
                         THREAD_INPUT_TIMESLICE);    // 时间片
    if (err == RT_EOK) {
        rt_thread_startup(&input_thread);
        LOG_I("io init input thread successful.");
    } else {
        LOG_E("io init input thread failed.");
        return -RT_ERROR;
    }

    return RT_EOK;
}
#ifdef FINSH_USING_MSH
INIT_APP_EXPORT(com_io_ctrl_init);
#endif

/**
 * @brief  输出控制线程。
 * @param  parameter 输入参数
 * @retval 无返回值。
 */
static void start_output_thread(void *parameter) {
    rt_ssize_t out_mq_size;
    rt_uint8_t out_msg;

    /** 初始化输出引脚 **/
    rt_pin_mode(OUT1, PIN_MODE_OUTPUT);
    rt_pin_write(OUT1, PIN_HIGH);

    for (;;) {
        /** 清除缓存 **/
        rt_memset(&out_msg, 0, sizeof(out_msg));

        /** 等待接收控制指令 **/
        out_mq_size = rt_mq_recv(&out_ctrl_mq, &out_msg, sizeof(out_msg), RT_WAITING_FOREVER);
        if (out_mq_size == sizeof(out_msg)) {
            switch (out_msg) {
                case IO_CMD_MOTOR_BRAKE_ON:
                    rt_pin_write(OUT1, PIN_HIGH);
                    break;
                case IO_CMD_MOTOR_BRAKE_OFF:
                    rt_pin_write(OUT1, PIN_LOW);
                    break;
                default:
                    break;
            }
        } else {
            LOG_E("io receive out ctrl mq failed.");
        }
    }
}

/**
 * @brief  输入检测线程
 * @param  parameter 输入参数
 * @retval 无返回值
 */
static void start_input_thread(void *parameter) {
    rt_base_t  level0;
    rt_uint8_t sensor_msg = 0;
    rt_int8_t  status = 0;

    /** 初始化输入引脚 **/
    rt_pin_mode(IN1, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(IN2, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(IN3, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(IN4, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(IN5, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(IN6, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(IN7, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(IN8, PIN_MODE_INPUT_PULLUP);

    for (;;) {
        /** 间隔时间 **/
        rt_thread_mdelay(40);

        /** 读输入引脚**/
        status = rt_pin_read(IN1);
        if (status == PIN_LOW) {
            sensor_msg &= ~0x01;
        } else {
            sensor_msg |= 0x01;
        }

        status = rt_pin_read(IN2);
        if (status == PIN_LOW) {
            sensor_msg &= ~0x02;
        } else {
            sensor_msg |= 0x02;
        }

        status = rt_pin_read(IN3);
        if (status == PIN_LOW) {
            sensor_msg &= ~0x04;
        } else {
            sensor_msg |= 0x04;
        }

        status = rt_pin_read(IN4);
        if (status == PIN_LOW) {
            sensor_msg &= ~0x08;
        } else {
            sensor_msg |= 0x08;
        }

        status = rt_pin_read(IN5);
        if (status == PIN_LOW) {
            sensor_msg &= ~0x10;
        } else {
            sensor_msg |= 0x10;
        }

        status = rt_pin_read(IN6);
        if (status == PIN_LOW) {
            sensor_msg &= ~0x20;
        } else {
            sensor_msg |= 0x20;
        }

        status = rt_pin_read(IN7);
        if (status == PIN_LOW) {
            sensor_msg &= ~0x40;
        } else {
            sensor_msg |= 0x40;
        }

        status = rt_pin_read(IN8);
        if (status == PIN_LOW) {
            sensor_msg &= ~0x80;
        } else {
            sensor_msg |= 0x80;
        }

        /** 更新传感器状态 **/
        level0 = rt_hw_interrupt_disable();
        io_sensor_msg = sensor_msg;
        rt_hw_interrupt_enable(level0);
    }
}

/******************************************************************************/
/** (C) COPYRIGHT 2024 WUJIAN                                    END OF FILE **/
/******************************************************************************/
