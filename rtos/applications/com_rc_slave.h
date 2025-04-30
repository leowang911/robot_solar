#ifndef __COM_RC_SLAVE_H__
#define __COM_RC_SLAVE_H__

/******************************************************************************/
/** Includes                                                                 **/
/******************************************************************************/
#include <rtdef.h>

/******************************************************************************/
/** Exported Define                                                          **/
/******************************************************************************/
#define  RC_UART_NAME             "uart3"

#define  THREAD_RC_RECV_STACK_SIZE  4096  // 接收线程的堆栈大小
#define  THREAD_RC_RECV_PRIORITY       3  // 接收线程的优先级
#define  THREAD_RC_RECV_TIMESLICE      5  // 接收线程的时间片

#define  MQ_RC_RECV_STACK_SIZE       128  // 接收消息队列的堆栈大小

#define  RC_CH_MIN_VALUE             282  // CH最小值
#define  RC_CH_MID_VALUE            1002  // CH中间值
#define  RC_CH_MAX_VALUE            1722  // CH最大值
#define  RC_CH_HALF_RANGE            720  // RC_CH_MID_VALUE - RC_CH_MIN_VALUE = RC_CH_MAX_VALUE - RC_CH_MID_VALUE

/******************************************************************************/
/** Exported Types                                                           **/
/******************************************************************************/
struct rc_recv_msg {
    rt_device_t dev;
    rt_size_t   size;
};

struct rc_slave {
    rt_uint16_t ch[16];
    rt_uint8_t align_last;
};

#endif /* __COM_RC_SLAVE_H__ */

/******************************************************************************/
/** (C) COPYRIGHT 2024 WUJIAN                                    END OF FILE **/
/******************************************************************************/
