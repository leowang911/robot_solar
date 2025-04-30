#ifndef __COM_DEV_PARAM_H__
#define __COM_DEV_PARAM_H__

/******************************************************************************/
/** Includes                                                                 **/
/******************************************************************************/
#include <rtdef.h>

/******************************************************************************/
/** Exported Types                                                           **/
/******************************************************************************/
enum auto_run_step
{
    MANUAL = 0, // 准备

    IDLE,          // 空闲
    AUTORUNING,    // 自动运行
    UNLOADING,     // 出仓
    LOADING,       // 入仓
    ENERGENCUSTOP, // 急停
    INIT,          // 重置
    AUTOCORNER,    // 自动寻角
    AUTOWASHING,   //  自动清洗
    

   
};


enum auto_coner_step
{
    FD_FORWARD_EDGE = 0,    //前进到边
    FD_TO_EDGE_ROTATION,//右转90度
    FD_FORWARD_CORNER,  //前进到角落
    FD_TO_CORNER       //旋转180度
};

enum auto_clean_step
{


    START_CALIBRATION = 0, // 校准 == X轴正向

    PX_FORWARD,        // 前进到边 == X轴正向
    PX_BACKWARD,       // 后退 == X轴正向
    PX_TO_PY_ROTATION, // 逆时针旋转90° == X轴正向 -> Y轴正向

    NX_FORWARD,        // 前进到边 == X轴负向
    NX_BACKWARD,       // 后退 == X轴负向
    NX_TO_PY_ROTATION, // 顺时针旋转90° == X轴负向 -> Y轴正向

    PY_FORWARD,        // 前进换道 == Y轴正向
    PY_BACKWARD,       // 后退 == Y轴正向
    PY_TO_PX_ROTATION, // 顺时针旋转90° == Y轴正向 -> X轴正向
    PY_TO_NX_ROTATION, // 逆时针旋转90° == X轴负向 -> Y轴负向

    /** 结束阶段 **/
    NX_TO_NY_ROTATION, // 逆时针旋转90° == X轴负向 -> Y轴负向
    NY_FORWARD,        // 前进到边 == Y轴负向
    NY_BACKWARD,       // 后退 == Y轴负向
    NY_TO_PX_ROTATION, // 逆时针旋转90° == Y轴负向 -> X轴正向
    
    END // 结束
};


struct dev_run_param
{
    rt_uint8_t dev_mode;     // 运行模式
    rt_uint8_t dev_auto;     // 自动运行
    rt_uint8_t brush_enable; // 毛刷启停标志
    rt_uint8_t align_enable; // 对齐标志
//    float yaw;               // 角度
//    float lon;
//    float lat;
    enum auto_run_step auto_step;
    enum auto_coner_step auto_coner_step;
    enum auto_clean_step auto_clean_step;      // 自动清洗阶段
    rt_uint32_t auto_cleaned_line_num; // 自动清洗行数
    rt_uint8_t end_flag;               // 自动结束返回标志
};

struct dev_param
{
    struct dev_run_param run;
};

/******************************************************************************/
/** Exported Define                                                          **/
/******************************************************************************/
#define DEV_MODE_STOP 0   // 停止模式
#define DEV_MODE_NORMAL 1 // 正常模式
#define DEV_MODE_AVOID 2  // 探边模式

#define DEV_AUTO_STOP 0  // 停止
#define DEV_AUTO_ONCE 1  // 自动 - 一圈
#define DEV_AUTO_CYCLE 2 // 自动 - 循环


#define AUTO_PX_BACKWARD_DIS  210.0f// 140.0f//  210.0f 无毛刷 // X轴正向 后退距离 单位毫米
#define AUTO_NX_BACKWARD_DIS  210.0f //140.0f  // X轴负向 后退距离
#define AUTO_PY_FORWARD_DIS 580.0f  // 580 380 Y轴正向 前进距离
#define AUTO_PY_BACKWARD_DIS  210.0f //140.0f  // Y轴正向 后退距离
#define AUTO_NY_BACKWARD_DIS  210.0f //130.0f  // Y轴负向 后退距离

#define AUTO_ALIGN_CNT 3 // 自动连续对齐次数 -- 判定校准完成

#define MAX_LIMIT(VALUE, MAX) \
    if (VALUE > MAX)          \
    {                         \
        VALUE = MAX;          \
    }
#define MIN_LIMIT(VALUE, MIN) \
    if (VALUE < MIN)          \
    {                         \
        VALUE = MIN;          \
    }
#define MID_LIMIT(VALUE, MID)               \
    if ((0 < VALUE) && (VALUE < MID))       \
    {                                       \
        VALUE = MID;                        \
    }                                       \
    else if ((0 > VALUE) && (VALUE > -MID)) \
    {                                       \
        VALUE = -MID;                       \
    }

/******************************************************************************/
/** Exported Variables                                                       **/
/******************************************************************************/
extern struct dev_param dev_param;

/*****************************************************************************/
/* External Variables and Functions                                          */
/*****************************************************************************/
int com_dev_param_init(void);

#endif /* __COM_DEV_PARAM_H__ */

/******************** (C) COPYRIGHT 2024 WUJIAN ********* END OF FILE *********/
