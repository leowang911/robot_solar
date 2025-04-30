/******************************************************************************/
/** Private Includes                                                         **/
/******************************************************************************/
#include <rtthread.h>
#define LOG_TAG "dev.param"
#define LOG_LVL LOG_LVL_DBG
#include <ulog.h>
#include "com_dev_param.h"

/******************************************************************************/
/** Private Variables                                                        **/
/******************************************************************************/
struct dev_param dev_param;

/******************************************************************************/
/** Private Function                                                         **/
/******************************************************************************/
/**
 * @brief  设备参数模块初始化。
 * @param  无
 * @retval 返回初始化结果。
 */
int com_dev_param_init(void) {
    /** 运行参数初始化 **/
    dev_param.run.dev_mode = DEV_MODE_STOP;
    dev_param.run.dev_auto = DEV_AUTO_STOP;
    dev_param.run.brush_enable = 0;
    dev_param.run.align_enable = 0;

    dev_param.run.auto_cleaned_line_num = 0;
    dev_param.run.auto_step = MANUAL;
    dev_param.run.end_flag = 0;

    return RT_EOK;
}
#ifdef FINSH_USING_MSH
INIT_APP_EXPORT(com_dev_param_init);
#endif

/******************************************************************************/
/** (C) COPYRIGHT 2024 WUJIAN                                    END OF FILE **/
/******************************************************************************/
