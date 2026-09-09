/* 本文件已加入“小白逐行解释”。【逐行 N】中的 N 是改写前的原始行号，便于和原工程对照。 */
/**
 * @file    RMLibHead.h
 * @author  Yao
 * @date    2020年11月4日
 * @brief   RMLib头文件，引用库文件并定义基本返回值，支持实时系统临界区和动态内存函数
 */

/**@mainpage
 * <table>
 * <tr><th>Project  <td>RMLib
 * <tr><th>Author   <td>yao
 * </table>
 * @section Robomaster整合库
 *
 * <h2><a href="http://www.kdrobot.top/git/KDRobot_RM/RM_LIB">Git页面</a></h2>
 *
 * 注意使用的HAL库的版本大于1.19.0,推荐1.24.x
 *
 * ## 配置方法
 *
 * 添加全局宏定义`__USE_RTOS`决定是否使用FreeRTOS的标志决定了CAN是否使用临界区保护以及巴特沃斯滤波器动态内存分配使用的函数</p>
 * 添加全局宏定义`WatchDoglength`，并赋值看门狗最大数量启用看门狗</p>
 *
 * <table>
 * <tr><th>CAN底层驱动模块  <td>@see CANDrive.h @see CANDrive.c
 * <tr><th>底盘驱动模块  <td>@see Chassis.h @see Chassis.c
 * <tr><th>任意阶IIR直接II型，二阶节滤波器模块  <td>@see IIRFilter.h @see IIRFilter.c
 * <tr><th>二阶卡尔曼滤波器模块  <td>@see kalmanII.h @see kalmanII.c
 * <tr><th>一阶卡尔曼滤波器模块  <td>@see kalman.h @see kalman.c
 * <tr><th>电机驱动模块  <td>@see motor.h @see motor.c
 * <tr><th>PID模块  <td>@see PID.h @see PID.c
 * <tr><th>斜坡函数模块  <td>@see ramp.h @see ramp.c
 * <tr><th>遥控器模块  <td>@see remote.h @see remote.c
 * <tr><th>队列模块  <td>@see RMQueue.h @see RMQueue.c
 * <tr><th>软件看门狗模块头  <td>@see WatchDog.h @see WatchDog.c
 * <tr><th>虚拟串口通信模块  <td>@see VCOMCOMM.h @see VCOMCOMM.c
 * <tr><th>CRC校验模块  <td>@see CRC.h @see CRC.c
 * <tr><th>串口printf缓冲区模块  <td>@see PrintfDMABuffer.h @see PrintfDMABuffer.c
 * <tr><th>RoboModule直流电机驱动器驱动模块  <td>@see RoboModule_DRV.h @see RoboModule_DRV.c
 * </table>
 */

#ifndef _RMLIBHEAD_H
#define _RMLIBHEAD_H

#ifdef __cplusplus
    #define RMLIB_CPP_BEGIN extern "C" {
    #define RMLIB_CPP_END }
#else
    #define RMLIB_CPP_BEGIN
    #define RMLIB_CPP_END
#endif

#define RM_LIB_VERSION 20210630

#include <string.h>
#include <stdlib.h>

RMLIB_CPP_BEGIN

/* 本移植工程目标固定为 DM-MC02 的 STM32H723，不再保留原库中会引入
 * F0/F1/F3/F4 头文件的跨芯片条件分支。这个变化只属于芯片适配层。 */
#include <stm32h7xx.h>

#ifdef __USE_RTOS

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#define RMLIB_ENTER_CRITICAL() vPortEnterCritical()
#define RMLIB_EXIT_CRITICAL() vPortExitCritical()
#define RMLIB_MALLOC(SIZE) pvPortMalloc(SIZE)
#define RMLIB_FREE(P) vPortFree(P)

#else //__USE_RTOS

#define RMLIB_ENTER_CRITICAL()
#define RMLIB_EXIT_CRITICAL()
#define RMLIB_MALLOC(SIZE) malloc(SIZE)
#define RMLIB_FREE(P) free(P)

#endif //__USE_RTOS

typedef enum {
    RM_SUCCESS = 1,
    RM_YES = RM_SUCCESS,
    RM_ENABLE = RM_SUCCESS,
    RM_ERROR = 0,
    RM_NO = RM_ERROR,
    RM_DISABLE = RM_ERROR,
} RM_Status;

RMLIB_CPP_END

#endif //_RMLIBHEAD_H
