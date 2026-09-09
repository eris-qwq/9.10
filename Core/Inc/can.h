/**
  ******************************************************************************
  * @file    can.h
  * @brief   学长工程 bxCAN 名称到 MC02 FDCAN 句柄的兼容层
  ******************************************************************************
  * @note
  * 业务代码继续使用 CAN_HandleTypeDef、hcan1、hcan2 等原名称；真正的底层
  * 外设已经替换为 STM32H723 的 FDCAN1/FDCAN2。这样可以减少控制层改动，
  * 同时明确把硬件差异隔离在底层。
  ******************************************************************************
  */

#ifndef __CAN_H__
#define __CAN_H__

#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 保留原工程的句柄类型名，实际类型是 H7 的 FDCAN 句柄。 */
typedef FDCAN_HandleTypeDef CAN_HandleTypeDef;

/* 保留原业务代码中的句柄变量名。 */
#define hcan1 hfdcan1
#define hcan2 hfdcan2

/* 保留原业务代码用于判断外设实例的名称。 */
#define CAN1 FDCAN1
#define CAN2 FDCAN2

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */
