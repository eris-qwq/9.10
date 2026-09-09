/**
 * @file    CANDrive.c
 * @brief   保留学长工程接口名的 STM32H723 FDCAN 底层
 * @note    CAN1接四个底盘M2006；CAN2接机构M2006与GM6020。
 */

#include "CANDrive.h"

uint8_t CAN1_buff[8];
uint8_t CAN2_buff[8];

/* 业务层原回调名称保持不变，由 H7 的 FDCAN 回调转入。 */
extern void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);

/**
 * @brief 配置指定 FDCAN 的标准帧接收滤波器。
 * @param hcan FDCAN 句柄（业务层仍沿用 CAN_HandleTypeDef 名称）。
 * @note  FDCAN1接收0x201~0x204；FDCAN2接收0x201~0x205。
 */
void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

    if (hcan->Instance == FDCAN1)
    {
        /* CAN1：四个底盘M2006。 */
        filter.FilterType = FDCAN_FILTER_RANGE;
        filter.FilterID1 = 0x201;
        filter.FilterID2 = 0x204;
    }
    else
    {
        /* CAN2：机构电机反馈ID均处于此范围。 */
        filter.FilterType = FDCAN_FILTER_RANGE;
        filter.FilterID1 = 0x201;
        filter.FilterID2 = 0x205;
    }

    if (HAL_FDCAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    /* 未命中过滤器的标准帧和扩展帧均丢弃，避免无关报文占满 FIFO。 */
    if (HAL_FDCAN_ConfigGlobalFilter(hcan,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief 发送一帧 8 字节经典 CAN 标准数据帧。
 * @param hcan  FDCAN 句柄。
 * @param StdId 11 位标准帧 ID。
 * @param msg   8 字节数据缓冲区。
 * @return HAL_OK 表示已放入发送 FIFO，其余值表示发送失败或 FIFO 忙。
 */
HAL_StatusTypeDef CAN_Send_StdDataFrame(CAN_HandleTypeDef *hcan,
                                        uint32_t StdId,
                                        uint8_t *msg)
{
    FDCAN_TxHeaderTypeDef header = {0};
    HAL_StatusTypeDef status;

    header.Identifier = StdId;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0;

    /* 保留原库的临界区保护，避免两个任务同时改写发送 FIFO。 */
    RMLIB_ENTER_CRITICAL();
    status = HAL_FDCAN_AddMessageToTxFifoQ(hcan, &header, msg);
    RMLIB_EXIT_CRITICAL();
    return status;
}

/**
 * @brief 从接收 FIFO0 取出一帧，并返回其标准 ID。
 * @param hcan FDCAN 句柄。
 * @param buf  接收 8 字节负载的缓冲区。
 * @return 接收到的标准 ID；读取失败时返回 0。
 */
uint32_t CAN_Receive_DataFrame(CAN_HandleTypeDef *hcan, uint8_t *buf)
{
    FDCAN_RxHeaderTypeDef header = {0};

    if (HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &header, buf) != HAL_OK)
    {
        return 0;
    }
    return header.Identifier;
}

/**
 * @brief H7 HAL 的 FDCAN FIFO0 新消息回调。
 * @param hfdcan 触发中断的 FDCAN 句柄。
 * @param RxFifo0ITs FIFO0 中断状态位。
 * @note 两路FDCAN收到新帧后，都转入保留原名的业务回调。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    if ((hfdcan->Instance == FDCAN1) || (hfdcan->Instance == FDCAN2))
    {
        HAL_CAN_RxFifo1MsgPendingCallback(hfdcan);
    }
}
