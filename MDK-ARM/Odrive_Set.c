/* 本文件已加入“小白逐行解释”。【逐行 N】中的 N 是改写前的原始行号，便于和原工程对照。 */
#include "Odrive_Set.h"
#include "usart.h"
float VelMAX = 0;
// #define _MAX_(MAX,Last,Now) ((Now > Last) ? (MAX = Now) : (MAX = Last))
OdriveState_t odrive_state;
OdriveAxisSetState_t odrive_set_axis0;
OdriveAxisGetState_t odrive_get_axis0;
OdriveAxisSetState_t odrive_set_axis1;
OdriveAxisGetState_t odrive_get_axis1;
OdriveAxisSetState_t odrive_set_axis2;
OdriveAxisGetState_t odrive_get_axis2;
OdriveAxisSetState_t odrive_set_axis3;
OdriveAxisGetState_t odrive_get_axis3;

CanMessage_t rx_msg_0,rx_msg_1;
OdriveMsg_t getmode;

uint8_t VOFA[12] = {0,0,0,0,0,0,0,0,0x00,0x00,0x80,0x7F};
    
uint8_t odrive_handle_msg(CanMessage_t *msg)
{
    uint8_t first_word[4];
    uint8_t second_word[4];
    if (msg->len == 8)
    {
        memcpy(&first_word, &msg->buf, 4);
				memcpy(&VOFA, &msg->buf, 4);
        second_word[3] = msg->buf[7];
        second_word[2] = msg->buf[6];
        second_word[1] = msg->buf[5];
        second_word[0] = msg->buf[4];
				VOFA[7] = msg->buf[7];
        VOFA[6] = msg->buf[6];
        VOFA[5] = msg->buf[5];
        VOFA[4] = msg->buf[4];
				HAL_UART_Transmit_DMA(&huart2,VOFA,sizeof VOFA);
    }
    else if (msg->len == 4)
    {
        memcpy(&first_word, &msg->buf, 4);
    }

    OdriveAxisGetState_t *odrive_get;
    if ((msg->id & AXIS0_NODE_ID) == AXIS0_NODE_ID) //  Mask off the first 5 bits of the 11 bit id
    {
        odrive_get = &odrive_get_axis0;
    }
    else if ((msg->id & AXIS1_NODE_ID) == AXIS1_NODE_ID)
    {
        odrive_get = &odrive_get_axis1;
    }
    else
    {
        return 1; // Not an axis
    }
        
        if (msg->rtr == 0)
    {
        switch(getmode) // Mask out the first 5 bits
        {
        case (MSG_ODRIVE_HEARTBEAT):
            memcpy(&odrive_get->axis_error, &first_word, sizeof(uint32_t));
            memcpy(&odrive_get->axis_current_state, &second_word, sizeof(uint32_t));
            break;
        case (MSG_GET_ENCODER_ESTIMATES):
            memcpy(&odrive_get->encoder_pos_estimate, &first_word, sizeof(float));
            memcpy(&odrive_get->encoder_vel_estimate, &second_word, sizeof(float));
            break;
        case (MSG_GET_ENCODER_COUNT):
            memcpy(&odrive_get->encoder_shadow_count, &first_word, sizeof(uint32_t));
            memcpy(&odrive_get->encoder_cpr_count, &second_word, sizeof(uint32_t));
            break;
        case (MSG_GET_MOTOR_ERROR):
            memcpy(&odrive_get->motor_error, &first_word, sizeof(uint32_t));
            break;
        case (MSG_GET_ENCODER_ERROR):
            memcpy(&odrive_get->encoder_error, &first_word, sizeof(uint32_t));
            break;
        case (MSG_GET_VBUS_VOLTAGE):
            memcpy(&odrive_state.vbus_voltage, &first_word, sizeof(float));
            break;
        default:
            break;
        }
    }
    return 0;
}

uint8_t odrive_can_write(Axis_t axis, OdriveMsg_t msg)
{   
    
    getmode = msg;
    
    uint32_t can_error = HAL_CAN_GetError(&hcan1);
    if (can_error == HAL_CAN_ERROR_NONE)
    {
        CAN_TxHeaderTypeDef header;
        header.IDE = CAN_ID_STD;
        
        uint8_t data[8];
        uint8_t *ptrTor;
        uint8_t *ptrControl;
        uint8_t *ptrInput;
        
        
        uint8_t tmp_word[4]; // TODO: Get rid of this with better mem management
        OdriveAxisSetState_t *odrive_set;
				switch(axis){
					case AXIS_0:
						header.StdId = AXIS0_NODE_ID + msg;
            odrive_set = &odrive_set_axis0;
					break;
					case AXIS_1:
						header.StdId = AXIS1_NODE_ID + msg;
            odrive_set = &odrive_set_axis1;
					break;
					case AXIS_2:
						header.StdId = AXIS2_NODE_ID + msg;
            odrive_set = &odrive_set_axis2;
					break;
					case AXIS_3:
						header.StdId = AXIS3_NODE_ID + msg;
            odrive_set = &odrive_set_axis3;
					break;
					default:return 0;break;
						
			 }
//        if (axis == AXIS_0)
//        {
//            
//        }
//        else if (axis == AXIS_1)
//        {
//           
//        } 
//        else 
//        {
//            return 1;
//        }

        switch (msg)
        {
        case MSG_ODRIVE_ESTOP:
            /* TODO: Implement */
            break;
        case MSG_GET_MOTOR_ERROR:
            header.RTR = CAN_RTR_REMOTE;
            header.DLC = 0;
            break;
        case MSG_GET_ENCODER_ERROR:
            header.RTR = CAN_RTR_REMOTE;
            header.DLC = 0;
            break;
        case MSG_GET_SENSORLESS_ERROR:
            /* TODO: Implement */
            break;
        case MSG_SET_AXIS_NODE_ID:
            /* TODO: Implement */
            break;
        case MSG_SET_AXIS_REQUESTED_STATE:
            memcpy(&data, &odrive_set->requested_state, 4);
            header.RTR = CAN_RTR_DATA;
            header.DLC = 4;
            break;
        case MSG_SET_AXIS_STARTUP_CONFIG:
            /* TODO: Implement */
            break;
        case MSG_GET_ENCODER_ESTIMATES:
            header.RTR = CAN_RTR_REMOTE;
            header.DLC = 0;
            break;
        case MSG_GET_ENCODER_COUNT:
            header.RTR = CAN_RTR_REMOTE;
            header.DLC = 0;
            break;
        case MSG_SET_CONTROLLER_MODES:
            
            ptrControl = (uint8_t *)&odrive_set->control_mode;
            ptrInput = (uint8_t *)&odrive_set->input_mode;
            data[0] = ptrControl[0];
            data[1] = ptrControl[1];
            data[2] = ptrControl[2];
            data[3] = ptrControl[3];
            data[4] = ptrInput[0];
            data[5] = ptrInput[1];
            data[6] = ptrInput[2];
            data[7] = ptrInput[3];
            header.RTR = CAN_RTR_DATA;
            header.DLC = 8;
            break;
        case MSG_SET_INPUT_POS:
            memcpy(&data, &odrive_set->input_pos, 4);
            data[4] = odrive_set->vel_ff & 0x00FF;
            data[5] = odrive_set->vel_ff >> 8;
            data[6] = odrive_set->current_ff & 0x00FF;
            data[7] = odrive_set->current_ff >> 8;
            header.RTR = CAN_RTR_DATA;
            header.DLC = 8;
            break;
        case MSG_SET_INPUT_VEL:
            memcpy(&data, &odrive_set->input_vel, 4); 
            ptrTor = (uint8_t *)&odrive_set->current_ff;
            data[4] = ptrTor[0];
            data[5] = ptrTor[1];
            data[6] = ptrTor[0];
            data[7] = ptrTor[1];
            header.RTR = CAN_RTR_DATA;
            header.DLC = 8;
            break;
        case MSG_SET_INPUT_CURRENT:
            memcpy(&data, &odrive_set->input_current, 4);
            header.RTR = CAN_RTR_DATA;
            header.DLC = 4;
            break;
        case MSG_SET_VEL_LIMIT:
            memcpy(&data, &odrive_set->vel_limit, 4);
            header.RTR = CAN_RTR_DATA;
            header.DLC = 4;
            break;
        case MSG_START_ANTICOGGING:
            header.RTR = CAN_RTR_REMOTE;
            header.DLC = 0;
            break;
        case MSG_SET_TRAJ_VEL_LIMIT:
            memcpy(&data, &odrive_set->traj_vel_limit, 4);
            header.RTR = CAN_RTR_DATA;
            header.DLC = 4;
            break;
        case MSG_SET_TRAJ_ACCEL_LIMITS:
            memcpy(&data, &odrive_set->traj_accel_limit, 4);
            memcpy(&tmp_word, &odrive_set->traj_decel_limit, 4);
            data[4] = tmp_word[0];
            data[5] = tmp_word[1];
            data[6] = tmp_word[2];
            data[7] = tmp_word[3];
            header.RTR = CAN_RTR_DATA;
            header.DLC = 4;
            break;
        case MSG_SET_TRAJ_A_PER_CSS:
            memcpy(&data, &odrive_set->traj_a_per_css, 4);
            header.RTR = CAN_RTR_DATA;
            header.DLC = 4;
            break;
        case MSG_GET_IQ:
            /* TODO: Implement */
            break;
        case MSG_GET_SENSORLESS_ESTIMATES:
            /* TODO: Implement */
            break;
        case MSG_RESET_ODRIVE:
            header.RTR = CAN_RTR_REMOTE;
            header.DLC = 0;
            break;
        case MSG_GET_VBUS_VOLTAGE:
            header.RTR = CAN_RTR_REMOTE;
            header.DLC = 0;
            break;
        case MSG_CLEAR_ERRORS:
            header.RTR = CAN_RTR_REMOTE;
            header.DLC = 0;
            break;
        case MSG_CO_HEARTBEAT_CMD:
            /* TODO: Implement */
            break;
        default:
            break;
        }
    uint32_t retTxMailbox = 0;
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
        {
            HAL_CAN_AddTxMessage(&hcan1, &header, data, &retTxMailbox);
        }

        return 1;
    }
    else
    {
        return can_error;
    }
}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0))
//    {
//        CAN_RxHeaderTypeDef header;
//        if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
//        {
//            CanMessage_t rxmsg;
//            HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, rxmsg.buf);

//            rxmsg.id = header.StdId;
//            rxmsg.len = header.DLC;
//            rxmsg.rtr = header.RTR;
//            if(rxmsg.id == AXIS0_NODE_ID + getmode)
//            {
//                memcpy(&rx_msg_0,&rxmsg,sizeof(CanMessage_t));
//            }
//            else if(rxmsg.id == AXIS1_NODE_ID + getmode)
//            {
//                memcpy(&rx_msg_1,&rxmsg,sizeof(CanMessage_t));
//            }
//        }
//    }

//}
