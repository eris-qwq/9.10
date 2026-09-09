/* 本文件已加入“小白逐行解释”。【逐行 N】中的 N 是改写前的原始行号，便于和原工程对照。 */
/**
 * @file    ODRIVE_SET.h
 * @author  guo
 * @date    25-Feb-2023
 * @brief   ÎÞË¢Ö±Á÷µç»úÍ¨ÓÃÇý¶¯Ä£¿é
 * @note
 *  Ê¹ÓÃpython»·¾³ÅäÖÃºÃodriveÇý¶¯°å£¬PID¼´¿É
 *  odrive¹Ù·½canÖ¸Áî£ºhttps://docs.odriverobotics.com/v/0.5.4/can-protocol.html
 *  ÀíÂÛÉÏÊÐ³¡ÉÏËùÓÐÎÞË¢µç»ú¾ù¿ÉÒÔÊ¹ÓÃ¸ÃÄ£¿é¿ØÖÆ
 */

#ifndef __ODRIVE_SET_H
#define __ODRIVE_SET_H

#include "main.h"
#include "string.h"
#include "can.h"

#define AXIS0_NODE_ID (0x000 << 5)
#define AXIS1_NODE_ID (0x010 << 5)

#define AXIS2_NODE_ID (0x020 << 5)
#define AXIS3_NODE_ID (0x030 << 5)

/*  odriveÖ¸Áî    */
typedef enum
{
    MSG_CO_NMT_CTRL = 0x000, 
    MSG_ODRIVE_HEARTBEAT,           //ODrive ¼ì²âÐÅºÅÏûÏ¢
    MSG_ODRIVE_ESTOP,               //ODrive Estop ÏûÏ¢
    MSG_GET_MOTOR_ERROR,            //»ñÈ¡µç»ú´íÎó*
    MSG_GET_ENCODER_ERROR,          //»ñÈ¡±àÂëÆ÷´íÎó*
    MSG_GET_SENSORLESS_ERROR,       //»ñÈ¡ÎÞ´«¸ÐÆ÷´íÎó*
    MSG_SET_AXIS_NODE_ID,           //ÉèÖÃÖá½Úµã ID
    MSG_SET_AXIS_REQUESTED_STATE,   //ÉèÖÃÖáÇëÇó×´Ì¬
    MSG_SET_AXIS_STARTUP_CONFIG,    //ÉèÖÃÖáÆô¶¯ÅäÖÃ
    MSG_GET_ENCODER_ESTIMATES,      //»ñÈ¡±àÂëÆ÷¹ÀËãÖµ*
    MSG_GET_ENCODER_COUNT,          //»ñÈ¡±àÂëÆ÷¼ÆÊý*
    MSG_SET_CONTROLLER_MODES,       //ÉèÖÃ¿ØÖÆÆ÷Ä£Ê½
    MSG_SET_INPUT_POS,              //ÉèÖÃÊäÈëÎ»ÖÃ
    MSG_SET_INPUT_VEL,              //ÉèÖÃÊäÈë Vel
    MSG_SET_INPUT_CURRENT,          //ÉèÖÃÊäÈëÅ¤¾Ø
    MSG_SET_VEL_LIMIT,              //ÉèÖÃÏÞÖÆ
    MSG_START_ANTICOGGING,          //¿ªÊ¼¿¹³Ý²ÛÐ§Ó¦
    MSG_SET_TRAJ_VEL_LIMIT,         //ÌÝÐÎ¹ì¼£ËÙ¶ÈÏÞÖÆ
    MSG_SET_TRAJ_ACCEL_LIMITS,      //ÌÝÐÎ¹ì¼£¼ÓËÙ¶ÈÏÞÖÆ
    MSG_SET_TRAJ_A_PER_CSS,         //ÌÝÐÎ¹ì¼£¹ßÐÔ
    MSG_GET_IQ,
    MSG_GET_SENSORLESS_ESTIMATES,  
    MSG_RESET_ODRIVE,               //ÖØÐÂÆô¶¯ ODrive
    MSG_GET_VBUS_VOLTAGE,           //»ñÈ¡ Vbus µçÑ¹
    MSG_CLEAR_ERRORS,               //Çå³ý´íÎó
    MSG_CO_HEARTBEAT_CMD = 0x700, 
} OdriveMsg_t;


typedef enum
{
    AXIS_0 = 0,
    AXIS_1,
	  AXIS_2,
		AXIS_3
} Axis_t;


typedef struct
{
    uint16_t id; // 11-bit max is 0x7ff, 29-bit max is 0x1FFFFFFF
    uint8_t rtr;
    uint8_t len;
    uint8_t buf[8];
} CanMessage_t;

typedef struct
{
    uint32_t axis_error;
    uint32_t axis_current_state;
    uint32_t motor_error;
    uint32_t encoder_error;
    uint32_t sensorless_error;
    float encoder_pos_estimate;
    float encoder_vel_estimate;
    int32_t encoder_shadow_count;
    int32_t encoder_cpr_count;
    float iq_setpoint;
    float iq_measured;
    float sensorless_pos_estimate;
    float sensorless_vel_estimate;
} OdriveAxisGetState_t;


typedef struct
{
    uint16_t axis_node_id;
    uint32_t requested_state;   //ÖáÇëÇó×´Ì¬
    int32_t control_mode;       //¿ØÖÆÄ£Ê½
    int32_t input_mode;         //Êä³öÄ£Ê½
    int16_t vel_ff;
    int16_t current_ff;           //Ç°À¡Å¤¾Ø
    float input_pos;
    float input_vel;
    int32_t input_current;
    float vel_limit;
    float traj_vel_limit;       //ÌÝÐÎ¹ì¼£ËÙ¶ÈÏÞÖÆ
    float traj_accel_limit;     //ÌÝÐÎ¹ì¼£¼ÓËÙ¶ÈÏÞÖÆ
    float traj_decel_limit;     //ÌÝÐÎ¹ì¼£¼õËÙ¶ÈÏÞÖÆ
    float traj_a_per_css;
} OdriveAxisSetState_t;

typedef struct
{
    float vbus_voltage;
} OdriveState_t;

//Axis States
typedef enum{
	UNDEFINED = 0x0,                    //<! will fall through to idle
	IDLE = 0x1,                         //<! disable PWM and do nothing
	STARTUP_SEQUENCE = 0x2,             //<! the actual sequence is defined by the config.startup_... flags
	FULL_CALIBRATION_SEQUENCE = 0x3,    //<! run all calibration procedures, then idle
	MOTOR_CALIBRATION = 0x4,            //µç»úÐ£×¼
	ENCODER_INDEX_SEARCH = 0x6,         //ÔËÐÐ±àÂëÆ÷Ë÷ÒýËÑË÷
	ENCODER_OFFSET_CALIBRATION = 0x7,   //ÔËÐÐ±àÂëÆ÷Æ«ÒÆÐ£×¼
	CLOSED_LOOP_CONTROL = 0x8,          //±Õ»·¿ØÖÆ
	LOCKIN_SPIN = 0x9,
	ENCODER_DIR_FIND = 0xA,
	HOMING = 0xB,
	ENCODER_HALL_POLARITY_CALIBRATION = 0xC,
	ENCODER_HALL_PHASE_CALIBRATION = 0xD
} Axis_State;

typedef enum{
	VOLTAGE_CONTROL = 0x0,
	TORQUE_CONTROL = 0x1,
	VELOCITY_CONTROL = 0x2,
	POSITION_CONTROL = 0x3,
	MOTOR_IDLE //没用
} Control_Mode;

typedef enum{
	INACTIVE = 0x0,
	PASSTHROUGH = 0x1,                  //Ö±½ÓÊä³ö
	VEL_RAMP = 0x2,                     //ËÙ¶ÈÅÀÉýÄ£Ê½
	POS_FILTER = 0x3,
	MIX_CHANNELS = 0x4,
	TRAP_TRAJ = 0x5,
	TORQUE_RAMP = 0x6,                  //Á¦¾ØÅÀÉýÄ£Ê½
	MIRROR = 0x7,
	TUNING = 0x8
} Input_Mode;

extern CanMessage_t rx_msg_0,rx_msg_1;
extern OdriveState_t odrive_state;
extern OdriveMsg_t getmode;
extern OdriveAxisSetState_t odrive_set_axis0;
extern OdriveAxisGetState_t odrive_get_axis0;
extern OdriveAxisSetState_t odrive_set_axis1;
extern OdriveAxisGetState_t odrive_get_axis1;
extern OdriveAxisSetState_t odrive_set_axis2;
extern OdriveAxisGetState_t odrive_get_axis2;
extern OdriveAxisSetState_t odrive_set_axis3;
extern OdriveAxisGetState_t odrive_get_axis3;

/*´¦ÀíÊÕµ½µÄÓÊÏäÊý¾Ý »ñÈ¡µç»ú¸÷ÖÖÐÅÏ¢*/
uint8_t odrive_handle_msg(CanMessage_t *msg);

/*
    ·¢ËÍÖ¸Áî¸øµç»ú£¬
    axis£ºµç»úºÅ£»
    msg£ºodriveÖ¸Áî
*/
uint8_t odrive_can_write(Axis_t axis, OdriveMsg_t msg);
#endif
