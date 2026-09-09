#ifndef MECHANISM_CONFIG_H
#define MECHANISM_CONFIG_H

/* ========================= 机构电机集中配置区 =========================
 * 只需在这里调机械相关参数，Auto.c中的原流程距离值继续沿用学长工程。
 * 当前版本：升降保留PA0脉冲步进电机；伸缩由C610+M2006（36:1减速箱）驱动。
 */

#define MECH_HEIGHT_USE_M2006             0U
#define MECH_LENGTH_USE_M2006             1U

/* CAN2反馈ID；C610的控制帧仍为0x200，ID决定它使用控制帧中的第几个电流值。 */
#define MECH_HEIGHT_CAN_ID                 0x201U
#define MECH_LENGTH_CAN_ID                 0x202U
#define MECH_GM6020_CAN_ID                 0x205U

/* 机械正方向与电机编码器正方向的关系。运动方向反了只改1.0f为-1.0f。 */
#define MECH_HEIGHT_MOTOR_SIGN             1.0f
#define MECH_LENGTH_MOTOR_SIGN             1.0f

/* 最重要的标定量：电机轴转多少度，等于原流程的1个StepCounter单位。
 * 目前先填1.0f以便编译和低速联调；机械安装好后必须实测再修改。
 * 计算方法：让机构运动已知距离，读取电机角度变化，再除以对应旧单位变化。
 */
#define MECH_HEIGHT_DEG_PER_LEGACY_UNIT    1.0f
#define MECH_LENGTH_DEG_PER_LEGACY_UNIT    1.0f

/* 原流程允许的逻辑行程。没有原点开关前，这只是软件保护，不能替代机械限位。 */
#define MECH_HEIGHT_MIN_UNIT               0.0f
#define MECH_HEIGHT_MAX_UNIT               3600.0f
#define MECH_LENGTH_MIN_UNIT               0.0f
#define MECH_LENGTH_MAX_UNIT               6500.0f

/* 位置外环：位置误差乘此值，得到M2006目标转速(rpm)。 */
#define MECH_HEIGHT_POSITION_KP             5.0f
#define MECH_LENGTH_POSITION_KP             5.0f
#define MECH_HEIGHT_MAX_SPEED_RPM        1800.0f
#define MECH_LENGTH_MAX_SPEED_RPM        1800.0f

/* 速度内环：把目标转速与反馈转速的误差变成C610电流命令。先小电流联调。 */
#define MECH_SPEED_KP                       3.0f
#define MECH_SPEED_KI                       0.015f
#define MECH_SPEED_KD                       0.0f
#define MECH_CURRENT_LIMIT               5000.0f

/* 连续满足位置和速度条件若干个2ms周期，才判定到位，避免刚经过目标点就结束。 */
#define MECH_POSITION_TOLERANCE_UNIT        5.0f
#define MECH_SPEED_TOLERANCE_RPM           60.0f
#define MECH_STABLE_CYCLES                 20U
#define MECH_FEEDBACK_WAIT_MS            2000U
#define MECH_MOVE_TIMEOUT_MS            15000U

/* 视觉对准时，每次AutoArm调用让伸缩目标移动的旧单位数；调用周期约2ms。 */
#define MECH_JOG_UNIT_PER_CALL              2.0f

/* 断电位置保存到MC02板载W25Q64最后一个4KB扇区。
 * 1=启用，0=禁用。它只能记住“断电前软件认为的位置”；断电后人工移动机构会造成偏差。
 */
#define MECH_POSITION_PERSIST_ENABLE        1U
#define MECH_POSITION_FLASH_ADDRESS         0x007FF000UL

/* 原点开关后续再接。现在保持0，不读取任何未接入的GPIO。
 * 将来触发原点开关时调用Mechanism_ApplyOrigin()，即可把累计误差重新归零。
 */
#define MECH_ORIGIN_SWITCH_ENABLE           0U

#endif
