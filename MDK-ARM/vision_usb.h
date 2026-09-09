/**
 * @file    vision_usb.h
 * @brief   上位机视觉协议到学长工程 Data1Pack/ 的 USB CDC 适配层
 */

#ifndef _VISION_USB_H_
#define _VISION_USB_H_

#include <stdint.h>

/** 接收 USB CDC 本次到达的字节，可处理拆包和粘包。 */
void Vision_USB_Receive(const uint8_t *data, uint32_t length);

/** 使用 MC02 原生 USB CDC 发送一段数据。 */
uint8_t Vision_USB_Transmit(uint8_t *data, uint16_t length);

/** 发送航向角监视帧（功能码0xA0），不改动原视觉协议帧。 */
uint8_t Vision_USB_TransmitYaw(float yaw_centideg,
                               int16_t omega_centideg_s,
                               uint32_t uptime_ms,
                               float temperature_c,
                               uint16_t heater_pwm,
                               float position_kp,
                               float position_ki,
                               float position_kd,
                               float speed_kp,
                               float speed_ki,
                               float speed_kd);

/** 上位机发来合法二进制帧后返回1；上电默认为可读文本模式。 */
uint8_t Vision_USB_IsBinaryMode(void);

/** 向普通串口监视器发送一行可读数据。 */
uint8_t Vision_USB_TransmitTextStatus(float yaw_centideg,
                                      float target_yaw_centideg,
                                      int16_t omega_centideg_s,
                                      float temperature_c,
                                      uint16_t heater_pwm,
                                      float position_kp,
                                      float position_ki,
                                      float position_kd,
                                      float speed_kp,
                                      float speed_ki,
                                      float speed_kd,
                                      uint8_t calibration_state,
                                      uint8_t calibration_progress,
                                      float gyro_bias_z_deg_s);

/** 打印四个底盘M2006的原始目标、斜坡目标、RPM反馈和PID输出。 */
uint8_t Vision_USB_TransmitChassisStatus(
    int32_t raw_1, int32_t raw_2, int32_t raw_3, int32_t raw_4,
    int32_t ramp_1, int32_t ramp_2, int32_t ramp_3, int32_t ramp_4,
    int16_t feedback_1, int16_t feedback_2, int16_t feedback_3, int16_t feedback_4,
    float output_1, float output_2, float output_3, float output_4);

#endif /* _VISION_USB_H_ */
