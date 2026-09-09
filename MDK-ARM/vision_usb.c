/**
 * @file    vision_usb.c
 * @brief   MC02 Type-C USB CDC 与原视觉数据结构之间的协议适配
 */

#include "vision_usb.h"

#include <string.h>
#include <stdio.h>

#include "CRC.h"
#include "PID_task.h"
#include "usbd_cdc_if.h"

/* 保留学长工程原全局变量的名称和类型。 */
Data2Pack Data2;
uint8_t ScreenGo;

/* 不再接遥控器，但保留零值按键对象，避免原 Updatakey() 解引用空指针。 */
static hw_key_t RemoteKeyZero;
Remote_Handle_t Remote_Control = {
    .Key_Control = &RemoteKeyZero
};

extern Data1Pack Sequence;

/* VCOM 帧最大有效载荷；坐标为4字节，car V2二维码结果为12字节。 */
#define VISION_USB_MAX_PAYLOAD 48U
#define VISION_USB_BUFFER_SIZE 96U

static uint8_t stream_buffer[VISION_USB_BUFFER_SIZE];
static uint16_t stream_length;
static uint8_t binary_mode;

/**
 * @brief 按小端顺序读取一个 16 位无符号数。
 * @param bytes 至少包含两个字节的地址。
 * @return 解析后的 16 位数值。
 */
static uint16_t ReadU16LE(const uint8_t *bytes)
{
    return (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
}

/**
 * @brief 把一帧完整视觉数据写入原工程的全局结构。
 * @param frame 完整帧，格式为 5A、功能码、ID、长度、负载、CRC16。
 */
static void HandleFrame(const uint8_t *frame)
{
    const uint8_t function = frame[1];
    const uint16_t id = ReadU16LE(frame + 2);
    const uint16_t payload_length = ReadU16LE(frame + 4);
    const uint8_t *payload = frame + 6;

    if ((function == 0U) && (payload_length == 4U))
    {
        binary_mode = 1U;
        /* 对准数据：保持 Data2Pack 的字段名、类型和 0x5A 有效标志。 */
        Data2.head = 0x5A;
        Data2.Fuction = function;
        Data2.ID = id;
        Data2.DataLen = payload_length;
        Data2.ex = (int16_t)ReadU16LE(payload);
        Data2.ey = (int16_t)ReadU16LE(payload + 2);
    }
    else if ((function == 1U) && (payload_length == 12U))
    {
        binary_mode = 1U;
        /* car V2负载排列为：第一轮3色、第一轮3位置、第二轮3色、第二轮3位置。
         * 原自动流程只需要两轮共6个颜色，所以取payload[0..2]和payload[6..8]。
         */
        Sequence.head = 0x5A;
        Sequence.Fuction = function;
        Sequence.ID = id;
        Sequence.DataLen = sizeof(Sequence.Sequence);
        memcpy(Sequence.Sequence, payload, 3U);
        memcpy(Sequence.Sequence + 3U, payload + 6U, 3U);
        ScreenGo = 1U;
    }
}

/**
 * @brief 接收 USB CDC 字节流并完成帧同步、拆包和粘包处理。
 * @param data 本次收到的字节地址。
 * @param length 本次收到的字节数。
 */
void Vision_USB_Receive(const uint8_t *data, uint32_t length)
{
    while (length-- > 0U)
    {
        /* 缓冲区异常满时丢弃旧数据并重新寻找 0x5A 帧头。 */
        if (stream_length >= VISION_USB_BUFFER_SIZE)
        {
            stream_length = 0U;
        }
        stream_buffer[stream_length++] = *data++;
    }

    for (;;)
    {
        uint16_t payload_length;
        uint16_t frame_length;
        uint16_t head = 0U;

        /* 跳过帧头之前的噪声字节。 */
        while ((head < stream_length) && (stream_buffer[head] != 0x5AU))
        {
            ++head;
        }
        if (head > 0U)
        {
            memmove(stream_buffer, stream_buffer + head, stream_length - head);
            stream_length = (uint16_t)(stream_length - head);
        }

        /* 六字节协议头尚未收齐，等待下一次 USB OUT 数据。 */
        if (stream_length < 6U)
        {
            return;
        }

        payload_length = ReadU16LE(stream_buffer + 4);
        if (payload_length > VISION_USB_MAX_PAYLOAD)
        {
            /* 非法长度通常说明当前 0x5A 不是帧头，丢掉它重新同步。 */
            --stream_length;
            memmove(stream_buffer, stream_buffer + 1, stream_length);
            continue;
        }

        frame_length = (uint16_t)(6U + payload_length + 2U);
        if (stream_length < frame_length)
        {
            return;
        }

        /* CRC16 只校验有效负载；空负载按原协议要求 CRC 为 0。 */
        {
            const uint16_t received_crc = ReadU16LE(stream_buffer + 6U + payload_length);
            const uint16_t calculated_crc = (payload_length == 0U)
                                                ? 0U
                                                : Verify_CRC16_Check_Sum(stream_buffer + 6U,
                                                                         payload_length);
            if (received_crc != calculated_crc)
            {
                /* CRC 错误：丢掉当前帧头，从后续字节重新寻找 0x5A。 */
                --stream_length;
                memmove(stream_buffer, stream_buffer + 1, stream_length);
                continue;
            }
        }

        HandleFrame(stream_buffer);
        memmove(stream_buffer,
                stream_buffer + frame_length,
                stream_length - frame_length);
        stream_length = (uint16_t)(stream_length - frame_length);
    }
}

/**
 * @brief 经 MC02 Type-C 的 USB CDC IN 端点发送原视觉状态帧。
 * @param data 发送缓冲区。
 * @param length 发送字节数。
 * @return USBD_OK、USBD_BUSY 或 USBD_FAIL。
 */
uint8_t Vision_USB_Transmit(uint8_t *data, uint16_t length)
{
    static uint8_t frame[VISION_USB_MAX_PAYLOAD + 8U];
    uint16_t payload_length;
    uint16_t frame_length;
    uint16_t crc;

    if ((data == NULL) || (length < 8U))
    {
        return USBD_FAIL;
    }

    payload_length = ReadU16LE(data + 4U);
    frame_length = (uint16_t)(payload_length + 8U);
    if ((payload_length > VISION_USB_MAX_PAYLOAD) || (length < frame_length))
    {
        return USBD_FAIL;
    }

    memcpy(frame, data, 6U + payload_length);
    crc = (payload_length == 0U)
              ? 0U
              : Verify_CRC16_Check_Sum(frame + 6U, payload_length);
    frame[6U + payload_length] = (uint8_t)(crc & 0xFFU);
    frame[7U + payload_length] = (uint8_t)(crc >> 8);

    return CDC_Transmit_HS(frame, frame_length);
}

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
                               float speed_kd)
{
    static uint8_t frame[48U];
    const int32_t yaw = (int32_t)yaw_centideg;
    const int16_t temperature_centideg = (int16_t)(temperature_c * 100.0f);
    uint16_t crc;

    frame[0] = 0x5AU;
    frame[1] = 0xA0U;
    frame[2] = 0x01U;
    frame[3] = 0x00U;
    frame[4] = 40U;
    frame[5] = 0U;

    frame[6] = (uint8_t)yaw;
    frame[7] = (uint8_t)(yaw >> 8);
    frame[8] = (uint8_t)(yaw >> 16);
    frame[9] = (uint8_t)(yaw >> 24);
    frame[10] = (uint8_t)omega_centideg_s;
    frame[11] = (uint8_t)(omega_centideg_s >> 8);
    frame[12] = (uint8_t)uptime_ms;
    frame[13] = (uint8_t)(uptime_ms >> 8);
    frame[14] = (uint8_t)(uptime_ms >> 16);
    frame[15] = (uint8_t)(uptime_ms >> 24);
    frame[16] = (uint8_t)temperature_centideg;
    frame[17] = (uint8_t)(temperature_centideg >> 8);
    frame[18] = (uint8_t)heater_pwm;
    frame[19] = (uint8_t)(heater_pwm >> 8);
    frame[20] = (uint8_t)4000U;
    frame[21] = (uint8_t)(4000U >> 8);

    memcpy(frame + 22U, &position_kp, sizeof(float));
    memcpy(frame + 26U, &position_ki, sizeof(float));
    memcpy(frame + 30U, &position_kd, sizeof(float));
    memcpy(frame + 34U, &speed_kp, sizeof(float));
    memcpy(frame + 38U, &speed_ki, sizeof(float));
    memcpy(frame + 42U, &speed_kd, sizeof(float));

    crc = Verify_CRC16_Check_Sum(frame + 6U, 40U);
    frame[46] = (uint8_t)crc;
    frame[47] = (uint8_t)(crc >> 8);
    return CDC_Transmit_HS(frame, sizeof(frame));
}

uint8_t Vision_USB_IsBinaryMode(void)
{
    return binary_mode;
}

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
                                      float gyro_bias_z_deg_s)
{
    static uint8_t text[240U];
    const char *calibration_text = (calibration_state == 2U) ? "Ready" :
                                   (calibration_state == 1U) ? "Calibrating" : "Heating";
    const int length = snprintf((char *)text, sizeof(text),
                                "CAL=%s %u%% BiasZ=%+.4f deg/s | Yaw=%+.2f deg | TargetYaw=%+.2f deg | Gyro=%+.2f deg/s | Temp=%.2f C | Heater=%.1f%% | "
                                "PosPID(%.4f,%.6f,%.4f) | SpeedPID(%.4f,%.6f,%.4f)\r\n",
                                calibration_text, calibration_progress, gyro_bias_z_deg_s,
                                yaw_centideg / 100.0f,
                                target_yaw_centideg / 100.0f,
                                (float)omega_centideg_s / 100.0f,
                                temperature_c,
                                (float)heater_pwm / 99.99f,
                                position_kp, position_ki, position_kd,
                                speed_kp, speed_ki, speed_kd);
    if ((length <= 0) || (length >= (int)sizeof(text)))
    {
        return USBD_FAIL;
    }
    return CDC_Transmit_HS(text, (uint16_t)length);
}

uint8_t Vision_USB_TransmitChassisStatus(
    int32_t raw_1, int32_t raw_2, int32_t raw_3, int32_t raw_4,
    int32_t ramp_1, int32_t ramp_2, int32_t ramp_3, int32_t ramp_4,
    int16_t feedback_1, int16_t feedback_2, int16_t feedback_3, int16_t feedback_4,
    float output_1, float output_2, float output_3, float output_4)
{
    static uint8_t text[256U];
    const int length = snprintf(
        (char *)text, sizeof(text),
        "RawTarget=[%ld,%ld,%ld,%ld] | RampTarget=[%ld,%ld,%ld,%ld] | "
        "FeedbackRPM=[%d,%d,%d,%d] | PIDOut=[%.0f,%.0f,%.0f,%.0f]\r\n",
        (long)raw_1, (long)raw_2, (long)raw_3, (long)raw_4,
        (long)ramp_1, (long)ramp_2, (long)ramp_3, (long)ramp_4,
        (int)feedback_1, (int)feedback_2, (int)feedback_3, (int)feedback_4,
        output_1, output_2, output_3, output_4);

    if ((length <= 0) || (length >= (int)sizeof(text)))
    {
        return USBD_FAIL;
    }
    return CDC_Transmit_HS(text, (uint16_t)length);
}
