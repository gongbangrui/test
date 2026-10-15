/**
 * @file gray_sensor.h
 * @brief Dual 8-channel grayscale sensors (CD4051) driver.
 *        Supports left-front and right-front sensors.
 */

#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ---------- 硬件引脚定义 ---------- */

/* 左前传感器 (Left) */
#define GRAY1_OUT_Pin       GPIO_PIN_5
#define GRAY1_OUT_GPIO_Port      GPIOE

#define GRAY1_0_Pin        GPIO_PIN_9
#define GRAY1_0_GPIO_Port       GPIOH
#define GRAY1_1_Pin        GPIO_PIN_0
#define GRAY1_1_GPIO_Port       GPIOE
#define GRAY1_2_Pin        GPIO_PIN_4
#define GRAY1_2_GPIO_Port       GPIOE

/* 右前传感器 (Right) */
#define GRAY2_OUT_Pin      GPIO_PIN_6
#define GRAY2_OUT_GPIO_Port     GPIOE

#define GRAY2_0_Pin       GPIO_PIN_7
#define GRAY2_0_GPIO_Port      GPIOC
#define GRAY2_1_Pin       GPIO_PIN_1
#define GRAY2_1_GPIO_Port      GPIOE
#define GRAY2_2_Pin       GPIO_PIN_4
#define GRAY2_2_GPIO_Port      GPIOI

/* ---------- 通用参数 ---------- */
#define GRAY_CHANNEL_COUNT      8
#define GRAY_CENTER_INDEX       3.5f
#define GRAY_NORMALIZE_SCALE    3.5f
#define GRAY_OFFSET_SCALE       1000.0f

/* 电平有效值：黑线为低电平（白底高电平） */
#define GRAY_DETECT_LEVEL       GPIO_PIN_RESET

/* ---------- 数据结构 ---------- */
typedef struct {
    uint16_t bitmask;       // 位图（低8位有效）
    uint8_t  line_count;    // 激活通道数
    uint8_t  out_line;      // 出界标志（0=左出，1=右出，2=无线）
    int16_t  offset_raw;    // 原始偏移
    float    position;      // 归一化位置 [-1, +1]
    uint8_t  online;        // 1=在线
    uint8_t  frame_ready;   // 新数据就绪
} gray_sensor_t;

/* ---------- API 函数 ---------- */

/* 左前传感器 */
void gray_sensor_left_init(void);
uint8_t gray_sensor_left_poll(void);
const gray_sensor_t* get_gray_sensor_left(void);

/* 右前传感器 */
void gray_sensor_right_init(void);
uint8_t gray_sensor_right_poll(void);
const gray_sensor_t* get_gray_sensor_right(void);

uint8_t gray_sensor_poll(void);                // 返回在线状态（0/1）
const gray_sensor_t* get_gray_sensor_point(void);
/* 为了兼容旧代码，保留原函数名（可选，可删除） */
// 但为了不冲突，建议删除或注释掉
// void gray_sensor_init(uint8_t id);
// uint8_t gray_sensor_poll(void);
// const gray_sensor_t* get_gray_sensor_point(void);

#endif