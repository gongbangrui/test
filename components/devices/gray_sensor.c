/**
 * @file gray_sensor.c
 * @brief Dual 8-channel grayscale sensor driver with merged 16-channel output.
 */

#include "gray_sensor.h"
#include <string.h>
#include "cmsis_os.h"
#include "debug_console.h"

/* ---- 内部静态数据 ---- */
static gray_sensor_t g_sensor_left  = {0};
static gray_sensor_t g_sensor_right = {0};
static gray_sensor_t g_sensor_merged = {0};   // 用于返回合并后的16通道数据

/* ---- 辅助函数 ---- */

static uint8_t count_bits(uint8_t x) {
    uint8_t cnt = 0;
    while (x) { cnt++; x &= (x - 1); }
    return cnt;
}

/* ---- 底层通道读取（复用之前的） ---- */

static void set_channel_gpio(GPIO_TypeDef *port_a0, uint16_t pin_a0,
                             GPIO_TypeDef *port_a1, uint16_t pin_a1,
                             GPIO_TypeDef *port_a2, uint16_t pin_a2,
                             uint8_t ch) {
    HAL_GPIO_WritePin(port_a0, pin_a0, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(port_a1, pin_a1, (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(port_a2, pin_a2, (ch & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    for (volatile uint32_t i = 0; i < 200; i++);
}

static uint8_t read_single_sensor(GPIO_TypeDef *out_port, uint16_t out_pin,
                                  GPIO_TypeDef *a0_port, uint16_t a0_pin,
                                  GPIO_TypeDef *a1_port, uint16_t a1_pin,
                                  GPIO_TypeDef *a2_port, uint16_t a2_pin) {
    uint8_t mask = 0;
    for (uint8_t ch = 0; ch < 8; ch++) {
        set_channel_gpio(a0_port, a0_pin, a1_port, a1_pin, a2_port, a2_pin, ch);
        if (HAL_GPIO_ReadPin(out_port, out_pin) == GRAY_DETECT_LEVEL) {
            mask |= (1 << ch);
        }
    }
    return mask;
}

/* ---- 解析单个8位掩码（填充左右结构体） ---- */
static void parse_single_mask(gray_sensor_t *sensor, uint8_t mask) {
    sensor->bitmask = mask;                         // 低8位有效
    sensor->line_count = count_bits(mask);
    sensor->online = (mask != 0) ? 1 : 0;
    sensor->frame_ready = 1;
    // 对于独立传感器，我们不计算详细位置，只存位图
}

/* ---- 左右传感器初始化（不变） ---- */

void gray_sensor_left_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_InitStruct.Pin = GRAY1_OUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GRAY1_OUT_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GRAY1_0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GRAY1_0_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GRAY1_1_Pin;
    HAL_GPIO_Init(GRAY1_1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GRAY1_2_Pin;
    HAL_GPIO_Init(GRAY1_2_GPIO_Port, &GPIO_InitStruct);

    memset(&g_sensor_left, 0, sizeof(g_sensor_left));
}

void gray_sensor_right_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

    GPIO_InitStruct.Pin = GRAY2_OUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GRAY2_OUT_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GRAY2_0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GRAY2_0_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GRAY2_1_Pin;
    HAL_GPIO_Init(GRAY2_1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GRAY2_2_Pin;
    HAL_GPIO_Init(GRAY2_2_GPIO_Port, &GPIO_InitStruct);

    memset(&g_sensor_right, 0, sizeof(g_sensor_right));
}

/* ---- 独立轮询（左右分别） ---- */

uint8_t gray_sensor_left_poll(void) {
    uint8_t mask = read_single_sensor(GRAY1_OUT_GPIO_Port, GRAY1_OUT_Pin,
                                      GRAY1_0_GPIO_Port, GRAY1_0_Pin,
                                      GRAY1_1_GPIO_Port, GRAY1_1_Pin,
                                      GRAY1_2_GPIO_Port, GRAY1_2_Pin);
    parse_single_mask(&g_sensor_left, mask);
    return g_sensor_left.online;
}

uint8_t gray_sensor_right_poll(void) {
    uint8_t mask = read_single_sensor(GRAY2_OUT_GPIO_Port, GRAY2_OUT_Pin,
                                      GRAY2_0_GPIO_Port, GRAY2_0_Pin,
                                      GRAY2_1_GPIO_Port, GRAY2_1_Pin,
                                      GRAY2_2_GPIO_Port, GRAY2_2_Pin);
    parse_single_mask(&g_sensor_right, mask);
    return g_sensor_right.online;
}

const gray_sensor_t* get_gray_sensor_left(void) {
    return &g_sensor_left;
}

const gray_sensor_t* get_gray_sensor_right(void) {
    return &g_sensor_right;
}

/* ---- 兼容旧接口：合并左右数据为16通道 ---- */

void gray_sensor_init(uint8_t id) {
    (void)id;
    gray_sensor_left_init();
    gray_sensor_right_init();
    memset(&g_sensor_merged, 0, sizeof(g_sensor_merged));
}

uint8_t gray_sensor_poll(void) {
    // 轮询左右传感器
    uint8_t left_online = gray_sensor_left_poll();
    uint8_t right_online = gray_sensor_right_poll();

    uint8_t left_mask  = g_sensor_left.bitmask;   // 低8位
    uint8_t right_mask = g_sensor_right.bitmask;  // 低8位，要移到高8位

    // 合并为16位位图：低8位 = 左前，高8位 = 右前
    uint16_t combined = right_mask | ((uint16_t)left_mask << 8);
    uint16_t combined_1 = left_mask | ((uint16_t)right_mask << 8);
    // 计算总活跃通道数
    uint8_t total_line_count = count_bits(left_mask) + count_bits(right_mask);

    // 计算基于16通道的重心位置（0~15），中心索引为7.5
    float position = 0.0f;
    if (total_line_count > 0) {
        int32_t sum = 0, cnt = 0;
        for (int i = 0; i < 16; i++) {
            if (combined_1 & (1 << i)) {
                sum += i;
                cnt++;
            }
        }
        float center = (float)sum / (float)cnt;        // 0~15
        position = (center - 7.5f) / 7.5f;              // 映射到 [-1, +1]
        if (position > 1.0f) position = 1.0f;
        if (position < -1.0f) position = -1.0f;
    }

    // 填充合并后的数据
    g_sensor_merged.bitmask = combined;
    g_sensor_merged.line_count = total_line_count;
    g_sensor_merged.position = position;
    g_sensor_merged.offset_raw = (int16_t)(position * 1000.0f);
    g_sensor_merged.online = (left_online || right_online) ? 1 : 0;
    g_sensor_merged.frame_ready = 1;

    // 计算 out_line（出界标志）：基于16通道的最左/最右
    if (combined == 0) {
        g_sensor_merged.out_line = 2;          // 无线
    } else if (combined & 0x0001) {
        g_sensor_merged.out_line = 0;          // 左出界（最左边通道0）
    } else if (combined & 0x8000) {
        g_sensor_merged.out_line = 1;          // 右出界（最右边通道15）
    } else {
        g_sensor_merged.out_line = 2;          // 内部
    }

    return 1;
}

const gray_sensor_t* get_gray_sensor_point(void) {
    return &g_sensor_merged;
}