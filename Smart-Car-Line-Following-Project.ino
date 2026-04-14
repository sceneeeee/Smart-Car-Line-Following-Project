#include "Adafruit_NeoPixel.h"  // 彩色灯珠驱动
#include "comm.h"               // 传感器数据读取
#include "motor.h"              // 电机控制

#define PIN 4
#define NUMPIXELS 2

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// ========== 巡线参数（高速模式） ==========
const int BASE_SPEED = 255;        // 基础前进速度【大幅提高到230】
const int MAX_CORRECTION = 150;    // 最大差速修正【提高到150，允许更大的转向差】
const float KP = 15.0f;            // 比例系数【降低到15，减少过度反应】
const float KD = 140.0f;           // 微分系数【增加到140，提高稳定性】
const float ALPHA = 0.2f;          // 误差低通滤波系数【降低到0.2，强力滤波】
const int SEARCH_TURN_SPEED = 200; // 丢线后原地找线速度【提高到200】
const bool ENABLE_REVERSE_TURN = true;   // 启用内轮倒车转弯
const int REVERSE_THRESHOLD = 40;  // 倒车启用阈值【降低到40，更早启用倒车】

static float filtered_error = 0.0f;
static float last_error = 0.0f;
static int last_turn_direction = 1;

float calc_line_error_and_update_direction(bool &line_seen)
{
    const int values[7] = {
        sensor.ir_left_3,
        sensor.ir_left_2,
        sensor.ir_left_1,
        sensor.ir_mid,
        sensor.ir_right_1,
        sensor.ir_right_2,
        sensor.ir_right_3
    };
    const int weights[7] = {-3, -2, -1, 0, 1, 2, 3};

    int sum = 0;
    int count = 0;
    for (int i = 0; i < 7; ++i)
    {
        if (values[i])
        {
            sum += weights[i];
            ++count;
        }
    }

    line_seen = (count > 0);
    if (!line_seen)
    {
        return last_error;
    }

    float error = float(sum) / float(count);
    if (error > 0.1f)
    {
        last_turn_direction = 1;
    }
    else if (error < -0.1f)
    {
        last_turn_direction = -1;
    }
    return error;
}

void set_led_by_error(float error, bool line_seen)
{
    uint8_t r0 = 0, g0 = 0, b0 = 0;
    uint8_t r1 = 0, g1 = 0, b1 = 0;

    if (!line_seen)
    {
        // 丢线：黄色警告
        r0 = r1 = 120;
        g0 = g1 = 120;
    }
    else if (error > 0.6f)
    {
        // 偏右明显：右灯偏红
        r1 = 120;
        b0 = 40;
    }
    else if (error < -0.6f)
    {
        // 偏左明显：左灯偏红
        r0 = 120;
        b1 = 40;
    }
    else
    {
        // 中线附近：白色
        r0 = g0 = b0 = 80;
        r1 = g1 = b1 = 80;
    }

    pixels.setPixelColor(0, pixels.Color(r0, g0, b0));
    pixels.setPixelColor(1, pixels.Color(r1, g1, b1));
    pixels.show();
}

void setup()
{
    shift_reg_init(); // 传感器初始化
    motor_init();     // 电机初始化
    pixels.begin();   // 彩色灯珠初始化
}

void loop()
{
    reload_shift_reg(); // 刷新传感器数据

    bool line_seen = false;
    float raw_error = calc_line_error_and_update_direction(line_seen);

    // 一阶低通：抑制误差跳变，减少左右抖动
    filtered_error = (1.0f - ALPHA) * filtered_error + ALPHA * raw_error;
    float d_error = filtered_error - last_error;
    last_error = filtered_error;

    int motor_left = 0;
    int motor_right = 0;

    if (line_seen)
    {
        int correction = int(KP * filtered_error + KD * d_error);
        if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
        if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;

        // ===== 高速转弯策略：内轮倒车，保持整体速度高 =====
        if (ENABLE_REVERSE_TURN && abs(correction) > REVERSE_THRESHOLD)
        {
            // 大转弯时，外轮加速，内轮倒车
            if (correction > 0)
            {
                // 向右转：左轮快速前进，右轮倒车
                motor_left = 255;  // 左轮最大速度
                motor_right = -abs(correction);  // 右轮倒车（速度与转向幅度成正比）
            }
            else
            {
                // 向左转：右轮快速前进，左轮倒车
                motor_left = -abs(correction);  // 左轮倒车
                motor_right = 255;  // 右轮最大速度
            }
        }
        else
        {
            // 小转弯时，使用差速前进（两轮都前进）
            motor_left = BASE_SPEED + correction;
            motor_right = BASE_SPEED - correction;

            // 如果任一轮超过255，缩放以保持转向比例
            if (motor_left > 255 || motor_right > 255)
            {
                float scale = 255.0f / max(motor_left, motor_right);
                motor_left = (int)(motor_left * scale);
                motor_right = (int)(motor_right * scale);
            }

            // 防止倒车
            if (motor_left < 0) motor_left = 0;
            if (motor_right < 0) motor_right = 0;
        }

        // 最终限制
        if (motor_left > 255) motor_left = 255;
        if (motor_left < -255) motor_left = -255;
        if (motor_right > 255) motor_right = 255;
        if (motor_right < -255) motor_right = -255;
    }
    else
    {
        // 丢线策略：高速找线
        if (last_turn_direction > 0)
        {
            motor_left = SEARCH_TURN_SPEED;
            motor_right = -50;  // 右轮倒车找线
        }
        else
        {
            motor_left = -50;  // 左轮倒车找线
            motor_right = SEARCH_TURN_SPEED;
        }
    }

    set_led_by_error(filtered_error, line_seen);
    motor_set_PWM(motor_left, motor_right);
    delay(15);
}
