#include "Adafruit_NeoPixel.h"  // 彩色灯珠驱动
#include "comm.h"               // 传感器数据读取
#include "motor.h"              // 电机控制

#define PIN 4
#define NUMPIXELS 2

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// ========== 巡线参数（先调这里） ==========
const int BASE_SPEED = 190;      // 基础前进速度（0~255）
const int MAX_CORRECTION = 140;  // 最大差速修正
const float KP = 40.0f;          // 比例系数
const float KD = 95.0f;          // 微分系数
const float ALPHA = 0.35f;       // 误差低通滤波系数（越小越平滑）
const int SEARCH_TURN_SPEED = 135; // 丢线后原地找线速度

static float filtered_error = 0.0f;
static float last_error = 0.0f;
static int last_turn_direction = 1; // -1=上次向左，1=上次向右

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

        // 右偏(error>0)时减小左轮、增大右轮实际会反向；
        // 当前电机接线下采用下面映射：右偏 -> 左快右慢。
        motor_left = BASE_SPEED + correction;
        motor_right = BASE_SPEED - correction;

        if (motor_left > 255) motor_left = 255;
        if (motor_left < 0) motor_left = 0;
        if (motor_right > 255) motor_right = 255;
        if (motor_right < 0) motor_right = 0;
    }
    else
    {
        // 丢线策略：沿上一次偏转方向原地小幅找线
        if (last_turn_direction > 0)
        {
            motor_left = SEARCH_TURN_SPEED;
            motor_right = 0;
        }
        else
        {
            motor_left = 0;
            motor_right = SEARCH_TURN_SPEED;
        }
    }

    set_led_by_error(filtered_error, line_seen);
    motor_set_PWM(motor_left, motor_right);
    delay(15);
}
