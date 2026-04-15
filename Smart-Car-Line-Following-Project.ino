#include "Adafruit_NeoPixel.h"  // 彩色灯珠驱动
#include "comm.h"               // 传感器数据读取
#include "motor.h"              // 电机控制

#define PIN 4
#define NUMPIXELS 2

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// ========== 巡线参数（高速模式） ==========
const int BASE_SPEED = 235;              // 默认巡航速度
const int STRAIGHT_SPEED = 255;          // 长直道目标速度
const int MIN_BASE_SPEED = 170;          // 大弯时最低基础速度（减速换平滑）
const int MAX_CORRECTION = 210;          // 最大转向差速
const float KP = 24.0f;                  // 比例系数
const float KD = 90.0f;                  // 微分系数
const float ERROR_ALPHA = 0.32f;         // 误差低通滤波
const float CORRECTION_ALPHA = 0.28f;    // 输出低通滤波（抑制“打舵”感）
const int MAX_CORRECTION_STEP = 24;      // 每周期最大转向变化，防止突变
const int TURN_BLEND_START = 70;         // 开始混入内轮倒车的转向阈值
const int TURN_BLEND_FULL = 160;         // 完全进入大弯模式阈值
const int MAX_REVERSE_SPEED = 130;       // 内轮最大倒车速度（限制突兀感）
const int SEARCH_TURN_SPEED = 170;       // 丢线后找线速度
const int LOST_LINE_REVERSE = 45;        // 丢线时内轮轻微倒车
const float STRAIGHT_ERR_WINDOW = 0.18f; // 直道判定误差阈值
const float STRAIGHT_DERR_WINDOW = 0.08f;// 直道判定误差变化阈值
const int STRAIGHT_BUILDUP_FRAMES = 6;   // 连续稳定后再拉到255，避免突然窜车

static float filtered_error = 0.0f;
static float last_error = 0.0f;
static int last_turn_direction = 1;
static float filtered_correction = 0.0f;
static int last_motor_left = 0;
static int last_motor_right = 0;
static int straight_frames = 0;

int clamp_int(int value, int low, int high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

float clamp_float(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

int slew_limit(int target, int last, int step)
{
    if (target > last + step) return last + step;
    if (target < last - step) return last - step;
    return target;
}

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
    filtered_error = (1.0f - ERROR_ALPHA) * filtered_error + ERROR_ALPHA * raw_error;
    float d_error = filtered_error - last_error;
    last_error = filtered_error;

    int motor_left = 0;
    int motor_right = 0;

    if (line_seen)
    {
        // PD 输出 + 双重滤波，避免“打一把、回一把”的顿挫
        float raw_correction = KP * filtered_error + KD * d_error;
        raw_correction = clamp_float(raw_correction, -MAX_CORRECTION, MAX_CORRECTION);
        filtered_correction = (1.0f - CORRECTION_ALPHA) * filtered_correction + CORRECTION_ALPHA * raw_correction;
        int correction = (int)filtered_correction;

        int abs_correction = abs(correction);
        float corner_ratio = clamp_float((float)abs_correction / (float)MAX_CORRECTION, 0.0f, 1.0f);

        // 直道稳定时逐步提升到255；弯道时再按曲率回落速度
        bool straight_stable = (abs(filtered_error) < STRAIGHT_ERR_WINDOW) && (abs(d_error) < STRAIGHT_DERR_WINDOW);
        if (straight_stable)
        {
            if (straight_frames < STRAIGHT_BUILDUP_FRAMES) ++straight_frames;
        }
        else
        {
            straight_frames = 0;
        }

        float straight_gain = (float)straight_frames / (float)STRAIGHT_BUILDUP_FRAMES;
        straight_gain = clamp_float(straight_gain, 0.0f, 1.0f);
        int cruise_base = BASE_SPEED + (int)((STRAIGHT_SPEED - BASE_SPEED) * straight_gain);

        // 转向越大，基础速度越低，给控制器更多余量，减少大弯抖动
        int adaptive_base = cruise_base - (int)((cruise_base - MIN_BASE_SPEED) * corner_ratio);

        // 前进差速（适合中小弯）
        int forward_left = adaptive_base + correction;
        int forward_right = adaptive_base - correction;
        forward_left = clamp_int(forward_left, 0, 255);
        forward_right = clamp_int(forward_right, 0, 255);

        // 大弯时逐步混入内轮倒车，替代“阈值触发”的突变
        float blend = (abs_correction - TURN_BLEND_START) / (float)(TURN_BLEND_FULL - TURN_BLEND_START);
        blend = clamp_float(blend, 0.0f, 1.0f);

        int target_left = forward_left;
        int target_right = forward_right;
        if (blend > 0.0f)
        {
            int pivot_outer = clamp_int(adaptive_base + abs_correction, 0, 255);
            int pivot_inner = -(int)(MAX_REVERSE_SPEED * blend);

            if (correction > 0)
            {
                // 向右转：左外轮快进，右内轮渐进倒车
                target_left = (int)((1.0f - blend) * forward_left + blend * pivot_outer);
                target_right = (int)((1.0f - blend) * forward_right + blend * pivot_inner);
            }
            else
            {
                // 向左转：右外轮快进，左内轮渐进倒车
                target_left = (int)((1.0f - blend) * forward_left + blend * pivot_inner);
                target_right = (int)((1.0f - blend) * forward_right + blend * pivot_outer);
            }
        }

        motor_left = clamp_int(target_left, -255, 255);
        motor_right = clamp_int(target_right, -255, 255);
    }
    else
    {
        straight_frames = 0;

        // 丢线策略：沿上一次方向平滑找线，避免剧烈抽动
        if (last_turn_direction > 0)
        {
            motor_left = SEARCH_TURN_SPEED;
            motor_right = -LOST_LINE_REVERSE;
        }
        else
        {
            motor_left = -LOST_LINE_REVERSE;
            motor_right = SEARCH_TURN_SPEED;
        }
    }

    // 输出限斜率，进一步抑制电机瞬时反向导致的“顿一下”
    motor_left = slew_limit(motor_left, last_motor_left, MAX_CORRECTION_STEP);
    motor_right = slew_limit(motor_right, last_motor_right, MAX_CORRECTION_STEP);
    last_motor_left = motor_left;
    last_motor_right = motor_right;

    set_led_by_error(filtered_error, line_seen);
    motor_set_PWM(motor_left, motor_right);
    delay(15);
}
