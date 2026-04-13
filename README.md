# Smart-Car-Line-Following-Project (智能寻迹小车)

这是一个基于 Arduino 的智能寻迹小车项目。小车通过红外传感器（IR Sensor）阵列来检测黑线，并根据传感器的反馈实时调整左右电机的 PWM 速度，从而实现平滑的自动寻线行驶。同时，小车配备了 WS2812 (NeoPixel) 彩色 LED 灯珠，可以通过不同颜色的灯光来指示当前触发的传感器位置和转向状态。

## 功能特点
- **多通道红外寻迹**：支持左、右各三个以及中间一个红外传感器，检测精度更高。
- **动态电机差速控制**：根据偏离中心线的程度，自适应调节左右轮 PWM 实现转向。
- **状态指示灯**：集成 Adafruit NeoPixel 驱动，根据小车转向和传感器触发状态，实时显示不同颜色的 RGB 氛围灯。

## 文件结构
- `xunji.ino`：Arduino 主程序，包含 `setup()` 初始化和 `loop()` 核心寻迹逻辑。
- `motor.cpp` / `motor.h`：底层电机 PWM 控制的封装代码。
- `comm.cpp` / `comm.h`：传感器状态读取与移位寄存器相关功能代码。
- `Adafruit_NeoPixel.cpp` / `Adafruit_NeoPixel.h`：RGB 彩色灯珠驱动库。

## 硬件依赖
- Arduino 主控板 (如 UNO / Nano)
- 直流电机及对应的电机驱动模块
- 红外寻迹传感器模块阵列
- WS2812 RGB LED 灯珠

## 快速上手
1. 按照接线定义连接好电机驱动、红外传感器阵列以及 RGB 灯珠（灯珠默认控制引脚为 Pin 4）。
2. 使用 Arduino IDE 打开 `xunji.ino` 文件。
3. 将代码编译并下载到 Arduino 主板中。
4. 将小车放置在带有黑线的浅色场地上，小车将自动开始寻迹运行。
