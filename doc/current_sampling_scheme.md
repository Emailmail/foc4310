# Nebula FOC 电流采样方案详细说明

> 目标读者：另一个 AI / 开发者，需要完整理解本项目的相电流采样硬件设计、ADC 配置、软件处理流程和 FOC 控制链。

---

## 目录

1. [硬件拓扑](#1-硬件拓扑)
2. [INA240 电流采样放大器](#2-ina240-电流采样放大器)
3. [采样电阻与量程计算](#3-采样电阻与量程计算)
4. [ADC 配置](#4-adc-配置)
5. [TIM1 PWM 与 ADC 触发时序](#5-tim1-pwm-与-adc-触发时序)
6. [ADC 中断回调与数据处理流程](#6-adc-中断回调与数据处理流程)
7. [偏置校准](#7-偏置校准)
8. [电流换算与滤波](#8-电流换算与滤波)
9. [FOC 控制链（在 ADC 中断上下文中执行）](#9-foc-控制链在-adc-中断上下文中执行)
10. [关键结构体与参数](#10-关键结构体与参数)
11. [总结：一条完整的采样路径](#11-总结一条完整的采样路径)
12. [附录：关键文件索引](#12-附录关键文件索引)

---

## 1. 硬件拓扑

### 1.1 整体架构

```
┌──────────────────────────────────────────────────────┐
│  STM32G431CBUx (Cortex-M4, 170MHz)                  │
│                                                      │
│  ADC1 ──── PA2 (CH3) ◄── INA240 U16 OUT ── U相采样  │
│      └──── PA3 (CH4) ◄── INA240 U17 OUT ── W相采样  │
│                                                      │
│  TIM1 ──── CH1/CH1N → U相上下桥 PWM                  │
│      ├──── CH2/CH2N → V相上下桥 PWM                  │
│      ├──── CH3/CH3N → W相上下桥 PWM                  │
│      └──── CH4 (内部) → ADC1 注入触发                 │
│                                                      │
│  SPI1 ──── AS5047P 磁编码器 (14-bit)                 │
└──────────────────────────────────────────────────────┘
```

### 1.2 双电阻采样（Two-Shunt）

本项目采用**双电阻**电流采样方案：

- **U 相**：采样电阻 U21 (5 mΩ) → INA240 U16 → ADC1_IN3 (PA2)
- **W 相**：采样电阻 U22 (5 mΩ) → INA240 U17 → ADC1_IN4 (PA3)
- **V 相**：不直接测量，通过基尔霍夫电流定律推导：`Ic = -(Ia + Ib)`

> **为什么选双电阻？** 对于星形连接的三相无刷电机（无中性线），`Ia + Ib + Ic = 0` 恒成立，只需测量两相即可得到第三相。相比三电阻方案节省一个放大器和一个 ADC 通道；相比单电阻方案，不需要复杂的 PWM 移相来在不同时刻采样不同相电流。

### 1.3 PCB 布局要点

- 采样电阻位于 MOSFET 半桥的下桥臂源极与 GND 之间（low-side sensing）
- INA240 尽量靠近采样电阻放置，走线使用开尔文连接（Kelvin connection）减少寄生电阻
- ADC 信号线远离 PWM 大电流走线

---

## 2. INA240 电流采样放大器

### 2.1 芯片选型

| 参数 | 值 |
|------|-----|
| 型号 | **INA240A2**（READNE.md 声明） |
| 增益 | **50 V/V** |
| 供电电压 | **3.3V**（VCC3V3） |
| 封装 | TSSOP-8 (PW) |
| 架构 | 零漂移（zero-drift），双向，增强型 PWM 抑制 |

> **为什么选 INA240？** INA240 专为电机驱动电流采样设计，具有增强的 PWM 共模抑制能力（enhanced PWM rejection），能在 PWM 开关产生的剧烈共模电压跳变（-4V ~ +80V）下稳定工作。零漂移架构使输入失调电压低至 25μV（典型值），温漂仅 250 nV/°C。

### 2.2 引脚连接

```
INA240 U16/U17 (TSSOP-8):
        ┌─────┐
   NC ──┤1   8├── OUT  → PA2/PA3 (ADC1_IN3/IN4)
  IN+ ──┤2   7├── REF1 → VCC3V3 (3.3V)
  IN- ──┤3   6├── REF2 → GND
  GND ──┤4   5├── VS   → VCC3V3 (3.3V)
        └─────┘
```

### 2.3 基准电压配置（双向电流检测）

**REF1 = VCC3V3 (3.3V), REF2 = GND**

根据 INA240 数据手册 8.4.3.2 节：
> "By connecting one reference pin to VS and the other to the GND pin, the output is set at half of the supply when there is no differential input."

此配置下零电流输出电压为 **Vs/2 = 1.65V**，可实现**双向电流检测**：

```
Vout = Vref + I × Rshunt × Gain
     = 1.65V + I × 0.005Ω × 50
     = 1.65V + I × 0.25V/A
```

- 正向电流（电机驱动）：Vout > 1.65V
- 反向电流（再生制动）：Vout < 1.65V
- 零电流：Vout ≈ 1.65V ≈ ADC 读数 2048（12-bit 中值）

---

## 3. 采样电阻与量程计算

### 3.1 参数

| 参数 | 值 |
|------|-----|
| 采样电阻 Rshunt | **5 mΩ** (0.005 Ω) |
| 放大器增益 G | **50 V/V** (INA240A2) |
| INA240 输出电压范围 | 0V ~ 3.3V（由 3.3V 供电限制） |
| 零电流输出电压 Vref | 1.65V（Vs/2） |

### 3.2 理论量程

```
I_max_pos = (3.3V - 1.65V) / (0.005Ω × 50) = 1.65V / 0.25 = +6.6 A
I_max_neg = (0V - 1.65V) / (0.005Ω × 50)   = -1.65V / 0.25 = -6.6 A
满量程范围: ±6.6 A
```

### 3.3 ADC 分辨率

```
ADC 参考电压 VREF+  = VDDA = 3.3V
ADC 分辨率          = 12-bit (0 ~ 4095)
ADC 每 LSB 电压     = 3.3V / 4096 = 0.8057 mV

电流每 ADC count = V_LSB / (Rshunt × Gain)
                  = 0.8057 mV / 0.25 Ω
                  = 3.22 mA / count
```

### 3.4 注意事项：固件换算因子

固件中实际使用的换算因子为：

```c
// AuroFOC.c:41-42
float Uu_ = ((float)((int)uu - (int)uu_offset)) * 0.001611328f;
```

其中 `0.001611328 = 3.3V / 2048`，而非 `3.3V / 4096`。

这与 INA240A3（增益=100V/V）的理论值完全吻合：

```
INA240A3 (G=100): I_per_count = 3.3 / (4096 × 0.005 × 100) = 3.3/2048 = 0.001611 A/count ✓
```

而 INA240A2 (G=50) 的理论值应为 `3.3/1024 = 0.003223 A/count`。

此外，IIR 滤波器具有 4 倍的 DC 增益——这使得 `foc->Ia/Ib/Ic` 中的数值**不是严格的安培值**，而是在一个缩放后的内部控制单位中。由于 FOC 是闭环控制系统，只要内部一致性保持，绝对单位的精度不影响控制性能——PID 增益已针对此缩放进行了调谐。

---

## 4. ADC 配置

### 4.1 ADC1 初始化参数

文件：[Core/Src/adc.c](Core/Src/adc.c)

| 参数 | 值 | 说明 |
|------|-----|------|
| 实例 | ADC1 | 主 ADC，用于 FOC |
| 时钟预分频 | SYSCLK / 4 | 170MHz / 4 = **42.5 MHz** |
| 分辨率 | 12-bit | 0 ~ 4095 |
| 数据对齐 | 右对齐 | |
| 扫描模式 | 使能 | 注入组需要 |
| 连续转换 | 禁用 | 由外部触发控制 |
| **注入触发源** | **TIM1 CC4** | 下降沿触发 |
| 注入通道数 | 2 | |
| 注入序列 Rank 1 | **CH3 (PA2)** | U 相电流，**采样时间 12.5 周期** |
| 注入序列 Rank 2 | **CH4 (PA3)** | W 相电流，**采样时间 12.5 周期** |
| 常规通道 | CH3 (PA2)，6.5 周期 | 配置了但未使用 |
| 过采样 | 禁用 | |
| DMA | 禁用 | |
| 硬件偏置校准 | 未使用 | 用软件实现了偏置校准 |

### 4.2 ADC 时序

```
ADC 时钟      = 42.5 MHz
采样时间      = 12.5 ADC 时钟周期 = 294 ns
转换时间      = 12.5 ADC 时钟周期（12-bit）= 294 ns
单通道总时间   = 25 周期 = 588 ns
两通道总时间   = 2 × 588 ns ≈ 1.18 μs（注入序列全程）
```

### 4.3 ADC 中断配置

| 参数 | 值 |
|------|-----|
| 中断向量 | ADC1_2_IRQn |
| 优先级 | 0（最高） |
| 子优先级 | 0 |
| 触发条件 | JEOC（注入转换完成） |

### 4.4 为什么不使用 ADC 硬件偏置功能

STM32G4 的 ADC 支持硬件偏置补偿（offset calibration），但本项目选择软件方案：

1. **软件偏置校准**：采集前 500 个样本求平均，作为零电流偏置
2. 优点：软件方案可以补偿**整个信号链**的偏置（INA240 输出失调 + ADC 自身失调），而不仅仅是 ADC 的偏置

---

## 5. TIM1 PWM 与 ADC 触发时序

### 5.1 TIM1 PWM 配置

文件：[Core/Src/tim.c](Core/Src/tim.c)

| 参数 | 值 |
|------|-----|
| 定时器 | TIM1（高级定时器） |
| 时钟源 | APB2 Timer Clock = **170 MHz** |
| 预分频器 | 0（不分频） |
| **计数模式** | **中心对齐模式 1** (Center-Aligned Mode 1) |
| ARR（自动重装值） | **4249** |
| 死区时间 | 50 ticks ≈ **294 ns** |
| CH1/CH1N | U 相上下桥，PWM1 模式 |
| CH2/CH2N | V 相上下桥，PWM1 模式 |
| CH3/CH3N | W 相上下桥，PWM1 模式 |
| **CH4** | **ADC 触发**，PWM1 模式，CCR4=4240，**无输出引脚** |

### 5.2 PWM 频率

```
计数器周期 = 2 × (ARR + 1) = 2 × 4250 = 8500 个 tick
PWM 频率   = 170 MHz / 8500 = 20 kHz
PWM 周期   = 50 μs
```

### 5.3 ADC 触发时刻

**为什么用中心对齐 PWM + CH4 触发？**

在中心对齐模式下，定时器计数器（CNT）先递增再递减：

```
CNT
4249 ┤     ╱ ╲         ← CH4=4240 处下降沿触发 ADC
     │    ╱   ╲
     │   ╱     ╲
     │  ╱       ╲
     │ ╱         ╲
    0 ┤╱           ╲
     └──────────────→ 时间
      ← 50μs PWM周期 →
```

**TIM1 CH4 配置（CCR4=4240, ARR=4249）：**
- CNT < 4240 时 CH4 输出高电平
- CNT ≥ 4240 时 CH4 输出低电平

**下降沿发生在：**
1. **递增阶段**：CNT 从 4239→4240，CH4 从高变低
2. **递减阶段**：CNT 从 4241→4240，CH4 从高变低

ADC 注入组在 **CH4 下降沿触发**，每个 PWM 周期触发 **2 次**。

```
ADC 采样频率 = 2 × PWM 频率 = 40 kHz
采样间隔    = 25 μs（两次触发之间的时间，在对称中心对齐模式下基本均匀）
```

### 5.4 采样点在 PWM 周期中的位置

CH4=4240 非常接近 ARR=4249（计数器峰值），采样发生在 PWM 开关接近其最大导通范围的时刻。在这个时刻：

- **至少有一个下桥臂 MOSFET 正在导通**（对于非 100% 占空比的情况）
- 相电流流过该下桥臂的采样电阻
- **开关瞬态已经消逝**（经过死区时间和 MOSFET 开关时间后电流已稳定）

这符合 FOC 电流采样的最佳实践：**在 PWM 载波峰值处采样，避开开关噪声**。

### 5.5 关键时序图

```
                      PWM 周期 50μs
              ├─────────────────────────────────┤
CNT           :    /╲        ╱╲        ╱
4249 ─────────:───/──╲──────/──╲──────/────────
              :  /    ╲    /    ╲    /
4240 ─────TRG:─/──────╲──/──────╲──/─────────── ← CH4 下降沿，触发 ADC
              :╱        ╲╱        ╲╱
0    ─────────:────────────────────────────────
              :                                 
U相上桥  ─────:████░░░░░░░░░░░░░░████░░░░░░░░░░░
U相下桥  ─────:░░░░██████████████░░░░███████████
              :                                 
U相电流  ─────:──────╱╲──────────────╱╲────────
              :    ╱    ╲          ╱    ╲
              :  ╱      ╲        ╱      ╲
              :╱        ╲██████╱        ╲██████
              :         ^                  ^
              :      采样点1            采样点2
              :   (CNT=4240 ↑)      (CNT=4240 ↓)
```

**图例：**
- `████` = MOSFET 导通
- `░░░░` = MOSFET 关断
- `TRG` = ADC 触发点
- `^` = 电流采样时刻

---

## 6. ADC 中断回调与数据处理流程

### 6.1 完整调用链

```
硬件层:
  TIM1 CH4 下降沿 (CNT=4240, 每 PWM 周期 2 次)
    │
    ▼
  ADC1 注入序列自动开始:
    Rank 1: ADC1_IN3 (PA2, U相) → 转换结果存入 JDR1
    Rank 2: ADC1_IN4 (PA3, W相) → 转换结果存入 JDR2
    │
    ▼
  ADC1_2_IRQHandler()                         [stm32g4xx_it.c]
    │
    ▼
  HAL_ADC_IRQHandler(&hadc1)                  [HAL 库]
    处理 JEOC (注入转换完成) 标志
    │
    ▼
  HAL_ADCEx_InjectedConvCpltCallback(&hadc1)   [MotorEvent.c:53]
    │
    ├─► foc_update_adc(&motorA, JDR1, JDR2, 0) [AuroFOC.c:28]
    │     ├─ 偏置校准 (前500样本)
    │     ├─ ADC raw → 电流换算
    │     ├─ IIR 低通滤波
    │     └─ 推导 Ic = -(Ia + Ib)
    │
    └─► foc_control(&motorA)                   [AuroFOC.c:56]
          ├─ _foc_get_angle()   读编码器角度
          ├─ _foc_cal_sincos()  计算 sin/cos
          ├─ _foc_clark()       Clarke 变换
          ├─ _foc_park()        Park 变换
          ├─ PI 电流环          Id/Iq → Ud/Uq
          ├─ _foc_ipark()       逆 Park 变换
          ├─ _foc_svpwm()       空间矢量调制
          └─ _foc_update_pwm()  更新 TIM1 CCR1/2/3
```

### 6.2 ISR 代码（MotorEvent.c）

```c
// MotorEvent.c:53-57
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc == &hadc1) {
        foc_update_adc(&motorA, hadc1.Instance->JDR1, hadc1.Instance->JDR2, 0);
        foc_control(&motorA);
    }
}
```

**关键事实：**
- 整个 FOC 控制链都在 ADC 中断上下文中执行（裸机，无 RTOS）
- 中断优先级为 0（最高），不会被其他中断打断
- 从 ADC 转换完成到 PWM 占空比更新，全程在一个 ISR 中完成
- `main()` 中的 `while(1)` 是空的——所有实时逻辑都在中断中

### 6.3 中断嵌套与优先级

```
中断优先级分配:
  0 (最高) : ADC1_2_IRQn      — FOC 主循环，不可被抢占
  0        : TIM1_UP_TIM16_IRQn — PWM 更新中断（已使能但未绑定用户回调）
  1        : TIM6_DAC_IRQn      — 1ms 遥测定时器
  2        : USART1_IRQn        — 主机协议接收
```

---

## 7. 偏置校准

### 7.1 校准过程

```c
// AuroFOC.c:28-39
static uint32_t u_time     = 0;   // 采样计数
static uint32_t uu_sum     = 0;   // U 相累加和
static uint32_t uv_sum     = 0;   // W 相累加和
static uint32_t uu_offset  = 0;   // U 相偏置值
static uint32_t uv_offset  = 0;   // W 相偏置值

void foc_update_adc(AuroFOC *foc, uint32_t uu, uint32_t uv, uint32_t uw) {
    if (u_time < 500) {
        uu_sum += uu;
        uv_sum += uv;
        u_time += 1;
    } else if (u_time == 500) {
        uu_offset = uu_sum / 500;   // 算术平均
        uv_offset = uv_sum / 500;
        u_time += 1;                // 确保只执行一次
    }
    // ... 后续使用 uu_offset / uv_offset
}
```

### 7.2 时序与前提条件

```
校准时长 = 500 样本 / 40 kHz = 12.5 ms
```

**关键前提条件：**
1. 校准期间**电机必须处于静止状态**（零相电流）
2. `foc_start_adc()` 在 `MotorEvent_Start()` 中被调用
3. 此时 `focmode = stop_loop_mode`（输出全关），所以电机确实静止
4. 零电流时，INA240 输出电压 = Vref = 1.65V，对应 ADC 读数 ≈ 2048

### 7.3 校准的内容

偏置校准补偿的是**整个信号链的综合零点偏移**：

| 偏置来源 | 典型值 |
|---------|--------|
| INA240 输入失调电压 | ≤ ±25 μV |
| INA240 输出失调（含 REF 分压公差） | ≤ ±几 mV |
| ADC 自身失调 | ≤ ±4 LSB |
| ADC 参考电压公差 | ≤ ±1% |
| PCB 地弹 / 热电动势 | 微量 |

综合偏置在 ADC 读数上通常表现为 2048 ± 几十个 counts 的偏移。

### 7.4 对比：为什么不用 STM32G4 的 ADC 硬件偏置？

STM32G4 的 ADC 支持通过 `ADC_OFFSETy` 寄存器做硬件数字偏置补偿（在转换结果上自动加减一个值）。本项目没有使用该功能，原因：

1. 硬件偏置只能补偿 ADC 自身的偏移
2. INA240 输出到 ADC 输入的整个模拟链路偏置无法被硬件偏置覆盖
3. 软件方案更灵活——500 样本的统计平均也降低了噪声影响

---

## 8. 电流换算与滤波

### 8.1 原始 ADC 值到电流值

```c
// AuroFOC.c:41-45
float Uu_ = ((float)((int)uu - (int)uu_offset)) * 0.001611328f;  // U相: ADC delta → 中间量
float Uv_ = ((float)((int)uv - (int)uv_offset)) * 0.001611328f;  // W相: ADC delta → 中间量

foc->Ia = foc->Ia * 0.2f + 3.2f * Uu_;   // U相: IIR 低通滤波
foc->Ib = foc->Ib * 0.2f + 3.2f * Uv_;   // W相: IIR 低通滤波
foc->Ic = (-(foc->Ia + foc->Ib));          // V相: 基尔霍夫定律推导
```

### 8.2 换算因子分析

```
V_LSB   = 3.3V / 4096 = 0.000805664 V/count

换算因子 = 3.3 / 2048 = 0.001611328

这个因子等于 2 × V_LSB，其含义是：
- 将偏置校正后的 ADC 差值映射到 [-3.3, +3.3] 范围
- 当 delta_counts = 2048（半量程）时，Uu_ = 3.3
- 当 delta_counts = -2048 时，Uu_ = -3.3
```

### 8.3 IIR 低通滤波器

```c
foc->Ia = foc->Ia * 0.2f + 3.2f * Uu_;
```

**z 域分析：**

```
H(z) = 3.2 / (1 - 0.2 × z⁻¹)

直流增益  = 3.2 / (1 - 0.2) = 3.2 / 0.8 = 4.0
极点      = 0.2
```

**时域分析：**

```
采样频率 fs    = 40 kHz
采样周期 Ts    = 25 μs
时间常数 τ     = -Ts / ln(0.2) = -25μs / (-1.609) ≈ 15.5 μs
截止频率 fc    = 1 / (2π × τ) ≈ 10.3 kHz
建立时间 (1%) ≈ 4.6 × τ ≈ 71 μs ≈ 3 个采样周期
```

**滤波器特性总结：**

| 参数 | 值 |
|------|-----|
| 类型 | 一阶 IIR 低通滤波 |
| 极点 | 0.2 |
| 直流增益 | 4.0 |
| 截止频率 (-3dB) | ~10.3 kHz |
| 建立时间 | ~71 μs |
| 实现开销 | 1 次乘法 + 1 次加法 / 通道 |

### 8.4 关于数值单位的说明

`foc->Ia`, `foc->Ib`, `foc->Ic` 中的值是**相对于 ADC 偏置校正后差值**的缩放量，并经过了滤波器的 4 倍增益和换算因子的转换。它们不严格等于安培值。

**这在闭环控制中完全可行**，原因：
1. FOC 是**闭环系统**，PID 通过反馈自动修正
2. Clark/Park 变换只需要电流的**比例关系**，不需要绝对精度
3. PI 控制器的增益 `Kp`, `Ki` 已经针对这个内部单位进行了调谐
4. `max_iq = 4`, `max_id = 4` 等限制值也是在这个缩放系统下定义的

如果在某一天需要将值映射回真实的物理安培数，校准方式是：用已知电流（如 1A）通过电机某一相，记录对应的 `foc->Ia` 值，建立映射关系。

---

## 9. FOC 控制链（在 ADC 中断上下文中执行）

### 9.1 完整控制序列

每个 ADC 中断（40 kHz）中执行以下全部步骤：

```
Step 0: foc_update_adc()
  ├─ 偏置校准（仅前 500 次）
  ├─ ADC → 数值换算
  ├─ IIR 低通滤波
  └─ 推导 Ic

Step 1: _foc_get_angle()
  └─ SPI 读取 AS5047P 14-bit 绝对角度
       ├─ 奇偶校验，错误时重读
       ├─ 施加机械偏置校准
       └─ 考虑正反转方向

Step 2: _foc_cal_sincos()
  └─ 计算 sin(θ_e), cos(θ_e)
       θ_e = (机械角度 × 极对数 + 偏置) mod 360°

Step 3: _foc_clark() — Clarke 变换
  └─ Iα = Ia
       Iβ = (1/√3) × (Ia + 2×Ib)
       使用幅值不变形式（amplitude-invariant）

Step 4: _foc_park() — Park 变换
  └─ Id =  Iα×cos(θ) + Iβ×sin(θ)
       Iq = -Iα×sin(θ) + Iβ×cos(θ)

Step 5: 级联 PID（根据当前 focmode）
  └─ stop_loop_mode:                 不执行 PID
  ├─ selftest_loop_mode:             自检模式
  ├─ calibration_angle_pole_pair:    自动检测机械偏置和极对数
  ├─ open_loop_mode:                 Ud=1.0V, 记录齿槽转矩
  ├─ i_loop_mode:                    电流环 PI → Ud, Uq
  ├─ speed_loop_mode:                速度环 PI → aim_iq → 电流环 PI → Ud, Uq
  └─ position_loop_mode:             位置环 PI → aim_speed → 速度环 PI → ... → Ud, Uq

Step 6: _foc_ipark() — 逆 Park 变换
  └─ Uα = Ud×cos(θ) - Uq×sin(θ)
       Uβ = Ud×sin(θ) + Uq×cos(θ)

Step 7: _foc_svpwm() — 空间矢量调制
  └─ 扇区判断 (6 区)
       T1, T2 导通时间计算
       时间标准化

Step 8: _foc_update_pwm()
  └─ 占空比钳位 [0, 0.8] (最大 80%)
       写入 TIM1 CCR1, CCR2, CCR3
```

**整条链的执行时间**必须在 25 μs（40 kHz 周期）内完成。以 170 MHz Cortex-M4 的性能（带 FPU），这段代码通常在 5-15 μs 内完成，留有充裕余量。

### 9.2 FOC 模式状态机

```
stop_loop_mode (初始)
  │
  ▼
selftest_loop_mode
  │
  ▼
calibration_angle_pole_pair_mode  ← 自动检测: 正转6电周期 + 反转6电周期
  │                                  ↓ Ud=1.0V，检测机械偏置/极对数/转向
  ▼
open_loop_mode                    ← 同时记录 1800 点齿槽转矩补偿表
  │
  ▼
i_loop_mode                       ← 仅电流闭环
  │
  ▼
speed_loop_mode                   ← 速度-电流级联
  │
  ▼
position_loop_mode                ← 位置-速度-电流三级级联
```

---

## 10. 关键结构体与参数

### 10.1 AuroFOC 结构体（电流相关字段）

```c
// AuroFOC.h:48-85
typedef struct AuroFOC {
    // ... 标识和配置 ...
    uint8_t Pole_Pair;           // 电机极对数
    uint8_t motor_rotate_direct; // 正/反转标志

    // 电流值（每 25μs 更新一次）
    float Ia;      // U 相电流（滤波后）
    float Ib;      // W 相电流（滤波后）
    float Ic;      // V 相电流（推导：-(Ia+Ib)）
    float Ialpha;  // α 轴电流 (Clarke 变换后)
    float Ibeta;   // β 轴电流 (Clarke 变换后)
    float Iq;      // 交轴电流 (Park 变换后)
    float Id;      // 直轴电流 (Park 变换后)

    // 电压值
    float Ud;      // 直轴电压 (PI 输出)
    float Uq;      // 交轴电压 (PI 输出)
    float Ualpha;  // α 轴电压
    float Ubeta;   // β 轴电压

    // SVPWM 参数
    float K;       // √3 × Ts / Udc
    float Ts;      // PWM 周期（归一化 = 1）
    float Udc;     // 直流母线电压（12V）

    // 目标值
    AuroAim aim;          // { aim_iq, aim_id, aim_speed, aim_speed_rpm, aim_position }

    // 限制值
    AuroFOCLimit limit;   // max/min 电压、电流、速度、位置

    // PID 控制器
    AuroPID iq_pid;       // Q 轴电流 PI
    AuroPID id_pid;       // D 轴电流 PI
    AuroPID speed_pid;    // 速度 PI
    AuroPID position_pid; // 位置 PI

    // 硬件句柄
    TIM_HandleTypeDef *htim;   // TIM1
    ADC_HandleTypeDef *hadc;   // ADC1
    AS5047P_bsp as5047p;       // 编码器
} AuroFOC;
```

### 10.2 默认参数值

```c
// MotorEvent.c — 上电初始化
motorA.Udc          = 12.0f;    // 母线电压 [V]
motorA.Ts           = 1.0f;     // 归一化周期
motorA.K            = 1.732f/12.0f;  // √3/Udc

// 电流限制
motorA.limit.max_iq = 4.0f;
motorA.limit.min_iq = -4.0f;
motorA.limit.max_id = 4.0f;
motorA.limit.min_id = -4.0f;

// 电流环 PID 增益
PID_Init(&motorA.iq_pid, 2.5f, 0.0045f, 0, 300);  // Kp=2.5, Ki=0.0045, Kd=0, limit=300
PID_Init(&motorA.id_pid, 2.1f, 0.0012f, 0, 300);  // Kp=2.1, Ki=0.0012, Kd=0, limit=300

// 速度环 PID
PID_Init(&motorA.speed_pid, 0.0985f, 0.00185f, 0, 400);

// 位置环 PID
PID_Init(&motorA.position_pid, 4.0f, 0.012f, 0, 200);
```

### 10.3 AuroPID 结构体

```c
// AuroPID.h
typedef struct AuroPID {
    float Kp;           // 比例增益
    float Ki;           // 积分增益
    float Kd;           // 微分增益
    float integral;     // 积分累加值
    float prev_error;   // 上次误差（用于微分）
    float integral_limit; // 积分抗饱和上限
    float output;       // PID 输出
} AuroPID;
```

---

## 11. 总结：一条完整的采样路径

下面按照**时间顺序**，追踪从 MOSFET 开关到 PWM 更新的一个完整周期：

```
时间 │ 事件
─────┼────────────────────────────────────────────────────────
     │
 t₀  │ PWM 周期开始 (CNT=0)
     │ TIM1 从 0 开始递增计数
     │ PWM 输出根据 CCR1/2/3 控制上下桥 MOSFET
     │ 相电流流过下桥臂采样电阻
     │
 t₁  │ CNT 达到 4240 (≈99.8% PWM 周期)
     │ TIM1 CH4 输出从高变低 → 下降沿
     │ **ADC1 注入序列被触发**
     │
     │ ADC1 自动采样:
     │   Rank 1 (1.18μs): CH3 (PA2) → JDR1 → U 相 ADC raw
     │   Rank 2 (1.18μs): CH4 (PA3) → JDR2 → W 相 ADC raw
     │
     │ ADC JEOC 标志置位
     │
 t₂  │ ADC1_2_IRQHandler 被调用
     │
     │ HAL_ADCEx_InjectedConvCpltCallback:
     │
     │   foc_update_adc():
     │     ├─ uu = JDR1, uv = JDR2  (读取 ADC 结果寄存器)
     │     ├─ 偏置校准: uu -= 2048±δ, uv -= 2048±δ
     │     ├─ 换算: Uu_ = delta × 0.001611328
     │     ├─ 滤波: Ia = 0.2×Ia + 3.2×Uu_
     │     └─ 推导: Ic = -(Ia + Ib)
     │
     │   foc_control():
     │     ├─ SPI 读 AS5047P (角度)
     │     ├─ sin/cos 计算
     │     ├─ Clarke: Ia,Ib → Iα,Iβ
     │     ├─ Park:   Iα,Iβ → Id,Iq
     │     ├─ PI 电流环: Id_err→Ud, Iq_err→Uq
     │     ├─ 逆 Park:  Ud,Uq → Uα,Uβ
     │     ├─ SVPWM:    Uα,Uβ → Ta,Tb,Tc
     │     └─ 写入 TIM1 CCR1/CCR2/CCR3 (新占空比)
     │
 t₃  │ ISR 返回
     │
 t₄  │ CNT 达到 ARR=4249, 开始递减
     │ TIM1 更新事件 (UEV) — 将 CCR 影子寄存器加载到活动寄存器
     │ 新的 PWM 占空比在下半周期生效
     │
 t₅  │ CNT 递减到 4240
     │ TIM1 CH4 第二次下降沿
     │ **ADC1 注入序列再次触发** (第 2 次采样/周期)
     │
     │ ... 重复 t₁ ~ t₄ 的过程 ...
     │
 t₆  │ CNT 回到 0，新的 PWM 周期开始
     │
─────┴────────────────────────────────────────────────────────
 总延迟: 从电流采样 (t₁) 到 PWM 更新生效 (t₄) ≈ 2~8 μs
 控制频率: 40 kHz (每 25 μs 一次完整的 FOC 迭代)
```

---

## 12. 附录：关键文件索引

| 文件 | 作用 |
|------|------|
| [APP/MotorEvent.c](APP/MotorEvent.c) | 全局 `motorA` 实例、ADC JEOC 回调、电机初始化 |
| [FOC/AuroFOC.c](FOC/AuroFOC.c) | `foc_update_adc()` 偏置校准+换算+滤波、`foc_control()` 完整 FOC 链 |
| [FOC/AuroFOC.h](FOC/AuroFOC.h) | `AuroFOC` 结构体、`AuroFOCLimit`、`AuroAim`、FOC 模式枚举 |
| [FOC/AuroPID.c](FOC/AuroPID.c) | PI/PID 控制器实现（积分抗饱和） |
| [Core/Src/adc.c](Core/Src/adc.c) | ADC1/ADC2 初始化：注入通道、TIM1 CH4 触发、采样时间 |
| [Core/Src/tim.c](Core/Src/tim.c) | TIM1 中心对齐 PWM、CH4 ADC 触发配置、死区时间 |
| [Core/Src/stm32g4xx_it.c](Core/Src/stm32g4xx_it.c) | 各中断服务例程入口 |
| [BSP/as5047p_bsp_drv.c](BSP/as5047p_bsp_drv.c) | AS5047P 编码器 SPI 驱动、角度读取、速度计算 |
| [COMM/](COMM/) | 主机通信协议：遥测数据选择（Ia/Ib/Ic 或 Iq/Id）、二进制包 |
| [Core/Inc/stm32g4xx_hal_conf.h](Core/Inc/stm32g4xx_hal_conf.h) | HAL 模块使能、VDD_VALUE=3300 |

---

## 参数速查表

| 参数 | 值 | 来源 |
|------|-----|------|
| MCU | STM32G431CBUx, Cortex-M4, 170MHz | main.h |
| PWM 频率 | ~20 kHz | tim.c: ARR=4249, 中心对齐 |
| ADC 采样频率 | 40 kHz (每 PWM 周期 2 次) | TIM1 CH4 下降沿, CCR4=4240 |
| ADC 触发方式 | TIM1 CH4 下降沿 → ADC1 注入组 | adc.c |
| ADC 分辨率 | 12-bit (0~4095) | adc.c |
| ADC 参考电压 | 3.3V (VDDA) | stm32g4xx_hal_conf.h |
| ADC 时钟 | 42.5 MHz (SYSCLK/4) | adc.c |
| ADC 采样时间 (注入) | 12.5 周期 / ~294 ns | adc.c |
| 电流采样拓扑 | 双电阻 (U+W, V 相推导) | MotorEvent.c, AuroFOC.c |
| 电流放大器 | INA240A2, Gain=50 V/V | README.md, 原理图 |
| 采样电阻 | 5 mΩ (U21/U22) | 原理图 |
| 零电流输出 | 1.65V (Vs/2, REF1=3.3V, REF2=GND) | INA240 数据手册 |
| 电流分辨率 (理论) | 3.22 mA/LSB | 3.3V/4096/0.25 |
| 电流满量程 (理论) | ±6.6 A | 1.65V / 0.25V/A |
| 偏置校准 | 前 500 样本 (12.5ms) 求平均 | AuroFOC.c |
| IIR 滤波极点 | 0.2, fc≈10.3kHz | AuroFOC.c |
| IIR 滤波 DC 增益 | 4.0 | AuroFOC.c |
| 电流环 PID 频率 | 40 kHz (与 ADC 采样同步) | AuroFOC.c |
| 死区时间 | ~294 ns | tim.c: DT=50 ticks |
| SVPWM 最大占空比 | 80% (钳位) | AuroFOC.c |
| 接地方式 | 采样电阻 low-side (下桥臂源极-GND) | 原理图 |

---

> **文档生成日期**: 2026-06-25
> **代码版本**: 基于 Nebula_st_mdk 项目的当前代码分析
