# FOC4310 项目说明

## 项目概述

基于 STM32G431CBUx (Cortex-M4, 170MHz) 的 FOC 电机控制固件。
双电阻低端电流采样 (U/W 相)，AS5047P 磁编码器，SVPWM 调制。
参考项目：`../doc/Nebula/` (Nebula_st_mdk)、`../doc/STM32G431CBU6_Template/`。

## 硬件配置

| 资源 | 用途 |
|------|------|
| ADC1_IN3 (PA2) | U 相电流，INA240A2 放大 (G=50)，注入 Rank1 → JDR1 |
| ADC1_IN4 (PA3) | W 相电流，INA240A2 放大 (G=50)，注入 Rank2 → JDR2 |
| ADC1 VREFINT | VDDA 校准（启动时单次 polling） |
| ADC2_IN3/IN4 (PA6/PA7) | NTC/VBUS 电压，连续 DMA（预留，未参与 FOC 控制） |
| TIM1 CH1/CH2/CH3 | U/V/W PWM 输出，CH3=U, CH2=V, CH1=W |
| TIM1 CH4 | ADC1 注入触发源 (CCR=4239, FALLING edge, 无输出引脚) |
| TIM6 | 10kHz 定时中断，VOFA CDC 数据发送 |
| SPI1 | AS5047P 编码器 (PA15=CS) |
| USART1 | VOFA UART (已弃用发送，改用 USB CDC) |
| USB CDC | VOFA 数据收发 (12Mbps Full Speed) |
| FDCAN1 | CAN 总线（CubeMX 已初始化，固件中未使用） |
| USART3 | UART3（CubeMX 已初始化，固件中未使用） |

## 时钟配置

- HSE 外部晶振 + HSI48 (USB) → PLL: /3 ×85, ÷2,÷2,÷2 → SYSCLK=170MHz
- ADC 时钟 42.5MHz (SYSCLK/4)，采样时间 6.5 cycle

## PWM & ADC 时序

- TIM1: 中心对齐模式1，ARR=4249，PSC=0，PWM 频率 ≈20kHz
- TIM1 CH4 CCR=4239，falling edge 触发，每 PWM 周期 1 次触发
- ADC1 注入组: 12-bit, 扫描模式, 双通道 (IN3→JDR1, IN4→JDR2)
- ISR 触发源改为 **JEOS**（两路全完成才进中断），避免读 JDR2 时数据竞争
- JDR 直接读寄存器 (`hadc1.Instance->JDR1/JDR2`)，不经过 HAL

## 中断优先级

| 中断 | 优先级 |
|------|--------|
| ADC1_2_IRQn | 0/0 (最高) |
| TIM1_UP_TIM16_IRQn | 0/1 |
| TIM6_DAC_IRQn | 2/0 |
| USART1_IRQn | 2/0 |

## 电流采样链路

```
INA240A2 (G=50, Vref=1.65V) → ADC1_IN3/IN4 → 注入组双通道转换
→ JEOS ISR → get_current_update() → 直接读 JDR1/JDR2
→ 换算: I = (adc_raw/4095 * vdda - vdda/2) / (50 * 0.005) - offset
→ Iv = -(Iu + Iw)  (基尔霍夫定律)
→ foc_update(): Clarke→Park + uvw/dq 低通滤波
```

滤波系数 (foc_register 中设置): uvw α=0.3, dq α=0.02

## 控制环路

### 速度环 + 电流环级联 (当前默认)
- **速度环**: 2kHz (每 10 个 FOC 周期)，PI 控制器输出 Iq_ref
- **电流环**: 20kHz (每周期)，Id/Iq PI → SVPWM
- **扭矩前馈**: 根据速度关系叠加摩擦补偿到 Iq_ref，克服静/动摩擦力

### 控制模式
- `foc_setspeed(&foc, speed_ref, Id_ref)` — 速度模式（当前使用）
- `foc_setcurrent(&foc, Id_ref, Iq_ref)` — 电流模式（调试用，当前注释）
- `foc_openloop_svpwm(&foc, Ud, Uq)` — 开环 SVPWM（调试用）
- `foc_openloop(&foc, Ud, Uq)` — 开环 SPWM（调试用）
- `foc_openloop_virtual(&foc, Ud, Uq, elec_theta)` — 虚拟电角度开环

### PI 参数 (setup 中设置)
| PID | Kp | Ki | 输出限幅 | 积分限幅 |
|-----|----|----|---------|---------|
| Id 电流环 | 30.0 | 1.0 | ±7V | ±10V |
| Iq 电流环 | 30.0 | 1.0 | ±7V | ±10V |
| 速度环 | 0.0395 | 0.0008 | ±3A (Iq) | ±1000A |

### 扭矩前馈参数
| 参数 | 值 | 说明 |
|------|-----|------|
| static_compensation | 0.073 | 静摩擦力补偿 (A) |
| static_threshold | 0.1 | 静摩擦阈值 (rad/s) |
| fric_slope | 0.015 | 摩擦斜率 |
| dynamic_compensation | 0.016 | 最小动态补偿 (A) |

## 速度计算

`as5047p_getspeed()` (在 [User/Src/as5047p_cali.c](User/Src/as5047p_cali.c)):
- 角度差分 Δθ/Δt，降采样到 2kHz (每 10 次 FOC 周期)
- 自动处理角度翻转 (±π wrap)
- 20 点滑动平均滤波，群延迟约 4.75ms
- Δt = 5×50μs = 0.5ms，1 LSB → 0.77 rad/s

## 通信 (VOFA+ JustFloat 协议)

### 发送 (USB CDC)
- `vofa_cdc_send(data, len)`: 帧头 4B (NaN) + len×4B float 数据，通过 CDC_Transmit_FS 发送
- 在 TIM6 回调中发送 8 个 float (10kHz 触发，实测 ≈1ms/次):

| 索引 | 数据 | 说明 |
|------|------|------|
| 0 | speed_pid.pout | 速度环比例输出 |
| 1 | speed_pid.iout | 速度环积分输出 |
| 2 | torque_feedforward | 扭矩前馈补偿值 |
| 3 | speed | 机械角速度 [rad/s] |
| 4 | Iab.b | Iβ 电流 |
| 5 | Idq.d | Id 电流 |
| 6 | Idq.q | Iq 电流 |
| 7 | mech_angle | 机械角度 [rad] |

### 接收 (USB CDC)
- `vofa_cdc_receive(buf, len)`: 解析 7 字节帧 `| 0x0A | id | float(4B) | 0x0B |`
- `vofa_cdc_rx_bind(id, &var)`: 注册绑定的 float 变量地址
- CDC 接收回调在 [usbd_cdc_if.c](USB_Device/App/usbd_cdc_if.c) 的 `CDC_Receive_FS` 中调用

| ID | 绑定变量 | 说明 |
|----|---------|------|
| 0 | Id | d 轴电流参考 |
| 1 | foc.speed_pid.Ki | 速度环 Ki |
| 2 | radpersec | 目标角速度 [rad/s] |
| 3 | foc.speed_pid.Kp | 速度环 Kp |

## 文件结构

```
Core/Src/
  main.c           main() → setup() → while(1) { loop() }; 包含 SystemClock_Config
  adc.c            ADC1/ADC2 CubeMX 配置 (EOCSelection=SINGLE_CONV)
  tim.c            TIM1/TIM6 CubeMX 配置 (TIM1 CH4 CCR=4239)
  stm32g4xx_it.c   ADC1_2_IRQHandler → HAL_ADC_IRQHandler(&hadc1/&hadc2)

User/Src/
  usr_code.c       setup(), loop(), TIM6 回调, ADC JEOS 回调
  get_vol.c        vbus 读取 (ADC2 DMA), 电流采样 (JDR 直读), 偏置校准, get_current_start (JEOS 切换)
  vofa_cdc.c       VOFA over CDC 收发 + rx_bind 注册
  as5047p_cali.c   速度计算 (角度差分+滑动平均), 电角度校准

User/Inc/
  usr_code.h, get_vol.h, vofa_cdc.h, as5047p_cali.h, usr_cfg.h

Module/Algorithm/Src/
  foc.c            InPark, Park, Clarke, InClarke, foc_pid_update,
                   setspwm, setsvpwm (扇区判断+SVPWM 计算),
                   foc_register, foc_init, foc_openloop, foc_openloop_svpwm,
                   foc_update, foc_setcurrent, foc_setspeed (速度+电流级联),
                   foc_setpid_*, foc_settorquefeedforward
Module/Algorithm/Inc/
  foc.h            foc_t (含 speed_pid, 扭矩前馈参数), foc_pid_t 结构体及 API 声明

Module/Hardware/Src/
  vofa.c           VOFA UART 版本 (发送接收，当前未使用)
  as5047p.c        AS5047P SPI 驱动 (阻塞读，返回弧度，内置低通滤波，支持角度翻转)
  led.c            LED 控制抽象
Module/Hardware/Inc/
  vofa.h, as5047p.h, led.h

Module/BSP/
  bsp_pwm.c/h, bsp_spi.c/h, bsp_uart.c/h, bsp_gpio.c/h   硬件抽象层

USB_Device/App/
  usbd_cdc_if.c    CDC_Transmit_FS (判忙避免覆盖), CDC_Receive_FS (调 vofa_cdc_receive)
```

## FOC 结构体 (foc_t) 关键字段

| 字段 | 说明 |
|------|------|
| Uuvw, Uab, Udq | 三相/αβ/dq 电压 |
| Iuvw, Iab, Idq | 三相/αβ/dq 电流 (滤波后) |
| elec_theta, mech_theta | 电角度 / 机械角度 |
| elec_theta_offset | 电角度零点偏移 (4.09710753 rad) |
| pole_pairs | 极对数 (14) |
| id_pid, iq_pid | d/q 轴电流 PI 控制器 |
| speed_pid | 速度环 PI 控制器 |
| speed | 当前机械角速度 [rad/s] |
| cnt | 速度环降采样计数器 |
| static_compensation, static_threshold, fric_slope, dynamic_compensation | 扭矩前馈参数 |
| torque_feedforward | 实时扭矩前馈补偿值 |

## ADC JEOS ISR 执行流程 (20kHz)

```
HAL_ADCEx_InjectedConvCpltCallback (hadc1)
  → get_current_update()        // 读 JDR1/JDR2, 换算 Iu/Iv/Iw
  → as5047p_read_angle()        // SPI 读编码器角度 (带低通)
  → as5047p_getspeed()          // 降采样 2kHz, 角度差分 + 滑动平均
  → foc_update()                // 电角度计算, Clarke+Park, 电流滤波
  → foc_setspeed()              // 速度环(2kHz) + 扭矩前馈 + 电流环(20kHz) + SVPWM
```

## 关键调试记录

1. **JDR 数据竞争**: TIM1 CH4 触发后 JEOC 中断时 Rank2 还在转换，JDR2 读到半更新值（8000/500 毛刺）。**解决**: `get_current_start` 里切到 JEOS 中断 + 直接读 JDR 寄存器。

2. **VOFA UART 丢帧**: 460800 波特率发 8 个 float (36B) 需要 781μs，TIM6 100μs 触发导致帧堆积。**解决**: 改用 USB CDC (12Mbps)，TIM6 回调中发送。

3. **SVPWM 方向**: SVPWM 电压方向与电角度增加方向相反，`foc_openloop_svpwm` 和 `foc_setcurrent` 中 Ud/Uq 需取反。

4. **偏置校准**: `get_current_offset()` 使用 polling + HAL_InjectedGetValue，100 次采样，约 110ms。该函数仅执行一次，不影响实时噪声。

5. **电机参数**: 14 极对，elec_theta_offset=4.09710753 rad，采样电阻 5mΩ，INA240A2 增益 50V/V，VBUS 分压比 1:19。

6. **速度环降采样**: 速度环 2kHz（每 10 个 FOC 周期），电流环 20kHz。速度计算也降采样到 2kHz，使用 20 点滑动平均降低噪声。

7. **CDC 发送判忙**: `CDC_Transmit_FS` 检查 `TxState != 0` 时返回 USBD_BUSY 避免覆盖正在发送的缓冲区，TIM6 10kHz 触发时若上次发送未完成则静默丢弃本帧。
