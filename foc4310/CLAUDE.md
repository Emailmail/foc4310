# FOC4310 项目说明

## 项目概述

基于 STM32G431CBUx (Cortex-M4, 170MHz) 的 FOC 电机控制固件。
双电阻低端电流采样 (U/W 相)，AS5047P 磁编码器，SVPWM 调制。
参考项目：`doc/Nebula/` (Nebula_st_mdk)、`doc/STM32G431CBU6_Template/`。

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

## PWM & ADC 时序

- TIM1: 中心对齐模式1，ARR=4249，PSC=0，PWM 频率 ≈20kHz
- TIM1 CH4 CCR=4239，falling edge 触发，每 PWM 周期 1 次触发
- ADC1 注入组: 12-bit, 扫描模式, 双通道 (IN3→JDR1, IN4→JDR2)
- ADC 时钟 42.5MHz (SYSCLK/4)，采样时间 6.5 cycle
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

滤波系数 (foc_register 中设置): uvw α=0.3, dq α=0.05

## 通信 (VOFA+ JustFloat 协议)

### 发送 (USB CDC)
- `vofa_cdc_send(data, len)`: 帧头 4B (NaN) + len×4B float 数据，通过 CDC_Transmit_FS 发送
- 当前在 TIM6 回调中发送 8 个 float: Iu,Iv,Iw,Iα,Iβ,Id,Iq,mech_angle
- 速率: 10kHz 触发，1ms/次发送

### 接收 (USB CDC)
- `vofa_cdc_receive(buf, len)`: 解析 7 字节帧 `| 0x0A | id | float(4B) | 0x0B |`
- `vofa_cdc_rx_bind(id, &var)`: 注册绑定的 float 变量地址
- 当前绑定: id0=Id_ref, id1=Iq_ref (在 usr_code.c setup() 中)
- CDC 接收回调在 `usbd_cdc_if.c` 的 `CDC_Receive_FS` 中调用

## 文件结构

```
Core/Src/
  adc.c           ADC1/ADC2 CubeMX 配置 (EOCSelection=SINGLE_CONV)
  tim.c           TIM1/TIM6 CubeMX 配置 (TIM1 CH4 CCR=4239)
  stm32g4xx_it.c  ADC1_2_IRQHandler → HAL_ADC_IRQHandler(&hadc1/&hadc2)
  main.c          main() → setup() → while(1) { loop() }

User/Src/
  usr_code.c      setup(), loop(), TIM6 回调, ADC JEOS 回调
  get_vol.c       vbus 读取, 电流采样, 偏置校准, get_current_start (JEOS 切换)
  vofa_cdc.c      VOFA over CDC 收发 + rx_bind 注册

User/Inc/
  usr_code.h, get_vol.h, vofa_cdc.h, as5047p_cali.h

Module/Algorithm/Src/
  foc.c           InPark, Park, Clarke, InClarke, foc_pid_update,
                  setspwm, setsvpwm (扇区判断+SVPWM 计算),
                  foc_register, foc_init, foc_openloop, foc_openloop_svpwm,
                  foc_update, foc_setcurrent, foc_setpid_*
Module/Algorithm/Inc/
  foc.h           foc_t, foc_pid_t 结构体及 API 声明

Module/Hardware/Src/
  vofa.c          VOFA UART 版本 (发送接收，当前未使用)
  as5047p.c       AS5047P SPI 驱动 (阻塞读，返回弧度，内置低通)
Module/Hardware/Inc/
  vofa.h, as5047p.h

Module/BSP/
  bsp_pwm.c/h, bsp_spi.c/h, bsp_uart.c/h    硬件抽象层

USB_Device/App/
  usbd_cdc_if.c   CDC_Transmit_FS, CDC_Receive_FS (调 vofa_cdc_receive)
```

## 关键调试记录

1. **JDR 数据竞争**: TIM1 CH4 触发后 JEOC 中断时 Rank2 还在转换，JDR2 读到半更新值（8000/500 毛刺）。**解决**: `get_current_start` 里切到 JEOS 中断 + 直接读 JDR 寄存器。

2. **VOFA UART 丢帧**: 460800 波特率发 8 个 float (36B) 需要 781μs，TIM6 100μs 触发导致帧堆积。**解决**: 改用 USB CDC (12Mbps)，`loop()` 中 1ms 发一次。

3. **SVPWM 方向**: SVPWM 电压方向与电角度增加方向相反，`foc_openloop_svpwm` 和 `foc_setcurrent` 中 Ud/Uq 需取反。

4. **偏置校准**: `get_current_offset()` 使用 polling + HAL_InjectedGetValue，100 次采样，约 110ms。该函数仅执行一次，不影响实时噪声。

5. **电机参数**: 14 极对，elec_theta_offset=4.09710753 rad，采样电阻 5mΩ，INA240A2 增益 50V/V。
