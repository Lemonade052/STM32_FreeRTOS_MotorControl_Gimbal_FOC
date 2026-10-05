# STM32 FreeRTOS 电机控制

基于 STM32F103 + FreeRTOS 的有刷电机位置/速度闭环控制（当前进度）

## 技术栈
- MCU：STM32F103C8T6（Blue Pill）
- RTOS：FreeRTOS（CMSIS_V1）
- 工具链：STM32CubeMX + HAL + Keil MDK
- 外设：TIM（PWM/编码器）、USART、GPIO

## 当前进度
- FreeRTOS 多任务 / 队列通信
- PWM 调速（20kHz）+ L298N 方向控制
- 编码器测速（TIM3 编码器模式 ×4 倍频）
- PID 速度闭环（PI + 前馈 + 积分限幅 anti-windup）
- 位置环串级闭环（P + D + 死区）
- 多任务重构（控制/通信任务 + 队列 + osDelayUntil）

## 硬件
- STM32F103C8T6 + ST-Link + usb-ttl
- L298N 电机驱动 + 18650电池两节（7.4v - 2v（L298N压降）= 5.4v）
- TT马达带霍尔编码器减速电机（1:48，13 PPR，300 rpm）

## 记录
- 时钟：HSE + PLL ×9 = 72MHz（AHB/1、APB1/2=36M、APB2/1=72M）
- TT马达 50% 占空比电机堵转，改 80%（CCR=360）成功运转。
- TIM3 编码器模式（PA6=CH1/PA7=CH2，×4 倍频）实测电机轴每圈 52 计数、输出轴每圈 2496 计数，80% 占空比实测 202~208 rpm
- PID 速度闭环跑通、抗负载测试跑通
- osDelay产生节拍偏移导致位置环极限环振荡，改用osDelayUntil后现象变为震荡后收敛。
- 位置控制用减速箱有"回差"：收敛后电机停在固定两个角度而非精确 90°。
- 实时性优化：① 位置环由 bang-bang 改为 P + D 控制（kp_pos=40、kd_pos=0.3、限幅 ±200、死区 ±5°）② 控制任务优先级 Normal→AboveNormal