/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "tim.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//电机控制参数
#define ENCODER_CPR 2496  //编码器输出轴每圈计数（13PRR*4倍频*48减速比）
#define SAMPLING_MS 5     //采样周期5ms

#define RPM_FACTOR (60000/SAMPLING_MS)  //转圈系数 = 60000/采样ms

#define PWM_PERIOD 449  //TIM计数上限（占空比0-449）
#define PWM_BASE 350    //速度环前馈base

#define SPEED_LIMIT 200.0f    //位置环输出限幅|200rpm|
#define INTEGRAL_LIMIT 150.0f //积分限幅（防饱和）

#define POS_KP 40.0f      //位置环P(误差1°-40rpm)
#define POS_DEADBAND 5.0f //位置死区|5|
#define POS_KD  0.3f   // 位置环 D 系数

#define SPEED_KP 0.5f   //速度环P
#define SPEED_KI 0.005f //速度环I

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Led2TaskHandle;
osMessageQId ledQueueHandle;
osMutexId uartMutexHandle;
osSemaphoreId ledSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static float   pos_control(float pos_error, float rpm, int *stop);
static int32_t speed_control(float target_speed, float rpm, float *integral);
static void    motor_set_direction(int forward);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartLed2Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of uartMutex */
  osMutexDef(uartMutex);
  uartMutexHandle = osMutexCreate(osMutex(uartMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of ledSem */
  osSemaphoreDef(ledSem);
  ledSemHandle = osSemaphoreCreate(osSemaphore(ledSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of ledQueue */
  osMessageQDef(ledQueue, 8, uint16_t);
  ledQueueHandle = osMessageCreate(osMessageQ(ledQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityAboveNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Led2Task */
  osThreadDef(Led2Task, StartLed2Task, osPriorityNormal, 0, 128);
  Led2TaskHandle = osThreadCreate(osThread(Led2Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */

    /* ===== 位置环（外环）变量：管「转到哪」 ===== */
    float target_angle  = 90.0f;   // 目标角度：让电机转到 90°（改这里测不同角度）
    float current_angle = 0.0f;    // 当前角度：编码器算出来的「现在在哪」
    float pos_error     = 0.0f;    // 位置误差 = 目标 − 当前
    float target_speed  = 0.0f;    // 位置环输出：目标速度

    /* ===== 速度环（内环 PI）变量：管「跑多快」 ===== */
    float integral = 0.0f;   // 积分（存钱罐），跨采样保持，传给 speed_control
    int32_t output = 0;      // 最终占空比（0~449）
    int stop = 0;            // 1 = 该停（死区内）

    /* ===== 编码器 ===== */
    int32_t cnt = 0, last_cnt = 0, delta = 0;  // 本次计数 / 上次计数 / 两次之差
    int32_t rpm = 0;        // 转速（有正负：正=正转，负=反转）
    int32_t pos_count = 0;  // 累计编码器计数

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);       // 启动 PWM
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // 启动编码器
    last_cnt = __HAL_TIM_GET_COUNTER(&htim3);       // 记下起始位置（当作 0°）
    uint32_t prevTick = osKernelSysTick();

    for(;;)
    {
      // ① 读编码器：现在在哪 + 转多快
      cnt = __HAL_TIM_GET_COUNTER(&htim3);
      delta = (int16_t)(cnt - last_cnt);
      last_cnt = cnt;
      rpm = delta * RPM_FACTOR / ENCODER_CPR;
      pos_count += delta;
      current_angle = pos_count * 360.0f / ENCODER_CPR;

      // ② 位置环：误差 → 目标速度（顺便决定"该不该停"）
      pos_error = target_angle - current_angle;
      target_speed = pos_control(pos_error, rpm, &stop);

      // ③ 速度环 + 方向：往哪转、踩多深
      if (stop)
      {
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);   // 双低 = 短路刹车
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
        integral = 0;              // 停时清空积分
        output = PWM_PERIOD;       // 100% 占空比 = 短路刹车
      }
      else
      {
        motor_set_direction(target_speed > 0);                 // 先定方向
        output = speed_control(target_speed, rpm, &integral);  // 再算占空比
      }

      // ④ 输出 + 上报
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, output);
      osMessagePut(ledQueueHandle, (uint32_t)(int16_t)rpm, 0);
      osDelayUntil(&prevTick, SAMPLING_MS);
    }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartLed2Task */
/**
* @brief Function implementing the Led2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLed2Task */
void StartLed2Task(void const * argument)
{
  /* USER CODE BEGIN StartLed2Task */
  char buf[80];  // 打印用的临时字符串
  osEvent evt;  // 「信封」：evt.status 判断有没有真收到，evt.value.v 装着数据
  int16_t r;   // 转回来的转速（有符号）

  /* Infinite loop */
  for(;;)
  {
    evt = osMessageGet(ledQueueHandle,osWaitForever);  // 没数据时，等信封送来
    if(evt.status == osEventMessage){  // 真收到数据才往下
      r = (int16_t)evt.value.v;  // 信封里装的是转速（int16_t），转成有符号的
      sprintf(buf,"R=%d\r\n",r);
      HAL_UART_Transmit(&huart1,(uint8_t*)buf,strlen(buf),100);

    }
    
  }
  /* USER CODE END StartLed2Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ===== 位置环：角度误差 → 目标速度（P+D，并报告"该不该停"） ===== */
static float pos_control(float pos_error, float rpm, int *stop)
{
    float target_speed = POS_KP * pos_error - POS_KD * rpm;   // P项追目标 + D项(rpm)阻尼

    if (target_speed >  SPEED_LIMIT) target_speed =  SPEED_LIMIT;   // 限幅 ±200
    if (target_speed < -SPEED_LIMIT) target_speed = -SPEED_LIMIT;

    if (pos_error < POS_DEADBAND && pos_error > -POS_DEADBAND) {    // 死区 ±5°
        target_speed = 0;
        *stop = 1;
    } else {
        *stop = 0;
    }
    return target_speed;
}

/* ===== 速度环：目标速度 → 占空比（integral 跨调用保持，用指针传） ===== */
static int32_t speed_control(float target_speed, float rpm, float *integral)
{
    float rpm_mag     = (rpm > 0) ? rpm : -rpm;                           // 实际转速大小
    float target_mag  = (target_speed > 0) ? target_speed : -target_speed; // 目标速度大小
    float speed_error = target_mag - rpm_mag;                              // 误差

    *integral += speed_error;                                             // 积分累加
    if (*integral >  INTEGRAL_LIMIT) *integral =  INTEGRAL_LIMIT;         // 限幅防饱和
    if (*integral < -INTEGRAL_LIMIT) *integral = -INTEGRAL_LIMIT;

    int32_t output = PWM_BASE + (int32_t)(SPEED_KP * speed_error + SPEED_KI * (*integral));
    if (output > PWM_PERIOD) output = PWM_PERIOD;                         // 占空比限幅
    if (output < 0)          output = 0;
    return output;
}

/* ===== 方向脚：按 L298N 真值表设正转/反转 ===== */
static void motor_set_direction(int forward)
{
    if (forward) {
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);     // 正转
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);   // 反转
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
    }
}

/* USER CODE END Application */

