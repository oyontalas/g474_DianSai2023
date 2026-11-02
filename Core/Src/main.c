/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "math.h"

#include "arm_math.h"
#include "ad9833.h"

#include "a_key.h"
#include "pic.h"
#include "lcd_init.h"
#include "lcd.h"

#include "syn6288.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 串口接收相关变量
#define UART_RX_MAX_SIZE 1024

uint8_t uart_rx_byte = 0;
uint8_t uart_rx_buffer[UART_RX_MAX_SIZE] = {0};
uint16_t uart_rx_buffer_size = 0;
uint8_t uart_rx_over_flag = 0;
uint8_t uart_rx_mode = 3;

// 定义发送缓冲区
#define TX_BUFFER_SIZE 600
uint8_t tx_buffer[TX_BUFFER_SIZE] = {0};

// // 定义dac输出缓冲区
// #define DAC_BUFFER_SIZE 100
// uint16_t dac_buffer_0[DAC_BUFFER_SIZE] = {0};
// uint16_t dac_buffer_1[DAC_BUFFER_SIZE] = {0};

// // 定义adc采样缓冲区
// #define ADC_BUFFER_SIZE 4096

// uint16_t adc_buffer[ADC_BUFFER_SIZE] = {0};
// uint8_t adc_buffer_full_flag = 0;

// // 定义fft相关变量
// #define FFT_SIZE 4096
// // fft句柄
// arm_rfft_fast_instance_f32 fft_handler;
// // fft输入缓冲区
// float32_t fft_input[FFT_SIZE] = {0};
// // fft输出缓冲区
// float32_t fft_output[FFT_SIZE * 2] = {0};

// // fft幅值输出缓冲区
// float32_t fft_output_magnitude[FFT_SIZE] = {0};

// 每帧是同一 ADC 的两个顺序通道交错采样，共 4096 个值，每通道 2048 点。
// 题目分离时第一通道接混合信号 C，第二通道接当前 DDS 输出反馈。
#define ADC_BUFFER_SIZE 4096

uint16_t adc_buffer[ADC_BUFFER_SIZE] = {0};
uint8_t adc_buffer_full_flag = 0;
// 后缀 _1 / _2 表示扫描序列的第一 / 第二通道，不是 ADC1 / ADC2 外设编号。
uint16_t adc_buffer_1[ADC_BUFFER_SIZE / 2] = {0};
uint16_t adc_buffer_2[ADC_BUFFER_SIZE / 2] = {0};

// 1: ADC1 采 C(PC2) 与 A 输出反馈(PC3)；2: ADC2 采 C(PC0) 与 B 输出反馈(PC1)。
// 两个 ADC 在相位反馈阶段交替使用，不是四通道同时采样。
uint8_t adc_mode = 1;

// 定义fft相关变量
#define FFT_SIZE 2048

// fft句柄
arm_rfft_fast_instance_f32 fft_handler;
// fft输入缓冲区
float32_t fft_input[FFT_SIZE];
// fft输出缓冲区
float32_t fft_output_1[FFT_SIZE * 2];
float32_t fft_output_2[FFT_SIZE * 2];

// fft幅值输出缓冲区
float32_t fft_output_magnitude_1[FFT_SIZE];
float32_t fft_output_magnitude_2[FFT_SIZE];

// 定义采样分辨率
float32_t freq_resolution = 0;

// 4 为题目自动分离模式；0/1、2/3 分别用于两组 ADC 的原始数据/频谱调试。
int8_t system_mode = 4;
// 记录已完成的波形判别轮数；达到 WAVE_TYPE_COUNT_MAX 后转入相位反馈。
uint8_t system_mode_full = 0;
// 定义保存tim8的ARR值的变量
uint16_t tim8_arr = 0;

// 单位 Hz；freq_1 始终对应低频 A，freq_2 对应高频 B，满足题目 fA < fB。
uint32_t freq_1 = 0;
uint32_t freq_2 = 0;
// 定义基波与谐波的频率比
float32_t freq_1_rate = 0;
float32_t freq_2_rate = 0;

// 当前有效分支使用 0=正弦波、1=三角波；下方 #if 0 的旧实现曾使用 1/2。
uint8_t wave_1_type = 0;
uint8_t wave_2_type = 0;
// 定义计数上限
#define WAVE_TYPE_COUNT_MAX 3
// 统计被判为三角波的轮数，其余轮次视为正弦波，最后以多数票决定波形。
uint16_t wave_1_type_count = 0;
uint16_t wave_2_type_count = 0;
// DDS 相位寄存器累计值，不是角度；一周对应 4096 个相位码。
int16_t phase_1 = 0;
int16_t phase_2 = 0;

// 用户调相设置，单位度；按键每次 5 度，串口每次 1 度，在 B 路误差中扣除。
// 题目初相位以 A 输出的时间原点定义；不同频率的瞬时相位差不能直接等同于初相位。
float32_t phase_diff = 0.0;

// 发挥部分的频率为 5 kHz 整数倍；该归整网格也覆盖基础部分的 10 kHz 步进。
#define FREQ_STEP 5000

void ADMIN_Init(void)
{
  // 开启串口接收中断
  // HAL_UART_Receive_IT(&hlpuart1, &uart_rx_byte, 1);
  HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
  // 打开定时器8
  HAL_TIM_Base_Start(&htim8);
  // 打开定时器17中断
  HAL_TIM_Base_Start_IT(&htim17);
  // 开启adc 校准
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUFFER_SIZE);
  AD9833_CS_Select = 0;
  AD9833_GPIOinit();
  HAL_Delay(10);
  AD9833_CS_Select = 1;
  AD9833_GPIOinit();
  HAL_Delay(10);

  AD9833_CS_Select = 0;
  AD9833_SetFrequencyQuick(0, AD9833_OUT_SINUS); // 识别前将频率设为 0 Hz，保留正弦输出模式
  HAL_Delay(10);
  AD9833_CS_Select = 1;
  AD9833_SetFrequencyQuick(0, AD9833_OUT_SINUS); // 识别前将频率设为 0 Hz，保留正弦输出模式
  HAL_Delay(10);

  LCD_Init();
  LCD_Fill(0, 0, 320, 240, BLACK);
  // LCD_ShowChinese(240, 0, "就", RED, WHITE, 16, 0); // 显示汉字
  // LCD_ShowChinese(240, 16, "是", RED, WHITE, 16, 0);
  // LCD_ShowChinese(240, 32, "啊", RED, WHITE, 16, 0);

  memset(tx_buffer, 0, strlen(tx_buffer));
  snprintf(tx_buffer, 40, "            Welcome to Use            ", (uint16_t)phase_diff);
  LCD_ShowString(8 * 0, 16 * 2, tx_buffer, WHITE, BLACK, 16, 0);
  // 初始化fft句柄
  arm_rfft_fast_init_f32(&fft_handler, FFT_SIZE);

  // TIM8: 160 MHz / (39+1) / (3+1) = 1 MHz，每次触发完成两通道顺序转换。
  // 每通道 FFT 的频率间隔为 1 MHz / 2048，约 488.28 Hz，不是 FREQ_STEP。
  freq_resolution = 1.0 * 160000000.0 / (htim8.Instance->PSC + 1) / (htim8.Instance->ARR + 1) / FFT_SIZE;
  // // 初始化dac输出缓冲区，sine波
  // for (uint16_t i = 0; i < DAC_BUFFER_SIZE; i++)
  // {
  //   // dac_buffer_0[i] = (uint16_t)(512 * sin(2 * 3.1415926 * i / DAC_BUFFER_SIZE) + 512 * sin(3 * 3.1415926 * i / DAC_BUFFER_SIZE + 3.1415926 / 2) + 2048);
  //   dac_buffer_0[i] = (uint16_t)(512 * sin(2 * 3.1415926 * i / DAC_BUFFER_SIZE) + 2048);
  //   dac_buffer_1[i] = dac_buffer_0[i];
  // }
  // // 开启dac dma传输
  // HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)dac_buffer_0, DAC_BUFFER_SIZE, DAC_ALIGN_12B_R);
  // HAL_DAC_Start_DMA(&hdac2, DAC_CHANNEL_1, (uint32_t *)dac_buffer_1, DAC_BUFFER_SIZE, DAC_ALIGN_12B_R);
  // 打开定时器15
  // HAL_TIM_Base_Start(&htim15);
}

void ADMIN_Mainloop(void)
{
  for (uint8_t i = 0; i < KEY_NUM; i++)
  {
    // 按键长按
    if (key_flag[i] == 1)
    {
      switch (i)
      {
      case 0:
        phase_diff = 0.0;

        break;
      }
    }
    // 按键短按1次
    else if (key_flag[i] == 2)
    {
      switch (i)
      {
      case 0:

        // LD1翻转
        // HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        // htim8.Instance->ARR = 4 - 1;
        // phase_diff+=5.0;
        // if (phase_diff >180.0)
        // {
        //   phase_diff -= 360.0;
        // }
        // if (system_mode == 4)
        // {
        //   system_mode_full = 0;
        //   adc_mode = 1;
        // }
        phase_diff -= 5.0;
        if (phase_diff < -180.0)
        {
          phase_diff += 360.0;
        }
        memset(tx_buffer, 0, strlen(tx_buffer));
        snprintf(tx_buffer, 24, "Phase Diff:%ddeg     ", (int16_t)phase_diff);
        LCD_ShowString(0, 16 * 0, tx_buffer, WHITE, BLACK, 16, 0);

        break;
      case 1:

        // LD2翻转
        // HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        // // 切换发送模式
        // system_mode--;
        // if (system_mode < 0)
        // {
        //   system_mode = 4;
        // }
        // if (system_mode == 4)
        // {
        //   // htim8.Instance->ARR = 4 - 1;
        //   system_mode_full = 0;
        //   adc_mode = 1;
        // }
        phase_diff = 0;

        memset(tx_buffer, 0, strlen(tx_buffer));
        snprintf(tx_buffer, 24, "Phase Diff:%ddeg     ", (int16_t)phase_diff);
        LCD_ShowString(0, 16 * 0, tx_buffer, WHITE, BLACK, 16, 0);

        break;
      case 2:

        // LED3翻转
        // HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
        // htim8.Instance->ARR = 4 - 1;
        // phase_diff-=5.0;
        // if (phase_diff <-180.0)
        // {
        //   phase_diff += 360.0;
        // }
        phase_diff += 5.0;
        if (phase_diff > 180.0)
        {
          phase_diff -= 360.0;
        }
        memset(tx_buffer, 0, strlen(tx_buffer));
        snprintf(tx_buffer, 24, "Phase Diff:%ddeg     ", (int16_t)phase_diff);
        LCD_ShowString(0, 16 * 0, tx_buffer, WHITE, BLACK, 16, 0);

        break;
      case 3:
        // // LD4翻转
        // HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
        // // 切换发送模式
        // system_mode++;
        // if (system_mode > 4)
        // {
        //   system_mode = 0;
        // }
        // if (system_mode == 4)
        // {
        //   // htim8.Instance->ARR = 4 - 1;
        //   system_mode_full = 0;
        //   adc_mode = 1;
        // }
        system_mode_full = 0;
        wave_1_type_count = 0;
        wave_2_type_count = 0;

        break;
      }
    }
    // 按键短按2次
    else if (key_flag[i] == 3)
    {
      switch (i)
      {
      case 0:

        adc_mode = 1;

        break;
      }
    }
    // 按键短按3次
    else if (key_flag[i] == 4)
    {
      switch (i)
      {
      case 0:

        break;
      }
    }
    if (key_flag[i] != 0)
    {
      key_flag[i] = 0;
    }
  }

  if (uart_rx_over_flag)
  {
    uart_rx_over_flag = 0;
    // 将接收到的数据通过串口发送回去
    HAL_UART_Transmit(&huart1, uart_rx_buffer, uart_rx_buffer_size, 0xffff);
    // 清空接收缓冲区
    memset(uart_rx_buffer, 0, uart_rx_buffer_size);
    uart_rx_buffer_size = 0;
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
  }
  if (adc_buffer_full_flag)
  {
    adc_buffer_full_flag = 0;
    // // 将adc_buffer中的数据复制到fft_input中
    // for (uint16_t i = 0; i < FFT_SIZE; i++)
    // {
    //   fft_input[i] = (float32_t)adc_buffer[i] * 3.3 / 4096;
    // }
    // // 计算fft
    // arm_rfft_fast_f32(&fft_handler, fft_input, fft_output, 0);

    // // 计算fft幅值
    // arm_cmplx_mag_f32(fft_output, fft_output_magnitude, FFT_SIZE/2);
    // // 归一化
    // for (uint16_t i = 0; i < FFT_SIZE; i++)
    // {
    //   if (i == 0)
    //   {
    //     fft_output_magnitude[i] = fft_output_magnitude[i] / FFT_SIZE;
    //   }
    //   else
    //   {
    //     fft_output_magnitude[i] = fft_output_magnitude[i] / FFT_SIZE * 2;
    //   }
    // }

    // DMA 完成中断已停止当前采样，此处处理这一帧快照。
    // 偶数位置是 C，奇数位置是反馈；两者属于同次触发，但顺序转换存在时间差。
    for (uint16_t i = 0; i < ADC_BUFFER_SIZE; i++)
    {
      if (i % 2 == 0)
      {
        adc_buffer_1[i / 2] = adc_buffer[i];
      }
      else
      {
        adc_buffer_2[i / 2] = adc_buffer[i];
      }
    }
    // 先对第一路adc采样数据进行fft
    // 将adc_buffer_1中的数据复制到fft_input中
    for (uint16_t i = 0; i < FFT_SIZE; i++)
    {
      fft_input[i] = (float32_t)adc_buffer_1[i] * 3.3 / 4096;
    }
    // 最后一个参数 0 表示正向 FFT；两通道独立变换，用于比较同频成分的相位。
    arm_rfft_fast_f32(&fft_handler, fft_input, fft_output_1, 0);

    // 计算fft幅值
    arm_cmplx_mag_f32(fft_output_1, fft_output_magnitude_1, FFT_SIZE);
    // 归一化
    for (uint16_t i = 0; i < FFT_SIZE; i++)
    {
      if (i == 0)
      {
        fft_output_magnitude_1[i] = fft_output_magnitude_1[i] / FFT_SIZE;
      }
      else
      {
        fft_output_magnitude_1[i] = fft_output_magnitude_1[i] / FFT_SIZE * 2;
      }
    }

    // 再对第二路adc采样数据进行fft
    // 将adc_buffer_2中的数据复制到fft_input中
    for (uint16_t i = 0; i < FFT_SIZE; i++)
    {
      fft_input[i] = (float32_t)adc_buffer_2[i] * 3.3 / 4096;
    }
    // 计算fft
    arm_rfft_fast_f32(&fft_handler, fft_input, fft_output_2, 0);
    // 计算fft幅值
    arm_cmplx_mag_f32(fft_output_2, fft_output_magnitude_2, FFT_SIZE);
    // 归一化
    for (uint16_t i = 0; i < FFT_SIZE; i++)
    {
      if (i == 0)
      {
        fft_output_magnitude_2[i] = fft_output_magnitude_2[i] / FFT_SIZE;
      }
      else
      {
        fft_output_magnitude_2[i] = fft_output_magnitude_2[i] / FFT_SIZE * 2;
      }
    }

    // 在混合信号频谱中找两个主峰；_1_1 / _1_2 是幅度排序，不是 A / B 的频率排序。
    // 从 1 开始跳过直流位置，在正频率半区搜索基波。
    uint16_t max_index_1_1 = 1;
    uint16_t max_index_1_2 = 1;
    // 第二峰避开主峰约正负 2.5 kHz（总宽约 5 kHz），避免把泄漏谱线当成另一路。
    // 2500 / freq_resolution 将频率间隔换算成 FFT bin 数。
    // 先找到最大值
    for (uint16_t i = 1; i < FFT_SIZE / 2; i++)
    {
      if (fft_output_magnitude_1[i] > fft_output_magnitude_1[max_index_1_1])
      {
        max_index_1_1 = i;
      }
    }
    // 再找到第二大值
    for (uint16_t i = 1; i < FFT_SIZE / 2; i++)
    {
      if (i < max_index_1_1 - (uint32_t)(2500.0 / freq_resolution) || i > max_index_1_1 + (uint32_t)(2500.0 / freq_resolution))
      {
        if (fft_output_magnitude_1[i] > fft_output_magnitude_1[max_index_1_2])
        {
          max_index_1_2 = i;
        }
      }
    }
    // 得到最大峰值的频率
    float32_t max_freq_1_1 = (float32_t)(1.0 * max_index_1_1 / FFT_SIZE * 160 * 1000 * 1000 / (htim8.Instance->PSC + 1) / (htim8.Instance->ARR + 1));
    // 对非直流、非奈奎斯特 bin k，实数 FFT 的实部/虚部分别位于 2*k、2*k+1。
    // atan2f 返回弧度，后续统一转换成度，再与输出反馈相位比较。
    float32_t max_phase_1_1 = atan2f(fft_output_1[max_index_1_1 * 2 + 1], fft_output_1[max_index_1_1 * 2]);
    // 弧度转换成角度
    max_phase_1_1 = max_phase_1_1 * 180 / 3.1415926;
    // 得到最第二大峰值的频率
    float32_t max_freq_1_2 = (float32_t)(1.0 * max_index_1_2 / FFT_SIZE * 160 * 1000 * 1000 / (htim8.Instance->PSC + 1) / (htim8.Instance->ARR + 1));
    // 得到第二大峰值的频率对应的相位
    float32_t max_phase_1_2 = atan2f(fft_output_1[max_index_1_2 * 2 + 1], fft_output_1[max_index_1_2 * 2]);
    // 弧度转换成角度
    max_phase_1_2 = max_phase_1_2 * 180 / 3.1415926;

    if (system_mode == 0 || system_mode == 2)
    {
      // if (system_mode == 0)
      // {
      //   HAL_UART_Transmit(&hlpuart1, "adc1\r\n", strlen("adc1\r\n"), 0xffff);
      // }
      // else
      // {
      //   HAL_UART_Transmit(&hlpuart1, "adc2\r\n", strlen("adc2\r\n"), 0xffff);
      // }
      // 将得到的fft数据发送出去
      for (uint16_t i = 0; i < FFT_SIZE; i++)
      {
        // 清空发送缓冲区
        memset(tx_buffer, 0, strlen(tx_buffer));
        // 将adc_value的值转换为字符串，存放在tx_buffer中
        sprintf(tx_buffer, "%d,%d\r\n", adc_buffer_1[i], adc_buffer_2[i]);
        // sprintf(tx_buffer, "%f\r\n", fft_output_magnitude[i]);
        // 通过usart1发送tx_buffer中的数据
        HAL_UART_Transmit(&huart1, tx_buffer, strlen(tx_buffer), 0xffff);
      }
    }
    else if (system_mode == 1 || system_mode == 3)
    {
      // if (system_mode == 1)
      // {
      //   HAL_UART_Transmit(&hlpuart1, "fft1\r\n", strlen("fft1\r\n"), 0xffff);
      // }
      // else
      // {
      //   HAL_UART_Transmit(&hlpuart1, "fft2\r\n", strlen("fft2\r\n"), 0xffff);
      // }
      // 将得到的幅值数据发送出去
      for (uint16_t i = 1; i < FFT_SIZE; i++)
      {
        // 清空发送缓冲区
        memset(tx_buffer, 0, strlen(tx_buffer));
        // 将adc_value的值转换为字符串，存放在tx_buffer中
        // sprintf(tx_buffer, "%d\r\n", adc_buffer[i]);
        sprintf(tx_buffer, "%f,%f\r\n", fft_output_magnitude_1[i], fft_output_magnitude_2[i]);
        // 通过usart1发送tx_buffer中的数据
        HAL_UART_Transmit(&huart1, tx_buffer, strlen(tx_buffer), 0xffff);
      }
    }
    else if (system_mode == 4)
    {
      // 定义本轮测量的基波频率
      uint32_t freq_1_this = 0;
      uint32_t freq_2_this = 0;
      // 先按频率排序，再就近归整到题目的 5 kHz 网格；恰好半步时当前表达式向下取。
      // DDS 使用归整后的 freq_1/2，反馈相位仍取实际检测到的 FFT bin。
      if (max_freq_1_1 < max_freq_1_2)
      {
        freq_1_this = ((uint32_t)(max_freq_1_1) % FREQ_STEP) > FREQ_STEP / 2 ? (uint32_t)(max_freq_1_1 / FREQ_STEP + 1) * FREQ_STEP : (uint32_t)(max_freq_1_1 / FREQ_STEP) * FREQ_STEP;
        freq_2_this = ((uint32_t)(max_freq_1_2) % FREQ_STEP) > FREQ_STEP / 2 ? (uint32_t)(max_freq_1_2 / FREQ_STEP + 1) * FREQ_STEP : (uint32_t)(max_freq_1_2 / FREQ_STEP) * FREQ_STEP;
      }
      else
      {
        freq_1_this = ((uint32_t)(max_freq_1_2) % FREQ_STEP) > FREQ_STEP / 2 ? (uint32_t)(max_freq_1_2 / FREQ_STEP + 1) * FREQ_STEP : (uint32_t)(max_freq_1_2 / FREQ_STEP) * FREQ_STEP;
        freq_2_this = ((uint32_t)(max_freq_1_1) % FREQ_STEP) > FREQ_STEP / 2 ? (uint32_t)(max_freq_1_1 / FREQ_STEP + 1) * FREQ_STEP : (uint32_t)(max_freq_1_1 / FREQ_STEP) * FREQ_STEP;
      }
      // 0.1 是按 3.3 V 标定并归一化后的主峰幅值门限，不是题目的 1 V 峰峰值指标。
      if (fft_output_magnitude_1[max_index_1_1] < 0.1)
      {

        if (freq_2 != 0 && freq_1 != 0)
        {
          system_mode_full = 0;
          freq_1 = 0;
          freq_2 = 0;
          // 设置频率
          AD9833_CS_Select = 0;
          AD9833_SetFrequencyQuick((float)freq_1, AD9833_OUT_SINUS);
          AD9833_CS_Select = 1;
          AD9833_SetFrequencyQuick((float)freq_2, AD9833_OUT_SINUS);

          // 屏幕显示
          memset(tx_buffer, 0, strlen(tx_buffer));
          snprintf(tx_buffer, 40, "                                       ");
          LCD_ShowString(0, 16 * 0, tx_buffer, WHITE, BLACK, 16, 0);
          LCD_ShowString(0, 16 * 1, tx_buffer, WHITE, BLACK, 16, 0);
          LCD_ShowString(0, 16 * 2, tx_buffer, WHITE, BLACK, 16, 0);
          LCD_ShowString(0, 16 * 3, tx_buffer, WHITE, BLACK, 16, 0);

          memset(tx_buffer, 0, strlen(tx_buffer));
          // 显示无信号
          snprintf(tx_buffer, 24, "NO SIGNAL", freq_1 / 1000);
          LCD_ShowString(8 * 15, 16 * 2, tx_buffer, WHITE, BLACK, 16, 0);

          memset(tx_buffer, 0, strlen(tx_buffer));
          snprintf(tx_buffer, 100, "无信号", freq_1 / 1000, freq_2 / 1000);
          SYN_FrameInfo(0, tx_buffer);
        }
      }

      // 识别阶段先确认归整后的频率组合，只有本帧与已存频率相同时才进行波形投票。
      else if (system_mode_full < WAVE_TYPE_COUNT_MAX)
      {

        if (freq_1_this != freq_1 || freq_2_this != freq_2)
        {
          system_mode_full = 0;

          freq_1 = freq_1_this;
          freq_2 = freq_2_this;
        }
        else
        {

          // 判断波形

          // 正弦波主要含基波，理想三角波的三次谐波/基波幅度约为 1/9。
          // 若 fB=3*fA，A 的三次谐波与 B 的基波重叠，不能再用该谱峰判断 A 的波形。
          // 题目范围内对应 20/60、25/75、30/90 kHz，下方特殊分支改查五次谐波。
          if (freq_1 * 3 != freq_2)
          {
            uint16_t max_index_1_1_harmonic = 0;
            uint16_t max_index_1_2_harmonic = 0;

            // 在三次谐波附近正负 2.5 kHz 搜索局部峰，容纳基波定位误差和频谱泄漏。
            // 扩展题目频率范围或修改采样率时，应同时核对谐波窗口与有效频谱边界。
            max_index_1_1_harmonic = max_index_1_1 * 3 - (uint32_t)(FREQ_STEP / freq_resolution / 2);
            for (uint16_t i = max_index_1_1 * 3 - (uint32_t)(FREQ_STEP / freq_resolution / 2); i < max_index_1_1 * 3 + (uint32_t)(FREQ_STEP / freq_resolution / 2); i++)
            {
              if (fft_output_magnitude_1[i] > fft_output_magnitude_1[max_index_1_1_harmonic])
              {
                max_index_1_1_harmonic = i;
              }
            }
            max_index_1_2_harmonic = max_index_1_2 * 3 - (uint32_t)(FREQ_STEP / freq_resolution / 2);
            for (uint16_t i = max_index_1_2 * 3 - (uint32_t)(FREQ_STEP / freq_resolution / 2); i < max_index_1_2 * 3 + (uint32_t)(FREQ_STEP / freq_resolution / 2); i++)
            {
              if (fft_output_magnitude_1[i] > fft_output_magnitude_1[max_index_1_2_harmonic])
              {
                max_index_1_2_harmonic = i;
              }
            }
            // // 清空发送缓冲区
            // memset(tx_buffer, 0, strlen(tx_buffer));
            // snprintf(tx_buffer, TX_BUFFER_SIZE, "finished!\r\n\
            // max_index_1_1:%d\tmax_index_1_1_harmonic:%d\tmax_1:%f\tmax_1_harmonic:%f\t,rate:%f\r\n\
            // max_index_1_2:%d\tmax_index_1_2_harmonic:%d\tmax_2:%f\tmax_2_harmonic:%f\t,rate:%f\r\n\
            // ",
            //          max_index_1_1, max_index_1_1_harmonic, fft_output_magnitude_1[max_index_1_1], fft_output_magnitude_1[max_index_1_1_harmonic], fft_output_magnitude_1[max_index_1_1_harmonic] / fft_output_magnitude_1[max_index_1_1],
            //          max_index_1_2, max_index_1_2_harmonic, fft_output_magnitude_1[max_index_1_2], fft_output_magnitude_1[max_index_1_2_harmonic], fft_output_magnitude_1[max_index_1_2_harmonic] / fft_output_magnitude_1[max_index_1_2]);
            // HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer, strlen(tx_buffer), 0xffff);
            // 2% 为工程判别阈值，不是理想三角波的 1/9；通过相对基波幅值降低幅度影响。
            // 两个峰按幅度编号，投票前必须根据频率大小映射回低频 A / 高频 B。
            if (fft_output_magnitude_1[max_index_1_1_harmonic] > 0.02 * (fft_output_magnitude_1[max_index_1_1]))
            {
              if (max_freq_1_1 < max_freq_1_2)
              {

                wave_1_type_count++;
                // AD9833_CS_Select = 0;
                // AD9833_SetFrequencyQuick((float)freq_1, AD9833_OUT_TRIANGLE);
              }
              else
              {
                wave_2_type_count++;
                // AD9833_CS_Select = 1;
                // AD9833_SetFrequencyQuick((float)freq_2, AD9833_OUT_TRIANGLE);
              }
            }
            else
            {
              if (max_freq_1_1 < max_freq_1_2)
              {

                // AD9833_CS_Select = 0;
                // AD9833_SetFrequencyQuick((float)freq_1, AD9833_OUT_SINUS);
              }
              else
              {
                // AD9833_CS_Select = 1;
                // AD9833_SetFrequencyQuick((float)freq_2, AD9833_OUT_SINUS);
              }
            }

            if (fft_output_magnitude_1[max_index_1_2_harmonic] > 0.02 * (fft_output_magnitude_1[max_index_1_2]))
            {
              if (max_freq_1_1 < max_freq_1_2)
              {
                wave_2_type_count++;
                // AD9833_CS_Select = 1;
                // AD9833_SetFrequencyQuick((float)freq_2, AD9833_OUT_TRIANGLE);
              }
              else
              {
                wave_1_type_count++;
                // AD9833_CS_Select = 0;
                // AD9833_SetFrequencyQuick((float)freq_1, AD9833_OUT_TRIANGLE);
              }
            }
            else
            {
              if (max_freq_1_1 < max_freq_1_2)
              {

                // AD9833_CS_Select = 1;
                // AD9833_SetFrequencyQuick((float)freq_2, AD9833_OUT_SINUS);
              }
              else
              {
                // AD9833_CS_Select = 0;
                // AD9833_SetFrequencyQuick((float)freq_1, AD9833_OUT_TRIANGLE);
              }
            }
          }
          else
          {
            uint16_t max_index_1_1_harmonic = 0;
            uint16_t max_index_1_2_harmonic = 0;

            // 三倍频重叠时，两路都改查五次谐波；三角波理论幅度比为 1/25。
            // 后面采用 1% 工程阈值，本分支针对三倍频关系，不是任意谐波重叠的通用消除。
            max_index_1_1_harmonic = max_index_1_1 * 5 - (uint32_t)(FREQ_STEP / freq_resolution / 2);
            for (uint16_t i = max_index_1_1 * 5 - (uint32_t)(FREQ_STEP / freq_resolution / 2); i < max_index_1_1 * 5 + (uint32_t)(FREQ_STEP / freq_resolution / 2); i++)
            {
              if (fft_output_magnitude_1[i] > fft_output_magnitude_1[max_index_1_1_harmonic])
              {
                max_index_1_1_harmonic = i;
              }
            }
            max_index_1_2_harmonic = max_index_1_2 * 5 - (uint32_t)(FREQ_STEP / freq_resolution / 2);
            for (uint16_t i = max_index_1_2 * 5 - (uint32_t)(FREQ_STEP / freq_resolution / 2); i < max_index_1_2 * 5 + (uint32_t)(FREQ_STEP / freq_resolution / 2); i++)
            {
              if (fft_output_magnitude_1[i] > fft_output_magnitude_1[max_index_1_2_harmonic])
              {
                max_index_1_2_harmonic = i;
              }
            }
            // // 清空发送缓冲区
            // memset(tx_buffer, 0, strlen(tx_buffer));
            // snprintf(tx_buffer, TX_BUFFER_SIZE, "finished!\r\n\
            // max_index_1_1:%d\tmax_index_1_1_harmonic:%d\tmax_1:%f\tmax_1_harmonic:%f\t,rate:%f\r\n\
            // max_index_1_2:%d\tmax_index_1_2_harmonic:%d\tmax_2:%f\tmax_2_harmonic:%f\t,rate:%f\r\n\
            // ",
            //          max_index_1_1, max_index_1_1_harmonic, fft_output_magnitude_1[max_index_1_1], fft_output_magnitude_1[max_index_1_1_harmonic], fft_output_magnitude_1[max_index_1_1_harmonic] / fft_output_magnitude_1[max_index_1_1],
            //          max_index_1_2, max_index_1_2_harmonic, fft_output_magnitude_1[max_index_1_2], fft_output_magnitude_1[max_index_1_2_harmonic], fft_output_magnitude_1[max_index_1_2_harmonic] / fft_output_magnitude_1[max_index_1_2]);
            // HAL_UART_Transmit(&huart1, (uint8_t *)tx_buffer, strlen(tx_buffer), 0xffff);

            if (fft_output_magnitude_1[max_index_1_1_harmonic] > 0.01 * fft_output_magnitude_1[max_index_1_1])
            {
              if (max_freq_1_1 < max_freq_1_2)
              {

                wave_1_type_count++;
                // AD9833_CS_Select = 0;
                // AD9833_SetFrequencyQuick((float)freq_1, AD9833_OUT_TRIANGLE);
              }
              else
              {
                wave_2_type_count++;
                // AD9833_CS_Select = 1;
                // AD9833_SetFrequencyQuick((float)freq_2, AD9833_OUT_TRIANGLE);
              }
            }
            else
            {
              if (max_freq_1_1 < max_freq_1_2)
              {
                // AD9833_CS_Select = 0;
                // AD9833_SetFrequencyQuick((float)freq_1, AD9833_OUT_SINUS);
              }
              else
              {
                // AD9833_CS_Select = 1;
                // AD9833_SetFrequencyQuick((float)freq_2, AD9833_OUT_SINUS);
              }
            }

            if (fft_output_magnitude_1[max_index_1_2_harmonic] > 0.01 * fft_output_magnitude_1[max_index_1_2])
            {
              if (max_freq_1_1 < max_freq_1_2)
              {
                wave_2_type_count++;
                // AD9833_CS_Select = 1;
                // AD9833_SetFrequencyQuick((float)freq_2, AD9833_OUT_TRIANGLE);
              }
              else
              {
                wave_1_type_count++;
                // AD9833_CS_Select = 0;
                // AD9833_SetFrequencyQuick((float)freq_1, AD9833_OUT_TRIANGLE);
              }
            }
            else
            {
              if (max_freq_1_1 < max_freq_1_2)
              {

                // AD9833_CS_Select = 1;
                // AD9833_SetFrequencyQuick((float)freq_2, AD9833_OUT_SINUS);
              }
              else
              {
                // AD9833_CS_Select = 0;
                // AD9833_SetFrequencyQuick((float)freq_1, AD9833_OUT_TRIANGLE);
              }
            }
          }
          // 每轮分别给 A、B 投票；三轮结束后比较三角波票数与其余票数，再统一配置 DDS。
          system_mode_full++;
          if (system_mode_full == WAVE_TYPE_COUNT_MAX)
          {
            AD9833_CS_Select = 0;
            if (wave_1_type_count < WAVE_TYPE_COUNT_MAX - wave_1_type_count)
            {
              wave_1_type = 0;
              AD9833_SetFrequencyQuick((float)freq_1, AD9833_OUT_SINUS);
            }
            else
            {
              wave_1_type = 1;
              AD9833_SetFrequencyQuick((float)freq_1, AD9833_OUT_TRIANGLE);
            }
            AD9833_CS_Select = 1;
            if (wave_2_type_count < WAVE_TYPE_COUNT_MAX - wave_2_type_count)
            {
              wave_2_type = 0;
              AD9833_SetFrequencyQuick((float)freq_2, AD9833_OUT_SINUS);
            }
            else
            {
              wave_2_type = 1;
              AD9833_SetFrequencyQuick((float)freq_2, AD9833_OUT_TRIANGLE);
            }
            // 清空发送缓冲区
            memset(tx_buffer, 0, strlen(tx_buffer));
            snprintf(tx_buffer, TX_BUFFER_SIZE, "finished!\r\n\
WAVE_TYPE_COUNT_MAX:%d\n\
freq_1:%d\twave_1_type:%d\twave_1_type_count:%d\n\
freq_2:%d\twave_2_type:%d\twave_2_type_count:%d\n\
            ",
                     WAVE_TYPE_COUNT_MAX,
                     freq_1, wave_1_type, wave_1_type_count,
                     freq_2, wave_2_type, wave_2_type_count);

            HAL_UART_Transmit(&huart1, tx_buffer, strlen(tx_buffer), 0xffff);

            // 屏幕显示
                // 将adc_buffer_1中的数据复制到fft_input中
              for (uint16_t i = 0; i < FFT_SIZE; i++)
              {
                fft_input[i] = (float32_t)adc_buffer_1[i] * 3.3 / 4096;
              }

            // Gui_ShowWave(float *InputWaveData, uint16_t c_len, float begin_x, float begin_y, float end_x, float end_y);
            Gui_ShowWave(fft_input+100, 100, 0.1, 0.3, 0.85, 0.95);

            memset(tx_buffer, 0, strlen(tx_buffer));
            snprintf(tx_buffer, 40, "             Mixed-signal             ");
            LCD_ShowString(8 * 0, 16 * 3, tx_buffer, WHITE, BLACK, 16, 0);

            memset(tx_buffer, 0, strlen(tx_buffer));
            snprintf(tx_buffer, 40, "  0us     25us    50us    75us   100us");
            LCD_ShowString(8 * 0, 16 * 14, tx_buffer, WHITE, BLACK, 16, 0);

            memset(tx_buffer, 0, strlen(tx_buffer));
            snprintf(tx_buffer, 40, "Phase Diff:%ddeg     ", (uint16_t)phase_diff);
            LCD_ShowString(8 * 0, 16 * 0, tx_buffer, WHITE, BLACK, 16, 0);
            memset(tx_buffer, 0, strlen(tx_buffer));
            snprintf(tx_buffer, 40, "Freq_A:%dkHz            ", freq_1 / 1000);
            LCD_ShowString(8 * 0, 16 * 1, tx_buffer, WHITE, BLACK, 16, 0);

            memset(tx_buffer, 0, strlen(tx_buffer));
            if (wave_1_type == 0)
            {
              snprintf(tx_buffer, 20, "wave_A:sin     ");
            }
            else if (wave_1_type == 1)
            {
              snprintf(tx_buffer, 20, "wave_A:tri     ");
            }
            LCD_ShowString(8 * 20, 16 * 1, tx_buffer, WHITE, BLACK, 16, 0);

            memset(tx_buffer, 0, strlen(tx_buffer));
            snprintf(tx_buffer, 40, "freq_B:%dkHz              ", freq_2 / 1000);
            LCD_ShowString(8 * 0, 16 * 2, tx_buffer, WHITE, BLACK, 16, 0);
            memset(tx_buffer, 0, strlen(tx_buffer));
            if (wave_2_type == 0)
            {
              snprintf(tx_buffer, 20, "wave_B:sin   ");
            }
            else if (wave_2_type == 1)
            {
              snprintf(tx_buffer, 20, "wave_B:tri   ");
            }
            LCD_ShowString(8 * 20, 16 * 2, tx_buffer, WHITE, BLACK, 16, 0);

            memset(tx_buffer, 0, strlen(tx_buffer));
            if (wave_1_type == 0 && wave_2_type == 0)
            {
              snprintf(tx_buffer, 100, "A频率为%dK赫兹 正弦波 B频率为%dK赫兹 正弦波 ", freq_1 / 1000, freq_2 / 1000);
            }
            else if (wave_1_type == 0 && wave_2_type == 1)
            {
              snprintf(tx_buffer, 100, "A频率为%dK赫兹 正弦波 B频率为%dK赫兹 三角波 ", freq_1 / 1000, freq_2 / 1000);
            }
            else if (wave_1_type == 1 && wave_2_type == 0)
            {
              snprintf(tx_buffer, 100, "A频率为%dK赫兹 三角波 B频率为%dK赫兹 正弦波 ", freq_1 / 1000, freq_2 / 1000);
            }
            else if (wave_1_type == 1 && wave_2_type == 1)
            {
              snprintf(tx_buffer, 100, "A频率为%dK赫兹 三角波 B频率为%dK赫兹 三角波 ", freq_1 / 1000, freq_2 / 1000);
            }

            SYN_FrameInfo(0, tx_buffer);


            // HAL_Delay(5000);
          }
        }
      }
      else
      {

        // 归整后的频率组合变化才自动重新识别；频率不变时继续反馈，不重复进行波形投票。
        // 因此仅改变输入波形而不改变频率时，可通过启动/重识别操作清空统计。
        if (freq_1_this != freq_1 || freq_2_this != freq_2)
        {
          system_mode_full = 0;
          wave_1_type_count = 0;
          wave_2_type_count = 0;
        }
        else
        {

          // 第二扫描通道是当前 DDS 的单路反馈，取其主峰相位，与 C 中对应基波比较。
          uint16_t max_index_2 = 1;
          // 过滤直流分量
          for (uint16_t i = 1; i < FFT_SIZE; i++)
          {
            if (fft_output_magnitude_2[i] >= fft_output_magnitude_2[max_index_2])
            {
              max_index_2 = i;
            }
          }
          // 将反馈谱峰索引换算为 Hz，采样时钟来自 TIM8。
          float32_t max_freq_2 = (float32_t)(1.0 * max_index_2 * 160 * 1000 * 1000 / (htim8.Instance->PSC + 1) / (htim8.Instance->ARR + 1)) / FFT_SIZE;
          // 得到最大值的频率对应的相位
          float32_t max_phase_2 = atan2f(fft_output_2[max_index_2 * 2 + 1], fft_output_2[max_index_2 * 2]);
          // 弧度转换成角度
          max_phase_2 = max_phase_2 * 180 / 3.1415926;

          // 误差单位为度：混合信号中目标基波的相位 - 当前反馈相位 + 链路补偿。
          // 1.351/10000 的单位为度/Hz，属于现有链路补偿系数，改硬件/采样时需重新标定。
          // A 跟随低频成分，B 跟随高频成分并扣除 phase_diff；不要按主峰幅度大小选 A/B。
          float32_t diff = 0.0;
          if (adc_mode == 1)
          {
            if (max_freq_1_1 < max_freq_1_2)
            {
              diff = max_phase_1_1 - max_phase_2 + 1.351 / 10000 * max_freq_1_1;
            }
            else
            {
              diff = max_phase_1_2 - max_phase_2 + 1.351 / 10000 * max_freq_1_2;
            }
          }
          else if (adc_mode == 2)
          {
            if (max_freq_1_1 < max_freq_1_2)
            {
              diff = max_phase_1_2 - max_phase_2 + 1.351 / 10000 * max_freq_1_2 - phase_diff;
            }
            else
            {
              diff = max_phase_1_1 - max_phase_2 + 1.351 / 10000 * max_freq_1_1 - phase_diff;
            }
          }
          // 跨越正负 180 度时修正一周，避免把边界两侧的相近相位当成长距离调相。
          if (diff > 180)
          {
            diff -= 360;
          }
          else if (diff < -180)
          {
            diff += 360;
          }
          // 按当前反馈链路累加 DDS 相位码：4096 对应一周，整数转换会截去不足一码的部分。
          // 这里更新相位而不重设频率；写入后切到另一路，在后续帧继续交替校正。
          if (adc_mode == 1)
          {

            phase_1 += (int16_t)(diff / 360 * 4096);
            if (phase_1 > 4096)
            {
              phase_1 -= 4096;
            }
            else if (phase_1 < 0)
            {
              phase_1 += 4096;
            }
            AD9833_CS_Select = 0;
            // 设置相位差进行跟随
            AD9833_SetPhase(AD9833_REG_PHASE0, (uint16_t)phase_1);
            // HAL_Delay(1);

            adc_mode = 2;
          }
          else if (adc_mode == 2)
          {

            phase_2 += (int16_t)(diff / 360 * 4096);
            if (phase_2 > 4096)
            {
              phase_2 -= 4096;
            }
            else if (phase_2 < 0)
            {
              phase_2 += 4096;
            }

            AD9833_CS_Select = 1;
            // 设置相位差进行跟随
            AD9833_SetPhase(AD9833_REG_PHASE0, (uint16_t)phase_2);
            // HAL_Delay(1);
            adc_mode = 1;
          }
        }
      }
#if 0
      if (system_mode_full == 0)
      {
        // // 找出第一个adc中的最大值和第二大值
        // uint16_t max_index_1_1 = 1;
        // uint16_t max_index_1_2 = 1;
        // // 要找出最大峰值和第二大峰值，有频谱泄露，
        // // 如果确定一个值是峰值的话，不再对其周围5k范围内的值进行比较
        // // 先找到最大值
        // for (uint16_t i = 1; i < FFT_SIZE / 2; i++)
        // {
        //   if (fft_output_magnitude_1[i] > fft_output_magnitude_1[max_index_1_1])
        //   {
        //     max_index_1_1 = i;
        //   }
        // }
        // // 再找到第二大值
        // for (uint16_t i = 1; i < FFT_SIZE / 2; i++)
        // {
        //   if (i < max_index_1_1 - (uint32_t)(2500.0 / freq_resolution) || i > max_index_1_1 + (uint32_t)(2500.0 / freq_resolution))
        //   {
        //     if (fft_output_magnitude_1[i] > fft_output_magnitude_1[max_index_1_2])
        //     {
        //       max_index_1_2 = i;
        //     }
        //   }
        // }
        // // 得到最大峰值的频率
        // float32_t max_freq_1_1 = (float32_t)(1.0 * max_index_1_1 / FFT_SIZE * 160 * 1000 * 1000 / (htim8.Instance->PSC + 1) / (htim8.Instance->ARR + 1));
        // // 得到最大值的频率对应的相位
        // float32_t max_phase_1_1 = atan2f(fft_output_1[max_index_1_1 * 2 + 1], fft_output_1[max_index_1_1 * 2]);
        // // 弧度转换成角度
        // max_phase_1_1 = max_phase_1_1 * 180 / 3.1415926;
        // // 得到最第二大峰值的频率
        // float32_t max_freq_1_2 = (float32_t)(1.0 * max_index_1_2 / FFT_SIZE * 160 * 1000 * 1000 / (htim8.Instance->PSC + 1) / (htim8.Instance->ARR + 1));
        // // 得到第二大峰值的频率对应的相位
        // float32_t max_phase_1_2 = atan2f(fft_output_1[max_index_1_2 * 2 + 1], fft_output_1[max_index_1_2 * 2]);
        // // 弧度转换成角度
        // max_phase_1_2 = max_phase_1_2 * 180 / 3.1415926;

        // 将得到的频率和
        // 清空发送缓冲区
        memset(tx_buffer, 0, strlen(tx_buffer));
        // 将值转换为字符串，存放在tx_buffer中
        snprintf(tx_buffer, TX_BUFFER_SIZE, "\
max_freq_1_1:%f\t,max_phase_1_1:%f\tmax_index_1_1:%d\tmax_value_1_1:%f\treal_1_1:%f\timag_1_1:%f\r\n\
max_freq_1_2:%f\t,max_phase_1_2:%f\tmax_index_1_2:%d\tmax_value_1_2:%f\treal_1_2:%f\timag_1_2:%f\r\n\
PSC:%d \tARR:%d\r\n\r\n\
",
                 max_freq_1_1, max_phase_1_1, max_index_1_1, fft_output_magnitude_1[max_index_1_1], fft_output_1[max_index_1_1 * 2], fft_output_1[max_index_1_1 * 2 + 1],
                 max_freq_1_2, max_phase_1_2, max_index_1_2, fft_output_magnitude_1[max_index_1_2], fft_output_1[max_index_1_2 * 2], fft_output_1[max_index_1_2 * 2 + 1],
                 htim8.Instance->PSC + 1, htim8.Instance->ARR + 1);
        // 通过usart1发送tx_buffer中的数据
        HAL_UART_Transmit(&huart1, tx_buffer, strlen(tx_buffer), 0xffff);
        // 通道1自适应精度.调整定时器的分频系数和重装载值，使得采样率为16000000/(PSC+1)/(ARR+1)

        // // 如果max_index_1<0.2*FFT_SIZE
        // if (max_index_1 < (uint16_t)(0.05 * FFT_SIZE))
        // {
        //   if (htim8.Instance->ARR < 8192)
        //   {
        //     htim8.Instance->ARR = (htim8.Instance->ARR + 1) * 2 - 1;
        //   }
        //   // else if (htim15.Instance->PSC < 20480)
        //   // {
        //   //     htim15.Instance->PSC = (htim15.Instance->PSC + 1) * 2 - 1;
        //   // }
        // }
        // 如果htim8的ARR值没有变化，证明达到了最精度了。等待两轮
        // if (tim8_arr == htim8.Instance->ARR)
        // {
        //   system_mode_full++;
        // }
        // else
        // {
        //   tim8_arr = htim8.Instance->ARR;
        //   system_mode_full = 0;
        // }

        // 两个混叠波的基频确定完成,uart1发送提示信息
        // 基波被确定为使5k的倍数所以取测量值最接近的5k的倍数的频率

        // 计算两个基波的频率
        if (max_freq_1_1 < max_freq_1_2)
        {
          freq_1 = ((uint32_t)(max_freq_1_1) % 5000) > 2500 ? (uint32_t)(max_freq_1_1 / 5000 + 1) * 5000 : (uint32_t)(max_freq_1_1 / 5000) * 5000;
          freq_2 = ((uint32_t)(max_freq_1_2) % 5000) > 2500 ? (uint32_t)(max_freq_1_2 / 5000 + 1) * 5000 : (uint32_t)(max_freq_1_2 / 5000) * 5000;
        }
        else
        {
          freq_1 = ((uint32_t)(max_freq_1_2) % 5000) > 2500 ? (uint32_t)(max_freq_1_2 / 5000 + 1) * 5000 : (uint32_t)(max_freq_1_2 / 5000) * 5000;
          freq_2 = ((uint32_t)(max_freq_1_1) % 5000) > 2500 ? (uint32_t)(max_freq_1_1 / 5000 + 1) * 5000 : (uint32_t)(max_freq_1_1 / 5000) * 5000;
        }
        // 如果freq_1不是freq_2的3倍，
        // 即不是freq_1==20000&&freq_2==60000
        // 不是freq_1==25000&&freq_2==75000
        // 不是freq_1==30000&&freq_2==90000
        // 即可用谐波方式判断
        if (!((freq_1 == 20000 && freq_2 == 60000) || (freq_1 == 25000 && freq_2 == 75000) || (freq_1 == 30000 && freq_2 == 90000)))
        {

          if (fft_output_magnitude_1[max_index_1_1 * 3] > 0.1 * fft_output_magnitude_1[max_index_1_1])
          {
            wave_1_type = 2;
            AD9833_CS_Select = 0;
            AD9833_SetFrequencyQuick((float)freq_1, AD9833_OUT_TRIANGLE);
            HAL_Delay(10);
          }
          else
          {
            wave_1_type = 1;
            AD9833_CS_Select = 0;
            AD9833_SetFrequencyQuick((float)freq_1, AD9833_OUT_SINUS);
            HAL_Delay(10);
          }

          if (fft_output_magnitude_1[max_index_1_2 * 3] > 0.1 * fft_output_magnitude_1[max_index_1_2])
          {
            wave_2_type = 2;
            AD9833_CS_Select = 1;
            AD9833_SetFrequencyQuick((float)freq_2, AD9833_OUT_TRIANGLE);
          }
          else
          {
            wave_2_type = 1;
            AD9833_CS_Select = 1;
            AD9833_SetFrequencyQuick((float)freq_2, AD9833_OUT_SINUS);
          }
        }
        else
        {
          wave_1_type = 1;
          AD9833_CS_Select = 0;
          AD9833_SetFrequencyQuick((float)freq_1, AD9833_OUT_SINUS);
          HAL_Delay(10);
          wave_2_type = 1;
          AD9833_CS_Select = 1;
          AD9833_SetFrequencyQuick((float)freq_2, AD9833_OUT_SINUS);
        }

        // 清空发送缓冲区
        memset(tx_buffer, 0, strlen(tx_buffer));
        snprintf(tx_buffer, TX_BUFFER_SIZE, "finished!\r\nfreq_1:%d\tfreq_2:%d,wave_1_type:%d,wave_2_type:%d\r\n",
                 freq_1, freq_2, wave_1_type, wave_2_type);

        HAL_UART_Transmit(&huart1, tx_buffer, strlen(tx_buffer), 0xffff);
        system_mode_full += 2;
      }
#endif

#if 0
    else if (system_mode_full == 1)
    {
      // 找出第一个adc中的最大值和第二大值
      uint16_t max_index_1_1 = 1;
      uint16_t max_index_1_2 = 1;
      // 要找出最大峰值和第二大峰值，有频谱泄露，
      // 如果确定一个值是峰值的话，不再对其周围5k范围内的值进行比较
      // 先找到最大值
      for (uint16_t i = 1; i < FFT_SIZE / 2; i++)
      {
        if (fft_output_magnitude_1[i] > fft_output_magnitude_1[max_index_1_1])
        {
          max_index_1_1 = i;
        }
      }

      // 再找到第二大值
      for (uint16_t i = 1; i < FFT_SIZE / 2; i++)
      {
        if (i < max_index_1_1 - (uint32_t)(2500.0 / freq_resolution) || i > max_index_1_1 + (uint32_t)(2500.0 / freq_resolution))
        {
          if (fft_output_magnitude_1[i] > fft_output_magnitude_1[max_index_1_2])
          {
            max_index_1_2 = i;
          }
        }
      }
      // 得到最大峰值的频率
      float32_t max_freq_1_1 = (float32_t)(1.0 * max_index_1_1 / FFT_SIZE * 160 * 1000 * 1000 / (htim8.Instance->PSC + 1) / (htim8.Instance->ARR + 1));
      // 得到最大值的频率对应的相位
      float32_t max_phase_1_1 = atan2f(fft_output_1[max_index_1_1 * 2 + 1], fft_output_1[max_index_1_1 * 2]);
      // 弧度转换成角度
      max_phase_1_1 = max_phase_1_1 * 180 / 3.1415926;
      // 得到最第二大峰值的频率
      float32_t max_freq_1_2 = (float32_t)(1.0 * max_index_1_2 / FFT_SIZE * 160 * 1000 * 1000 / (htim8.Instance->PSC + 1) / (htim8.Instance->ARR + 1));
      // 得到第二大峰值的频率对应的相位
      float32_t max_phase_1_2 = atan2f(fft_output_1[max_index_1_2 * 2 + 1], fft_output_1[max_index_1_2 * 2]);
      // 弧度转换成角度
      max_phase_1_2 = max_phase_1_2 * 180 / 3.1415926;

      // 将得到的频率和
      // 清空发送缓冲区
      memset(tx_buffer, 0, strlen(tx_buffer));
      // 将值转换为字符串，存放在tx_buffer中
      snprintf(tx_buffer, TX_BUFFER_SIZE, "\
max_freq_1_1:%f\t,max_phase_1_1:%f\tmax_index_1_1:%d\tmax_value_1_1:%f\treal_1_1:%f\timag_1_1:%f\r\n\
max_freq_1_2:%f\t,max_phase_1_2:%f\tmax_index_1_2:%d\tmax_value_1_2:%f\treal_1_2:%f\timag_1_2:%f\r\n\
PSC:%d \tARR:%d\r\n\r\n\
",
               max_freq_1_1, max_phase_1_1, max_index_1_1, fft_output_magnitude_1[max_index_1_1], fft_output_1[max_index_1_1 * 2], fft_output_1[max_index_1_1 * 2 + 1],
               max_freq_1_2, max_phase_1_2, max_index_1_2, fft_output_magnitude_1[max_index_1_2], fft_output_1[max_index_1_2 * 2], fft_output_1[max_index_1_2 * 2 + 1],
               htim8.Instance->PSC + 1, htim8.Instance->ARR + 1);

      // 判断一次谐波的幅度是不是到达了基波的幅度的0.1倍，如果是，就认为这个谐波是存在的
      if (max_index_1_2 * 2 < max_index_1_1 - (uint32_t)(2500.0 / freq_resolution) || max_index_1_2 * 2 > max_index_1_1 + (uint32_t)(2500.0 / freq_resolution))
      {
        if (fft_output_magnitude_1[max_index_1_1 * 2] > 0.1 * fft_output_magnitude_1[max_index_1_1])
        {
          wave_1_type = 2;
          AD9833_CS_Select = 0;
          AD9833_SetFrequencyQuick((float)freq_1, AD9833_OUT_TRIANGLE);
          HAL_Delay(10);
        }
        else
        {
          wave_1_type = 1;
          AD9833_CS_Select = 0;
          AD9833_SetFrequencyQuick((float)freq_1, AD9833_OUT_SINUS);
          HAL_Delay(10);
        }
      }
      else
      {
        wave_1_type = 1;
        AD9833_CS_Select = 0;
        AD9833_SetFrequencyQuick((float)freq_1, AD9833_OUT_SINUS);
        HAL_Delay(10);
      }

      if (fft_output_magnitude_1[max_index_1_2 * 2] > 0.1 * fft_output_magnitude_1[max_index_1_2])
      {
        wave_2_type = 2;
        AD9833_CS_Select = 1;
        AD9833_SetFrequencyQuick((float)freq_2, AD9833_OUT_TRIANGLE);
      }
      else
      {
        wave_2_type = 1;
        AD9833_CS_Select = 1;
        AD9833_SetFrequencyQuick((float)freq_2, AD9833_OUT_SINUS);
      }
    }
#endif
#if 0
      else if (system_mode_full == 2)
      {

        // 获取最大值的索引
        uint16_t max_index_2 = 1;
        // 过滤直流分量
        for (uint16_t i = 1; i < FFT_SIZE; i++)
        {
          if (fft_output_magnitude_2[i] >= fft_output_magnitude_2[max_index_2])
          {
            max_index_2 = i;
          }
        }
        // 得到最大值的频率，采样率是定时15的频率
        float32_t max_freq_2 = (float32_t)(1.0 * max_index_2 * 160 * 1000 * 1000 / (htim8.Instance->PSC + 1) / (htim8.Instance->ARR + 1)) / FFT_SIZE;
        // 得到最大值的频率对应的相位
        float32_t max_phase_2 = atan2f(fft_output_2[max_index_2 * 2 + 1], fft_output_2[max_index_2 * 2]);
        // 弧度转换成角度
        max_phase_2 = max_phase_2 * 180 / 3.1415926;

        // // 找出第一个adc中的最大值和第二大值
        // uint16_t max_index_1_1 = 1;
        // uint16_t max_index_1_2 = 1;
        // // 要找出最大峰值和第二大峰值，有频谱泄露，
        // // 如果确定一个值是峰值的话，不再对其周围5k范围内的值进行比较
        // // 先找到最大值
        // for (uint16_t i = 1; i < FFT_SIZE / 2; i++)
        // {
        //   if (fft_output_magnitude_1[i] > fft_output_magnitude_1[max_index_1_1])
        //   {
        //     max_index_1_1 = i;
        //   }
        // }
        // // 再找到第二大值
        // for (uint16_t i = 1; i < FFT_SIZE / 2; i++)
        // {
        //   if (i < max_index_1_1 - (uint32_t)(2500.0 / freq_resolution) || i > max_index_1_1 + (uint32_t)(2500.0 / freq_resolution))
        //   {
        //     if (fft_output_magnitude_1[i] > fft_output_magnitude_1[max_index_1_2])
        //     {
        //       max_index_1_2 = i;
        //     }
        //   }
        // }
        // // 得到最大峰值的频率
        // float32_t max_freq_1_1 = (float32_t)(1.0 * max_index_1_1 / FFT_SIZE * 160 * 1000 * 1000 / (htim8.Instance->PSC + 1) / (htim8.Instance->ARR + 1));

        // // 得到最第二大峰值的频率
        // float32_t max_freq_1_2 = (float32_t)(1.0 * max_index_1_2 / FFT_SIZE * 160 * 1000 * 1000 / (htim8.Instance->PSC + 1) / (htim8.Instance->ARR + 1));
        // 定义相位差，已经加矫正
        float32_t diff = 0.0;
        // float32_t max_phase_1_1 = 0.0;
        // float32_t max_phase_1_2 = 0.0;
        if (adc_mode == 1)
        {
          if (max_freq_1_1 < max_freq_1_2)
          {
            // // 得到最大值的频率对应的相位
            // max_phase_1_1 = atan2f(fft_output_1[max_index_1_1 * 2 + 1], fft_output_1[max_index_1_1 * 2]);
            // // 弧度转换成角度
            // max_phase_1_1 = max_phase_1_1 * 180 / 3.1415926;
            diff = max_phase_1_1 - max_phase_2 + 1.351 / 10000 * max_freq_1_1;
          }
          else
          {
            // // 得到第二大峰值的频率对应的相位
            // max_phase_1_2 = atan2f(fft_output_1[max_index_1_2 * 2 + 1], fft_output_1[max_index_1_2 * 2]);
            // // 弧度转换成角度
            // max_phase_1_2 = max_phase_1_2 * 180 / 3.1415926;
            diff = max_phase_1_2 - max_phase_2 + 1.351 / 10000 * max_freq_1_2;
          }
        }
        else if (adc_mode == 2)
        {

          if (max_freq_1_1 < max_freq_1_2)
          {
            // // 得到第二大峰值的频率对应的相位
            // max_phase_1_2 = atan2f(fft_output_1[max_index_1_2 * 2 + 1], fft_output_1[max_index_1_2 * 2]);
            // // 弧度转换成角度
            // max_phase_1_2 = max_phase_1_2 * 180 / 3.1415926;
            diff = max_phase_1_2 - max_phase_2 + 1.351 / 10000 * max_freq_1_2;
          }
          else
          {
            // // 得到最大值的频率对应的相位
            // max_phase_1_1 = atan2f(fft_output_1[max_index_1_1 * 2 + 1], fft_output_1[max_index_1_1 * 2]);
            // // 弧度转换成角度
            // max_phase_1_1 = max_phase_1_1 * 180 / 3.1415926;

            diff = max_phase_1_1 - max_phase_2 + 1.351 / 10000 * max_freq_1_1;
          }
        }
        if (diff > 180)
        {
          diff -= 360;
        }
        else if (diff < -180)
        {
          diff += 360;
        }
        // 根绝adc模式进行锁相
        if (adc_mode == 1)
        {

          phase_1 += (int16_t)(diff / 360 * 4096);
          if (phase_1 > 4096)
          {
            phase_1 -= 4096;
          }
          else if (phase_1 < 0)
          {
            phase_1 += 4096;
          }
          AD9833_CS_Select = 0;
          // 设置相位差进行跟随
          AD9833_SetPhase(AD9833_REG_PHASE0, (uint16_t)phase_1);
          // HAL_Delay(1);

          adc_mode = 2;
        }
        else if (adc_mode == 2)
        {

          phase_2 += (int16_t)(diff / 360 * 4096);
          if (phase_2 > 4096)
          {
            phase_2 -= 4096;
          }
          else if (phase_2 < 0)
          {
            phase_2 += 4096;
          }

          AD9833_CS_Select = 1;
          // 设置相位差进行跟随
          AD9833_SetPhase(AD9833_REG_PHASE0, (uint16_t)phase_2);
          // HAL_Delay(1);
          adc_mode = 1;
        }
        //         // 清空发送缓冲区
        //         memset(tx_buffer, 0, strlen(tx_buffer));
        //         snprintf(tx_buffer, TX_BUFFER_SIZE, "\
// adc_mode=%d,diff=%.2f\r\n\
// max_phase_1_1=%.2f,max_freq_1_1=%.2f,max_index_1_1=%d\r\n\
// max_phase_1_2=%.2f,max_freq_1_2=%.2f,max_index_1_2=%d\r\n\
// max_phase_2=%.2f,max_freq_2=%.2f,max_index_2=%d\r\n\
// diff_1=%.2f,diff_2=%.2f\
// phase_1=%d,phase_2=%d\r\n\
// bais_1=%.2f,bais_2=%.2f\r\n\
// \r\n\r\n",
        //                   adc_mode, diff,
        //                   max_phase_1_1, max_freq_1_1,max_index_1_1,
        //                   max_phase_1_2, max_freq_1_2,max_index_1_2,
        //                   max_phase_2, max_freq_2,max_index_2,
        //                   max_phase_1_1-max_phase_2, max_phase_1_2-max_phase_2,
        //                  phase_1, phase_2,
        //                   1.351 / 10000 * max_freq_1_1,1.351 / 10000 * max_freq_1_2);
        //         // 发送数据
        //         HAL_UART_Transmit(&huart1, tx_buffer, strlen(tx_buffer), 0xffff);
      }
#endif
    }
  }

  if (system_mode == 0 || system_mode == 1)
  {
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUFFER_SIZE);
  }
  else if (system_mode == 2 || system_mode == 3)
  {
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_buffer, ADC_BUFFER_SIZE);
  }
  else if (system_mode == 4)
  {
    // 两个 ADC 共用 adc_buffer 和完成标志，当前流程只选择其中一路启动采样。
    if (adc_mode == 1)

      HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUFFER_SIZE);
    else if (adc_mode == 2)
      HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_buffer, ADC_BUFFER_SIZE);
  }

  HAL_Delay(1);
  // for(int i = 0; i < 500; i++);
}

// 串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // 定义lpuart1接收函数
  if (huart->Instance == LPUART1)
  {
    if (uart_rx_buffer_size < UART_RX_MAX_SIZE - 1)
    {
      switch (uart_rx_mode)
      {
      case 0:
        // 如果最后收到的数据是“\n”，则结束接收
        if (uart_rx_byte == '\n')
        {
          uart_rx_over_flag = 1;
        }
        break;
      case 1:
        // 如果最后收到的数据是“\r\n”，则结束接收
        if (uart_rx_byte == '\n' && uart_rx_buffer[uart_rx_buffer_size - 1] == '\r')
        {
          // 将末尾所有'\r'替换为'\0'
          uart_rx_buffer[uart_rx_buffer_size - 1] = '\0';
          uart_rx_buffer_size -= 1;
          uart_rx_over_flag = 1;
        }

        break;
      case 2:
        // 如果最后收到的数据是“\r\n\r\n”，则结束接收
        if (uart_rx_byte == '\n' && uart_rx_buffer[uart_rx_buffer_size - 1] == '\r' && uart_rx_buffer[uart_rx_buffer_size - 2] == '\n' && uart_rx_buffer[uart_rx_buffer_size - 3] == '\r')
        {
          // 将末尾的“\r\n\r\n”替换为“\0”
          uart_rx_buffer[uart_rx_buffer_size - 3] = '\0';
          uart_rx_buffer_size -= 3;
          uart_rx_over_flag = 1;
        }
        break;
      }
    }
    else
    {
      // 缓存满了，结束接收，uart_rx_over_flag设置为2
      uart_rx_buffer[UART_RX_MAX_SIZE - 1] = '\0';
      uart_rx_over_flag = 2;
    }
    if (!uart_rx_over_flag)
    {
      uart_rx_buffer[uart_rx_buffer_size++] = uart_rx_byte;
      HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
  }
  // 定义husart1接收函数
  if (huart->Instance == USART1)
  {
    if (uart_rx_buffer_size < UART_RX_MAX_SIZE - 1)
    {
      switch (uart_rx_mode)
      {
      case 0:
        // 如果最后收到的数据是“\n”，则结束接收
        if (uart_rx_byte == '\n')
        {
          uart_rx_over_flag = 1;
        }
        break;
      case 1:
        // 如果最后收到的数据是“\r\n”，则结束接收
        if (uart_rx_byte == '\n' && uart_rx_buffer[uart_rx_buffer_size - 1] == '\r')
        {
          // 将末尾所有'\r'替换为'\0'
          uart_rx_buffer[uart_rx_buffer_size - 1] = '\0';
          uart_rx_buffer_size -= 1;
          uart_rx_over_flag = 1;
        }

        break;
      case 2:
        // 如果最后收到的数据是“\r\n\r\n”，则结束接收
        if (uart_rx_byte == '\n' && uart_rx_buffer[uart_rx_buffer_size - 1] == '\r' && uart_rx_buffer[uart_rx_buffer_size - 2] == '\n' && uart_rx_buffer[uart_rx_buffer_size - 3] == '\r')
        {
          // 将末尾的“\r\n\r\n”替换为“\0”
          uart_rx_buffer[uart_rx_buffer_size - 3] = '\0';
          uart_rx_buffer_size -= 3;
          uart_rx_over_flag = 1;
        }
        break;
      case 3:
        if (uart_rx_byte == '1')
        {
          phase_diff += 1;
          if (phase_diff > 180)
          {
            phase_diff -= 360;
          }
          memset(tx_buffer, 0, strlen(tx_buffer));
          snprintf(tx_buffer, 24, "Phase Diff:%ddeg     ", (int16_t)phase_diff);
          LCD_ShowString(0, 16 * 0, tx_buffer, WHITE, BLACK, 16, 0);
        }
        else if (uart_rx_byte == '2')
        {
          phase_diff -= 1;
          if (phase_diff < -180)
          {
            phase_diff += 360;
          }
          memset(tx_buffer, 0, strlen(tx_buffer));
          snprintf(tx_buffer, 24, "Phase Diff:%ddeg     ", (int16_t)phase_diff);
          LCD_ShowString(0, 16 * 0, tx_buffer, WHITE, BLACK, 16, 0);
        }
        else if (uart_rx_byte == '3')
        {
          phase_diff = 0;
          memset(tx_buffer, 0, strlen(tx_buffer));
          snprintf(tx_buffer, 24, "Phase Diff:%ddeg     ", (int16_t)phase_diff);
          LCD_ShowString(0, 16 * 0, tx_buffer, WHITE, BLACK, 16, 0);
        }
        else if (uart_rx_byte == '4')
        {
          system_mode_full = 0;
          wave_1_type_count = 0;
          wave_2_type_count = 0;
        }

        break;
      }
    }
    else
    {
      // 缓存满了，结束接收，uart_rx_over_flag设置为2
      uart_rx_buffer[UART_RX_MAX_SIZE - 1] = '\0';
      uart_rx_over_flag = 2;
    }
    if (!uart_rx_over_flag)
    {
      uart_rx_buffer[uart_rx_buffer_size++] = uart_rx_byte;
      HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
  }
}

// 定义定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  // 定义定时器16中断回调函数
  if (htim->Instance == TIM17)
  {
    KEY_scan();
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  ADMIN_Init();
  while (1)
  {
    ADMIN_Mainloop();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
