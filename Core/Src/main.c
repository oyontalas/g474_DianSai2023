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

// 定义adc采样缓冲区
#define ADC_BUFFER_SIZE 4096

uint16_t adc_buffer[ADC_BUFFER_SIZE] = {0};
uint8_t adc_buffer_full_flag = 0;
// 将两路ADC采样值分离保存的数组
uint16_t adc_buffer_1[ADC_BUFFER_SIZE / 2] = {0};
uint16_t adc_buffer_2[ADC_BUFFER_SIZE / 2] = {0};

// 定义adc采样模式
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

// 定义系统模式
int8_t system_mode = 4;
// 定义完整功能的系统模式
uint8_t system_mode_full = 0;
// 定义保存tim8的ARR值的变量
uint16_t tim8_arr = 0;

// 定义两个基波的频率
uint32_t freq_1 = 0;
uint32_t freq_2 = 0;
// 定义基波与谐波的频率比
float32_t freq_1_rate = 0;
float32_t freq_2_rate = 0;

// 定义波的类型：1正弦波，2三角波
uint8_t wave_1_type = 0;
uint8_t wave_2_type = 0;
// 定义计数上限
#define WAVE_TYPE_COUNT_MAX 3
// 定义波类型计数
uint16_t wave_1_type_count = 0;
uint16_t wave_2_type_count = 0;
// 定义相位
int16_t phase_1 = 0;
int16_t phase_2 = 0;

// 定义相位差
float32_t phase_diff = 0.0;

// 定义频率步进
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
  AD9833_SetFrequencyQuick(0, AD9833_OUT_SINUS); // 写输出频率1000.0Hz,输出正弦波
  HAL_Delay(10);
  AD9833_CS_Select = 1;
  AD9833_SetFrequencyQuick(0, AD9833_OUT_SINUS); // 写输出频率1000.0Hz,输出正弦波
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

  // 定义频率分辨率
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

    // 将两路adc采样数据分离
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
    // 计算fft
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
      // 计算两个基波的频率
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
      // 判断无信号
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

          // 如果freq_2不是freq_1的3倍，
          // 即不是freq_1==20000&&freq_2==60000
          // 不是freq_1==25000&&freq_2==75000
          // 不是freq_1==30000&&freq_2==90000
          // 即可用谐波方式判断
          if (freq_1 * 3 != freq_2)
          {
            uint16_t max_index_1_1_harmonic = 0;
            uint16_t max_index_1_2_harmonic = 0;

            // 定义谐波位置最大值的索引，搜索范围为谐波的正负2.5k以内
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

            // 定义谐波位置最大值的索引，搜索范围为谐波的正负2.5k以内
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

        // 检查是否有频率变化
        if (freq_1_this != freq_1 || freq_2_this != freq_2)
        {
          system_mode_full = 0;
          wave_1_type_count = 0;
          wave_2_type_count = 0;
        }
        else
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
