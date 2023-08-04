#include "a_key.h"

uint16_t key_state[KEY_NUM] = {0}; // 按键状态
uint16_t key_case[KEY_NUM] = {0};  // 按键状态机
uint16_t key_time[KEY_NUM] = {0};  // 按键时间
uint16_t key_flag[KEY_NUM] = {0};  // 按键标志位
#if KEY_MULTI_PRESS
uint16_t key_presses[KEY_NUM] = {0}; // 按键多次短按计数
#endif

void KEY_scan(void)
{
    key_state[0] = !HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
    key_state[1] = HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin);
    key_state[2] = HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin);
    key_state[3] = HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin);

    for (uint8_t i = 0; i < KEY_NUM; i++)
    {
        switch (key_case[i])
        {
        case 0:
            if (key_state[i] == KEY_DOWN)
            {
                key_case[i] = 1;
                key_time[i] = 0;
            }
#if KEY_MULTI_PRESS
            else if (key_presses[i])
            {
                key_time[i]++;
                if (key_time[i] > 200)
                {
                    key_time[i] = 0;
                    key_flag[i] = 1 + key_presses[i];
                    key_presses[i] = 0;
                }
            }
#endif

            break;
        case 1:
            key_time[i]++;
            if (key_time[i] > 15)
            {
                if (key_state[i] == KEY_DOWN)
                {
                    key_case[i] = 2;
                }
            }
            else if (key_time[i] > 30)
            {
                key_case[i] = 0;
                key_time[i] = 0;
            }
            break;
        case 2:
            key_time[i]++;
            if (key_state[i] == KEY_UP)
            {
                // 短按
                if (key_time[i] < 400)
                {
                    key_case[i] = 0;
                    key_time[i] = 0;
#if KEY_MULTI_PRESS
                    key_presses[i]++;
#else
                    key_flag[i] = 2;
#endif
                }
#if KEY_LONG_PRESS
                // 长按
                else if (key_time[i] < 3000)
                {
                    key_case[i] = 0;
                    key_time[i] = 0;
#if KEY_MULTI_PRESS
                    if (key_presses[i])
                        key_presses[i] = 0;

                    else
#endif
                        key_flag[i] = 1;
                }
#endif
                else
                {
                    key_case[i] = 0;
                    key_time[i] = 0;
#if KEY_MULTI_PRESS
                    key_presses[i] = 0;
#endif
                }
            }
            break;
        }
    }
}
