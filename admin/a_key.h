
#ifndef _A_KEY_H_
#define _A_KEY_H_

#include "main.h"

// 按键数量
#define KEY_NUM 4

// 按键状态
#define KEY_UP 1
#define KEY_DOWN 0

// 多次短按功能使能
#define KEY_MULTI_PRESS 0
// 长按功能使能
#define KEY_LONG_PRESS 1


extern uint16_t key_flag[KEY_NUM];

void KEY_scan(void);

#endif
