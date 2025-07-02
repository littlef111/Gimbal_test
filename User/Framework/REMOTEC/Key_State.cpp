//
// Created by 2b superman on 25-6-30.
//

#include "Key_State.h"
/*
    * @name   portHandle
    * @brief  非连续键值处理 即上一次是0，本次是1，判断为按了一次，用于处理状态切换等不适用于连续用手按的按键
    * @param  port 键值状态结构体
      * @retval None
*/
void Key_State::portHandle()
{
    if (Now_State == 1 && Last_State == 0) Is_Click_Once = 1;
    else Is_Click_Once = 0;
    Last_State = Now_State;
}//用于判断按键是否按下了一次

uint8_t Key_State::Get_Is_Click_Once() const
{
    return Is_Click_Once;
}//用于把按键按下的信息传给外界

void Key_State::Set_Now_State(uint8_t state)
{
    Now_State = state;
}//用于设置现在的按压状态

uint8_t Key_State::Get_Now_State() const
{
    return Now_State;
}//用于得到当下按键状态
