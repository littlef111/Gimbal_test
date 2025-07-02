//
// Created by 2b superman on 25-6-30.
//

#ifndef RC_CTRL_T_H
#define RC_CTRL_T_H

#include "gimbalc.h"
#include "Key_State.h"

class __attribute__((packed)) RC_Ctrl_t {
    private:
    // 遥控器控制结构体
    class __attribute__((packed)) RC {
        public:
        int16_t ch[5];
        char s[2];
    } rc;

    // 鼠标控制结构体
    class __attribute__((packed)) Mouse {
        public:
        int16_t x;
        int16_t y;
        int16_t z;
        Key_State press_l;
        Key_State press_r;
    } mouse;

    // 键盘控制结构体
    class __attribute__((packed)) Key {
        public:
        uint16_t value;
        uint8_t W : 1;
        uint8_t S : 1;
        uint8_t A : 1;
        uint8_t D : 1;
        Key_State SHIFT;
        Key_State CONTRL;
        Key_State Q, E, R, F, G;
        Key_State Z, X, C, V, B;
    } key;

    public:
    // RC 访问接口
    RC& Get_RC();

    // Mouse 访问接口
    Mouse& Get_Mouse();

    // Key 访问接口
    Key& Get_Key();

};


#endif //RC_CTRL_T_H
