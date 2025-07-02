//
// Created by 2b superman on 25-6-30.
//

#ifndef KEY_STATE_H
#define KEY_STATE_H

#include "gimbalc.h"

class __attribute__((packed))Key_State {
public:
    Key_State():Last_State(0),Now_State(0),Is_Click_Once(0){};
    void portHandle();
    [[nodiscard]]uint8_t Get_Is_Click_Once() const;
    void Set_Now_State(uint8_t state);
    [[nodiscard]]uint8_t Get_Now_State() const;
private:
    uint8_t Last_State: 1;
    uint8_t Now_State: 1;
    uint8_t Is_Click_Once: 1;
};



#endif //KEY_STATE_H
