//
// Created by 2b superman on 25-6-30.
//

#include "RC_Ctrl_t.h"

// RC 访问接口
RC_Ctrl_t::RC& RC_Ctrl_t::Get_RC()
{
    return rc;
}

// Mouse 访问接口
RC_Ctrl_t::Mouse& RC_Ctrl_t::Get_Mouse()
{
    return mouse;
}

// Key 访问接口
RC_Ctrl_t::Key& RC_Ctrl_t::Get_Key()
{
    return key;
}

