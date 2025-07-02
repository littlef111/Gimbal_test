//
// Created by mjw on 2022/9/23.
//

#ifndef ROBO_TEST_REMOTEC_H
#define ROBO_TEST_REMOTEC_H

#include "gimbalc.h"
#include <cctype>
#include "Key_State.h"
#include "RC_Ctrl_t.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "remoteio.h"
#include "stm32f4xx_hal.h"


#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Internal Data ----------------------------------- */

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void REMOTEC_Init(void);

class remotec {
 public:
	//remote control data
	//遥控器控制变量
	RC_Ctrl_t rc_ctrl;
	int16_t RC_GetNewData = 0;//检测键值是否在发送/更新
	uint8_t Control_Mode = RC_MODE;
	bool is_online = 0;

	void RC_DataHandle();

	float yaw_speed;
	float pih_speed;
	float vx = 0;
	float vy = 0;
//	float vz = 0;
	int8_t CarMode;
	int8_t Last_CarMode = SUIDONG; //默认自由
	int8_t ProtectMode ;
	int8_t Last_ProtectMode = OPENFRIC;

	uint8_t portIsZimiao(void);
	uint8_t portIsRedrawing(void);

	void ShootSpeedTarget(float Shoot_Speed, float Ram_Speed, int8_t mode); //bug:有时候目标值弹跳 已解决
	void sbus_to_rc(volatile const uint8_t* sbus_buf);
	void update();
 private:
	void portSetYawSpeed(void);
	void portSetPihSpeed(void);
	void portSetVx(void);
	void portSetVy(void);
	void portSetCarMode(void);
	void portSetProtect(void);
	void portSetHead(void);
	void Swich_ControlMode(void);
};
extern remotec MyRemote;
#ifdef __cplusplus
}
#endif
#endif //ROBO_TEST_REMOTEC_H
