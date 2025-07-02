//
// Created by mjw on 2022/9/23.
//

#include "remotec.h"
#include "usart.h"
#include "debugc.h"
#include "stdlib.h"
#include "cmath"
#include "cstring"
#include "cstdio"
#include "INS_task.h"
#include "gimbalc.h"
#include "normal_pid.h"
#include "shootc.h"
#include "drv_can.h"
#include "packet.hpp"
#include "ITtask.h"

remotec MyRemote;
//receive data, 18 bytes one frame, but set 36 bytes
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         nonehh
  */
void REMOTEC_Init(void)
{
	REMOTEIO_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

void REMOTEC_UartIrqHandler(void)
{
	if (huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
	{
		__HAL_UART_CLEAR_PEFLAG(&huart3);
	}
	else if (USART3->SR & UART_FLAG_IDLE)
	{
		static uint16_t this_time_rx_len = 0;

		__HAL_UART_CLEAR_PEFLAG(&huart3);

		if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			/* Current memory buffer used is Memory 0 */

			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart3_rx);

			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

			//reset set_data_length
			//重新设定数据长度
			hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

			//set memory buffer 1
			//设定缓冲区1
			hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

			//enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart3_rx);

			if (this_time_rx_len == RC_FRAME_LENGTH)
			{
				MyRemote.sbus_to_rc(sbus_rx_buf[0]);
			}
		}
		else
		{
			/* Current memory buffer used is Memory 1 */
			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart3_rx);

			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

			//reset set_data_lenght
			//重新设定数据长度
			hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

			//set memory buffer 0
			//设定缓冲区0
			DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

			//enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart3_rx);

			if (this_time_rx_len == RC_FRAME_LENGTH)
			{
				//处理遥控器数据
				MyRemote.sbus_to_rc(sbus_rx_buf[1]);
			}
		}
		MyRemote.RC_DataHandle();
	}
}

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指针
  * @retval         none
  */

void remotec::sbus_to_rc(volatile const uint8_t* sbus_buf)
{
	if (sbus_buf == NULL || &rc_ctrl == NULL)
	{
		return;
	}
	rc_ctrl.Get_RC().ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
	rc_ctrl.Get_RC().ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
	rc_ctrl.Get_RC().ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
			(sbus_buf[4] << 10)) & 0x07ff;
	rc_ctrl.Get_RC().ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
	rc_ctrl.Get_RC().s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
	rc_ctrl.Get_RC().s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
	rc_ctrl.Get_Mouse().x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
	rc_ctrl.Get_Mouse().y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
	rc_ctrl.Get_Mouse().z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis 滚轮
	rc_ctrl.Get_Mouse().press_l.Set_Now_State(sbus_buf[12]);                                  //!< Mouse Left Is Press ?
	rc_ctrl.Get_Mouse().press_r.Set_Now_State(sbus_buf[13]);                                  //!< Mouse Right Is Press ?
	rc_ctrl.Get_Key().value =
			sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value  W S A D Q E SHIFT CTRL
	rc_ctrl.Get_Key().W = (rc_ctrl.Get_Key().value & 0x01);
	rc_ctrl.Get_Key().S = (rc_ctrl.Get_Key().value & 0x02) >> 1;
	rc_ctrl.Get_Key().A = (rc_ctrl.Get_Key().value & 0x04) >> 2;
	rc_ctrl.Get_Key().D = (rc_ctrl.Get_Key().value & 0x08) >> 3;
	rc_ctrl.Get_Key().SHIFT.Set_Now_State((rc_ctrl.Get_Key().value & 0x10) >> 4);
	rc_ctrl.Get_Key().CONTRL.Set_Now_State((rc_ctrl.Get_Key().value & 0x20) >> 5);
	rc_ctrl.Get_Key().Q.Set_Now_State((rc_ctrl.Get_Key().value & 0x40) >> 6);
	rc_ctrl.Get_Key().E.Set_Now_State((rc_ctrl.Get_Key().value & 0x80) >> 7);
	rc_ctrl.Get_Key().R.Set_Now_State((rc_ctrl.Get_Key().value & 0x100) >> 8);
	rc_ctrl.Get_Key().F.Set_Now_State((rc_ctrl.Get_Key().value & 0x200) >> 9);
	rc_ctrl.Get_Key().G.Set_Now_State((rc_ctrl.Get_Key().value & 0x400) >> 10);
	rc_ctrl.Get_Key().Z.Set_Now_State((rc_ctrl.Get_Key().value & 0x800) >> 11);
	rc_ctrl.Get_Key().X.Set_Now_State((rc_ctrl.Get_Key().value & 0x1000) >> 12);
	rc_ctrl.Get_Key().C.Set_Now_State((rc_ctrl.Get_Key().value & 0x2000) >> 13);
	rc_ctrl.Get_Key().V.Set_Now_State((rc_ctrl.Get_Key().value & 0x4000) >> 14);
	rc_ctrl.Get_Key().B.Set_Now_State((rc_ctrl.Get_Key().value & 0x8000) >> 15);
	rc_ctrl.Get_RC().ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //拨盘

	rc_ctrl.Get_RC().ch[0] -= RC_CH_VALUE_OFFSET;
	rc_ctrl.Get_RC().ch[1] -= RC_CH_VALUE_OFFSET;
	rc_ctrl.Get_RC().ch[2] -= RC_CH_VALUE_OFFSET;
	rc_ctrl.Get_RC().ch[3] -= RC_CH_VALUE_OFFSET;
	rc_ctrl.Get_RC().ch[4] -= RC_CH_VALUE_OFFSET;
	RC_GetNewData = 0;
}

void remotec::RC_DataHandle()
{
	if (abs(rc_ctrl.Get_RC().ch[0]) < 5)rc_ctrl.Get_RC().ch[0] = 0;
	if (abs(rc_ctrl.Get_RC().ch[1]) < 5)rc_ctrl.Get_RC().ch[1] = 0;
	if (abs(rc_ctrl.Get_RC().ch[2]) < 5)rc_ctrl.Get_RC().ch[2] = 0;
	if (abs(rc_ctrl.Get_RC().ch[3]) < 5)rc_ctrl.Get_RC().ch[3] = 0;
	if (abs(rc_ctrl.Get_RC().ch[4]) < 5)rc_ctrl.Get_RC().ch[4] = 0;
	if (abs(rc_ctrl.Get_RC().ch[0]) > 670 || abs(rc_ctrl.Get_RC().ch[3]) > 670)
	{
		memset(&rc_ctrl, 0, sizeof(RC_Ctrl_t)); //异常值
	}
}

void remotec::portSetYawSpeed(void)
{
	switch (Control_Mode)
	{
	case KEY_MODE:
		if (abs(rc_ctrl.Get_Mouse().x) < 50)
			yaw_speed = -rc_ctrl.Get_Mouse().x * 0.5; //这儿后期加等级分档位
		else
			yaw_speed = -rc_ctrl.Get_Mouse().x * 2.0; //这儿后期加等级分档位
		break;
	case RC_MODE:
		yaw_speed = -rc_ctrl.Get_RC().ch[0] * 360 / 660.0f;
		break;
	}
//	return yaw_speed;
}

void remotec::portSetPihSpeed(void)
{
	switch (Control_Mode)
	{
	case KEY_MODE:
		if (abs(rc_ctrl.Get_Mouse().y) < 50)
			pih_speed = -rc_ctrl.Get_Mouse().y * 0.5;
		else
			pih_speed = -rc_ctrl.Get_Mouse().y * 1.5;
		break;
	case RC_MODE:
		pih_speed = rc_ctrl.Get_RC().ch[1] * 360 / 660.0f;
		break;
	}
}

void remotec::portSetVx(void)
{
	switch (Control_Mode)
	{
	case KEY_MODE:
		vx = (rc_ctrl.Get_Key().D - rc_ctrl.Get_Key().A) * 80.0; //这儿后期加等级分档位
		if (rc_ctrl.Get_Key().SHIFT.Get_Now_State() == 1)
			vx *= 2.4;
		break;
	case RC_MODE:
		vx = rc_ctrl.Get_RC().ch[2] * 200.0f / 660.0f;
		break;
	}
}

void remotec::portSetVy(void)
{
	switch (Control_Mode)
	{
	case KEY_MODE:
		vy = (rc_ctrl.Get_Key().W - rc_ctrl.Get_Key().S) * 80.0;
		if (rc_ctrl.Get_Key().SHIFT.Get_Now_State() == 1)
			vy *= 2.4;
		break;
	case RC_MODE:
		vy = rc_ctrl.Get_RC().ch[3] * 200.0f / 660.0f;
		break;
	}
}

void remotec::portSetCarMode(void)
{
	rc_ctrl.Get_Key().V.portHandle();
	rc_ctrl.Get_Key().F.portHandle();
	switch (Control_Mode)
	{
	case KEY_MODE:
		if (rc_ctrl.Get_Key().V.Get_Is_Click_Once() && Last_CarMode == SUIDONG)
		{
			CarMode = TUOLUO;
		}
		else if (rc_ctrl.Get_Key().V.Get_Is_Click_Once() && Last_CarMode == TUOLUO)
		{
			CarMode = SUIDONG;
		}
		else CarMode = Last_CarMode;

		break;
	case RC_MODE:
		CarMode = rc_ctrl.Get_RC().s[0];
		break;
	}
	Last_CarMode = CarMode;
}

void remotec::portSetProtect(void)
{
	rc_ctrl.Get_Key().G.portHandle();
	rc_ctrl.Get_Key().F.portHandle();
	switch (Control_Mode)
	{
	case KEY_MODE:
		if (rc_ctrl.Get_Key().G.Get_Is_Click_Once() && Last_ProtectMode != OPENFRIC)
		{
			ProtectMode = OPENFRIC;
		}
		else if (rc_ctrl.Get_Key().G.Get_Is_Click_Once() && Last_ProtectMode != CLOSEFRIC)
		{
			ProtectMode = CLOSEFRIC;
		}
		else ProtectMode = Last_ProtectMode;

		break;
	case RC_MODE:
		ProtectMode = rc_ctrl.Get_RC().s[1];
		break;
	}
	Last_ProtectMode = ProtectMode;
}

uint8_t remotec::portIsZimiao(void)
{
	switch (Control_Mode)
	{
		case RC_MODE:
		{
			if (rc_ctrl.Get_RC().s[0] == 1)
				return 1;
			else
				return 0;
		}
		case KEY_MODE:
		{
			if (rc_ctrl.Get_Mouse().press_r.Get_Now_State())
				return 0x01;  //带数字的自瞄
			else
				return 0;
		}
	}
}

uint8_t remotec::portIsRedrawing(void)
{
	rc_ctrl.Get_Key().B.portHandle();
	return rc_ctrl.Get_Key().B.Get_Now_State();



}

void remotec::portSetHead(void)
{
	int16_t Target = Get_ChassisTarget();
	rc_ctrl.Get_Key().X.portHandle();
	rc_ctrl.Get_Key().Q.portHandle();
	rc_ctrl.Get_Key().E.portHandle();
	if (rc_ctrl.Get_Key().Q.Get_Is_Click_Once())
	{
		Set_ChassisTarget(Target + 45);
	}

	if (rc_ctrl.Get_Key().E.Get_Is_Click_Once())
	{
		Set_ChassisTarget(Target - 45);
	}

	if (rc_ctrl.Get_Key().X.Get_Is_Click_Once())
	{
		Set_YawTarget(180);
		Set_ChassisTarget(Target + 180);
	}
}

void remotec::Swich_ControlMode(void)
{
	switch (rc_ctrl.Get_RC().s[1])
	{
	case TOKEY:
		Control_Mode = KEY_MODE;
		break;
	default:
		Control_Mode = RC_MODE;
		break;
	}
}


void remotec::update()
{
	RC_GetNewData++;
	if (RC_GetNewData > 50)
	{
		is_online = 0;
	}
	else is_online = 1;
	// 复位
	if (RC_GetNewData >= 1000) {
		RC_GetNewData = 51;
	}

	Swich_ControlMode();
	portSetProtect();
	portSetCarMode();
	portSetVx();
	portSetVy();
	portSetPihSpeed();
	portSetYawSpeed();
	portSetHead();
}