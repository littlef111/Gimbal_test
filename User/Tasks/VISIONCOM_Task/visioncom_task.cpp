//
// Created by mjw on 2023/1/12.
//

#include "packet.hpp"
#include "crc.h" //��������û��ض������
#include "visioncom_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "remotec.h"
#include "INS_task.h"
#include "drv_can.h"
#include "debugc.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

//通信用变量
SendPacket send_packet;
uint8_t Buf[sizeof(SendPacket)];
uint8_t New_Buf[sizeof(Buf)+1];

void VisionChattingLoop(uint8_t mode)
{
	float ShootSpeed = judge.shoot_spd_now;
	float pitch = IMU_NaiveAngle().pitch;
	float yaw = IMU_NaiveAngle().yaw;
	uint8_t state = 0;

	send_packet.header = 0xA5;
	// send_packet.shoot_spd = ShootSpeed;
	send_packet.state = state;
	send_packet.pitch = pitch;
	send_packet.yaw = yaw;
	send_packet.autoaim = 1;
	send_packet.enemy_color = 1;//打红
	// send_packet.enemy_color = 0;//打蓝

	//  send_packet.packat_id = id;
	std::copy(reinterpret_cast<const uint8_t*>(&send_packet),
	   reinterpret_cast<const uint8_t*>(&send_packet) + sizeof (SendPacket),Buf);
	Append_CRC16_Check_Sum(Buf, sizeof(SendPacket));
	memcpy(New_Buf, Buf, sizeof(SendPacket));
	New_Buf[sizeof(SendPacket)] = '\n';
	CDC_Transmit_FS(New_Buf, sizeof(Buf)+1);
}

void VisionComTask(void const* argument)
{
	/* USER CODE BEGIN VisionComTask */
	portTickType CurrentTime;
	/* Infinite loop */
	for (;;)
	{
		CurrentTime = xTaskGetTickCount();
		VisionChattingLoop(MyRemote.portIsZimiao());
		vTaskDelayUntil(&CurrentTime, 10 / portTICK_RATE_MS);
	}
	/* USER CODE END VisionComTask */
}