/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include <math.h>
#include "INS_task.h"

#include "main.h"

#include "cmsis_os.h"

#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "BMI088driver.h"
#include "ist8310driver.h"
#include "pid.h"
#include "AHRS.h"
#include "tim.h"
#include "QuaternionEKF.h"
#include "bsp_dwt.h"
#include "TIM5.h"
#include "ITtask.h"


#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定

#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


static head_cali_t head_cali;       //head cali data
/**
  * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have
  *                 different install derection.
  * @param[out]     gyro: after plus zero drift and rotate
  * @param[out]     accel: after plus zero drift and rotate
  * @param[out]     mag: after plus zero drift and rotate
  * @param[in]      bmi088: gyro and accel data
  * @param[in]      ist8310: mag data
  * @retval         none
  */
/**
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[out]     mag: 加上零漂和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @param[in]      ist8310: 磁力计数据
  * @retval         none
  */
static void imu_cali_slove(fp32 gyro[3],
	fp32 accel[3],
	fp32 mag[3],
	bmi088_real_data_t* bmi088,
	ist8310_real_data_t* ist8310);

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp);
/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_cmd_spi_dma(void);

void EarthFrameToBodyFrame(const float* vecEF, float* vecBF, float* q);
void BodyFrameToEarthFrame(const float* vecBF, float* vecEF, float* q);

extern SPI_HandleTypeDef hspi1;

static TaskHandle_t INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = { 0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = { 0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = { 0xA2, 0xFF, 0xFF, 0xFF };

volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

bmi088_real_data_t bmi088_real_data;
fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];
fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];

static uint8_t first_temperate;
static const fp32 imu_temp_PID[3] = { TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD };
static pid_type_def imu_temp_pid;

static const float timing_time = 0.001f;   //tast run time , unit s.任务运行的时间 单位 s


//加速度计低通滤波
static fp32 accel_fliter_1[3] = { 0.0f, 0.0f, 0.0f };
static fp32 accel_fliter_2[3] = { 0.0f, 0.0f, 0.0f };
static fp32 accel_fliter_3[3] = { 0.0f, 0.0f, 0.0f };
static const fp32 fliter_num[3] = { 1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f };

static fp32 INS_gyro[3] = { 0.0f, 0.0f, 0.0f };
static fp32 INS_accel[3] = { 0.0f, 0.0f, 0.0f };
static fp32 INS_mag[3] = { 0.0f, 0.0f, 0.0f };
static fp32 INS_quat[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
fp32 INS_angle[3] = { 0.0f, 0.0f, 0.0f };      //euler angle, unit rad.欧拉角 单位 rad

TickType_t StartTime, lastTime;
INS_t INS;
IMU_Param_t IMU_Param;
#define cheat TRUE  //作弊模式 去掉较小的gyro值
#define correct_Time_define 1000    //上电去0飘 1000次取平均
#define X 0
#define Y 1
#define Z 2
#define temp_times 300       //探测温度阈值
uint8_t attitude_flag = 0;
uint32_t correct_times = 0;
float RefTemp = 40;   //Destination
float gyro_correct[3] = { 0 };  //0飘初始值
uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
const float xb[3] = { 1, 0, 0 };
const float yb[3] = { 0, 1, 0 };
const float zb[3] = { 0, 0, 1 };

bool_t IS_IMU_OK = 0;

void INS_Init(void)
{
	IMU_Param.scale[X] = 1;
	IMU_Param.scale[Y] = 1;
	IMU_Param.scale[Z] = 1;
	IMU_Param.Yaw = 0;
	IMU_Param.Pitch = 0;
	IMU_Param.Roll = 0;
	IMU_Param.flag = 1;

	IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
	// imu heat init
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

	INS.AccelLPF = 0.0085;
}
/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void INS_task(void const* pvParameters)
{
	INS_Init();
	const float gravity[3] = { 0, 0, 9.81f };
	static uint32_t count = 0;
	//wait a time
	osDelay(INS_TASK_INIT_TIME);
	while (BMI088_init())
	{
		osDelay(100);
	}
	while (ist8310_init())
	{
		osDelay(100);
	}

	BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
	//rotate and zero drift
	imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

	PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
	AHRS_init(INS_quat, INS_accel, INS_mag);

	accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
	accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
	accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
	//get the handle of task
	//获取当前任务的任务句柄，
	INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

	//set spi frequency
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}

	SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

	imu_start_dma_flag = 1;

	// TIM5_IT_Init();
	while (1)
	{
		if ((count % 1) == 0)
		{
			//wait spi DMA tansmit done
			//等待SPI DMA传输
			while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
			{
			}

			if (gyro_update_flag & (1 << IMU_NOTIFY_SHFITS))
			{
				gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
				BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
			}

			if (accel_update_flag & (1 << IMU_UPDATE_SHFITS))
			{
				accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
				BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel,
					&bmi088_real_data.time);

			}

			if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
			{
				accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
				BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET,
					&bmi088_real_data.temp);

			}

			if (attitude_flag == 2)  //ekf的姿态解算
			{
				dt = DWT_GetDeltaT(&INS_DWT_Count);
				t += dt;

				bmi088_real_data.gyro[0] -= gyro_correct[0];   //减去陀螺仪0飘
				bmi088_real_data.gyro[1] -= gyro_correct[1];
				bmi088_real_data.gyro[2] -= gyro_correct[2];

#if cheat              //作弊 可以让yaw很稳定 去掉比较小的值
				if (fabsf(bmi088_real_data.gyro[2]) < 0.02f)
					bmi088_real_data.gyro[2] = 0;
#endif

				INS.Accel[X] = bmi088_real_data.accel[X];
				INS.Accel[Y] = bmi088_real_data.accel[Y];
				INS.Accel[Z] = bmi088_real_data.accel[Z];
				INS.Gyro[X] = bmi088_real_data.gyro[X];
				INS.Gyro[Y] = bmi088_real_data.gyro[Y];
				INS.Gyro[Z] = bmi088_real_data.gyro[Z];

				// 核心函数,EKF更新四元数
				StartTime = xTaskGetTickCount();

				IMU_QuaternionEKF_Update(INS.Gyro[X],
					INS.Gyro[Y],
					INS.Gyro[Z],
					INS.Accel[X],
					INS.Accel[Y],
					INS.Accel[Z],
					dt);
				lastTime = StartTime;

				//rotate and zero drift
				imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

				memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

				// 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
				BodyFrameToEarthFrame(xb, INS.xn, INS.q);
				BodyFrameToEarthFrame(yb, INS.yn, INS.q);
				BodyFrameToEarthFrame(zb, INS.zn, INS.q);

				// 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
				float gravity_b[3];
				EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
				for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
				{
					INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt)
						+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
				}
				BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

				// 获取最终数据
				INS.Yaw = QEKF_INS.Yaw;
				INS.Pitch = QEKF_INS.Pitch;
				INS.Roll = QEKF_INS.Roll;
				INS.YawTotalAngle = QEKF_INS.YawTotalAngle;

//                usart_printf("%f,%f\n", INS.YawTotalAngle, INS.Roll);
				//because no use ist8310 and save time, no use
				if (mag_update_flag &= 1 << IMU_DR_SHFITS)
				{
					mag_update_flag &= ~(1 << IMU_DR_SHFITS);
					mag_update_flag |= (1 << IMU_SPI_SHFITS);
				}
			}
			else if (attitude_flag == 1)   //状态1 开始1000次的陀螺仪0飘初始化
			{
				gyro_correct[0] += bmi088_real_data.gyro[0];
				gyro_correct[1] += bmi088_real_data.gyro[1];
				gyro_correct[2] += bmi088_real_data.gyro[2];
				correct_times++;
				if (correct_times >= correct_Time_define)
				{
					gyro_correct[0] /= correct_Time_define;
					gyro_correct[1] /= correct_Time_define;
					gyro_correct[2] /= correct_Time_define;
					attitude_flag = 2; //go to 2 state
				}
			}
		}

		if ((count % 10) == 0)
		{
			// 100hz 的温度控制pid
//            imu_temp_control(bmi088_real_data.temp);
			//========================================================================================
//            static uint32_t temp_Ticks=0;
//            if((fabsf(bmi088_real_data.temp-RefTemp)<0.5f)&&attitude_flag==0) //接近额定温度之差小于0.5° 开始计数
//            {
//                temp_Ticks++;
//                if(temp_Ticks>temp_times)   //计数达到一定次数后 才进入0飘初始化 说明温度已经达到目标
//                {
			attitude_flag = 1;  //go to correct state
//                }
//            }
		}
		count++;
		IS_IMU_OK = 1;
	}
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float* vecBF, float* vecEF, float* q)
{
	vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
		(q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
		(q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

	vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
		(0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
		(q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

	vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
		(q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
		(0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float* vecEF, float* vecBF, float* q)
{
	vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
		(q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
		(q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

	vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
		(0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
		(q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

	vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
		(q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
		(0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}
/**
  * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have
  *                 different install derection.
  * @param[out]     gyro: after plus zero drift and rotate
  * @param[out]     accel: after plus zero drift and rotate
  * @param[out]     mag: after plus zero drift and rotate
  * @param[in]      bmi088: gyro and accel data
  * @param[in]      ist8310: mag data
  * @retval         none
  */
/**
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[out]     mag: 加上零漂和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @param[in]      ist8310: 磁力计数据
  * @retval         none
  */
static void imu_cali_slove(fp32 gyro[3],
	fp32 accel[3],
	fp32 mag[3],
	bmi088_real_data_t* bmi088,
	ist8310_real_data_t* ist8310)
{
	for (uint8_t i = 0; i < 3; i++)
	{
		gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1]
			+ bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
		accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1]
			+ bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
		mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1]
			+ ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
	}
}

/**
  * @brief          get imu control temperature, unit ℃
  * @param[in]      none
  * @retval         imu control temperature
  */
/**
  * @brief          获取imu控制温度, 单位℃
  * @param[in]      none
  * @retval         imu控制温度
  */
int8_t get_control_temperature(void)
{

	return head_cali.temperature;
}

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          控制bmi088的温度
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_temp_control(fp32 temp)
{
	uint16_t tempPWM;
	static uint8_t temp_constant_time = 0;
	if (first_temperate)
	{
		PID_calc(&imu_temp_pid, temp, 40);
		if (imu_temp_pid.out < 0.0f)
		{
			imu_temp_pid.out = 0.0f;
		}
		tempPWM = (uint16_t)imu_temp_pid.out;
		TIM_Set_PWM(&htim10, TIM_CHANNEL_1, tempPWM);
	}
	else
	{
		//在没有达到设置的温度，一直最大功率加热
		//in beginning, max power
		if (temp > 40)
		{
			temp_constant_time++;
			if (temp_constant_time > 200)
			{
				//达到设置温度，将积分项设置为一半最大功率，加速收敛
				//
				first_temperate = 1;
				imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
			}
		}
//		usart_printf("%f,%f\n",bmi088_real_data.temp,imu_temp_pid.out);
		TIM_Set_PWM(&htim10, TIM_CHANNEL_1, 2000 - 1);
	}

}

/**
  * @brief          calculate gyro zero drift
  * @param[out]     gyro_offset:zero drift
  * @param[in]      gyro:gyro data
  * @param[out]     offset_time_count: +1 auto
  * @retval         none
  */
/**
  * @brief          计算陀螺仪零漂
  * @param[out]     gyro_offset:计算零漂
  * @param[in]      gyro:角速度数据
  * @param[out]     offset_time_count: 自动加1
  * @retval         none
  */
void gyro_offset_calc(fp32 gyro_offset[3], fp32 gyro[3], uint16_t* offset_time_count)
{
	if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
	{
		return;
	}

	gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
	gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
	gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
	(*offset_time_count)++;
}

/**
  * @brief          calculate gyro zero drift
  * @param[out]     cali_scale:scale, default 1.0
  * @param[out]     cali_offset:zero drift, collect the gyro ouput when in still
  * @param[out]     time_count: time, when call gyro_offset_calc
  * @retval         none
  */
/**
  * @brief          校准陀螺仪
  * @param[out]     陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[out]     陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
  * @param[out]     陀螺仪的时刻，每次在gyro_offset调用会加1,
  * @retval         none
  */
void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t* time_count)
{
	if (*time_count == 0)
	{
		gyro_offset[0] = gyro_cali_offset[0];
		gyro_offset[1] = gyro_cali_offset[1];
		gyro_offset[2] = gyro_cali_offset[2];
	}
	gyro_offset_calc(gyro_offset, INS_gyro, time_count);

	cali_offset[0] = gyro_offset[0];
	cali_offset[1] = gyro_offset[1];
	cali_offset[2] = gyro_offset[2];
	cali_scale[0] = 1.0f;
	cali_scale[1] = 1.0f;
	cali_scale[2] = 1.0f;

}

/**
  * @brief          get gyro zero drift from flash
  * @param[in]      cali_scale:scale, default 1.0
  * @param[in]      cali_offset:zero drift,
  * @retval         none
  */
/**
  * @brief          校准陀螺仪设置，将从flash或者其他地方传入校准值
  * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[in]      陀螺仪的零漂
  * @retval         none
  */
void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3])
{
	gyro_cali_offset[0] = cali_offset[0];
	gyro_cali_offset[1] = cali_offset[1];
	gyro_cali_offset[2] = cali_offset[2];
	gyro_offset[0] = gyro_cali_offset[0];
	gyro_offset[1] = gyro_cali_offset[1];
	gyro_offset[2] = gyro_cali_offset[2];
}

/**
  * @brief          get the quat
  * @param[in]      none
  * @retval         the point of INS_quat
  */
/**
  * @brief          获取四元数
  * @param[in]      none
  * @retval         INS_quat的指针
  */
const fp32* get_INS_quat_point(void)
{
	return INS_quat;
}
/**
  * @brief          get the euler angle, 0:yaw, 1:pitch, 2:roll unit rad
  * @param[in]      none
  * @retval         the point of INS_angle
  */
/**
  * @brief          获取欧拉角, 0:yaw, 1:pitch, 2:roll 单位 rad
  * @param[in]      none
  * @retval         INS_angle的指针
  */
const fp32* get_INS_angle_point(void)
{
	return INS_angle;
}

/**
  * @brief          get the rotation speed, 0:x-axis, 1:y-axis, 2:roll-axis,unit rad/s
  * @param[in]      none
  * @retval         the point of INS_gyro
  */
/**
  * @brief          获取角速度,0:x轴, 1:y轴, 2:roll轴 单位 rad/s
  * @param[in]      none
  * @retval         INS_gyro的指针
  */
extern const fp32* get_gyro_data_point(void)
{
	return INS_gyro;
}
/**
  * @brief          get aceel, 0:x-axis, 1:y-axis, 2:roll-axis unit m/s2
  * @param[in]      none
  * @retval         the point of INS_accel
  */
/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
  * @param[in]      none
  * @retval         INS_accel的指针
  */
extern const fp32* get_accel_data_point(void)
{
	return INS_accel;
}
/**
  * @brief          get aceel fliter data, 0:x-axis, 1:y-axis, 2:roll-axis unit m/s2
  * @param[in]      none
  * @retval         the point of accel_fliter_3
  */
/**
  * @brief          ��ȡ�˲���ļ��ٶ�����,0:x��, 1:y��, 2:roll�� ��λ m/s2
  * @param[in]      none
  * @retval         accel_fliter_3��ָ��
  */
extern const fp32* get_accel_fliter_data_point(void)
{
	return accel_fliter_3;
}

float getIMUTemp(void)
{
	return bmi088_real_data.temp;
}

bmi088_real_data_t* getBMI088RealData(void)
{
	return &bmi088_real_data;
}
/**
  * @brief          get mag, 0:x-axis, 1:y-axis, 2:roll-axis unit ut
  * @param[in]      none
  * @retval         the point of INS_mag
  */
/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 ut
  * @param[in]      none
  * @retval         INS_mag的指针
  */
extern const fp32* get_mag_data_point(void)
{
	return INS_mag;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == INT1_ACCEL_Pin)
	{
//        detect_hook(BOARD_ACCEL_TOE);
		accel_update_flag |= 1 << IMU_DR_SHFITS;
		accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
		if (imu_start_dma_flag)
		{
			imu_cmd_spi_dma();
		}
	}
	else if (GPIO_Pin == INT1_GYRO_Pin)
	{
//        detect_hook(BOARD_GYRO_TOE);
		gyro_update_flag |= 1 << IMU_DR_SHFITS;
		if (imu_start_dma_flag)
		{
			imu_cmd_spi_dma();
		}
	}
	else if (GPIO_Pin == DRDY_IST8310_Pin)
	{
//        detect_hook(BOARD_MAG_TOE);
		mag_update_flag |= 1 << IMU_DR_SHFITS;
	}
	else if (GPIO_Pin == GPIO_PIN_0)
	{

		//wake up the task
		//唤醒任务
		if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
		{
			static BaseType_t xHigherPriorityTaskWoken;
			vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}

	}

}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          根据imu_update_flag的值开启SPI DMA
  * @param[in]      temp:bmi088的温度
  * @retval         none
  */
static void imu_cmd_spi_dma(void)
{
	UBaseType_t uxSavedInterruptStatus;
	uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

	//开启陀螺仪的DMA传输
	if ((gyro_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN)
		&& !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
		&& !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
	{
		gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
		gyro_update_flag |= (1 << IMU_SPI_SHFITS);

		HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
		SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
		taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
		return;
	}
	//开启加速度计的DMA传输
	if ((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN)
		&& !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
		&& !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
	{
		accel_update_flag &= ~(1 << IMU_DR_SHFITS);
		accel_update_flag |= (1 << IMU_SPI_SHFITS);

		HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
		SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
		taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
		return;
	}

	if ((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN)
		&& !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
		&& !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
	{
		accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
		accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

		HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
		SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
		taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
		return;
	}
	taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
	/* Prevent unused argument(s) compilation warning */
	//gyro read over
	//陀螺仪读取完毕
	if (gyro_update_flag & (1 << IMU_SPI_SHFITS))
	{
		gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
		gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

		HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);

	}

	//accel read over
	//加速度计读取完毕
	if (accel_update_flag & (1 << IMU_SPI_SHFITS))
	{
		accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
		accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

		HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
	}
	//temperature read over
	//温度读取完毕
	if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
	{
		accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
		accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

		HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
	}

	imu_cmd_spi_dma();

	if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
	{
		gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
		gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
		__HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
	}

	/* NOTE : This function should not be modified, when the callback is needed,
			  the HAL_SPI_RxCpltCallback should be implemented in the user file
	 */
}

//原名称：void DMA2_Stream2_IRQHandler(void) 创建一个函数，在这里调用
void SPI_RxCallBack(void)
{

	if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
	{
		__HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

		//gyro read over
		//陀螺仪读取完毕
		if (gyro_update_flag & (1 << IMU_SPI_SHFITS))
		{
			gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
			gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

			HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);

		}

		//accel read over
		//加速度计读取完毕
		if (accel_update_flag & (1 << IMU_SPI_SHFITS))
		{
			accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
			accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

			HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
		}
		//temperature read over
		//温度读取完毕
		if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
		{
			accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
			accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

			HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
		}

		imu_cmd_spi_dma();

		if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
		{
			gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
			gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
			__HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
		}
	}
}

//static float last_angle;
//static int32_t rotate_times;
//float IMU_AngleIncreLoop(float angle_now)
//{
//	float this_angle;
//	this_angle = angle_now;
//	if ((this_angle - last_angle) > 300)
//		rotate_times--;
//	if ((this_angle - last_angle) < -300)
//		rotate_times++;
//	angle_now = this_angle + rotate_times * 360.0f;
//	last_angle = this_angle;
//	return angle_now;
//}

float Yaw_Angle, Pih_Angle;
float IMU_Angle(int8_t Witch_angle)
{
//	Angle = get_INS_angle_point();
	Yaw_Angle = INS.YawTotalAngle;
	Pih_Angle = INS.Pitch;
	switch (Witch_angle)
	{
	case 1:
		return Yaw_Angle;
	case 2:
		return Pih_Angle;
	default:
		return 0;
	}
}

float Yaw_Speed, Pih_Speed;
const fp32* Speed;
float IMU_Speed(int8_t Witch_angle)
{
	Speed = get_gyro_data_point();
	Yaw_Speed = INS.Gyro[2] * 9.55f;
	Pih_Speed = INS.Gyro[0] * 9.55f;
	switch (Witch_angle)
	{
	case 1:
		return Yaw_Speed;
	case 2:
		return Pih_Speed;
	default:
		return 0;
	}
}

Get_IMUNaiveAngle IMU_NaiveAngle(void)
{
	Get_IMUNaiveAngle NaiveAngle;
	NaiveAngle.yaw = INS.Yaw;
	NaiveAngle.roll = INS.Roll;
	NaiveAngle.pitch = INS.Pitch;
	return NaiveAngle;
}