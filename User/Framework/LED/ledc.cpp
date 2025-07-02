//
// Created by cr on 22-9-12.
//

/* Private includes ----------------------------------------------------------*/
#include "ledc.h"
#include "ledio.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

void (*setLedLevel)(LedC led, uint8_t level);
/* Private user code ---------------------------------------------------------*/
void LED_SetLedOn(LedC &led)
{
    led.Set_Status(LED_ON);
    setLedLevel(led, led.Get_On_Level());
}

void LED_SetLedOff(LedC &led)
{
    led.Set_Status(LED_OFF);
    setLedLevel(led, !led.Get_On_Level());
}

void LED_SetLedToggle(LedC &led)
{
    led.Set_Status(LED_TOGGLE);
    setLedLevel(led, 3);
}

/**
  * @brief  设置led的状态
  * @param  status: LED_OFF, LED_ON, LED_TOGGLE
  * @retval None
  */
void LedC::setStatus(LED_State_e status) {
    this->state = status;
    switch (status) {
        case LED_ON:
            LED_SetLedOn(*this);
            break;
        case LED_OFF:
            LED_SetLedOff(*this);
            break;
        case LED_TOGGLE:
            LED_SetLedToggle(*this);
            break;
    }
}

LED_Port_IO LedC::Get_Led_Port()
{
    return led_port;
}

LED_Port_Pin LedC::Get_Led_Pin()
{
    return led_pin;
}

uint8_t LedC::Get_On_Level()
{
    return on_level;
}

void LedC::Set_Status(LED_State_e set_state)
{
    state = set_state;
}

void LED_Init(void)
{
	LEDIO_ConfigInit();
	LedC led(GPIOE, GPIO_PIN_11, LED_ON);
	led.setStatus(LED_TOGGLE);
}