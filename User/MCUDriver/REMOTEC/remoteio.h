//
// Created by cr on 22-9-12.
//

#ifndef ROBO_TEST_REMOTEIO_H
#define ROBO_TEST_REMOTEIO_H

//#include <cstdint>
#include "stm32f4xx_hal.h"

extern void REMOTEIO_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
extern void RC_unable(void);
extern void RC_restart(uint16_t dma_buf_num);
//class remoteio {
//
//};


#endif //ROBO_TEST_REMOTEIO_H
