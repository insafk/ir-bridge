#ifndef __IR_DECODE_H
#define __IR_DECODE_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf_drv_swi.h"

#define DECODE_ARRAY_SIZE				1000
#define DECODE_TIMEOUT_TIMER_MILLISECONDS 1500 //TODO optimal value is 20

#define DECODE_TIMEOUT_TIMER                NRF_TIMER1
#define DECODE_TIMEOUT_TIMER_IRQn        	  TIMER1_IRQn
#define DECODE_TIMEOUT_TIMER_IRQHandler  	  TIMER1_IRQHandler
#define DECODE_TIMEOUT_TIMER_IRQ_Priority	  NRF_APP_PRIORITY_HIGH

#define DECODE_INTERVAL_TIMER              NRF_TIMER2

#define DECODE_PPI_CH_A							5


#define DECODE_PPI_GROUP        		1

uint32_t ir_decode_init(uint32_t tsop_pin, uint16_t** received_data, nrf_swi_t* my_swi);
void ir_decode_uninit(void);
#endif
