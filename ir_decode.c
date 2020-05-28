#include "ir_decode.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_soc.h"
#include "app_util_platform.h"

//Insaf
#include <stdio.h>
#include "nrf_drv_gpiote.h"
#include "nrf_drv_ppi.h"
#include "app_error.h"
#include "nrf_drv_swi.h"


static uint16_t m_received_data[DECODE_ARRAY_SIZE];
static bool			m_timer_on;
static uint32_t m_tsop_pin;
static uint32_t err_code = NRF_SUCCESS;
static uint16_t m_array_index;
static nrf_swi_t* m_swi;

static nrf_ppi_channel_t ppi_channel1;

static void get_timer2_val_and_store_in_array()
{
	DECODE_INTERVAL_TIMER->TASKS_CAPTURE[0]=1;
	m_received_data[m_array_index++] = (uint16_t)DECODE_INTERVAL_TIMER->CC[0];
		printf("%d --%4x ",m_array_index-1, m_received_data[m_array_index-1]);
	DECODE_INTERVAL_TIMER->TASKS_CLEAR = 1;
}

//static void print_array_val()
//{	
//	printf("\n\nArray Values: Length(%d) ", m_received_data[0]);
//	for(int i = 1; i < m_array_index; i+=2)
//	{
//		printf("(%d, %d)\n", m_received_data[i], m_received_data[i+1]);
//	}
//	printf("\n\n");
//}

void DECODE_TIMEOUT_TIMER_IRQHandler(void)
{    
//	printf("Timeout event handler called\n");
    if(DECODE_TIMEOUT_TIMER->EVENTS_COMPARE[0])
    {
			//STOP everything
			void ir_decode_uninit();
		  DECODE_TIMEOUT_TIMER->EVENTS_COMPARE[0] = 0;
			
			get_timer2_val_and_store_in_array();
			printf("Timeout");
			DECODE_TIMEOUT_TIMER->TASKS_SHUTDOWN = 1;
			DECODE_INTERVAL_TIMER->TASKS_SHUTDOWN = 1;
		  nrf_drv_ppi_channel_disable(ppi_channel1);
			nrf_drv_gpiote_in_uninit(m_tsop_pin);
			
			m_received_data[0] = m_array_index-1;
			ir_decode_uninit();
			nrf_drv_swi_trigger(*m_swi, 3);
    }    
}

void tsop_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	printf("~~\n");
	if(pin == m_tsop_pin && action == NRF_GPIOTE_POLARITY_TOGGLE )
	{
		if(!m_timer_on)
		{
			printf("Starting decoding...\n");
			m_timer_on = true;
			err_code = nrf_drv_ppi_channel_enable(ppi_channel1);
			APP_ERROR_CHECK(err_code);
			
			DECODE_TIMEOUT_TIMER->TASKS_CLEAR = 1;
			DECODE_TIMEOUT_TIMER->TASKS_START = 1;
    
			DECODE_INTERVAL_TIMER->TASKS_CLEAR = 1;
			DECODE_INTERVAL_TIMER->TASKS_START = 1;
		}
		get_timer2_val_and_store_in_array();
	}
}

static void gpiote_init()
{
	if(!nrf_drv_gpiote_is_init())
	{
    err_code = nrf_drv_gpiote_init();
	}
	
	nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
	config.pull = NRF_GPIO_PIN_PULLUP;
	
	err_code =  nrf_drv_gpiote_in_init(m_tsop_pin, &config, tsop_pin_handler);
	nrf_drv_gpiote_in_event_enable(m_tsop_pin, true);

}

static void decode_timeout_timer_init()
{
		DECODE_TIMEOUT_TIMER->MODE          = TIMER_MODE_MODE_Timer;
    DECODE_TIMEOUT_TIMER->BITMODE       = TIMER_BITMODE_BITMODE_16Bit;
    DECODE_TIMEOUT_TIMER->PRESCALER     = 9;
		DECODE_TIMEOUT_TIMER->EVENTS_COMPARE[0] = 0;
    DECODE_TIMEOUT_TIMER->INTENSET    = TIMER_INTENSET_COMPARE0_Msk;
	
		NVIC_SetPriority(DECODE_TIMEOUT_TIMER_IRQn, APP_IRQ_PRIORITY_HIGH);
    NVIC_EnableIRQ(DECODE_TIMEOUT_TIMER_IRQn);

		uint32_t time_out_ms = DECODE_TIMEOUT_TIMER_MILLISECONDS; //Timeout time(in milliseconds).
		uint32_t time_ticks;
		time_ticks =  time_out_ms / .032;
		
    DECODE_TIMEOUT_TIMER->CC[0] = time_ticks; 
}

static void timer2_init()
{
		DECODE_INTERVAL_TIMER->MODE          = TIMER_MODE_MODE_Timer;
    DECODE_INTERVAL_TIMER->BITMODE       = TIMER_BITMODE_BITMODE_16Bit;
    DECODE_INTERVAL_TIMER->PRESCALER     = 4;
}

static void ppi_init(void)
{
    uint32_t err_code = NRF_SUCCESS;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    // Configure 1st available PPI channel to restart timeout timer
    err_code = nrf_drv_ppi_channel_alloc(&ppi_channel1);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_assign(ppi_channel1,
                                          nrf_drv_gpiote_in_event_addr_get(m_tsop_pin),
                                          (uint32_t)&DECODE_TIMEOUT_TIMER->TASKS_CLEAR);
    APP_ERROR_CHECK(err_code);

}

void ir_decode_uninit(void)
{
	nrf_drv_gpiote_in_uninit(m_tsop_pin);
	nrf_drv_gpiote_uninit ();
	nrf_drv_ppi_uninit();
	DECODE_TIMEOUT_TIMER->INTENCLR    = (TIMER_INTENCLR_COMPARE0_Clear << TIMER_INTENCLR_COMPARE0_Pos);
}


uint32_t ir_decode_init(uint32_t tsop_pin, uint16_t** received_data, nrf_swi_t* my_swi)
{
		printf("in irdecode.h");
		m_tsop_pin = tsop_pin;
		m_swi = my_swi;
		*received_data = m_received_data;
		m_timer_on = false; // Whenever a new signal appears, the timer has to be turned on once, by the GPIOTE interrupt handler
		m_array_index = 1; //Used to point to the next empty location in array
    err_code = NRF_SUCCESS;
		decode_timeout_timer_init();
		timer2_init();
		gpiote_init();
		ppi_init();
    return err_code;
}
