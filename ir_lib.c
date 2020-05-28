#include "ir_lib.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_soc.h"
#include "app_util_platform.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_gpiote.h"

static uint16_t *m_current_data_ptr;
static uint32_t m_bits_remaining;
static uint32_t m_integrated_time_us;
static uint32_t m_next_cc_for_update;
static bool     m_busy;
static uint32_t m_ir_pin;


static uint32_t pulse_count_calculate(uint32_t time_us)
{
    return (time_us + (IR_CARRIER_LOW_US + IR_CARRIER_HIGH_US) / 2) / (IR_CARRIER_LOW_US + IR_CARRIER_HIGH_US);
}

static nrf_ppi_channel_t ppi_channel1, ppi_channel2, ppi_channel3, ppi_channel4, ppi_channel5;
static nrf_ppi_channel_group_t ppi_group1;
uint32_t ir_lib_init(uint32_t ir_pin)
{
    uint32_t err_code = 0;
		m_ir_pin = ir_pin;

		if(!nrf_drv_gpiote_is_init())
		{
				err_code = nrf_drv_gpiote_init();
		}

		nrf_drv_ppi_init();		
		/*
    NRF_GPIOTE->CONFIG[0] = GPIOTE_CONFIG_MODE_Task         << GPIOTE_CONFIG_MODE_Pos |
                            GPIOTE_CONFIG_OUTINIT_Low       << GPIOTE_CONFIG_OUTINIT_Pos |
                            GPIOTE_CONFIG_POLARITY_Toggle   << GPIOTE_CONFIG_POLARITY_Pos |
                            ir_pin                          << GPIOTE_CONFIG_PSEL_Pos;
    */
		nrf_drv_gpiote_out_config_t config =  
		{                                              
        .init_state = NRF_GPIOTE_INITIAL_VALUE_LOW,
        .task_pin   = true,                        
        .action     = NRF_GPIOTE_POLARITY_TOGGLE
    };

		err_code = nrf_drv_gpiote_out_init(ir_pin, &config);
		nrf_drv_gpiote_out_task_enable(ir_pin);
    // Carrier timer init
		//!! GENERATES CARRIER WAVE, I THINK
    IR_TIMER_CARRIER->MODE          = TIMER_MODE_MODE_Timer;
    IR_TIMER_CARRIER->BITMODE       = TIMER_BITMODE_BITMODE_16Bit;
    IR_TIMER_CARRIER->PRESCALER     = 4;
    IR_TIMER_CARRIER->CC[0]         = IR_CARRIER_LOW_US;    
    IR_TIMER_CARRIER->CC[1]         = IR_CARRIER_LOW_US + IR_CARRIER_HIGH_US; 
    IR_TIMER_CARRIER->SHORTS        = TIMER_SHORTS_COMPARE1_CLEAR_Msk;

    // Modulation timer init
    IR_CARRIER_COUNTER->MODE        = TIMER_MODE_MODE_Counter;  
    IR_CARRIER_COUNTER->BITMODE     = TIMER_BITMODE_BITMODE_16Bit;
    IR_CARRIER_COUNTER->INTENSET    = TIMER_INTENSET_COMPARE0_Msk | TIMER_INTENSET_COMPARE1_Msk;
    IR_CARRIER_COUNTER->EVENTS_COMPARE[0] = 0;
    IR_CARRIER_COUNTER->EVENTS_COMPARE[1] = 0;

    err_code |= sd_nvic_SetPriority(IR_CARRIER_COUNTER_IRQn, IR_CARRIER_COUNTER_IRQ_Priority);
    err_code |= sd_nvic_EnableIRQ(IR_CARRIER_COUNTER_IRQn);

		nrf_drv_ppi_channel_alloc(&ppi_channel1);
		nrf_drv_ppi_channel_alloc(&ppi_channel2);
		nrf_drv_ppi_channel_alloc(&ppi_channel3);
		nrf_drv_ppi_channel_alloc(&ppi_channel4);
		nrf_drv_ppi_channel_alloc(&ppi_channel5);
		
		//!! Whenever CC[1] == 1 increment the timer, ie count each carrier waves
    //err_code |= sd_ppi_channel_assign(IR_PPI_CH_A, &IR_TIMER_CARRIER->EVENTS_COMPARE[1], &IR_CARRIER_COUNTER->TASKS_COUNT);
    nrf_drv_ppi_channel_assign(ppi_channel1, (uint32_t) &IR_TIMER_CARRIER->EVENTS_COMPARE[1], (uint32_t) &IR_CARRIER_COUNTER->TASKS_COUNT);
		//!! Generate the 38Khz waveform by toggling the ir_pin
		//!! Used in MARK i think
    //err_code |= sd_ppi_channel_assign(IR_PPI_CH_B, &IR_TIMER_CARRIER->EVENTS_COMPARE[0], &NRF_GPIOTE->TASKS_OUT[0]);
		nrf_drv_ppi_channel_assign(ppi_channel2, (uint32_t) &IR_TIMER_CARRIER->EVENTS_COMPARE[0], (uint32_t) nrf_drv_gpiote_out_task_addr_get(ir_pin));
    //err_code |= sd_ppi_channel_assign(IR_PPI_CH_C, &IR_TIMER_CARRIER->EVENTS_COMPARE[1], &NRF_GPIOTE->TASKS_OUT[0]);
		nrf_drv_ppi_channel_assign(ppi_channel3, (uint32_t) &IR_TIMER_CARRIER->EVENTS_COMPARE[1], (uint32_t) nrf_drv_gpiote_out_task_addr_get(ir_pin));

		//!!	Group MARK PPIs together
		nrf_drv_ppi_group_alloc(&ppi_group1);
		nrf_drv_ppi_channel_include_in_group(ppi_channel2, ppi_group1);
    //err_code |= sd_ppi_group_assign(IR_PPI_GROUP, 1 << IR_PPI_CH_B | 1 << IR_PPI_CH_C);
		nrf_drv_ppi_channel_include_in_group(ppi_channel3, ppi_group1);
    //err_code |= sd_ppi_group_task_disable(IR_PPI_GROUP);
		nrf_drv_ppi_group_disable(ppi_group1);
    
		//!! IR_CARRIER_COUNTER's CC[0] stores number of pulses needed for MARK, after reaching the we need SPACES
		//!! so turn of the output.
    //err_code |= sd_ppi_channel_assign(IR_PPI_CH_D, &IR_CARRIER_COUNTER->EVENTS_COMPARE[0], &NRF_PPI->TASKS_CHG[IR_PPI_GROUP].DIS);
		nrf_drv_ppi_channel_assign(ppi_channel4, (uint32_t) &IR_CARRIER_COUNTER->EVENTS_COMPARE[0], (uint32_t) nrf_drv_ppi_task_addr_group_disable_get(ppi_group1));
		//!! SPACES finished, turn on the output again
    //err_code |= sd_ppi_channel_assign(IR_PPI_CH_E, &IR_CARRIER_COUNTER->EVENTS_COMPARE[1], &NRF_PPI->TASKS_CHG[IR_PPI_GROUP].EN);
		nrf_drv_ppi_channel_assign(ppi_channel5, (uint32_t) &IR_CARRIER_COUNTER->EVENTS_COMPARE[1], (uint32_t) nrf_drv_ppi_task_addr_group_enable_get(ppi_group1));
		
		//err_code |= sd_ppi_channel_enable_set(1 << IR_PPI_CH_A | 1 << IR_PPI_CH_B | 1 << IR_PPI_CH_C | 1 << IR_PPI_CH_D | 1 << IR_PPI_CH_E);
    
		nrf_drv_ppi_channel_enable(ppi_channel1);
		nrf_drv_ppi_channel_enable(ppi_channel2);
		nrf_drv_ppi_channel_enable(ppi_channel3);
		nrf_drv_ppi_channel_enable(ppi_channel4);
		nrf_drv_ppi_channel_enable(ppi_channel5);
		
    
    m_busy = false;
    return err_code;
}

void ir_lib_uninit()
{
	// Small hack to uninit timers, since I am lazy to convert Timer HAL codes, to Driver codes :P
	IR_TIMER_CARRIER->TASKS_SHUTDOWN = 1;
	IR_CARRIER_COUNTER->TASKS_SHUTDOWN = 1;
	IR_TIMER_CARRIER->CC[0] = 0;
	IR_TIMER_CARRIER->CC[1] = 0;
	IR_TIMER_CARRIER->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Disabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
	IR_CARRIER_COUNTER->INTENCLR = (TIMER_INTENCLR_COMPARE0_Clear << TIMER_INTENCLR_COMPARE0_Pos) | 
																	(TIMER_INTENCLR_COMPARE1_Clear << TIMER_INTENCLR_COMPARE1_Pos);
	nrf_drv_ppi_uninit();
	nrf_drv_gpiote_out_uninit(m_ir_pin);
}

void ir_lib_send(uint16_t *time_us, uint32_t length)
{
    while(m_busy);
    m_busy = true;
    
    m_current_data_ptr = time_us;
    
    m_integrated_time_us = *m_current_data_ptr++;
    IR_CARRIER_COUNTER->CC[0] = pulse_count_calculate(m_integrated_time_us);
    m_integrated_time_us += *m_current_data_ptr++;
    IR_CARRIER_COUNTER->CC[1] = pulse_count_calculate(m_integrated_time_us);
    
    m_bits_remaining = length;
    m_next_cc_for_update = 0;

    //sd_ppi_group_task_enable(IR_PPI_GROUP);
		nrf_drv_ppi_group_enable(ppi_group1);
    
    IR_CARRIER_COUNTER->TASKS_CLEAR = 1;
    IR_CARRIER_COUNTER->TASKS_START = 1;
    
    IR_TIMER_CARRIER->TASKS_CLEAR = 1;
    IR_TIMER_CARRIER->TASKS_START = 1;
}

void IR_CARRIER_COUNTER_IRQHandler(void)
{
    while(IR_CARRIER_COUNTER->EVENTS_COMPARE[m_next_cc_for_update])
    {
        IR_CARRIER_COUNTER->EVENTS_COMPARE[m_next_cc_for_update] = 0;
        
        m_bits_remaining--;
        if(m_bits_remaining >= 2)
        {            
            m_integrated_time_us += *m_current_data_ptr++;
            IR_CARRIER_COUNTER->CC[m_next_cc_for_update] = pulse_count_calculate(m_integrated_time_us); 
        }
        else if(m_bits_remaining == 0)
        {
            IR_TIMER_CARRIER->TASKS_STOP = 1;
            m_busy = false;
					
					//!! While creating the Decoder, I think I can disable CARRIER_COUNTER here.
					//!! LET'S SEE
        }
        m_next_cc_for_update = 1 - m_next_cc_for_update;
    }
}
