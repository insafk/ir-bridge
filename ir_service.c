#include <stdint.h>
#include <string.h>
#include "ir_service.h"
#include "ble_srv_common.h"
#include "app_error.h"

#include <stdio.h>
#include "ir_lib.h"
#include "ir_decode.h"
#include "app_timer.h"

static ble_ir_service_t * service_ptr_from_main;
static uint16_t *m_decoded_results_ptr;
static bool m_new_decoded_data = true;
static nrf_swi_t my_swi;
static bool m_decoder_busy = false;
static uint16_t array_index = 0;	//Used while ir transmission
static bool startReading = false;

APP_TIMER_DEF(m_ir_array_reception_timer_id);

uint16_t ir_transmit_data[1000];
static uint16_t ir_transmit_data_length  = 0;

static void send_decoded_results_init(void)
{
	m_new_decoded_data = true;
}

//Used to make the app more robust.. Timeout when the ir transmit array is not received on time from the peer device(android device, may be)
static void ir_array_reception_timeout_handler(void * p_context)
{
		printf("%d milliseconds passed, IR transmission failed, no data received in time.. Retransmit again(Neglect this msg, if IR transmission has been started)\n", IR_SERVICE_RECEPTION_TIMEOUT);
		array_index = 0;
		startReading = false;
}

//Used to stop reception when the remaining elements of the IR transmit array is not sent by the phone
static void ir_array_reception_timeout_timer_init()
{
		uint32_t err_code;
    err_code = app_timer_create(&m_ir_array_reception_timer_id, APP_TIMER_MODE_SINGLE_SHOT, ir_array_reception_timeout_handler);
    APP_ERROR_CHECK(err_code);
}

static void ir_array_reception_timeout_timer_start()
{
	app_timer_start(m_ir_array_reception_timer_id, APP_TIMER_TICKS(IR_SERVICE_RECEPTION_TIMEOUT, APP_TIMER_PRESCALER), NULL);
}


static void ir_array_reception_timeout_timer_stop()
{
	app_timer_stop(m_ir_array_reception_timer_id);
}

static void ir_array_reception_timeout_timer_restart()
{
	ir_array_reception_timeout_timer_stop();
	ir_array_reception_timeout_timer_start();
}
static void on_write(ble_ir_service_t * p_nus, ble_evt_t * p_ble_evt)
{
		ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_nus->decode_char_handle.cccd_handle)
        &&
        (p_evt_write->len == 2))
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_nus->is_notification_enabled = true;
        }
        else
        {
            p_nus->is_notification_enabled = false;
        }
    }
    else if (p_evt_write->handle == p_nus->transmit_char_handle.value_handle)
    {
			//Code to receive all IR segments
			uint8_t length_of_segment = p_evt_write->len;
			uint8_t seg_index = 0;
			
			if(!startReading)
			{
				//First 16bit should be the length of the IR data to be transmitted.
				ir_transmit_data_length = p_evt_write->data[0] | (p_evt_write->data[1] << 8);
				startReading = true;
				seg_index += 2;
				array_index = 0;
				printf("Length is 0x%x\n",ir_transmit_data_length);
			}
			
			//Restart timeout timer
			ir_array_reception_timeout_timer_restart();
			
			for(; seg_index < length_of_segment; seg_index+=2)
			{
				ir_transmit_data[array_index++] = p_evt_write->data[seg_index] |  (p_evt_write->data[seg_index+1] << 8);
				printf("ReceivedArray[%d] = %d\n", array_index, ir_transmit_data[array_index-1]);
				if(array_index >= ir_transmit_data_length)
				{
					ir_array_reception_timeout_timer_stop();
					startReading = false;
	
//					printf("Received data is...\n");
//					for(uint16_t i = 0; i < ir_transmit_data_length; ++i)
//						printf("0x%x, ", ir_transmit_data[i]);
//					printf("\nEnds\n");
					
					if(m_decoder_busy)
					{
						printf("Transmitter and Decoder cannot work simultaneously, turning off decoder...\n");
						ir_decode_uninit();
						m_decoder_busy = false;
					}
					
					ir_array_reception_timeout_timer_stop();
					ir_lib_init(IR_TRANSMIT_PIN);
					printf("Transmitting IR...\n");
					ir_lib_send(ir_transmit_data, ir_transmit_data_length);
					
					break;
				}
			}
    }
   else if (p_evt_write->handle == p_nus->start_decode_char_handle.value_handle)
    {
			for(uint16_t i = 0; i < p_evt_write->len; ++i)
				printf("%d, ", p_evt_write->data[i]);
			if(!m_decoder_busy)
			{
				m_decoder_busy = true;
				printf("Starting decoding..\n");
				ir_lib_uninit();
				ir_decode_init(IR_DECODE_PIN, &m_decoded_results_ptr, &my_swi);
			}
			else
			{
				printf("Decoder already on, try again later...\n");
			}
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}

uint32_t  send_chunks_of_decoded_results(ble_ir_service_t *p_our_service, uint8_t *decoded_data, uint16_t offset, uint16_t length)
{
		uint32_t err_code = NRF_SUCCESS;
		if(p_our_service->is_notification_enabled == false)
		{
				printf("Notification not enabled\n");
				return err_code;
		}
   
		if (p_our_service->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
				ble_gatts_hvx_params_t hvx_params;
        uint16_t               hvx_len;
				memset(&hvx_params, 0, sizeof(hvx_params));

				hvx_len = length;
				
				decoded_data += offset;
			
				hvx_params.handle = p_our_service->decode_char_handle.value_handle;
				hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
				hvx_params.offset = 0;
				hvx_params.p_len  = &hvx_len;
				hvx_params.p_data = decoded_data; 
			
	
				err_code = sd_ble_gatts_hvx(p_our_service->conn_handle, &hvx_params);
     /*   if ((err_code == NRF_SUCCESS) && (hvx_len != length))
        {
            err_code = NRF_ERROR_DATA_SIZE;
        }
			*/
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}


static void send_decoded_results_wrapper(void)
{
	//In BLE MTU is 20bytes, so in this function we make chunks of 20bytes decoded_data and send it over BLE using
	// send_chunks_of_decoded_results function
	static uint32_t cnt = 0;
	static bool transfer_over = false;	//This function is called again when TX buffer is empty, so this flag is used to denote whether the transaction is over or not
	if(m_new_decoded_data)
	{
		cnt = 0;
		transfer_over = false;
		m_new_decoded_data = false;
	}
	if(transfer_over)
		return;
	
	uint32_t        err_code = NRF_SUCCESS;
	uint16_t				length = m_decoded_results_ptr[0]+1;	//1 is added because first position contains the length
	
	//Need to convert the 16bit array to 8bit array
	//Let's make LSB comes first(Little Endian)
	//But the data is already in little endian, so let's cast the uint16_t to uint8_t
	
	uint8_t					*decoded_8bit_data = (uint8_t*) m_decoded_results_ptr;
	uint16_t				eight_bit_len = length * 2;
	//After this we only deal with 8bit data.
	int n_chunks = eight_bit_len/20;
	
	//This loop works only if data is more than 20bytes. Send last chunk of data after this loop
	for (; cnt < n_chunks; ++cnt)
	{

		uint16_t offset = 20*cnt;
		err_code = send_chunks_of_decoded_results(service_ptr_from_main, decoded_8bit_data, offset, 20);
		if (err_code == BLE_ERROR_NO_TX_BUFFERS ||
				err_code == NRF_ERROR_INVALID_STATE || 
				err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
		{
				printf("Buffer Full... Waiting...(Loop)\n");
				return;
		}
		else if (err_code != NRF_SUCCESS) 
		{
				printf("Error in send_decoded_results_wrapper loop...\n");
				APP_ERROR_HANDLER(err_code);
		}
	}
	//Sending last chunk of packet, if any
	
	if(eight_bit_len % 20)
	{
		uint16_t offset = cnt*20;
		uint16_t len = eight_bit_len - offset;
		err_code = send_chunks_of_decoded_results(service_ptr_from_main, decoded_8bit_data, offset, len);
		if (err_code == BLE_ERROR_NO_TX_BUFFERS ||
				err_code == NRF_ERROR_INVALID_STATE || 
				err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
		{
				printf("Buffer Full... Waiting...(IF)\n");
				return;
		}
		else if (err_code != NRF_SUCCESS) 
		{
				printf("Error in send_decoded_results_wrapper last IF...\n");
				APP_ERROR_HANDLER(err_code);
		}
	}
	//If control reaches here, then the transaction is success
	transfer_over = true;
}



void ble_ir_service_on_ble_evt(ble_ir_service_t * p_our_service, ble_evt_t * p_ble_evt)
{
		switch (p_ble_evt->header.evt_id)
		{
				case BLE_GAP_EVT_CONNECTED:
						p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						break;
				case BLE_GAP_EVT_DISCONNECTED:
						p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
				
						break;
				case BLE_GATTS_EVT_WRITE:
						printf("BLE write received\n");
            on_write(p_our_service, p_ble_evt);
            break;
				 case BLE_EVT_TX_COMPLETE:
						send_decoded_results_wrapper();
            break;
				default:
						// No implementation needed.
						break;
		}
}

static uint32_t ir_char_decode_add(ble_ir_service_t * p_our_service)
{
    uint32_t            err_code;
		ble_uuid_t          char_uuid;
		ble_uuid128_t       base_uuid = BLE_UUID_IR_BASE_UUID;
		char_uuid.uuid      = BLE_UUID_IR_DECODE_CHARACTERISTIC_UUID;
		err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
		APP_ERROR_CHECK(err_code);
	
		ble_gatts_char_md_t char_md;
		memset(&char_md, 0, sizeof(char_md));
	
		char_md.char_props.read = 1;
		char_md.char_props.write = 1;
    char_md.char_props.write_wo_resp = 1;
    
		ble_gatts_attr_md_t cccd_md;
		memset(&cccd_md, 0, sizeof(cccd_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
	
		char_md.p_cccd_md           = &cccd_md;
		char_md.char_props.notify   = 1; 
	
	
    
		ble_gatts_attr_md_t attr_md;
		memset(&attr_md, 0, sizeof(attr_md));
		attr_md.vloc        = BLE_GATTS_VLOC_STACK; 
		attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen 				= 1;			//Variable length flag
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
		ble_gatts_attr_t    attr_char_value;
		memset(&attr_char_value, 0, sizeof(attr_char_value));    
		attr_char_value.p_uuid      = &char_uuid;
		attr_char_value.p_attr_md   = &attr_md;
		
		attr_char_value.max_len     = BLE_IR_DECODE_CHARACTERISTIC_LENGTH;
		attr_char_value.init_len    = BLE_IR_DECODE_CHARACTERISTIC_LENGTH;
		attr_char_value.p_value     = (uint8_t*) 0;
		
		err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
																			 &char_md,
																			 &attr_char_value,
																			 &p_our_service->decode_char_handle);
		APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

static uint32_t ir_char_transmit_add(ble_ir_service_t * p_our_service)
{
    // Add a custom characteristic UUID
    uint32_t            err_code;
		ble_uuid_t          char_uuid;
		ble_uuid128_t       base_uuid = BLE_UUID_IR_BASE_UUID;
		char_uuid.uuid      = BLE_UUID_IR_TRANSMIT_CHARACTERISTIC_UUID;
		err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
		APP_ERROR_CHECK(err_code);
	
    // Add read/write properties to our characteristic
		ble_gatts_char_md_t char_md;
		memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1;
		char_md.char_props.write = 1;
    
    // Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
		ble_gatts_attr_md_t cccd_md;
		memset(&cccd_md, 0, sizeof(cccd_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
		char_md.p_cccd_md           = &cccd_md;
		char_md.char_props.notify   = 1; 
	
	
    
    // Configure the attribute metadata
		ble_gatts_attr_md_t attr_md;
		memset(&attr_md, 0, sizeof(attr_md));
		attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    attr_md.vlen 				=1;			//Variable length flag 
    
    // Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    // OConfigure the characteristic value attribute
		ble_gatts_attr_t    attr_char_value;
		memset(&attr_char_value, 0, sizeof(attr_char_value));    
		attr_char_value.p_uuid      = &char_uuid;
		attr_char_value.p_attr_md   = &attr_md;
    // Set characteristic length in number of bytes
		attr_char_value.max_len     = BLE_IR_TRANSMIT_CHARACTERISTIC_LENGTH;
		attr_char_value.init_len    = 4;
		uint8_t value[4]            = {0x12,0xab,0x56,0x78};
		attr_char_value.p_value     = value;
    // OAdd our new characteristic to the service
		err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
																			 &char_md,
																			 &attr_char_value,
																			 &p_our_service->transmit_char_handle);
		APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}	

static uint32_t ir_char_start_decode_add(ble_ir_service_t * p_our_service)
{
    // Add a custom characteristic UUID
    uint32_t            err_code;
		ble_uuid_t          char_uuid;
		ble_uuid128_t       base_uuid = BLE_UUID_IR_BASE_UUID;
		char_uuid.uuid      = BLE_UUID_START_DECODE_CHARACTERISTIC_UUID;
		err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
		APP_ERROR_CHECK(err_code);
	
    // Add read/write properties to our characteristic
		ble_gatts_char_md_t char_md;
		memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1;
		char_md.char_props.write = 1;
    
    // Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure
		ble_gatts_attr_md_t cccd_md;
		memset(&cccd_md, 0, sizeof(cccd_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
		char_md.p_cccd_md           = &cccd_md;
		char_md.char_props.notify   = 1; 
	
	
    
    // Configure the attribute metadata
		ble_gatts_attr_md_t attr_md;
		memset(&attr_md, 0, sizeof(attr_md));
		attr_md.vloc        = BLE_GATTS_VLOC_STACK;    
    // Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    // Configure the characteristic value attribute
		ble_gatts_attr_t    attr_char_value;
		memset(&attr_char_value, 0, sizeof(attr_char_value));    
		attr_char_value.p_uuid      = &char_uuid;
		attr_char_value.p_attr_md   = &attr_md;
    // Set characteristic length in number of bytes
		attr_char_value.max_len     = 1;
		attr_char_value.init_len    = 1;
		uint8_t value[1]            = {0};
		attr_char_value.p_value     = value;
    // Add our new characteristic to the service
		err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
																			 &char_md,
																			 &attr_char_value,
																			 &p_our_service->start_decode_char_handle);
		APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}	


void swi_event_handler(nrf_swi_t swi, nrf_swi_flags_t flags)
{
    /* If "my_swi" was triggered and flag #3 is present... */
		
    if ((swi == my_swi) && (flags & (1 << 3)))
    {
				//printf("Decoded results received in ir_service.c, transmitting to the phone\n");
	
			printf("data has been received in ir_service.c, length is %d\n", m_decoded_results_ptr[0]);
		
			m_decoder_busy = false; 	//Decoder is free now
			send_decoded_results_init();
			send_decoded_results_wrapper();
    }
		
    return;
}


void ir_service_init(ble_ir_service_t * p_our_service)
{
    // STEP 3: Declare 16 bit service and 128 bit base UUIDs and add them to BLE stack table     
		uint32_t 					err_code;
		ble_uuid_t        service_uuid;
		ble_uuid128_t 		base_uuid = BLE_UUID_IR_BASE_UUID;
	
		service_uuid.uuid = BLE_UUID_IR_SERVICE;
		err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
		APP_ERROR_CHECK(err_code);

		p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
		service_ptr_from_main = p_our_service;
	
		// Add our service
		err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                    &service_uuid,
                                    &p_our_service->service_handle);
		APP_ERROR_CHECK(err_code);
	
	
	    //  Call the function ..._char_add() to add our new characteristic to the service. 
			ir_char_decode_add(p_our_service);
			ir_char_transmit_add(p_our_service);
			ir_char_start_decode_add(p_our_service);
			
			  
		  //SWI
			err_code = nrf_drv_swi_init();
			APP_ERROR_CHECK(err_code);
			err_code = nrf_drv_swi_alloc(&my_swi, swi_event_handler, APP_IRQ_PRIORITY_LOW);
			APP_ERROR_CHECK(err_code);
			
			//Used in ir_service.c for timeout
			APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);


			ir_array_reception_timeout_timer_init();
}
