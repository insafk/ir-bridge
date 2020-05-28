#ifndef IR_SERVICE_H__
#define IR_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BLE_UUID_IR_BASE_UUID              {0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00} // 128-bit base UUID
#define BLE_UUID_IR_SERVICE                0xFACE // Just a random, but recognizable value

#define BLE_UUID_START_DECODE_CHARACTERISTIC_UUID		 0xDEAD
#define BLE_UUID_IR_DECODE_CHARACTERISTIC_UUID		 0xCAFE
#define BLE_UUID_IR_TRANSMIT_CHARACTERISTIC_UUID   0xF00D
#define BLE_IR_TRANSMIT_CHARACTERISTIC_LENGTH 			20	//Max payload size is 20 bytes
#define BLE_IR_DECODE_CHARACTERISTIC_LENGTH 				20  //Max payload size is 20 bytes
#define IR_DECODE_PIN												4
#define IR_TRANSMIT_PIN											21
#define IR_SERVICE_RECEPTION_TIMEOUT				5000			//in millisecs

#define APP_TIMER_PRESCALER              12                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */

/**
 * @brief This structure contains various status information for our service. 
 * It only holds one entry now, but will be populated with more items as we go.
 * The name is based on the naming convention used in Nordic's SDKs. 
 * 'ble’ indicates that it is a Bluetooth Low Energy relevant structure and 
 * ‘os’ is short for IR Service). 
 */
typedef struct
{
	uint16_t                    conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
  uint16_t    								service_handle;     /**< Handle of Our Service (as provided by the BLE stack). */
	ble_gatts_char_handles_t    decode_char_handle;
	ble_gatts_char_handles_t    transmit_char_handle;
	ble_gatts_char_handles_t    start_decode_char_handle;
		bool                     is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
    
	    // OUR_JOB: Step 2.D, Add handles for the characteristic attributes to our struct

}ble_ir_service_t;

/**@brief Function for initializing our new service.
 *
 * @param[in]   p_our_service       Pointer to Our Service structure.
 */
void ir_service_init(ble_ir_service_t * p_our_service);



/**@brief Function for handling BLE Stack events related to our service and characteristic.
 *
 * @details Handles all events from the BLE stack of interest to Our Service.
 *
 * @param[in]   p_our_service       Our Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_ir_service_on_ble_evt(ble_ir_service_t * p_our_service, ble_evt_t * p_ble_evt);


/**@brief Function for updating and sending new characteristic values
 *
 * @details The application calls this function whenever our timer_timeout_handler triggers
 *
 * @param[in]   p_our_service                     Our Service structure.
 * @param[in]   characteristic_value     New characteristic value.
 */
uint32_t  send_decoded_results_back_to_client(ble_ir_service_t *p_our_service, uint16_t *decoded_data);

#endif  /* _ IR_SERVICE_H__ */
