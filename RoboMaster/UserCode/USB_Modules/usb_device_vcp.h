/*
 * usb_comm.h
 *
 *  Created on: 2021年1月20日
 *      Author: zhang
 */

#ifndef USB_DEVICE_VCP_H_
#define USB_DEVICE_VCP_H_



#include "stf.h"
#include "usbd_cdc_if.h"

#include "message_buffer.h"
#include <iostream>

#define PACKET_SIZE 64 // 64 bytes is the default packet size for USB2.0 FS
#define RX_MSG_BUF_SIZE 100 // bytes

/* This is not a complete library class that deals with much more edge cases,
 * but it can be upgraded to be like one of the stm32-thalamus-framework(stf)
 * source without too much difficulty.
 *
 * Unlike other peripherals oop-lized in stf library, this usb_vcp doesn't
 * support instantiating multiple USB devices, please use this class as singleton. (for reducing usage styling inconsistency, the real singleton pattern is not used)
 *
 * (priority is given to simplicity & rapid development for the current project instead
 * of its library generalizing capability, I might integrate this usb usage into stf library
 * in the future) */
class USB_VCP {
public:
	USB_VCP(void);

	void init(void);

	/* Tx methods*/
	void send_packet(std::string& str);
	void send_packet(const char* str);
	void send_packet(byte_t* bytes_ptr, uint16_t num_bytes);

	/* Rx methods*/
	std::string& read_some(void);
	std::string& read_line(char delim = '\r');


	inline uint32_t get_tx_buffer_size(void) {return APP_TX_DATA_SIZE;}
	inline uint32_t get_rx_buffer_size(void) {return APP_RX_DATA_SIZE;}
private:
	std::stringstream rx_ss;
	std::string some_str;
	std::string line_str;
	char rx_msg_buf[RX_MSG_BUF_SIZE];
};


extern "C" void CDC_Received_FS_Callback(char* buf, uint32_t len);



#endif /* USB_COMMU_H_ */
