/*
 * usb_comm.cpp
 *
 *  Created on: 2021年1月20日
 *      Author: zhang
 */


#include "usb_device_vcp.h"
#include "FreeRTOS.h"

uint32_t num_usbvcps = 0;

MessageBufferHandle_t msg_buf = NULL;

USB_VCP::USB_VCP(void) {
	/* for consistent usage styling, real singleton pattern is chosen not to be implemented */
	if(++num_usbvcps > 1) {
		stf::exception("Can't instantiate more than one USB_VCP (as device)");
	}
	if(msg_buf == NULL) {
		msg_buf = xMessageBufferCreate(RX_MSG_BUF_SIZE);
		if(msg_buf == NULL) {
			stf::exception("No enough heap for USB Rx Msg Buf");
		}
	}
}

void USB_VCP::init(void) {
	stf::delay(1000);
}

/* Tx methods*/
void USB_VCP::send_packet(std::string& str) {
	char *cstr = (char*)str.c_str();
	CDC_Transmit_FS((byte_t*)cstr, strlen(cstr));
}
void USB_VCP::send_packet(const char* str) {
	CDC_Transmit_FS((byte_t*)str, strlen(str));
}

void USB_VCP::send_packet(byte_t* bytes_ptr, uint16_t num_bytes) {
	CDC_Transmit_FS(bytes_ptr, num_bytes);
}

/* Rx methods*/
std::string& USB_VCP::read_some() {
	size_t num_bytes_received;
	do {
		num_bytes_received = xMessageBufferReceive(msg_buf,
				(void*)rx_msg_buf, sizeof(rx_msg_buf), 300);
	} while(num_bytes_received <= 0);

	some_str = std::string((const char*)rx_msg_buf, num_bytes_received);
	return some_str;

}

std::string& USB_VCP::read_line(char delim) {
	std::string str;
	do {
		str = read_some();
		rx_ss << str;
	} while(str.find(delim) == std::string::npos); // if delimiter not found, keep read_some()

    std::getline(rx_ss, line_str, delim);
	return line_str;
}





char tmp[PACKET_SIZE];
// invoked during the interrupt that a packet is received (ISR Callback)
void CDC_Received_FS_Callback(char* buf, uint32_t len) {
	BaseType_t higher_priority_task_woken = pdFALSE; // change to true if a higher priority task is waiting for msg from this ISR callback
	size_t num_bytes_sent;
	// Avoid using c++ exclusive things in this part that runs in an ISR
	if(len <= PACKET_SIZE) {
		strncpy(tmp, buf, len);

		// send string to msg buf
		num_bytes_sent = xMessageBufferSendFromISR(msg_buf,
				(void*)tmp, len, &higher_priority_task_woken);

		if(num_bytes_sent != len) {
			// when buffer have no space left to enqueue
			/* No good way to handle this situation, simple discard any
			 * received packet once buffer is full */
		}

		// (user defined) taskYIELD_FROM_ISR(higher_priority_task_woken);

	}
}



