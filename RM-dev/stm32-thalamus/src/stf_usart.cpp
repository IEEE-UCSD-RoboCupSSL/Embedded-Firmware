#include "stf_usart.h"
#include "stf_systick.h"

#ifdef HAL_UART_MODULE_ENABLED


namespace stf {
    std::string endl("\n\r");
}

using namespace stf;

/*==================================================================*/
/* active_usarts store pointers to all instantiated USART objects
 * globally, for ISR(interrupt service routine) extern functions to assign 
 * interrupt task for each different usart object. Ideally a hashtable or 
 * vector container should be implemented instead of a pointer array with 
 * a magic number of capacity, but ISR only support simple C-style 
 * operations, let's stick with the old-school fashion!
 * */
uint32_t num_usarts = 0;
USART* active_usarts[Max_Num_USARTs];
/*==================================================================*/



USART::USART(UART_HandleTypeDef *huartx, uint32_t tx_buffer_size, uint32_t rx_buffer_size) {
    this->huartx = huartx;

    tx_buffer = new char(tx_buffer_size);
    rx_buffer = new char(rx_buffer_size);

    tx_status = Initialized;
    rx_status = Initialized;
    active_usarts[num_usarts++] = this;
}

USART::~USART() {
    delete tx_buffer;
    delete rx_buffer;
}



void USART::hal_transmit(char* str_ptr, periph_mode mode) {
	if(tx_status == NotReady) return;

    HAL_StatusTypeDef status;
    if(mode == Polling) {
        status = HAL_UART_Transmit(huartx, (uint8_t*)(str_ptr), strlen(str_ptr), tx_timeout);
        if(status == HAL_BUSY) tx_status = InProgress;
        else if(status == HAL_TIMEOUT) {
            tx_status = TimeOut;

            //Unlock Usart
            __HAL_UNLOCK(huartx);
            huartx->gState = HAL_UART_STATE_READY;
            
            exception("usart_transmit Polling | TimeOut");
        }
        else if(status == HAL_ERROR) {
            tx_status = Error;
            exception("usart_transmit Polling | Error");
        }
        else if(status == HAL_OK) {
            tx_status = Completed;
        }
    }

    // Make sure interrupt is enabled within CubeMx software
    if(mode == Interrupt) {
        //check if the previous transmission is completed
        if(tx_status == InProgress) return;
        status = HAL_UART_Transmit_IT(huartx, (uint8_t*)str_ptr, strlen(str_ptr));
        if(status == HAL_ERROR) {
            tx_status = Error;
            exception("usart_transmit interrupt | Error");
            return;
        }
        tx_status = InProgress;
    }

    // Make sure DMA and interrupt are both enabled within CubeMx software
    if(mode == DMA) {
        //check if the previous transmission is completed
        if(tx_status == InProgress) return;
        status = HAL_UART_Transmit_DMA(huartx, (uint8_t*)str_ptr, strlen(str_ptr));
        if(status == HAL_ERROR) {
            tx_status = Error;
            exception("usart_transmit DMA | Error");
            return;
        }
        tx_status = InProgress;
    }
}



void USART::transmit(std::string& str, periph_mode mode) {
	hal_transmit((char*)str.c_str(), mode);
}


void USART::transmit(char* str_ptr, periph_mode mode) {
    hal_transmit(str_ptr, mode);
}

void USART::transmit(byte_t byte, periph_mode mode) {
    tx_buffer[0] = (char)byte;
    tx_buffer[1] = '\0';
    hal_transmit(tx_buffer, mode);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {    
	for(uint32_t i = 0; i < num_usarts; i++) {
		if(active_usarts[i]->get_huartx() == huart) {
            usart_transmit_completed_interrupt_task(active_usarts[i]);
            active_usarts[i]->set_tx_status(Completed);
		}
	}
}

__weak void usart_transmit_completed_interrupt_task(USART* instance) {
    UNUSED(instance);
}


void USART::hal_receive(char* str_ptr, uint16_t num_bytes, periph_mode mode) {
    if(rx_status == NotReady) return;

    HAL_StatusTypeDef status;
    if(mode == Polling) {
        
	    memset(str_ptr, 0, strlen(str_ptr));
        status = HAL_UART_Receive(huartx, (uint8_t*)(str_ptr), num_bytes, rx_timeout);
        if(status == HAL_BUSY) rx_status = InProgress;
        else if(status == HAL_TIMEOUT) {
            rx_status = TimeOut;

            //Unlock Usart
            __HAL_UNLOCK(huartx);
            huartx->gState = HAL_UART_STATE_READY;
            
            exception("usart_receive Polling | TimeOut");
        }
        else if(status == HAL_ERROR) {
            rx_status = Error;
            exception("usart_receive Polling | Error");
        }
        else if(status == HAL_OK) {
            rx_status = Completed;
        }
    }

    // Make sure interrupt is enabled within CubeMx software
    // Not recommended, input data size has to be exact, or weird behavior might occur 
    if(mode == Interrupt) {
        //check if the previous receiption is completed
        if(rx_status == InProgress) return;
        memset(str_ptr, 0, strlen(str_ptr));
        status = HAL_UART_Receive_IT(huartx, (uint8_t*)str_ptr, num_bytes);
        if(status == HAL_ERROR) {
            rx_status = Error;
            exception("usart_receive interrupt | Error");
            return;
        }
        rx_status = InProgress;
    }

    // Make sure DMA and interrupt are both enabled within CubeMx software
    if(mode == DMA) {
        //check if the previous reception is completed
        if(rx_status == InProgress) return;
        memset(str_ptr, 0, strlen(str_ptr));
        status = HAL_UART_Receive_DMA(huartx, (uint8_t*)str_ptr, num_bytes);
        if(status == HAL_ERROR) {
            rx_status = Error;
            exception("usart_receive DMA | Error");
            return;
        }
        rx_status = InProgress;
    }
}

/* only support Polling mode if the return type is a string, 
 * this is due to c++ string constructor only does deep copy 
 * instead of shallow copy, can't copy a char array (rx_buffer) right away
 * when non-blocking receive is still in the middle of filling
 * content into the array
 * (non-blocking methods are {Interrupt, DMA})
 */
std::string USART::receive(uint16_t num_bytes) {
    hal_receive(rx_buffer, num_bytes, Polling);
    if(rx_status == Completed) 
        return std::string(rx_buffer);
    else return ""; // if error occurs
}


void USART::receive(char* buffer_ptr, uint16_t num_bytes, periph_mode mode) {
    hal_receive(buffer_ptr, num_bytes, mode);
}


/*only support Polling mode due to the peripheral needs 
 *to finish receiption before returning the results
 *it's inherently a blocking method
 */
byte_t USART::receive(void) {
    hal_receive(rx_buffer, 1, Polling);
    if(rx_status == Completed) return (byte_t)rx_buffer[0];
    else return 0; // if error occurs
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    for(uint32_t i = 0; i < num_usarts; i++) {
		if(active_usarts[i]->get_huartx() == huart) {
            usart_receive_completed_interrupt_task(active_usarts[i]);
            active_usarts[i]->set_rx_status(Completed);
		}
	}
}

__weak void usart_receive_completed_interrupt_task(USART* instance) {
    UNUSED(instance);
}


// polling mode
const std::string USART::readWord(void) {
    uint32_t t0 = millis();

    int i = 0;
    char str[USART_Default_Rx_BufferSize];
	str[i] = (char)(this->receive());
	while(str[i] != ' ' && str[i] != '\r' && str[i] != '\n') {
		str[++i] = (char)(this->receive());
        if(millis() - t0 > USART_Default_RxTimeOut) break;
    }
	str[i] = '\0';
    const std::string rtn(str);
    return rtn;
}

// polling mode
const std::string USART::readLine(void) {
	
    uint32_t t0 = millis();

    int i = 0;
    char str[USART_Default_Rx_BufferSize];
	str[i] = (char)(this->receive());
	while(str[i] != '\r' && str[i] != '\n') {
		str[++i] = (char)(this->receive());
        if(millis() - t0 > USART_Default_RxTimeOut) break;
    }
	str[i] = '\0';
    const std::string rtn(str);
    return rtn;
}

#endif
