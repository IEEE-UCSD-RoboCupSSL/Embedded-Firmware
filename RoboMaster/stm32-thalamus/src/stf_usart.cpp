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

    tx_buffer = new byte_t(tx_buffer_size);
    rx_buffer = new byte_t(rx_buffer_size);

    tx_status = Initialized;
    rx_status = Initialized;
    active_usarts[num_usarts++] = this;
}

USART::~USART() {
    delete tx_buffer;
    delete rx_buffer;
}


void USART::hal_transmit(byte_t* bytes_ptr, uint16_t num_bytes, periph_mode mode) {
	if(tx_status == NotReady) return;

    HAL_StatusTypeDef status;
    if(mode == Polling) {
        status = HAL_UART_Transmit(huartx, bytes_ptr, num_bytes, tx_timeout);
        if(status == HAL_BUSY) tx_status = InProgress;
        else if(status == HAL_TIMEOUT) {
            tx_status = TimeOut;
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
        status = HAL_UART_Transmit_IT(huartx, bytes_ptr, num_bytes);
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
        status = HAL_UART_Transmit_DMA(huartx, bytes_ptr, num_bytes);
        if(status == HAL_ERROR) {
            tx_status = Error;
            exception("usart_transmit DMA | Error");
            return;
        }
        tx_status = InProgress;
    }
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

void USART::hal_receive(byte_t* bytes_ptr, uint16_t num_bytes, periph_mode mode) {
    if(rx_status == NotReady) return;

    HAL_StatusTypeDef status;
    if(mode == Polling) {
        
	    memset(bytes_ptr, 0, num_bytes);
        status = HAL_UART_Receive(huartx, bytes_ptr, num_bytes, rx_timeout);
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
        memset(bytes_ptr, 0, num_bytes);
        status = HAL_UART_Receive_IT(huartx, bytes_ptr, num_bytes);
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
        memset(bytes_ptr, 0, num_bytes);
        status = HAL_UART_Receive_DMA(huartx, bytes_ptr, num_bytes);
        if(status == HAL_ERROR) {
            rx_status = Error;
            exception("usart_receive DMA | Error");
            return;
        }
        rx_status = InProgress;
    }
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
