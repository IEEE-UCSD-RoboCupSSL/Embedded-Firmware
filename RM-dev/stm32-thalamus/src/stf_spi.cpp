#include "stf_spi.h"
#include "stf_systick.h"

#ifdef HAL_SPI_MODULE_ENABLED


using namespace stf;

/*==================================================================*/
/* active_spis store pointers to all instantiated SPI objects
 * globally, for ISR(interrupt service routine) extern functions to assign 
 * interrupt task for each different spi object. Ideally a hashtable or 
 * vector container should be implemented instead of a pointer array with 
 * a magic number of capacity, but ISR only support simple C-style 
 * operations, let's stick with the old-school fashion!
 * */
uint32_t num_spis = 0;
SPI* active_spis[Max_Num_SPIs];
/*==================================================================*/



SPI::SPI(SPI_HandleTypeDef *hspix, uint32_t tx_buffer_size, uint32_t rx_buffer_size) {
    this->hspix = hspix;

    tx_buffer = new char(tx_buffer_size);
    rx_buffer = new char(rx_buffer_size);

    tx_status = Initialized;
    rx_status = Initialized;
    active_spis[num_spis++] = this;
}

SPI::~SPI() {
    delete tx_buffer;
    delete rx_buffer;
}



void SPI::hal_transmit(char* str_ptr, periph_mode mode) {
	if(tx_status == NotReady) return;

    HAL_StatusTypeDef status;
    if(mode == Polling) {
        status = HAL_SPI_Transmit(hspix, (uint8_t*)(str_ptr), strlen(str_ptr), tx_timeout);
        if(status == HAL_BUSY) tx_status = InProgress;
        else if(status == HAL_TIMEOUT) {
            tx_status = TimeOut;

            //Unlock Usart
            __HAL_UNLOCK(hspix);
            hspix->State = HAL_SPI_STATE_READY;
            
            exception("spi_transmit Polling | TimeOut");
        }
        else if(status == HAL_ERROR) {
            tx_status = Error;
            exception("spi_transmit Polling | Error");
        }
        else if(status == HAL_OK) {
            tx_status = Completed;
        }
    }

    // Make sure interrupt is enabled within CubeMx software
    if(mode == Interrupt) {
        //check if the previous transmission is completed
        if(tx_status == InProgress) return;
        status = HAL_SPI_Transmit_IT(hspix, (uint8_t*)str_ptr, strlen(str_ptr));
        if(status == HAL_ERROR) {
            tx_status = Error;
            exception("spi_transmit interrupt | Error");
            return;
        }
        tx_status = InProgress;
    }

    // Make sure DMA and interrupt are both enabled within CubeMx software
    if(mode == DMA) {
        //check if the previous transmission is completed
        if(tx_status == InProgress) return;
        status = HAL_SPI_Transmit_DMA(hspix, (uint8_t*)str_ptr, strlen(str_ptr));
        if(status == HAL_ERROR) {
            tx_status = Error;
            exception("spi_transmit DMA | Error");
            return;
        }
        tx_status = InProgress;
    }
}



void SPI::transmit(std::string& str, periph_mode mode) {
	hal_transmit((char*)str.c_str(), mode);
}


void SPI::transmit(char* str_ptr, periph_mode mode) {
    hal_transmit(str_ptr, mode);
}

void SPI::transmit(byte_t byte, periph_mode mode) {
    tx_buffer[0] = (char)byte;
    tx_buffer[1] = '\0';
    hal_transmit(tx_buffer, mode);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {    
	for(uint32_t i = 0; i < num_spis; i++) {
		if(active_spis[i]->get_hspix() == hspi) {
            spi_transmit_completed_interrupt_task(active_spis[i]);
            active_spis[i]->set_tx_status(Completed);
		}
	}
}

__weak void spi_transmit_completed_interrupt_task(SPI* instance) {
    UNUSED(instance);
}


void SPI::hal_receive(char* str_ptr, uint16_t num_bytes, periph_mode mode) {
    if(rx_status == NotReady) return;

    HAL_StatusTypeDef status;
    if(mode == Polling) {
        
	    memset(str_ptr, 0, strlen(str_ptr));
        status = HAL_SPI_Receive(hspix, (uint8_t*)(str_ptr), num_bytes, rx_timeout);
        if(status == HAL_BUSY) rx_status = InProgress;
        else if(status == HAL_TIMEOUT) {
            rx_status = TimeOut;

            //Unlock Usart
            __HAL_UNLOCK(hspix);
            hspix->State = HAL_SPI_STATE_READY;
            
            exception("spi_receive Polling | TimeOut");
        }
        else if(status == HAL_ERROR) {
            rx_status = Error;
            exception("spi_receive Polling | Error");
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
        status = HAL_SPI_Receive_IT(hspix, (uint8_t*)str_ptr, num_bytes);
        if(status == HAL_ERROR) {
            rx_status = Error;
            exception("spi_receive interrupt | Error");
            return;
        }
        rx_status = InProgress;
    }

    // Make sure DMA and interrupt are both enabled within CubeMx software
    if(mode == DMA) {
        //check if the previous reception is completed
        if(rx_status == InProgress) return;
        memset(str_ptr, 0, strlen(str_ptr));
        status = HAL_SPI_Receive_DMA(hspix, (uint8_t*)str_ptr, num_bytes);
        if(status == HAL_ERROR) {
            rx_status = Error;
            exception("spi_receive DMA | Error");
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
std::string SPI::receive(uint16_t num_bytes) {
    hal_receive(rx_buffer, num_bytes, Polling);
    if(rx_status == Completed) 
        return std::string(rx_buffer);
    else return ""; // if error occurs
}


void SPI::receive(char* buffer_ptr, uint16_t num_bytes, periph_mode mode) {
    hal_receive(buffer_ptr, num_bytes, mode);
}


/*only support Polling mode due to the peripheral needs 
 *to finish receiption before returning the results
 *it's inherently a blocking method
 */
byte_t SPI::receive(void) {
    hal_receive(rx_buffer, 1, Polling);
    if(rx_status == Completed) return (byte_t)rx_buffer[0];
    else return 0; // if error occurs
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    for(uint32_t i = 0; i < num_spis; i++) {
		if(active_spis[i]->get_hspix() == hspi) {
            spi_receive_completed_interrupt_task(active_spis[i]);
            active_spis[i]->set_rx_status(Completed);
		}
	}
}

__weak void spi_receive_completed_interrupt_task(SPI* instance) {
    UNUSED(instance);
}


void SPI::hal_tranceive(char* tx_str_ptr, char* rx_str_ptr, periph_mode mode) {
    if(txrx_status == NotReady) return;

    HAL_StatusTypeDef status;
    if(mode == Polling) {
        status = HAL_SPI_TransmitReceive(hspix, (uint8_t*)(tx_str_ptr), (uint8_t*)(rx_str_ptr), 
                                        strlen(tx_str_ptr), tx_timeout);
        if(status == HAL_BUSY) txrx_status = InProgress;
        else if(status == HAL_TIMEOUT) {
            txrx_status = TimeOut;

            //Unlock Usart
            __HAL_UNLOCK(hspix);
            hspix->State = HAL_SPI_STATE_READY;
            
            exception("spi_tranceive Polling | TimeOut");
        }
        else if(status == HAL_ERROR) {
            txrx_status = Error;
            exception("spi_tranceive Polling | Error");
        }
        else if(status == HAL_OK) {
            txrx_status = Completed;
        }
    }

    // Make sure interrupt is enabled within CubeMx software
    if(mode == Interrupt) {
        //check if the previous transmission is completed
        if(txrx_status == InProgress) return;
        status = HAL_SPI_TransmitReceive_IT(hspix, (uint8_t*)tx_str_ptr, (uint8_t*)(rx_str_ptr), strlen(tx_str_ptr));
        if(status == HAL_ERROR) {
            txrx_status = Error;
            exception("spi_tranceive interrupt | Error");
            return;
        }
        txrx_status = InProgress;
    }

    // Make sure DMA and interrupt are both enabled within CubeMx software
    if(mode == DMA) {
        //check if the previous transmission is completed
        if(txrx_status == InProgress) return;
        status = HAL_SPI_TransmitReceive_DMA(hspix, (uint8_t*)tx_str_ptr, (uint8_t*)(rx_str_ptr), strlen(tx_str_ptr));
        if(status == HAL_ERROR) {
            txrx_status = Error;
            exception("spi_tranceive DMA | Error");
            return;
        }
        txrx_status = InProgress;
    }
}

//polling mode only
std::string SPI::tranceive(std::string& str) {
    hal_tranceive((char*)str.c_str(), rx_buffer, Polling);
    if(txrx_status == Completed) 
        return std::string(rx_buffer);
    else return ""; // if error occurs
}

void SPI::tranceive(char* tx_str_ptr, char* rx_str_ptr, periph_mode mode) {
    hal_tranceive(tx_str_ptr, rx_str_ptr, mode);
}

//polling mode only
byte_t SPI::tranceive(byte_t byte) {
    tx_buffer[0] = (char)byte;
    tx_buffer[1] = '\0';
    hal_tranceive(tx_buffer, rx_buffer, Polling);
    if(txrx_status == Completed) return (byte_t)rx_buffer[0];
    else return 0; // if error occurs
}


// polling mode
const std::string SPI::readWord(void) {
    uint32_t t0 = millis();

    int i = 0;
    char str[SPI_Default_Rx_BufferSize];
	str[i] = (char)(this->receive());
	while(str[i] != ' ' && str[i] != '\r' && str[i] != '\n') {
		str[++i] = (char)(this->receive());
        if(millis() - t0 > SPI_Default_RxTimeOut) break;
    }
	str[i] = '\0';
    const std::string rtn(str);
    return rtn;
}

// polling mode
const std::string SPI::readLine(void) {
	
    uint32_t t0 = millis();

    int i = 0;
    char str[SPI_Default_Rx_BufferSize];
	str[i] = (char)(this->receive());
	while(str[i] != '\r' && str[i] != '\n') {
		str[++i] = (char)(this->receive());
        if(millis() - t0 > SPI_Default_RxTimeOut) break;
    }
	str[i] = '\0';
    const std::string rtn(str);
    return rtn;
}

#endif
