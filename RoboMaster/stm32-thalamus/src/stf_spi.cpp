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

    tx_buffer = new byte_t(tx_buffer_size);
    rx_buffer = new byte_t(rx_buffer_size);

    tx_status = Initialized;
    rx_status = Initialized;
    txrx_status = Initialized;

    active_spis[num_spis++] = this;
}

SPI::~SPI() {
    delete tx_buffer;
    delete rx_buffer;
}



void SPI::hal_transmit(byte_t* bytes_ptr, uint16_t num_bytes, periph_mode mode) {
	if(tx_status == NotReady) return;

    HAL_StatusTypeDef status;
    if(mode == Polling) {
        status = HAL_SPI_Transmit(hspix, bytes_ptr, num_bytes , tx_timeout);
        if(status == HAL_BUSY) tx_status = InProgress;
        else if(status == HAL_TIMEOUT) {
            tx_status = TimeOut;
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
        status = HAL_SPI_Transmit_IT(hspix, bytes_ptr, num_bytes);
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
        status = HAL_SPI_Transmit_DMA(hspix, bytes_ptr, num_bytes);
        if(status == HAL_ERROR) {
            tx_status = Error;
            exception("spi_transmit DMA | Error");
            return;
        }
        tx_status = InProgress;
    }
}


void SPI::hal_receive(byte_t* bytes_ptr, uint16_t num_bytes, periph_mode mode) {
    if(rx_status == NotReady) return;

    HAL_StatusTypeDef status;
    if(mode == Polling) {
        
	    memset(bytes_ptr, 0, num_bytes);
        status = HAL_SPI_Receive(hspix, bytes_ptr, num_bytes, rx_timeout);
        if(status == HAL_BUSY) rx_status = InProgress;
        else if(status == HAL_TIMEOUT) {
            rx_status = TimeOut;
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
        memset(bytes_ptr, 0, num_bytes);
        status = HAL_SPI_Receive_IT(hspix, bytes_ptr, num_bytes);
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
        memset(bytes_ptr, 0, num_bytes);
        status = HAL_SPI_Receive_DMA(hspix, bytes_ptr, num_bytes);
        if(status == HAL_ERROR) {
            rx_status = Error;
            exception("spi_receive DMA | Error");
            return;
        }
        rx_status = InProgress;
    }
}


void SPI::hal_tranceive(byte_t* tx_bytes_ptr, byte_t* rx_bytes_ptr, uint16_t num_bytes, periph_mode mode) {
    if(txrx_status == NotReady) return;

    HAL_StatusTypeDef status;
    if(mode == Polling) {
        status = HAL_SPI_TransmitReceive(hspix, tx_bytes_ptr, rx_bytes_ptr, num_bytes, txrx_timeout);
        if(status == HAL_BUSY) txrx_status = InProgress;
        else if(status == HAL_TIMEOUT) {
            txrx_status = TimeOut;
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
        status = HAL_SPI_TransmitReceive_IT(hspix, tx_bytes_ptr, rx_bytes_ptr, num_bytes);
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
        status = HAL_SPI_TransmitReceive_DMA(hspix, tx_bytes_ptr, rx_bytes_ptr, num_bytes);
        if(status == HAL_ERROR) {
            txrx_status = Error;
            exception("spi_tranceive DMA | Error");
            return;
        }
        txrx_status = InProgress;
    }
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
