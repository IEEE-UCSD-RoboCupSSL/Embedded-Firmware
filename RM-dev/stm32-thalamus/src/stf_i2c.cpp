#include "stf_i2c.h"
#include "stf_systick.h"

#ifdef HAL_I2C_MODULE_ENABLED


using namespace stf;

/*==================================================================*/
/* active_i2cs store pointers to all instantiated I2C objects
 * globally, for ISR(interrupt service routine) extern functions to assign 
 * interrupt task for each different i2c object. Ideally a hashtable or 
 * vector container should be implemented instead of a pointer array with 
 * a magic number of capacity, but ISR only support simple C-style 
 * operations, let's stick with the old-school fashion!
 * */
uint32_t num_i2cs = 0;
I2C* active_i2cs[Max_Num_I2Cs];
/*==================================================================*/



I2C::I2C(I2C_HandleTypeDef *hi2cx, uint32_t tx_buffer_size, uint32_t rx_buffer_size) {
    this->hi2cx = hi2cx;

    tx_buffer = new char(tx_buffer_size);
    rx_buffer = new char(rx_buffer_size);

    tx_status = Initialized;
    rx_status = Initialized;
    active_i2cs[num_i2cs++] = this;
}

I2C::~I2C() {
    delete tx_buffer;
    delete rx_buffer;
}



void I2C::hal_transmit(char* str_ptr, i2c_mode imode, byte_t target_address, periph_mode mode) {
	if(tx_status == NotReady) return;

    uint16_t tgt_add = (uint16_t)target_address << 1;

    HAL_StatusTypeDef status;
    if(mode == Polling) {
        if(imode == Master) {
            status = HAL_I2C_Master_Transmit(hi2cx, tgt_add, (uint8_t*)(str_ptr), strlen(str_ptr), tx_timeout);
        }
        if(imode == Slave) {
            status = HAL_I2C_Slave_Transmit(hi2cx, (uint8_t*)(str_ptr), strlen(str_ptr), tx_timeout);
        }

        if(status == HAL_BUSY) tx_status = InProgress;
        else if(status == HAL_TIMEOUT) {
            tx_status = TimeOut;

            //Unlock Usart
            __HAL_UNLOCK(hi2cx);
            hi2cx->State = HAL_I2C_STATE_READY;
            
            exception("i2c_transmit Polling | TimeOut");
        }
        else if(status == HAL_ERROR) {
            tx_status = Error;
            exception("i2c_transmit Polling | Error");
        }
        else if(status == HAL_OK) {
            tx_status = Completed;
        }
    }

    // Make sure interrupt is enabled within CubeMx software
    if(mode == Interrupt) {
        //check if the previous transmission is completed
        if(tx_status == InProgress) return;

        if(imode == Master) {
            status = HAL_I2C_Master_Transmit_IT(hi2cx, tgt_add, (uint8_t*)(str_ptr), strlen(str_ptr));
        }
        if(imode == Slave) {
            status = HAL_I2C_Slave_Transmit_IT(hi2cx, (uint8_t*)(str_ptr), strlen(str_ptr));
        }

        if(status == HAL_ERROR) {
            tx_status = Error;
            exception("i2c_transmit interrupt | Error");
            return;
        }
        tx_status = InProgress;
    }

    // Make sure DMA and interrupt are both enabled within CubeMx software
    if(mode == DMA) {
        //check if the previous transmission is completed
        if(tx_status == InProgress) return;

        if(imode == Master) {
            status = HAL_I2C_Master_Transmit_DMA(hi2cx, tgt_add, (uint8_t*)(str_ptr), strlen(str_ptr));
        }
        if(imode == Slave) {
            status = HAL_I2C_Slave_Transmit_DMA(hi2cx, (uint8_t*)(str_ptr), strlen(str_ptr));
        }

        if(status == HAL_ERROR) {
            tx_status = Error;
            exception("i2c_transmit DMA | Error");
            return;
        }
        tx_status = InProgress;
    }
}

//I2C mode: slave mode
void I2C::transmit(std::string& str, periph_mode mode) {
    hal_transmit((char*)str.c_str(), Slave, 0, mode);
} 

//[I2C mode: master mode]
void I2C::transmit(std::string& str, byte_t target_address, periph_mode mode) {
    hal_transmit((char*)str.c_str(), Master, target_address, mode);
}

//[I2C mode: slave mode]
void I2C::transmit(char* str_ptr, periph_mode mode) {
    hal_transmit(str_ptr, Slave, 0, mode);
}

//[I2C mode: master mode]
void I2C::transmit(char* str_ptr, byte_t target_address, periph_mode mode) {
    hal_transmit(str_ptr, Master, target_address, mode);
}

//[I2C mode: slave mode]
void I2C::transmit(byte_t byte, periph_mode mode) {
    tx_buffer[0] = (char)byte;
    tx_buffer[1] = '\0';
    hal_transmit(tx_buffer, Slave, 0, mode);
}

//[I2C mode: master mode]
void I2C::transmit(byte_t byte, byte_t target_address, periph_mode mode) {
    tx_buffer[0] = (char)byte;
    tx_buffer[1] = '\0';
    hal_transmit(tx_buffer, Master, target_address, mode);
}


void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    for(uint32_t i = 0; i < num_i2cs; i++) {
		if(active_i2cs[i]->get_hi2cx() == hi2c) {
            i2c_transmit_completed_interrupt_task(active_i2cs[i]);
            active_i2cs[i]->set_tx_status(Completed);
		}
	}
}
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    for(uint32_t i = 0; i < num_i2cs; i++) {
		if(active_i2cs[i]->get_hi2cx() == hi2c) {
            i2c_transmit_completed_interrupt_task(active_i2cs[i]);
            active_i2cs[i]->set_tx_status(Completed);
		}
	}
}

__weak void i2c_transmit_completed_interrupt_task(I2C* instance) {
    UNUSED(instance);
}


void I2C::hal_receive(char* str_ptr, uint16_t num_bytes, byte_t target_address, i2c_mode imode, periph_mode mode) {
    if(rx_status == NotReady) return;

    uint16_t tgt_add = (uint16_t)target_address << 1;

    HAL_StatusTypeDef status;
    if(mode == Polling) {
        memset(str_ptr, 0, strlen(str_ptr));

        if(imode == Master) {
            status = HAL_I2C_Master_Receive(hi2cx, tgt_add, (uint8_t*)(str_ptr), num_bytes, rx_timeout);
        }
        if(imode == Slave) {
            status = HAL_I2C_Slave_Receive(hi2cx, (uint8_t*)(str_ptr), num_bytes, rx_timeout);
        }

        if(status == HAL_BUSY) rx_status = InProgress;
        else if(status == HAL_TIMEOUT) {
            rx_status = TimeOut;

            //Unlock Usart
            __HAL_UNLOCK(hi2cx);
            hi2cx->State = HAL_I2C_STATE_READY;
            
            exception("i2c_receive Polling | TimeOut");
        }
        else if(status == HAL_ERROR) {
            rx_status = Error;
            exception("i2c_receive Polling | Error");
        }
        else if(status == HAL_OK) {
            rx_status = Completed;
        }
    }

    // Make sure interrupt is enabled within CubeMx software
    if(mode == Interrupt) {
        //check if the previous receiption is completed
        if(rx_status == InProgress) return;
        memset(str_ptr, 0, strlen(str_ptr));

        if(imode == Master) {
            status = HAL_I2C_Master_Receive_IT(hi2cx, tgt_add, (uint8_t*)(str_ptr), num_bytes);
        }
        if(imode == Slave) {
            status = HAL_I2C_Slave_Receive_IT(hi2cx, (uint8_t*)(str_ptr), num_bytes);
        }

        if(status == HAL_ERROR) {
            rx_status = Error;
            exception("i2c_receive interrupt | Error");
            return;
        }
        rx_status = InProgress;
    }

    // Make sure DMA and interrupt are both enabled within CubeMx software
    if(mode == DMA) {
        //check if the previous reception is completed
        if(rx_status == InProgress) return;
        memset(str_ptr, 0, strlen(str_ptr));

        if(imode == Master) {
            status = HAL_I2C_Master_Receive_DMA(hi2cx, tgt_add, (uint8_t*)(str_ptr), num_bytes);
        }
        if(imode == Slave) {
            status = HAL_I2C_Slave_Receive_DMA(hi2cx, (uint8_t*)(str_ptr), num_bytes);
        }

        if(status == HAL_ERROR) {
            rx_status = Error;
            exception("i2c_receive DMA | Error");
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
 *  [I2C mode: slave mode]
 */
std::string I2C::receive(uint16_t num_bytes) {
    hal_receive(rx_buffer, num_bytes, 0, Slave, Polling);
    if(rx_status == Completed) 
        return std::string(rx_buffer);
    else return ""; // if error occurs
}

// [I2C mode: master mode]
std::string I2C::receive(uint16_t num_bytes, byte_t target_address) {
    hal_receive(rx_buffer, num_bytes, target_address, Master, Polling);
    if(rx_status == Completed) 
        return std::string(rx_buffer);
    else return ""; // if error occurs
}
        

// [I2C mode: slave mode]
void I2C::receive(char* buffer_ptr, uint16_t num_bytes, periph_mode mode) {
    hal_receive(buffer_ptr, num_bytes, 0, Slave, mode);
}

// [I2C mode: master mode]
void I2C::receive(char* buffer_ptr, uint16_t num_bytes, byte_t target_address, periph_mode mode) {
    hal_receive(buffer_ptr, num_bytes, target_address, Master, mode);
}

/*only support Polling mode due to the peripheral needs 
 *to finish receiption before returning the results
 *it's inherently a blocking method
 * [I2C mode: slave mode]                  
 */
byte_t I2C::receive(void) {
    hal_receive(rx_buffer, 1, 0, Slave, Polling);
    if(rx_status == Completed) return (byte_t)rx_buffer[0];
    else return 0; // if error occurs
}

// [I2C mode: master mode]   
byte_t I2C::receive(byte_t target_address) {
    hal_receive(rx_buffer, 1, target_address, Master, Polling);
    if(rx_status == Completed) return (byte_t)rx_buffer[0];
    else return 0; // if error occurs
}


void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    for(uint32_t i = 0; i < num_i2cs; i++) {
		if(active_i2cs[i]->get_hi2cx() == hi2c) {
            i2c_receive_completed_interrupt_task(active_i2cs[i]);
            active_i2cs[i]->set_rx_status(Completed);
		}
	}
}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    for(uint32_t i = 0; i < num_i2cs; i++) {
		if(active_i2cs[i]->get_hi2cx() == hi2c) {
            i2c_receive_completed_interrupt_task(active_i2cs[i]);
            active_i2cs[i]->set_rx_status(Completed);
		}
	}
}

__weak void i2c_receive_completed_interrupt_task(I2C* instance) {
    UNUSED(instance);
}


// polling mode
const std::string I2C::readWord(void) {
    uint32_t t0 = millis();

    int i = 0;
    char str[I2C_Default_Rx_BufferSize];
	str[i] = (char)(this->receive());
	while(str[i] != ' ' && str[i] != '\r' && str[i] != '\n') {
		str[++i] = (char)(this->receive());
        if(millis() - t0 > I2C_Default_RxTimeOut) break;
    }
	str[i] = '\0';
    const std::string rtn(str);
    return rtn;
}

// polling mode
const std::string I2C::readLine(void) {
	
    uint32_t t0 = millis();

    int i = 0;
    char str[I2C_Default_Rx_BufferSize];
	str[i] = (char)(this->receive());
	while(str[i] != '\r' && str[i] != '\n') {
		str[++i] = (char)(this->receive());
        if(millis() - t0 > I2C_Default_RxTimeOut) break;
    }
	str[i] = '\0';
    const std::string rtn(str);
    return rtn;
}



void I2C::hal_write_reg(char* str_ptr, byte_t target_address, uint16_t target_register_address, periph_mode mode) {
    if(tx_status == NotReady) return;

    uint16_t tgt_add = (uint16_t)target_address << 1;

    HAL_StatusTypeDef status;
    if(mode == Polling) {
        
        status = HAL_I2C_Mem_Write(hi2cx, tgt_add, target_register_address,
						(regsize == Reg8bit ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT) ,
						(uint8_t*)(str_ptr), strlen(str_ptr) , tx_timeout);
        if(status == HAL_BUSY) tx_status = InProgress;
        else if(status == HAL_TIMEOUT) {
            tx_status = TimeOut;

            //Unlock Usart
            __HAL_UNLOCK(hi2cx);
            hi2cx->State = HAL_I2C_STATE_READY;
            
            exception("i2c_write_reg Polling | TimeOut");
        }
        else if(status == HAL_ERROR) {
            tx_status = Error;
            exception("i2c_write_reg Polling | Error");
        }
        else if(status == HAL_OK) {
            tx_status = Completed;
        }
    }

    // Make sure interrupt is enabled within CubeMx software
    if(mode == Interrupt) {
        //check if the previous transmission is completed
        if(tx_status == InProgress) return;

        status = HAL_I2C_Mem_Write_IT(hi2cx, tgt_add, target_register_address,
						(regsize == Reg8bit ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT) ,
						(uint8_t*)(str_ptr), strlen(str_ptr));

        if(status == HAL_ERROR) {
            tx_status = Error;
            exception("i2c_write_reg interrupt | Error");
            return;
        }
        tx_status = InProgress;
    }

    // Make sure DMA and interrupt are both enabled within CubeMx software
    if(mode == DMA) {
        //check if the previous transmission is completed
        if(tx_status == InProgress) return;

        
        status = HAL_I2C_Mem_Write_DMA(hi2cx, tgt_add, target_register_address,
						(regsize == Reg8bit ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT) ,
						(uint8_t*)(str_ptr), strlen(str_ptr));

        if(status == HAL_ERROR) {
            tx_status = Error;
            exception("i2c_write_reg DMA | Error");
            return;
        }
        tx_status = InProgress;
    }
}
void I2C::hal_read_reg(char* str_ptr, uint16_t num_bytes, byte_t target_address, 
                       uint16_t target_register_address, periph_mode mode) {
    if(rx_status == NotReady) return;

    uint16_t tgt_add = (uint16_t)target_address << 1;

    HAL_StatusTypeDef status;
    if(mode == Polling) {
	    memset(str_ptr, 0, strlen(str_ptr));

        status = HAL_I2C_Mem_Read(hi2cx, tgt_add, target_register_address,
			(regsize == Reg8bit ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT) ,
			(uint8_t*)(str_ptr), num_bytes, rx_timeout);

        if(status == HAL_BUSY) rx_status = InProgress;
        else if(status == HAL_TIMEOUT) {
            rx_status = TimeOut;

            //Unlock Usart
            __HAL_UNLOCK(hi2cx);
            hi2cx->State = HAL_I2C_STATE_READY;
            
            exception("i2c_read_reg Polling | TimeOut");
        }
        else if(status == HAL_ERROR) {
            rx_status = Error;
            exception("i2c_read_reg Polling | Error");
        }
        else if(status == HAL_OK) {
            rx_status = Completed;
        }
    }

    // Make sure interrupt is enabled within CubeMx software
    if(mode == Interrupt) {
        //check if the previous receiption is completed
        if(rx_status == InProgress) return;
        memset(str_ptr, 0, strlen(str_ptr));

        status = HAL_I2C_Mem_Read_IT(hi2cx, tgt_add, target_register_address,
			(regsize == Reg8bit ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT) ,
			(uint8_t*)(str_ptr), num_bytes);

        if(status == HAL_ERROR) {
            rx_status = Error;
            exception("i2c_receive interrupt | Error");
            return;
        }
        rx_status = InProgress;
    }

    // Make sure DMA and interrupt are both enabled within CubeMx software
    if(mode == DMA) {
        //check if the previous reception is completed
        if(rx_status == InProgress) return;
        memset(str_ptr, 0, strlen(str_ptr));

        status = HAL_I2C_Mem_Read_DMA(hi2cx, tgt_add, target_register_address,
			(regsize == Reg8bit ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT) ,
			(uint8_t*)(str_ptr), num_bytes);

        if(status == HAL_ERROR) {
            rx_status = Error;
            exception("i2c_receive DMA | Error");
            return;
        }
        rx_status = InProgress;
    }
}



void I2C::write_reg(byte_t target_address, uint16_t target_register_address,
                        std::string& str, periph_mode mode) {
    hal_write_reg((char*)str.c_str(), target_address, target_register_address, mode);                      
}

void I2C::write_reg(byte_t target_address, uint16_t target_register_address,
                char* str_ptr, periph_mode mode) {
    hal_write_reg(str_ptr, target_address, target_register_address, mode);
}

void I2C::write_reg(byte_t target_address, uint16_t target_register_address,
                byte_t byte_to_write, periph_mode mode) {
    tx_buffer[0] = (char)byte_to_write;
    tx_buffer[1] = '\0';
    hal_write_reg(tx_buffer, target_address, target_register_address, mode);
}

//polling mode only
std::string I2C::read_reg(uint16_t num_bytes, byte_t target_address, uint16_t target_register_address) {
    hal_read_reg(rx_buffer, num_bytes, target_address, target_register_address, Polling);
    if(rx_status == Completed) 
        return std::string(rx_buffer);
    else return ""; // if error occurs
}

void I2C::read_reg(char* buffer_ptr, uint16_t num_bytes, byte_t target_address, 
                uint16_t target_register_address, periph_mode mode) {
    hal_read_reg(buffer_ptr, num_bytes, target_address, target_register_address, mode);
}

//polling mode only
byte_t I2C::read_reg(byte_t target_address, uint16_t target_register_address) {
    hal_read_reg(rx_buffer, 1, target_address, target_register_address, Polling);
    if(rx_status == Completed) return (byte_t)rx_buffer[0];
    else return 0; // if error occurs
}


#endif
