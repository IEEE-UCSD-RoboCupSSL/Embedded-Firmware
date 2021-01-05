#ifndef __STF_I2C_H
#define __STF_I2C_H 

#include "stf_dependancy.h"


#ifdef HAL_I2C_MODULE_ENABLED

#define I2C_Default_TxTimeOut 10000 // 10 seconds 
#define I2C_Default_RxTimeOut 10000 // 10 seconds


#define I2C_Default_Tx_BufferSize 3
#define I2C_Default_Rx_BufferSize 50
#define Max_Num_I2Cs 10 


enum i2c_mode {
    Master,
    Slave
};

enum i2c_reg_size {
    Reg8bit,
    Reg16bit
};

namespace stf {
    class I2C {
    private:
        I2C_HandleTypeDef *hi2cx;
        i2c_reg_size regsize = Reg8bit;

        // time-out default
        uint32_t tx_timeout = I2C_Default_TxTimeOut;
        uint32_t rx_timeout = I2C_Default_RxTimeOut;

        volatile periph_status tx_status = NotReady;
        volatile periph_status rx_status = NotReady;

        byte_t *tx_buffer;
        byte_t *rx_buffer;

        void hal_transmit(byte_t* bytes_ptr, uint16_t num_bytes, i2c_mode i_mode, byte_t target_address, periph_mode mode);
        void hal_receive( byte_t* bytes_ptr, uint16_t num_bytes, i2c_mode i_mode, byte_t target_address, periph_mode mode);

        //i2c_mode is Master only for the 2 methods below
        void hal_write_reg(byte_t* bytes_ptr, uint16_t num_bytes, byte_t target_address, uint16_t target_register_address, periph_mode mode);
        void hal_read_reg( byte_t* bytes_ptr, uint16_t num_bytes, byte_t target_address, uint16_t target_register_address, periph_mode mode);

    public:

        // Constructors & Destructors
        I2C(I2C_HandleTypeDef *hi2cx, uint32_t tx_buffer_size = I2C_Default_Tx_BufferSize, 
            uint32_t rx_buffer_size = I2C_Default_Rx_BufferSize);
        ~I2C();


        /** Transmit & Receive **/

        //[I2C mode: slave mode]
        void transmit(std::string& str, periph_mode mode = Polling) { 
            char *cstr = (char*)str.c_str();
            hal_transmit((byte_t*)cstr, strlen(cstr), Slave, 0, mode);
        }
        //[I2C mode: master mode]
        void transmit(std::string& str, byte_t target_address, periph_mode mode = Polling) { 
            char *cstr = (char*)str.c_str();
            hal_transmit((byte_t*)cstr, strlen(cstr), Master, target_address, mode);
        }

        //[I2C mode: slave mode]
        void transmit(char* str_ptr, periph_mode mode = Polling) {
            hal_transmit((byte_t*)str_ptr, strlen(str_ptr), Slave, 0, mode);
        }
        //[I2C mode: master mode]
        void transmit(char* str_ptr, byte_t target_address, periph_mode mode = Polling) {
            hal_transmit((byte_t*)str_ptr, strlen(str_ptr), Master, target_address, mode);
        }
        
        //[I2C mode: slave mode]
        void transmit(byte_t* bytes_ptr, uint16_t num_bytes, periph_mode mode = Polling) {
            hal_transmit(bytes_ptr, num_bytes, Slave, 0, mode);
        }
        //[I2C mode: master mode]
        void transmit(byte_t* bytes_ptr, uint16_t num_bytes, byte_t target_address, periph_mode mode = Polling) {
            hal_transmit(bytes_ptr, num_bytes, Master, target_address, mode);
        }


        //[I2C mode: slave mode]
        void transmit(byte_t byte, periph_mode mode = Polling) {
            tx_buffer[0] = byte;
            hal_transmit(tx_buffer, 1, Slave, 0, mode);
        } 
        //[I2C mode: master mode]
        void transmit(byte_t byte, byte_t target_address, periph_mode mode = Polling) {
            tx_buffer[0] = byte;
            hal_transmit(tx_buffer, 1, Master, target_address, mode);
        }


        /* only support Polling mode if the return type is a string, 
         * this is due to c++ string constructor only does deep copy 
         * instead of shallow copy, can't copy a char array (rx_buffer) right away
         * when non-blocking receive is still in the middle of filling
         * content into the array
         * (non-blocking methods are {Interrupt, DMA})
         *  [I2C mode: slave mode]
         */
        // [I2C mode: slave mode]
        std::string receive(uint16_t num_bytes) {
            hal_receive(rx_buffer, num_bytes, Slave, 0, Polling);
            if(rx_status == Completed) return std::string((char*)rx_buffer);
            else return ""; // if error occurs
        }

        // [I2C mode: master mode]                  
        std::string receive(uint16_t num_bytes, byte_t target_address) {
            hal_receive(rx_buffer, num_bytes, Master, target_address, Polling);
            if(rx_status == Completed) return std::string((char*)rx_buffer);
            else return ""; // if error occurs
        }

        // [I2C mode: slave mode]                                     
        void receive(char* buffer_ptr, uint16_t num_bytes, periph_mode mode = Polling) {
            hal_receive((byte_t*)buffer_ptr, num_bytes, Slave, 0, mode);
        } 
        // [I2C mode: master mode] 
        void receive(char* buffer_ptr, uint16_t num_bytes, byte_t target_address, periph_mode mode = Polling) {
            hal_receive((byte_t*)buffer_ptr, num_bytes, Master, target_address, mode);
        }

        // [I2C mode: slave mode]                                     
        void receive(byte_t* buffer_ptr, uint16_t num_bytes, periph_mode mode = Polling) {
            hal_receive(buffer_ptr, num_bytes, Slave, 0, mode);
        } 
        // [I2C mode: master mode] 
        void receive(byte_t* buffer_ptr, uint16_t num_bytes, byte_t target_address, periph_mode mode = Polling) {
            hal_receive(buffer_ptr, num_bytes, Master, target_address, mode);
        }


        /*only support Polling mode due to the peripheral needs 
         *to finish receiption before returning the results
         *it's inherently a blocking method
         * [I2C mode: slave mode]                  
         */
        // [I2C mode: slave mode]
        byte_t receive(void) {
            hal_receive(rx_buffer, 1, Slave, 0, Polling);
            if(rx_status == Completed) return (byte_t)rx_buffer[0];
            else return 0; // if error occurs
        }
        // [I2C mode: master mode]                  
        byte_t receive(byte_t target_address) {
            hal_receive(rx_buffer, 1, Master, target_address, Polling);
            if(rx_status == Completed) return (byte_t)rx_buffer[0];
            else return 0; // if error occurs
        }

        const std::string readWord(void); // polling mode
        const std::string readLine(void); // polling mode


        /* Read/Write an external device's registers, used when configuring this stm32 as the master device 
         * and accessing registers of a slave device, which is often time a sensor.
         * [All are in master mode] 
         */
        void write_reg(byte_t target_address, uint16_t target_register_address,
                        std::string& str, periph_mode mode = Polling) {
            char *cstr = (char*)str.c_str();
            hal_write_reg((byte_t*)cstr, strlen(cstr), target_address, target_register_address, mode);
        }

        void write_reg(byte_t target_address, uint16_t target_register_address,
                        char* str_ptr, periph_mode mode = Polling) {
            hal_write_reg((byte_t*)str_ptr, strlen(str_ptr), target_address, target_register_address, mode);
        }

        void write_reg(byte_t target_address, uint16_t target_register_address,
                        byte_t* bytes_ptr, uint16_t num_bytes, periph_mode mode = Polling) {
            hal_write_reg(bytes_ptr, num_bytes, target_address, target_register_address, mode);
        }

        void write_reg(byte_t target_address, uint16_t target_register_address,
                        byte_t byte_to_write, periph_mode mode = Polling) {
            tx_buffer[0] = byte_to_write;
            hal_write_reg(tx_buffer, 1, target_address, target_register_address, mode);
        }

        //polling mode only
        std::string read_reg(uint16_t num_bytes, byte_t target_address, uint16_t target_register_address) {
            hal_read_reg(rx_buffer, num_bytes, target_address, target_register_address, Polling);
            if(rx_status == Completed) return std::string((char*)rx_buffer);
            else return ""; // if error occurs
        }

        void read_reg(char* buffer_ptr, uint16_t num_bytes, byte_t target_address, 
                        uint16_t target_register_address, periph_mode mode = Polling) {
            hal_read_reg((byte_t*)buffer_ptr, num_bytes, target_address, target_register_address, mode);
        }

        void read_reg(byte_t* buffer_ptr, uint16_t num_bytes, byte_t target_address, 
                        uint16_t target_register_address, periph_mode mode = Polling) {
            hal_read_reg(buffer_ptr, num_bytes, target_address, target_register_address, mode);
        }

        //polling mode only
        byte_t read_reg(byte_t target_address, uint16_t target_register_address) {
            hal_read_reg(rx_buffer, 1, target_address, target_register_address, Polling);
            if(rx_status == Completed) return rx_buffer[0];
            else return 0; // if error occurs
        }
        


        // Getters & Setters
        inline I2C_HandleTypeDef *get_hi2cx(void) {  return hi2cx; }
        inline void set_tx_timeout(uint32_t t) { tx_timeout = t; }
        inline void set_rx_timeout(uint32_t t) { rx_timeout = t;}
        inline periph_status get_tx_status() { return tx_status; }
        inline periph_status get_rx_status() { return rx_status; }
        inline void set_tx_status(periph_status status) { tx_status = status; }
        inline void set_rx_status(periph_status status) { rx_status = status; } 
        inline byte_t* get_tx_buffer_ptr(void) {return tx_buffer;}                                                                                         
        inline byte_t* get_rx_buffer_ptr(void) {return rx_buffer;}
        inline void set_reg_size(i2c_reg_size reg_size) {regsize = reg_size;}
        inline i2c_reg_size get_reg_size_setting(void) { return regsize;}
    };
}

/* Operators */
// polling mode
template<typename T>
stf::I2C& operator<<(stf::I2C& obj, const T& output) {
    std::stringstream newStream;
    newStream << output;
    std::string str = newStream.str();
    obj.transmit(str, stf::Polling);
    return obj;
}

/* Callbacks */
__weak void i2c_transmit_completed_interrupt_task(stf::I2C* instance);
extern "C" void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
extern "C" void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);



__weak void i2c_receive_completed_interrupt_task(stf::I2C* instance);
extern "C" void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
extern "C" void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);




#endif

#endif 
