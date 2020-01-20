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

        char *tx_buffer;
        char *rx_buffer;

        void hal_transmit(char* str_ptr, i2c_mode imode, byte_t target_address, periph_mode mode);
        void hal_receive(char* str_ptr, uint16_t num_bytes, byte_t target_address, i2c_mode imode, periph_mode mode);

        void hal_write_reg(char* str_ptr, byte_t target_address, uint16_t target_register_address, periph_mode mode);
        void hal_read_reg(char* str_ptr, uint16_t num_bytes, byte_t target_address, uint16_t target_register_address, 
                          periph_mode mode);

    public:

        // Constructors & Destructors
        I2C(I2C_HandleTypeDef *hi2cx, uint32_t tx_buffer_size = I2C_Default_Tx_BufferSize, 
            uint32_t rx_buffer_size = I2C_Default_Rx_BufferSize);
        ~I2C();


        // Transmit & Receive 
        void transmit(std::string& str, periph_mode mode = Polling); //I2C mode: slave mode
        void transmit(std::string& str, byte_t target_address, periph_mode mode = Polling); //[I2C mode: master mode]
        
        void transmit(char* str_ptr, periph_mode mode = Polling); //[I2C mode: slave mode]
        void transmit(char* str_ptr, byte_t target_address, periph_mode mode = Polling); //[I2C mode: master mode]
        
        void transmit(byte_t byte, periph_mode mode = Polling); //[I2C mode: slave mode]
        void transmit(byte_t byte, byte_t target_address, periph_mode mode = Polling); //[I2C mode: master mode]

        std::string receive(uint16_t num_bytes); // [I2C mode: slave mode]
        std::string receive(uint16_t num_bytes, byte_t target_address); // [I2C mode: master mode]                  
                                                  
        void receive(char* buffer_ptr, uint16_t num_bytes, periph_mode mode = Polling); // [I2C mode: slave mode]
        void receive(char* buffer_ptr, uint16_t num_bytes, byte_t target_address, periph_mode mode = Polling); // [I2C mode: master mode] 

        byte_t receive(void); // [I2C mode: slave mode]
        byte_t receive(byte_t target_address); // [I2C mode: master mode]                  

        const std::string readWord(void); // polling mode
        const std::string readLine(void); // polling mode


        /* Read/Write an external device's registers, used when configuring this stm32 as the master device 
         * and accessing registers of a slave device, which is often time a sensor.
         * [All are in master mode] 
         */
        void write_reg(byte_t target_address, uint16_t target_register_address,
                        std::string& str, periph_mode = Polling);
        void write_reg(byte_t target_address, uint16_t target_register_address,
                        char* str_ptr, periph_mode = Polling);
        void write_reg(byte_t target_address, uint16_t target_register_address,
                        byte_t byte_to_write, periph_mode = Polling);

        std::string read_reg(uint16_t num_bytes, byte_t target_address, uint16_t target_register_address); //polling mode only
        void read_reg(char* buffer_ptr, uint16_t num_bytes, byte_t target_address, 
                        uint16_t target_register_address, periph_mode mode = Polling);
        byte_t read_reg(byte_t target_address, uint16_t target_register_address);
        


        // Getters & Setters
        inline I2C_HandleTypeDef *get_hi2cx(void) {  return hi2cx; }
        inline void set_tx_timeout(uint32_t t) { tx_timeout = t; }
        inline void set_rx_timeout(uint32_t t) { rx_timeout = t;}
        inline periph_status get_tx_status() { return tx_status; }
        inline periph_status get_rx_status() { return rx_status; }
        inline void set_tx_status(periph_status status) { tx_status = status; }
        inline void set_rx_status(periph_status status) { rx_status = status; } 
        inline char* get_tx_buffer_ptr(void) {return tx_buffer;}                                                                                         
        inline char* get_rx_buffer_ptr(void) {return rx_buffer;}
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
