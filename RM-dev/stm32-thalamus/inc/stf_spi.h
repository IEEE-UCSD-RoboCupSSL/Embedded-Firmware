#ifndef __STF_SPI_H
#define __STF_SPI_H 


#include "stf_dependancy.h"


#ifdef HAL_SPI_MODULE_ENABLED

#define SPI_Default_TxTimeOut 10000 // 10 seconds
#define SPI_Default_RxTimeOut 10000 // 10 seconds


#define SPI_Default_Tx_BufferSize 3
#define SPI_Default_Rx_BufferSize 50
#define Max_Num_SPIs 10 



namespace stf {
    class SPI {
    private:
        SPI_HandleTypeDef *hspix;

        // time-out default
        uint32_t tx_timeout = SPI_Default_TxTimeOut;
        uint32_t rx_timeout = SPI_Default_RxTimeOut;

        volatile periph_status tx_status = NotReady;
        volatile periph_status rx_status = NotReady;
        volatile periph_status txrx_status = NotReady;

        char *tx_buffer;
        char *rx_buffer;

        void hal_transmit(char* str_ptr, periph_mode mode);
        void hal_receive(char* str_ptr, uint16_t num_bytes, periph_mode mode);
        void hal_tranceive(char* tx_str_ptr, char* rx_str_ptr, periph_mode mode);

    public:

        // Constructors & Destructors
        SPI(SPI_HandleTypeDef *hspix, uint32_t tx_buffer_size = SPI_Default_Tx_BufferSize, 
                                          uint32_t rx_buffer_size = SPI_Default_Rx_BufferSize);
        ~SPI();

        // Transmit & Receive & Tranceive
        void transmit(std::string& str, periph_mode mode = Polling);
        void transmit(char* str_ptr, periph_mode mode = Polling);
        void transmit(byte_t byte, periph_mode mode = Polling);
        std::string receive(uint16_t num_bytes); 
        void receive(char* buffer_ptr, uint16_t num_bytes, periph_mode mode = Polling);
        byte_t receive(void); 
        std::string tranceive(std::string& str); //polling mode only
        void tranceive(char* tx_str_ptr, char* rx_str_ptr, periph_mode mode = Polling);
        byte_t tranceive(byte_t byte); //polling mode only

        

        const std::string readWord(void); // polling mode
        const std::string readLine(void); // polling mode

        // Getters & Setters
        inline SPI_HandleTypeDef *get_hspix(void) {  return hspix; }
        inline void set_tx_timeout(uint32_t t) { tx_timeout = t; }
        inline void set_rx_timeout(uint32_t t) { rx_timeout = t;}
        inline periph_status get_tx_status() { return tx_status; }
        inline periph_status get_rx_status() { return rx_status; }
        inline periph_status get_txrx_status() { return txrx_status; }
        inline void set_tx_status(periph_status status) { tx_status = status; }
        inline void set_rx_status(periph_status status) { rx_status = status; } 
        inline void set_txrx_status(periph_status status) { txrx_status = status; } 
        inline char* get_tx_buffer_ptr(void) {return tx_buffer;}                                                                                         
        inline char* get_rx_buffer_ptr(void) {return rx_buffer;}

    };
}

/* Operators */ 

// polling mode print
template<typename T>
stf::SPI& operator<<(stf::SPI& obj, const T& output) {
    std::stringstream newStream;
    newStream << output;
    std::string str = newStream.str();
    obj.transmit(str, stf::Polling);
    return obj;
}

/* Callbacks */
__weak void spi_transmit_completed_interrupt_task(stf::SPI* instance);
extern "C" void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);

extern "C" void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
__weak void spi_receive_completed_interrupt_task(stf::SPI* instance);

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
__weak void spi_tranceive_completed_interrupt_task(stf::SPI* instance);



#endif

#endif 
