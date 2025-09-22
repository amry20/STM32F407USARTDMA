#ifndef USART_H
#define USART_H
#include <Arduino.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_dma.h"

// ========================================================================
// CONFIGURATION OPTIONS
// ========================================================================
// Uncomment this line to disable DMA interrupt handlers in usart.cpp
// Use this when your project already has DMA handlers defined elsewhere
// #define USART_DISABLE_DMA_HANDLERS

// FIFO Buffer configuration - power of 2 for fast bitwise operations
#define FIFO_SIZE 8192   // 8KB FIFO buffer (2^13)
#define FIFO_MASK 8191   // FIFO_SIZE - 1 for fast modulo
#define DMA_CHUNK_SIZE 1024  // 1KB DMA chunks for efficiency
#define RX_BUFFER_SIZE 4096  // 4KB RX buffer

// DMA flag clear values for optimization (pre-computed)
#define DMA2_STREAM7_CLEAR_FLAGS (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7)
#define DMA1_STREAM6_CLEAR_FLAGS (DMA_HIFCR_CTCIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6)
#define DMA1_STREAM3_CLEAR_FLAGS (DMA_LIFCR_CTCIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3)

// Transmission threshold for batching - optimized for performance
#define TX_THRESHOLD 1  // Start DMA when >=256 bytes available (reduce overhead)

// Pre-computed DMA control register base value (without channel and enable bit)
#define DMA_CR_BASE (DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_TCIE)

// Performance optimization macros
#define LIKELY(x)   __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)

// USART instance mapping structure
typedef struct {
    USART_TypeDef* instance;
    DMA_Stream_TypeDef* dma_tx_stream;
    DMA_Stream_TypeDef* dma_rx_stream;
    uint32_t dma_channel;
    GPIO_TypeDef* gpio_port;
    uint16_t tx_pin;
    uint16_t rx_pin;
    uint8_t gpio_af;
    IRQn_Type dma_tx_irq;
    IRQn_Type dma_rx_irq;
} USART_Config_t;

class USARTClass{
    public:
        USARTClass();  // Constructor
        void begin(USART_TypeDef *uart, uint32_t baudrate);
        size_t write(uint8_t data);
        size_t WriteBytes(uint8_t *data, size_t len);
        size_t printf(const char* format, ...);  // Printf function with variable arguments
        size_t availableForWrite();              // Available space in FIFO for writing
        void println(const char* str);
        bool isTxBusy();
        void resetTxStatus();
        void setTxBusy(bool busy);  // For interrupt handler
        uint32_t getErrorCount();
        void debugStatus();
        
        // FIFO management
        void processTransmission();  // Process FIFO and trigger DMA
        void startDmaTransmission(uint16_t len);  // Start DMA with specified length
        size_t getFifoAvailable();
        size_t getFifoUsed();
        void flushFifo();
        void forceTransmit();         // Force transmit remaining FIFO data (ignore threshold)
        
        // RX functions
        int read();                   // Read single byte (-1 if no data)
        int available();              // Number of bytes available to read
        int peek();                   // Peek at next byte without removing it
        void flushRx();               // Flush RX buffer
        void handleRxComplete();      // DMA RX complete callback
        void handleRxHalfComplete();  // DMA RX half complete callback
        size_t readBytes(uint8_t* buffer, size_t length);  // Read multiple bytes
        String readStringUntil(char terminator);  // Read string until terminator character
        String readStringUntil(char terminator, size_t maxLength);  // Read string with max length limit
        String readString();  // Read all available data as string
        
        // Static method to get config for USART instance
        static const USART_Config_t* getConfig(USART_TypeDef* instance);
        
        USART_TypeDef *uart;
        UART_HandleTypeDef huart;
        
        // Make these public for HAL_UART_MspInit access
        DMA_HandleTypeDef hdma_tx;
        DMA_HandleTypeDef hdma_rx;
        const USART_Config_t* config;
        
        // Public access for optimized interrupt handlers
        volatile bool txBusy;
        volatile uint16_t fifoCount;    // Number of bytes in FIFO
        volatile uint32_t errorCount;
        volatile uint16_t rxHead;       // Current DMA write position
        
    private:
        // Instance-specific buffers (aligned for DMA)
        uint8_t fifoBuffer[FIFO_SIZE] __attribute__((aligned(4)));
        uint8_t txBuffer[DMA_CHUNK_SIZE] __attribute__((aligned(4)));
        uint8_t rxBuffer[RX_BUFFER_SIZE] __attribute__((aligned(4)));
        
        // Instance-specific state variables
        volatile uint16_t fifoHead;     // Write index
        volatile uint16_t fifoTail;     // Read index
        
        // RX Buffer management
        volatile uint16_t rxTail;       // Application read position
        volatile uint16_t rxOverflow;   // Overflow counter
};

// Global instances array for interrupt handlers
extern USARTClass* g_usart_instances[3];  // For USART1, USART2, USART3

#endif