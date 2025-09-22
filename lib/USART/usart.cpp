#include "usart.h"
#include <string.h>
#include <stdarg.h>  // For variable arguments (va_list, va_start, va_end)
#include <stdio.h>   // For vsnprintf

// Global instances array for interrupt handlers
USARTClass* g_usart_instances[3] = {nullptr, nullptr, nullptr};

// USART configuration mapping for STM32F407
static const USART_Config_t usart_configs[3] = {
    // USART1: PA9(TX), PA10(RX), DMA2_Stream7(TX), DMA2_Stream2(RX)
    {
        .instance = USART1,
        .dma_tx_stream = DMA2_Stream7,
        .dma_rx_stream = DMA2_Stream2,
        .dma_channel = DMA_CHANNEL_4,
        .gpio_port = GPIOA,
        .tx_pin = GPIO_PIN_9,
        .rx_pin = GPIO_PIN_10,
        .gpio_af = GPIO_AF7_USART1,
        .dma_tx_irq = DMA2_Stream7_IRQn,
        .dma_rx_irq = DMA2_Stream2_IRQn
    },
    // USART2: PA2(TX), PA3(RX), DMA1_Stream6(TX), DMA1_Stream5(RX)
    {
        .instance = USART2,
        .dma_tx_stream = DMA1_Stream6,
        .dma_rx_stream = DMA1_Stream5,
        .dma_channel = DMA_CHANNEL_4,
        .gpio_port = GPIOA,
        .tx_pin = GPIO_PIN_2,
        .rx_pin = GPIO_PIN_3,
        .gpio_af = GPIO_AF7_USART2,
        .dma_tx_irq = DMA1_Stream6_IRQn,
        .dma_rx_irq = DMA1_Stream5_IRQn
    },
    // USART3: PB10(TX), PB11(RX), DMA1_Stream3(TX), DMA1_Stream1(RX)
    {
        .instance = USART3,
        .dma_tx_stream = DMA1_Stream3,
        .dma_rx_stream = DMA1_Stream1,
        .dma_channel = DMA_CHANNEL_4,
        .gpio_port = GPIOB,
        .tx_pin = GPIO_PIN_10,
        .rx_pin = GPIO_PIN_11,
        .gpio_af = GPIO_AF7_USART3,
        .dma_tx_irq = DMA1_Stream3_IRQn,
        .dma_rx_irq = DMA1_Stream1_IRQn
    }
};

// Error handler implementation
void Error_Handler_Custom(void) {
    // Disable all interrupts to prevent further issues
    __disable_irq();
    
    // System error - halt execution
    while (1) {
        // System halted due to error
    }
}

// Constructor
USARTClass::USARTClass() {
    // Initialize all member variables
    fifoHead = 0;
    fifoTail = 0;
    fifoCount = 0;
    txBusy = false;
    errorCount = 0;
    rxHead = 0;
    rxTail = 0;
    rxOverflow = 0;
    config = nullptr;
    uart = nullptr;
    
    // Clear buffers
    memset(fifoBuffer, 0, FIFO_SIZE);
    memset(txBuffer, 0, DMA_CHUNK_SIZE);
    memset(rxBuffer, 0, RX_BUFFER_SIZE);
}

// Static method to get config for USART instance
const USART_Config_t* USARTClass::getConfig(USART_TypeDef* instance) {
    for (int i = 0; i < 3; i++) {
        if (usart_configs[i].instance == instance) {
            return &usart_configs[i];
        }
    }
    return nullptr;
}

// Get instance index for interrupt handlers
static int getInstanceIndex(USART_TypeDef* instance) {
    if (instance == USART1) return 0;
    if (instance == USART2) return 1;
    if (instance == USART3) return 2;
    return -1;
}

// Generic callback functions that work for all USART instances
void USART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == NULL) return;
    
    int idx = getInstanceIndex(huart->Instance);
    if (idx >= 0 && g_usart_instances[idx] != nullptr) {
        g_usart_instances[idx]->setTxBusy(false);
    }
}

void USART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == NULL) return;
    
    int idx = getInstanceIndex(huart->Instance);
    if (idx >= 0 && g_usart_instances[idx] != nullptr) {
        // RX complete handling per instance
        g_usart_instances[idx]->handleRxComplete();
    }
}

void USART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == NULL) return;
    
    int idx = getInstanceIndex(huart->Instance);
    if (idx >= 0 && g_usart_instances[idx] != nullptr) {
        // RX half complete handling per instance
        g_usart_instances[idx]->handleRxHalfComplete();
    }
}

void USART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == NULL) return;
    
    int idx = getInstanceIndex(huart->Instance);
    if (idx >= 0 && g_usart_instances[idx] != nullptr) {
        g_usart_instances[idx]->setTxBusy(false);
        // Error count will be incremented by the class internally
    }
    
    // Clear error flags for STM32F4
    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_NE | 
                         UART_FLAG_PE | UART_FLAG_FE);
    
    // Abort DMA to prevent hang
    if (huart->hdmatx != NULL) {
        HAL_DMA_Abort(huart->hdmatx);
    }
    
    // Restart RX DMA after error
    if (huart->hdmarx != NULL) {
        HAL_DMA_Abort(huart->hdmarx);
        HAL_UART_Receive_DMA(huart, (uint8_t*)huart->pRxBuffPtr, huart->RxXferSize);
    }
}

// DMA Interrupt Handlers for USART1
extern "C" void DMA2_Stream7_IRQHandler(void) {
    // USART1 TX DMA (DMA2_Stream7)
    if (DMA2->HISR & DMA_HISR_TCIF7) {
        // Clear ALL DMA flags
        DMA2->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7;
        
        // Disable DMA completely
        DMA2_Stream7->CR = 0;
        
        // Disable USART DMA
        USART1->CR3 &= ~USART_CR3_DMAT;
        
        // Mark transmission as complete
        if (g_usart_instances[0]) {
            g_usart_instances[0]->setTxBusy(false);
            
            // Auto-start next transmission if FIFO has data
            g_usart_instances[0]->processTransmission();
        }
    }
}

extern "C" void DMA2_Stream2_IRQHandler(void) {
    // USART1 RX DMA (DMA2_Stream2)
    uint32_t isr = DMA2->LISR;
    
    // Transfer Complete (TC) - full buffer processed
    if (isr & DMA_LISR_TCIF2) {
        DMA2->LIFCR = DMA_LIFCR_CTCIF2;  // Clear TC flag
        
        if (g_usart_instances[0]) {
            g_usart_instances[0]->handleRxComplete();
        }
    }
    
    // Half Transfer (HT) - half buffer processed  
    if (isr & DMA_LISR_HTIF2) {
        DMA2->LIFCR = DMA_LIFCR_CHTIF2;  // Clear HT flag
        
        if (g_usart_instances[0]) {
            g_usart_instances[0]->handleRxHalfComplete();
        }
    }
    
    // Transfer Error (TE)
    if (isr & DMA_LISR_TEIF2) {
        DMA2->LIFCR = DMA_LIFCR_CTEIF2;  // Clear TE flag
        // Handle error if needed
        if (g_usart_instances[0]) {
            // Error handling will be done in class method
        }
    }
}

// DMA Interrupt Handlers for USART2
extern "C" void DMA1_Stream6_IRQHandler(void) {
    // USART2 TX DMA (DMA1_Stream6)
    if (DMA1->HISR & DMA_HISR_TCIF6) {
        // Clear ALL DMA flags
        DMA1->HIFCR = DMA_HIFCR_CTCIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;
        
        // Disable DMA completely
        DMA1_Stream6->CR = 0;
        
        // Disable USART DMA
        USART2->CR3 &= ~USART_CR3_DMAT;
        
        // Mark transmission as complete
        if (g_usart_instances[1]) {
            g_usart_instances[1]->setTxBusy(false);
            g_usart_instances[1]->processTransmission();
        }
    }
}

extern "C" void DMA1_Stream5_IRQHandler(void) {
    // USART2 RX DMA (DMA1_Stream5)
    uint32_t isr = DMA1->HISR;
    
    if (isr & DMA_HISR_TCIF5) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF5;
        if (g_usart_instances[1]) {
            g_usart_instances[1]->handleRxComplete();
        }
    }
    
    if (isr & DMA_HISR_HTIF5) {
        DMA1->HIFCR = DMA_HIFCR_CHTIF5;
        if (g_usart_instances[1]) {
            g_usart_instances[1]->handleRxHalfComplete();
        }
    }
    
    if (isr & DMA_HISR_TEIF5) {
        DMA1->HIFCR = DMA_HIFCR_CTEIF5;
    }
}

// DMA Interrupt Handlers for USART3
extern "C" void DMA1_Stream3_IRQHandler(void) {
    // USART3 TX DMA (DMA1_Stream3)
    if (DMA1->LISR & DMA_LISR_TCIF3) {
        // Clear ALL DMA flags
        DMA1->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;
        
        // Disable DMA completely
        DMA1_Stream3->CR = 0;
        
        // Disable USART DMA
        USART3->CR3 &= ~USART_CR3_DMAT;
        
        // Mark transmission as complete
        if (g_usart_instances[2]) {
            g_usart_instances[2]->setTxBusy(false);
            g_usart_instances[2]->processTransmission();
        }
    }
}

extern "C" void DMA1_Stream1_IRQHandler(void) {
    // USART3 RX DMA (DMA1_Stream1)
    uint32_t isr = DMA1->LISR;
    
    if (isr & DMA_LISR_TCIF1) {
        DMA1->LIFCR = DMA_LIFCR_CTCIF1;
        if (g_usart_instances[2]) {
            g_usart_instances[2]->handleRxComplete();
        }
    }
    
    if (isr & DMA_LISR_HTIF1) {
        DMA1->LIFCR = DMA_LIFCR_CHTIF1;
        if (g_usart_instances[2]) {
            g_usart_instances[2]->handleRxHalfComplete();
        }
    }
    
    if (isr & DMA_LISR_TEIF1) {
        DMA1->LIFCR = DMA_LIFCR_CTEIF1;
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    // Get USART instance index
    int idx = getInstanceIndex(huart->Instance);
    if (idx < 0 || g_usart_instances[idx] == nullptr) return;
    
    USARTClass* instance = g_usart_instances[idx];
    const USART_Config_t* cfg = instance->config;
    
    // Enable clocks
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    
    if (huart->Instance == USART1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART1_CLK_ENABLE();
    } else if (huart->Instance == USART2) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART2_CLK_ENABLE();
    } else if (huart->Instance == USART3) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_USART3_CLK_ENABLE();
    }

    // Configure GPIO pins
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // TX Pin
    GPIO_InitStruct.Pin = cfg->tx_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = cfg->gpio_af;
    HAL_GPIO_Init(cfg->gpio_port, &GPIO_InitStruct);
    
    // RX Pin
    GPIO_InitStruct.Pin = cfg->rx_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = cfg->gpio_af;
    HAL_GPIO_Init(cfg->gpio_port, &GPIO_InitStruct);

    // Configure DMA TX
    instance->hdma_tx.Instance = cfg->dma_tx_stream;
    instance->hdma_tx.Init.Channel = cfg->dma_channel;
    instance->hdma_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    instance->hdma_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    instance->hdma_tx.Init.MemInc = DMA_MINC_ENABLE;
    instance->hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    instance->hdma_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    instance->hdma_tx.Init.Mode = DMA_NORMAL;
    instance->hdma_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    instance->hdma_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    
    if (HAL_DMA_Init(&instance->hdma_tx) != HAL_OK) {
        Error_Handler_Custom();
    }
    __HAL_LINKDMA(huart, hdmatx, instance->hdma_tx);

    // Configure DMA RX - CIRCULAR MODE
    instance->hdma_rx.Instance = cfg->dma_rx_stream;
    instance->hdma_rx.Init.Channel = cfg->dma_channel;
    instance->hdma_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    instance->hdma_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    instance->hdma_rx.Init.MemInc = DMA_MINC_ENABLE;
    instance->hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    instance->hdma_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    instance->hdma_rx.Init.Mode = DMA_CIRCULAR;  // Circular mode for continuous RX
    instance->hdma_rx.Init.Priority = DMA_PRIORITY_HIGH;  // Higher priority for RX
    instance->hdma_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    
    if (HAL_DMA_Init(&instance->hdma_rx) != HAL_OK) {
        Error_Handler_Custom();
    }
    __HAL_LINKDMA(huart, hdmarx, instance->hdma_rx);

    // Enable DMA TX interrupt
    HAL_NVIC_SetPriority(cfg->dma_tx_irq, 2, 0);
    HAL_NVIC_EnableIRQ(cfg->dma_tx_irq);
    
    // Enable DMA RX interrupt for half-transfer and transfer complete
    HAL_NVIC_SetPriority(cfg->dma_rx_irq, 1, 0);  // Higher priority than TX
    HAL_NVIC_EnableIRQ(cfg->dma_rx_irq);
}
void USARTClass::begin(USART_TypeDef *uart, uint32_t baudrate){
    // Get configuration for this USART instance
    this->config = getConfig(uart);
    if (this->config == nullptr) {
        this->errorCount++;
        Error_Handler_Custom();
        return;
    }
    
    // Register this instance in global array BEFORE HAL_UART_Init
    int idx = getInstanceIndex(uart);
    if (idx >= 0) {
        g_usart_instances[idx] = this;
    }
    
    // Force enable all required clocks first
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    if (uart == USART1) __HAL_RCC_USART1_CLK_ENABLE();
    else if (uart == USART2) __HAL_RCC_USART2_CLK_ENABLE();
    else if (uart == USART3) __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    // Initialize FIFO and state BEFORE HAL_UART_Init
    this->fifoHead = 0;
    this->fifoTail = 0;
    this->fifoCount = 0;
    this->txBusy = false;
    this->errorCount = 0;
    this->rxHead = 0;
    this->rxTail = 0;
    this->rxOverflow = 0;

    this->uart = uart;
    this->huart.Instance = uart;
    this->huart.Init.BaudRate = baudrate;
    this->huart.Init.WordLength = UART_WORDLENGTH_8B;
    this->huart.Init.StopBits = UART_STOPBITS_1;
    this->huart.Init.Parity = UART_PARITY_NONE;
    this->huart.Init.Mode = UART_MODE_TX_RX;
    this->huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    this->huart.Init.OverSampling = UART_OVERSAMPLING_16;

    // Initialize UART - this will call HAL_UART_MspInit
    HAL_StatusTypeDef result = HAL_UART_Init(&this->huart);
    if (result != HAL_OK) {
        this->errorCount++;
        Error_Handler_Custom();
    }
    
    // Register custom callbacks (only if HAL_UART_Init succeeded)
    if (result == HAL_OK) {
        HAL_UART_RegisterCallback(&this->huart, HAL_UART_TX_COMPLETE_CB_ID, USART_TxCpltCallback);
        HAL_UART_RegisterCallback(&this->huart, HAL_UART_RX_COMPLETE_CB_ID, USART_RxCpltCallback);
        HAL_UART_RegisterCallback(&this->huart, HAL_UART_RX_HALFCOMPLETE_CB_ID, USART_RxHalfCpltCallback);
        HAL_UART_RegisterCallback(&this->huart, HAL_UART_ERROR_CB_ID, USART_ErrorCallback);
    }
    
    // Start DMA RX in circular mode
    if (HAL_UART_Receive_DMA(&this->huart, this->rxBuffer, RX_BUFFER_SIZE) != HAL_OK) {
        this->errorCount++;
        Error_Handler_Custom();
    }
    
    // Add delay to stabilize
    delay(100);
}
size_t USARTClass::write(uint8_t data){
    if (WriteBytes((uint8_t*)&data, 1)) {
        return 1;
    }
    return 0;
}
size_t USARTClass::WriteBytes(uint8_t *data, size_t len) {
    if (len == 0) return 0;
    
    // Calculate available space in FIFO
    size_t available = FIFO_SIZE - this->fifoCount;
    size_t toWrite = (len > available) ? available : len;
    
    if (toWrite == 0) return 0; // FIFO full
    
    // Disable interrupts for atomic FIFO operations
    __disable_irq();
    
    // Optimized bulk write to FIFO
    if (this->fifoHead + toWrite <= FIFO_SIZE) {
        // No wrapping - single fast memcpy
        memcpy(&this->fifoBuffer[this->fifoHead], data, toWrite);
        this->fifoHead = (this->fifoHead + toWrite) & FIFO_MASK;  // Fast bitwise AND
    } else {
        // Wrapping case - two memcpy operations
        size_t firstPart = FIFO_SIZE - this->fifoHead;
        size_t secondPart = toWrite - firstPart;
        
        memcpy(&this->fifoBuffer[this->fifoHead], data, firstPart);
        memcpy(&this->fifoBuffer[0], &data[firstPart], secondPart);
        
        this->fifoHead = secondPart;  // Already within bounds
    }
    
    // Bulk update count
    this->fifoCount += toWrite;
    
    __enable_irq();
    
    // Try to start transmission if DMA is idle
    processTransmission();
    
    return toWrite;
}

// Process FIFO and start DMA transmission if needed
void USARTClass::processTransmission() {
    // Skip if DMA is busy or FIFO is empty
    if (this->txBusy || this->fifoCount == 0) {
        return;
    }
    
    // Start DMA transmission whenever there's data (simplified logic)
    // Determine how many bytes to send (up to DMA_CHUNK_SIZE)
    uint16_t bytesToSend = (this->fifoCount > DMA_CHUNK_SIZE) ? DMA_CHUNK_SIZE : this->fifoCount;
    
    // Disable interrupts for atomic FIFO read
    __disable_irq();
    
    // Optimized FIFO copy - handle circular buffer efficiently
    if (this->fifoTail + bytesToSend <= FIFO_SIZE) {
        // No wrapping - single fast memcpy
        memcpy(this->txBuffer, &this->fifoBuffer[this->fifoTail], bytesToSend);
        this->fifoTail = (this->fifoTail + bytesToSend) & FIFO_MASK;  // Fast bitwise AND instead of modulo
    } else {
        // Wrapping case - two memcpy operations
        uint16_t firstPart = FIFO_SIZE - this->fifoTail;
        uint16_t secondPart = bytesToSend - firstPart;
        
        memcpy(this->txBuffer, &this->fifoBuffer[this->fifoTail], firstPart);
        memcpy(&this->txBuffer[firstPart], &this->fifoBuffer[0], secondPart);
        
        this->fifoTail = secondPart;
    }
    
    // Bulk update count
    this->fifoCount -= bytesToSend;
    
    __enable_irq();
    
    // Start DMA transmission
    startDmaTransmission(bytesToSend);
}

// Start DMA transmission with specified length
void USARTClass::startDmaTransmission(uint16_t len) {
    if (this->config == nullptr) return;
    
    // Get the correct DMA stream and USART register
    DMA_Stream_TypeDef* dma_stream = this->config->dma_tx_stream;
    USART_TypeDef* usart_reg = this->uart;
    
    // COMPLETE DMA RESET before configuration with timeout
    dma_stream->CR = 0; // Disable DMA
    uint32_t timeout = 1000;
    while ((dma_stream->CR & DMA_SxCR_EN) && timeout--) {
        // Wait for DMA to be completely disabled
    }
    
    if (timeout == 0) {
        // DMA stuck - abort and increment error
        this->txBusy = false;
        this->errorCount++;
        return;
    }
    
    // Clear all flags - optimized with pre-computed values
    if (dma_stream == DMA2_Stream7) {
        DMA2->HIFCR = DMA2_STREAM7_CLEAR_FLAGS;
    } else if (dma_stream == DMA1_Stream6) {
        DMA1->HIFCR = DMA1_STREAM6_CLEAR_FLAGS;
    } else if (dma_stream == DMA1_Stream3) {
        DMA1->LIFCR = DMA1_STREAM3_CLEAR_FLAGS;
    }
    
    // Disable USART DMA first
    usart_reg->CR3 &= ~USART_CR3_DMAT;

    // Configure DMA Stream
    dma_stream->PAR = (uint32_t)&(usart_reg->DR); // Peripheral address
    dma_stream->M0AR = (uint32_t)this->txBuffer; // Memory address
    dma_stream->NDTR = len; // Number of data
    
    // Configure DMA control register (optimized with pre-computed base)
    dma_stream->CR = this->config->dma_channel | DMA_CR_BASE;
    
    // Enable USART DMA TX
    usart_reg->CR3 |= USART_CR3_DMAT;
    
    // Start DMA
    dma_stream->CR |= DMA_SxCR_EN;
    
    this->txBusy = true;
}

// Fungsi untuk cek status DMA
bool USARTClass::isTxBusy() {
    return this->txBusy;
}

// Fungsi untuk force reset status jika macet
void USARTClass::resetTxStatus() {
    this->txBusy = false;
    HAL_DMA_Abort(&this->hdma_tx);
}

// Fungsi untuk set txBusy status (for interrupt handler)
void USARTClass::setTxBusy(bool busy) {
    this->txBusy = busy;
}

// FIFO utility functions
size_t USARTClass::getFifoAvailable() {
    return FIFO_SIZE - this->fifoCount;
}

size_t USARTClass::getFifoUsed() {
    return this->fifoCount;
}

// Arduino-style availableForWrite function
size_t USARTClass::availableForWrite() {
    return FIFO_SIZE - this->fifoCount;
}

// Printf function with variable arguments
size_t USARTClass::printf(const char* format, ...) {
    char buffer[512];  // Buffer for formatted string
    va_list args;
    
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len < 0) {
        return 0;  // Formatting error
    }
    
    // Truncate if too long
    if (len >= sizeof(buffer)) {
        len = sizeof(buffer) - 1;
    }
    
    // Send via WriteBytes (DMA)
    return WriteBytes((uint8_t*)buffer, len);
}

void USARTClass::flushFifo() {
    __disable_irq();
    this->fifoHead = 0;
    this->fifoTail = 0;
    this->fifoCount = 0;
    __enable_irq();
}

// Force transmit remaining data (ignore threshold for immediate send)
void USARTClass::forceTransmit() {
    // Skip if DMA is busy or FIFO is empty
    if (this->txBusy || this->fifoCount == 0) {
        return;
    }
    
    // Send whatever is in FIFO, regardless of threshold
    uint16_t bytesToSend = (this->fifoCount > DMA_CHUNK_SIZE) ? DMA_CHUNK_SIZE : this->fifoCount;
    
    // Disable interrupts for atomic FIFO read
    __disable_irq();
    
    // Copy from FIFO to TX buffer
    if (this->fifoTail + bytesToSend <= FIFO_SIZE) {
        memcpy(this->txBuffer, &this->fifoBuffer[this->fifoTail], bytesToSend);
        this->fifoTail = (this->fifoTail + bytesToSend) & FIFO_MASK;
    } else {
        uint16_t firstPart = FIFO_SIZE - this->fifoTail;
        uint16_t secondPart = bytesToSend - firstPart;
        
        memcpy(this->txBuffer, &this->fifoBuffer[this->fifoTail], firstPart);
        memcpy(&this->txBuffer[firstPart], &this->fifoBuffer[0], secondPart);
        
        this->fifoTail = secondPart;
    }
    
    this->fifoCount -= bytesToSend;
    
    __enable_irq();
    
    // Start DMA transmission
    startDmaTransmission(bytesToSend);
}

// Fungsi untuk mendapatkan error count
uint32_t USARTClass::getErrorCount() {
    return this->errorCount;
}

// Status monitoring function  
void USARTClass::debugStatus() {
    char status[120];
    sprintf(status, "UART:%d DMA:%d Busy:%d Err:%lu FIFO:%d/%d\r\n", 
            this->huart.gState, 
            this->hdma_tx.State,
            this->txBusy ? 1 : 0,
            this->errorCount,
            this->fifoCount,
            FIFO_SIZE);
    
    // Send via non-DMA
    for (int i = 0; status[i] != '\0'; i++) {
        this->write((uint8_t)status[i]);
    }
}

// RX DMA interrupt handlers
void USARTClass::handleRxComplete() {
    // Full buffer transfer complete - update head to end of buffer
    this->rxHead = RX_BUFFER_SIZE;
    // Buffer wraps to beginning automatically due to circular mode
}

void USARTClass::handleRxHalfComplete() {
    // Half buffer transfer complete - update head to middle of buffer
    this->rxHead = RX_BUFFER_SIZE / 2;
}

// Calculate number of bytes available to read
int USARTClass::available() {
    if (this->config == nullptr) return 0;
    
    // Atomic read to prevent race condition with DMA
    __disable_irq();
    uint16_t dmaRemaining = (uint16_t)this->config->dma_rx_stream->NDTR;
    uint16_t rxTailCopy = this->rxTail;
    __enable_irq();
    
    uint16_t currentPos = RX_BUFFER_SIZE - dmaRemaining;
    
    // Calculate available bytes with proper wrap-around handling
    int available_bytes;
    if (currentPos >= rxTailCopy) {
        available_bytes = currentPos - rxTailCopy;
    } else {
        available_bytes = (RX_BUFFER_SIZE - rxTailCopy) + currentPos;
    }
    
    return available_bytes;
}

// Read single byte from RX buffer
int USARTClass::read() {
    if (available() == 0) {
        return -1;  // No data available
    }
    
    // Atomic read and update
    __disable_irq();
    uint8_t data = this->rxBuffer[this->rxTail];
    this->rxTail = (this->rxTail + 1) % RX_BUFFER_SIZE;
    __enable_irq();
    
    return data;
}

// Peek at next byte without removing it
int USARTClass::peek() {
    if (available() == 0) {
        return -1;  // No data available
    }
    
    return this->rxBuffer[this->rxTail];
}

// Read multiple bytes from RX buffer
size_t USARTClass::readBytes(uint8_t* buffer, size_t length) {
    size_t bytesRead = 0;
    
    while (bytesRead < length && available() > 0) {
        int data = read();
        if (data == -1) break;
        
        buffer[bytesRead] = (uint8_t)data;
        bytesRead++;
    }
    
    return bytesRead;
}

// Flush RX buffer
void USARTClass::flushRx() {
    if (this->config == nullptr) return;
    
    __disable_irq();
    
    // Get current DMA position and reset tail to it
    uint16_t dmaRemaining = (uint16_t)this->config->dma_rx_stream->NDTR;
    uint16_t currentPos = RX_BUFFER_SIZE - dmaRemaining;
    
    this->rxHead = currentPos;
    this->rxTail = currentPos;
    this->rxOverflow = 0;
    
    __enable_irq();
}

// Read string until terminator character
String USARTClass::readStringUntil(char terminator) {
    return readStringUntil(terminator, 1024);  // Default max length 1KB
}

// Read string until terminator character with custom max length
String USARTClass::readStringUntil(char terminator, size_t maxLength) {
    String result = "";
    
    if (this->config == nullptr) {
        return result;  // Return empty string if not initialized
    }
    
    // Reserve space for efficiency (estimate)
    if (maxLength < 512) {
        result.reserve(maxLength);
    } else {
        result.reserve(512);
    }
    
    // Read characters until terminator or no more data
    while (available() > 0 && result.length() < maxLength) {
        int data = read();
        
        if (data == -1) {
            break;  // No more data available
        }
        
        char ch = (char)data;
        
        // Check if we found the terminator
        if (ch == terminator) {
            break;  // Stop reading, don't include terminator in result
        }
        
        // Add character to result string
        result += ch;
    }
    
    return result;
}

// Read all available data as string
String USARTClass::readString() {
    String result = "";
    
    if (this->config == nullptr) {
        return result;  // Return empty string if not initialized
    }
    
    int availableBytes = available();
    if (availableBytes == 0) {
        return result;  // No data available
    }
    
    // Reserve space for efficiency
    if (availableBytes < 1024) {
        result.reserve(availableBytes);
    } else {
        result.reserve(1024);  // Limit to 1KB
        availableBytes = 1024;
    }
    
    // Read all available characters
    int count = 0;
    while (available() > 0 && count < availableBytes) {
        int data = read();
        
        if (data == -1) {
            break;  // No more data available
        }
        
        result += (char)data;
        count++;
    }
    
    return result;
}
void USARTClass::println(const char* str)
{
    WriteBytes((uint8_t*)str, strlen(str));
    WriteBytes((uint8_t*)"\r\n",2);
}