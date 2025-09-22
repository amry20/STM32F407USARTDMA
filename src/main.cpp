#include <Arduino.h>
#include <usart.h>
#include "hal_conf_extra.h"
#include <HardwareSerial.h>
#define Heartbeat PA_6
USARTClass SerialPort;

void setup()
{
  // put your setup code here, to run once:
  pinMode(Heartbeat, OUTPUT);
  digitalWriteFast(Heartbeat, LOW);
  
  // Test LED untuk memastikan sistem berjalan
  for (int i = 0; i < 10; i++) {
    digitalToggleFast(Heartbeat);
    delay(100);
  }
  SerialPort.begin(USART1, 230400);
  
  // Test kirim karakter langsung setelah init
  char initMsg[] = "=== UART DMA Test Start ===\n";
  for (int i = 0; initMsg[i] != '\0'; i++) {
    SerialPort.write((uint8_t)initMsg[i]);
  }
}

void loop()
{
  static uint32_t HeartbeatTime = millis();
  static uint32_t FifoTestTime = millis();
  static uint32_t RxTestTime = millis();
  static uint32_t TestCount = 0;
  
  // Toggle heartbeat setiap 2 detik (much slower)
  if (millis() - HeartbeatTime >= 500) {
    HeartbeatTime = millis();
    digitalToggleFast(Heartbeat);
    // Remove heartbeat serial output to reduce FIFO load
  }

  if (millis() - FifoTestTime >= 1000) { // Test setiap 1 detik
    FifoTestTime = millis();
  
    // Test simple 4KB tanpa debug overhead untuk pure performance
    uint8_t testData[4096];
    
    // Fill dengan pattern
    for (int i = 0; i < 4096; i++) {
      testData[i] = (uint8_t)('A' + (i % 26));
    }
    
    // Ukur waktu pure WriteBytes tanpa debug
    uint32_t startMicros = micros();
    
    size_t written = SerialPort.WriteBytes(testData, 4096);
    
    uint32_t endMicros = micros();
    uint32_t totalUs = endMicros - startMicros;

    // Single result output
    char result[120];
    sprintf(result, "[#%d] Written:%u, Time:%u us, FIFO:%d\r\n", 
            TestCount, (unsigned)written, (unsigned)totalUs, SerialPort.getFifoUsed());
    
    for (int i = 0; result[i] != '\0'; i++) {
      SerialPort.write((uint8_t)result[i]);
    }
    
    // Debug only every 5th test to reduce overhead
    if (TestCount % 5 == 0) {
      SerialPort.debugStatus();
    }
    
    TestCount++;
  }
}
//16MHZ HSE
/*extern "C" void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}
    */
//8MHz Crystal
extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON; // 🔹 pakai crystal 8 MHz, bukan bypass
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  RCC_OscInitStruct.PLL.PLLM = 8;             // 8 MHz / 8 = 1 MHz (PLL input)
  RCC_OscInitStruct.PLL.PLLN = 336;           // 1 MHz * 336 = 336 MHz (VCO freq)
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // 336 / 2 = 168 MHz (SYSCLK)
  RCC_OscInitStruct.PLL.PLLQ = 7;             // 336 / 7 = 48 MHz (untuk USB)

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // 168 MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  // 42 MHz (max APB1 = 42 MHz)
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  // 84 MHz (max APB2 = 84 MHz)

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

