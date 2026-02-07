/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g0xx_hal.h"
#include "stm32g0xx_hal_conf.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define JK_BUFFER_SIZE 512
#define DWIN_BUFFER_SIZE 32
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VP_VOLTAGE      0x1000 // Voltaj (0.01V biriminde)
#define VP_CURRENT      0x1002 // Akım (0.01A biriminde)
#define VP_SOC          0x1004 // Yüzde (%)
#define VP_TEMP_MOS     0x1006 // Mosfet Sıcaklığı
#define VP_TEMP_BAT     0x1008 // Batarya Sıcaklığı
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
uint8_t jk_rx_buffer[JK_BUFFER_SIZE];   // DMA'nın doldurduğu ham tampon
uint8_t jk_main_buffer[JK_BUFFER_SIZE]; // İşlem yapılacak güvenli kopya
volatile uint8_t data_ready_flag = 0;   // Veri geldi bayrağı
uint16_t received_len = 0;              // Gelen veri uzunluğu
/* --- DWIN VP ADRESLERİ (GÜNCELLENDİ) --- */
#define VP_SOC          0x1000 // SOC (0-100)
#define VP_TEMP1        0x1020 // Sıcaklık 1 (Pil)
#define VP_TEMP2        0x1040 // Sıcaklık 2 (MOS/Kutu)
#define VP_VOLTAGE      0x1060 // Voltaj (1 ondalık hane için ölçeklenecek)
#define VP_CURRENT      0x1080 // Akım (1 ondalık hane için ölçeklenecek)
// JK BMS "Read All" Komutu (Standart)
const uint8_t jk_request_cmd[] = {
    0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00,
    0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x68, 0x00, 0x00, 0x01, 0x29
};

/* --- BMS VERİ YAPISI --- */
typedef struct {
    uint16_t voltage_disp; // Ekrana gidecek (Örn: 546 => 54.6V)
    int16_t  current_disp; // Ekrana gidecek (Örn: 125 => 12.5A)
    uint16_t soc;
    int16_t  temp1;
    int16_t  temp2;
} BMS_Data_t;

BMS_Data_t bmsData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART5_UART_Init(void);
/* USER CODE BEGIN PFP */
void JKBMS_Request(void);
void JKBMS_Parse(uint8_t *buffer, uint16_t len);
void DWIN_SendVal16(uint16_t vp_addr, int16_t val); // 2 Byte gönderir (Sıcaklık, SOC için)
void DWIN_SendVal32(uint16_t vp_addr, int32_t val); // 4 Byte gönderir (Voltaj, Akım için)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t counter = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USART5_UART_Init();
  /* USER CODE BEGIN 2 */
// 1. USART5 üzerinden DMA ile dinlemeye başla (Idle Line Interrupt ile)
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, jk_rx_buffer, JK_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   
 
    // 2. BMS'ten Veri İste
        JKBMS_Request();

        // Verinin gelmesi için biraz bekle (BMS işlem süresi)
        HAL_Delay(250); 

        // 3. Veri geldiyse işle ve DWIN'e bas
        if (data_ready_flag)
        {
            // Veriyi ayrıştır
            JKBMS_Parse(jk_main_buffer, received_len);

            // DWIN Ekrana Gönder (Adresler yukarıda tanımlı)
            // JK Voltajı 0.01V birimindedir, DWIN'de ondalık hane ayarını 2 yaparsanız direkt gönderebilirsiniz.
           DWIN_SendVal16(VP_SOC, (int16_t)bmsData.soc);
            HAL_Delay(10);
            
            DWIN_SendVal16(VP_TEMP1, bmsData.temp1);
            HAL_Delay(10);
            
            DWIN_SendVal16(VP_TEMP2, bmsData.temp2);
            HAL_Delay(10);

            // ÖNEMLİ: Voltaj ve Akım 4 Byte olarak gidiyor
            DWIN_SendVal32(VP_VOLTAGE, bmsData.voltage_disp);
            HAL_Delay(10);
            
            DWIN_SendVal32(VP_CURRENT, bmsData.current_disp);
            HAL_Delay(10);

            data_ready_flag = 0; // Bayrağı indir
        }

        // Döngü hızı (1 saniyede bir güncelleme)
         counter++;
        HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART5_UART_Init(void)
{

  /* USER CODE BEGIN USART5_Init 0 */

  /* USER CODE END USART5_Init 0 */

  /* USER CODE BEGIN USART5_Init 1 */

  /* USER CODE END USART5_Init 1 */
  huart5.Instance = USART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART5_Init 2 */

  /* USER CODE END USART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// JK BMS'e veri okuma komutu gönderir
void JKBMS_Request(void) {
    HAL_UART_Transmit(&huart5, (uint8_t*)jk_request_cmd, sizeof(jk_request_cmd), 100);
}
// UART Rx Event Callback (Veri alımı bittiğinde veya IDLE olduğunda tetiklenir)
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART5)
    {
        // Gelen veriyi güvenli tampona al
        memcpy(jk_main_buffer, jk_rx_buffer, Size);
        received_len = Size;
        data_ready_flag = 1;

        // DMA'yı tekrar başlat (Bir sonraki paket için)
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, jk_rx_buffer, JK_BUFFER_SIZE);
    }
}
// JK Veri Ayrıştırma (Parsing)
void JKBMS_Parse(uint8_t *buffer, uint16_t len) {
    if (len < 20 || buffer[0] != 0x4E || buffer[1] != 0x57) return;

    for (uint16_t i = 0; i < len - 2; i++) {
        // Voltaj (0x83) -> Ekranda 1 decimal istendiği için 10'a bölüyoruz.
        // JK: 5460 (54.60V) -> /10 -> 546. Ekran "54.6" gösterir.
        if (buffer[i] == 0x83) {
            uint16_t raw_vol = (buffer[i+1] << 8) | buffer[i+2];
            bmsData.voltage_disp = raw_vol / 10; 
        }
        // Akım (0x84) -> Ekranda 1 decimal istendiği için 10'a bölüyoruz.
        else if (buffer[i] == 0x84) {
            uint16_t raw_curr = (buffer[i+1] << 8) | buffer[i+2];
            if (raw_curr & 0x8000) { // Negatif (Deşarj)
                 bmsData.current_disp = -((raw_curr & 0x7FFF) / 10);
            } else { // Pozitif (Şarj)
                 bmsData.current_disp = (raw_curr) / 10;
            }
        }
        // SOC (0x85)
        else if (buffer[i] == 0x85) {
            bmsData.soc = buffer[i+1];
        }
        // MOS Sıcaklığı (0x80) -> Temp2 (1040)
        else if (buffer[i] == 0x80) {
            bmsData.temp2 = (int16_t)buffer[i+1];
            if(bmsData.temp2 > 100) bmsData.temp2 -= 100; 
        }
        // Batarya Sıcaklığı (0x81) -> Temp1 (1020)
        else if (buffer[i] == 0x81) {
            bmsData.temp1 = (int16_t)buffer[i+1];
            if(bmsData.temp1 > 100) bmsData.temp1 -= 100;
        }
    }
}
// DWIN Ekrana Veri Gönderme Fonksiyonu (USART3)
// 5A A5 [Len] 82 [AddrH] [AddrL] [ValH] [ValL]
// DWIN Ekrana Veri Basma (Standart 2 Byte Integer Yazma)
void DWIN_SendVal16(uint16_t vp_addr, int16_t val) {
    uint8_t dwin_tx[8];
    
    dwin_tx[0] = 0x5A;
    dwin_tx[1] = 0xA5;
    dwin_tx[2] = 0x05; // Uzunluk: Cmd(1) + Addr(2) + Data(2) = 5
    dwin_tx[3] = 0x82;
    dwin_tx[4] = (vp_addr >> 8) & 0xFF;
    dwin_tx[5] = vp_addr & 0xFF;
    dwin_tx[6] = (val >> 8) & 0xFF;
    dwin_tx[7] = val & 0xFF;

    HAL_UART_Transmit(&huart3, dwin_tx, 8, 50);
}
// 4 Byte (Long Integer) Gönderme Fonksiyonu
void DWIN_SendVal32(uint16_t vp_addr, int32_t val) {
    uint8_t dwin_tx[10]; // 4 byte veri için paket uzar
    
    dwin_tx[0] = 0x5A;
    dwin_tx[1] = 0xA5;
    dwin_tx[2] = 0x07; // Uzunluk: Cmd(1) + Addr(2) + Data(4) = 7
    dwin_tx[3] = 0x82; // Yazma Komutu
    dwin_tx[4] = (vp_addr >> 8) & 0xFF;
    dwin_tx[5] = vp_addr & 0xFF;
    
    // 32 Bit veriyi 4 parçaya böl (Big Endian)
    dwin_tx[6] = (val >> 24) & 0xFF;
    dwin_tx[7] = (val >> 16) & 0xFF;
    dwin_tx[8] = (val >> 8) & 0xFF;
    dwin_tx[9] = val & 0xFF;

    HAL_UART_Transmit(&huart3, dwin_tx, 10, 50);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
