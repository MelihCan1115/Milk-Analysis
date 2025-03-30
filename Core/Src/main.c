/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_lcd.h"
#include "stdio.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum{
	LOW,
	HIGH
}POL_t;

typedef enum{
	False,
	True
}BOOL_t;

uint8_t RxBufTemp[2]; //uart gelen byte
uint8_t RxBuf[30]; //rfid anten gelen eid
uint8_t RxIndex=0; //uart index
uint8_t CalibrateIndex=0; //calibre verisi index
uint8_t PcReq[2]; //pc sorgusu
char kupebuffer[14];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
I2C_LCD_HandleTypeDef lcd1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

        if(huart == &huart3)
        {
        	RxBuf[RxIndex]=RxBufTemp[0];
        			HAL_UART_Receive_IT(&huart3, RxBufTemp, 1); //interrupt yeniden aktif edildi
        			if(RxIndex>0 && RxBuf[RxIndex]==35 && RxBuf[RxIndex-1]==119)
        			{
        				RxBuf[RxIndex]=0;
        				RxBuf[RxIndex-1]=0;
        				RxBuf[0]=119;
        				RxBuf[1]=35;
        				RxIndex=2;
        				return;
        			}
        			RxIndex++;
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	  uint8_t sensorDurumu = 0;
	  uint8_t oncekiSensorDurumu = 0;
	  unsigned long baslangicZamani = 0; // Su akışı başladığında zamanı kaydet
	  int suAktigiSure = 0;    // Su akışı olduğu süre boyunca geçen süre
	  int16_t toplamSure = 0;      // İlk su akışı başladığı andan itibaren geçen toplam süre
	  float kalibrasyon = 57;
	  uint8_t suAkiyor = 0;
	  int toplamSuMiktari = 0;
	  uint8_t ilkSuBasladi = 0; // İlk su akışı başladı mı?
	  unsigned long ilkSuBaslangicZamani = 0;
	  char buffer[50];
//	  uint16_t serialData[2];

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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  HAL_UART_Receive_IT(&huart3, RxBufTemp, 1);
  /* USER CODE BEGIN 2 */

  lcd1.hi2c = &hi2c1;
  lcd1.address = 0x4E;
  /*  lcd_init(&lcd1);

  lcd_clear(&lcd1);
  lcd_gotoxy(&lcd1, 0, 0);
  lcd_puts(&lcd1, "Sistem basladi");
  lcd_gotoxy(&lcd1, 0, 1);
  lcd_puts(&lcd1,"Toplam : 0 ml");
  lcd_gotoxy(&lcd1, 0, 2);
  lcd_puts(&lcd1,"Kalibrasyon : ");
  sprintf(buffer,"%.2f", kalibrasyon);
  lcd_puts(&lcd1,buffer);
  lcd_gotoxy(&lcd1,0, 3);
  lcd_puts(&lcd1, "Zaman :     sn");
  HAL_Delay(100);
*/
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,LOW);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,LOW);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,HIGH);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,LOW);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  sensorDurumu = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

	  if(( sensorDurumu == HIGH ) && ( oncekiSensorDurumu == LOW) ){
//		  lcd_clear(&lcd1);
//		  lcd_puts(&lcd1, "Su Geliyor...");
		  baslangicZamani = HAL_GetTick();
		  suAkiyor = True;


		  if (!ilkSuBasladi) {
			 ilkSuBasladi = True;
			 ilkSuBaslangicZamani = HAL_GetTick(); // İlk su akışı başladığı andaki zamanı kaydet
		   }

//		  lcd_clear(&lcd1);
//		  lcd_puts(&lcd1, "Su Geliyor...");
//		  HAL_Delay(50);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,LOW);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,HIGH);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,LOW);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,LOW);
		  HAL_Delay(50);
		}





	  if (sensorDurumu == LOW && oncekiSensorDurumu == HIGH) {
	    if (suAkiyor) {
	      suAktigiSure += (HAL_GetTick() - baslangicZamani) / 1000; // Su akış süresini güncelle
	      HAL_Delay(50);

		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,HIGH);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,LOW);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,LOW);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,HIGH);
		  HAL_Delay(50);


	      unsigned long suMiktari = ((HAL_GetTick() - baslangicZamani) / 1000) * kalibrasyon;
	      toplamSuMiktari += suMiktari;

//		  UART SPESIFIK VERI OZELLIKLERI

//	      huart2.Instance = USART2;
//	      huart2.Init.BaudRate = 115200;
//	      huart2.Init.WordLength = UART_WORDLENGTH_8B;
//	      huart2.Init.StopBits = UART_STOPBITS_1;
//	      huart2.Init.Parity = UART_PARITY_NONE;
//	      huart2.Init.Mode = UART_MODE_TX_RX; // Hem TX hem RX aktif
//	      Alternatif: Binary Veri Gönderme
//	      uint8_t serialBuffer[4];
//	      serialBuffer[0] = (toplamSuMiktari >> 8) & 0xFF; // High byte
//	      serialBuffer[1] = toplamSuMiktari & 0xFF;        // Low byte
//	      serialBuffer[2] = (suAktigiSure >> 8) & 0xFF;    // High byte
//	      serialBuffer[3] = suAktigiSure & 0xFF;           // Low byte
//	      HAL_UART_Transmit(&huart2, serialBuffer, 4, HAL_MAX_DELAY);


	      char serialBuffer[50];

	      kupebuffer[0]= RxBuf[0];
	      kupebuffer[1]= RxBuf[1];
	      kupebuffer[2]= RxBuf[2];
	      kupebuffer[3]= RxBuf[3];
	      kupebuffer[4]= RxBuf[4];
	      kupebuffer[5]= RxBuf[5];
	      kupebuffer[6]= RxBuf[6];
	      kupebuffer[7]= RxBuf[7];
	      kupebuffer[8]= RxBuf[8];
	      kupebuffer[9]= RxBuf[9];
	      kupebuffer[10]= RxBuf[10];
	      kupebuffer[11]= RxBuf[11];
	      kupebuffer[12]= RxBuf[12];
	      kupebuffer[13]= RxBuf[13];

	      HAL_Delay(50);

//	      for(int i = 0; i < 14; i++) {
//	          kupebuffer[i] = RxBuf[i];
//	      }

	      // RxBuf'ın ilk 14 byte'ını temizle
//	      for(int i = 0; i < 14; i++) {
//	          RxBuf[i] = 0;
//	      }

	      // Veya daha kısa yolu:
//	      memcpy(kupebuffer, RxBuf, 14); // Kopyalama
//	      memset(RxBuf, 0, 14);          // Temizleme

	      sprintf(serialBuffer,
	          "Toplam: %d ml, Sure: %d sn,\nRFID: "
	          "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
	          toplamSuMiktari, suAktigiSure,
	          kupebuffer[0], kupebuffer[1], kupebuffer[2], kupebuffer[3],
	          kupebuffer[4], kupebuffer[5], kupebuffer[6], kupebuffer[7],
	          kupebuffer[8], kupebuffer[9], kupebuffer[10], kupebuffer[11],
	          kupebuffer[12], kupebuffer[13]);
	      if (HAL_UART_Transmit(&huart2, (uint8_t*)serialBuffer, strlen(serialBuffer), HAL_MAX_DELAY) != HAL_OK) {
	          Error_Handler();
		      HAL_Delay(50);
	      }
//	         lcd_clear(&lcd1);
/*	      lcd_gotoxy(&lcd1, 0, 0);
	      lcd_puts(&lcd1, "Su durdu..... ");
	      lcd_gotoxy(&lcd1,9,1);
//	      lcd_puts(&lcd1,"Toplam : ");
	      sprintf(buffer,"%d",toplamSuMiktari);
	      lcd_gotoxy(&lcd1, 15, 2);
	      sprintf(buffer,"%.2f", kalibrasyon);
	      lcd_puts(&lcd1 ,buffer);
*/
	      suAkiyor = False;
	    }
	  }

	  if (suAkiyor) {
	    // Su akışı devam ederken geçen süreyi hesapla
	    int anlikSuAktigiSure = (HAL_GetTick()- baslangicZamani) / 1000;
	    unsigned long anlikSuMiktari = anlikSuAktigiSure * kalibrasyon;
	    int toplamSu = toplamSuMiktari + anlikSuMiktari;
	    /*
	    lcd_gotoxy(&lcd1 ,0, 1);
	    lcd_puts(&lcd1 ,"Toplam : ");
	    HAL_Delay(50);
	    sprintf(buffer,"%d",toplamSu);
	    lcd_puts(&lcd1 ,buffer);
	   // sprintf()
	    lcd_puts(&lcd1 ," ml  ");

	    // Su akışı sırasında X değerini güncelle
	    lcd_gotoxy(&lcd1 ,0, 3);
	    lcd_puts(&lcd1 ,"Zaman : ");
	    sprintf(buffer,"%d",suAktigiSure + anlikSuAktigiSure);
	    lcd_puts(&lcd1 ,buffer); // X değeri (su akış süresi)
	    lcd_puts(&lcd1 ," / ");
	    sprintf(buffer,"%d",toplamSure);
	    lcd_puts(&lcd1 ,buffer); // Y değeri (toplam süre)
	    lcd_puts(&lcd1 ," sn  ");
	    */
	  }

	  // İlk su akışı başladıysa toplam süreyi hesapla
	  if (ilkSuBasladi) {
	    toplamSure = ((HAL_GetTick()) - ilkSuBaslangicZamani) / 1000;
	  }

	  // Su akışı olmadığında da X ve Y değerlerini güncelle
	  if (!suAkiyor) {

	//	lcd_clear(&lcd1);
//		lcd_gotoxy(&lcd1 ,0, 3);
//		lcd_puts(&lcd1 ,"Zaman : ");
		//lcd_puts(&lcd1 ,(suAktigiSure + anlikSuAktigiSure)); // X değeri (su akış süresi)
//		lcd_puts(&lcd1 ," / ");
/*		lcd_gotoxy(&lcd1 ,12, 3);
		sprintf(buffer,"%d",toplamSure);
		lcd_puts(&lcd1 ,buffer); // Y değeri (toplam süre)
//		lcd_puts(&lcd1 ," sn  ");
 */
	  }

	  oncekiSensorDurumu = sensorDurumu;
	  HAL_Delay(50);


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
