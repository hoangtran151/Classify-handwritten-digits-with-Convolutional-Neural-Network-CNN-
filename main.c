/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int file_ready = 0;
extern uint8_t rtext[4096];
extern uint8_t name[10];
float * out_img;
float val1;
float val2;
int idx = 0;
float temp = 0;
float temp_input;
float pool_test[] = {1,4.5,2,3.5,3,2.5,4,1.5,5,8.5,6,7.5,7,6.5,8,5.5,9,12.5,10,11.5,11,10.5,12,9.5,13,16.5,14,15.5,15,14.5,16,13.5};
float dense_test[] = {1.5,2.5,3.5,4.5,5.5,2.5,3.5,4.5,5.5,6.5,3.5,4.5,5.5,6.5,7.5,4.5,5.5,6.5,7.5,8.5,5.5,6.5,7.5,8.5,9.5,6.5,7.5,8.5,9.5,10.5,7.5,8.5,9.5,10.5,11.5,8.5,9.5,10.5,11.5,12.5};
int y = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

                void dense(float input[], float kernel[], float bias[], float output[], int input_size, int output_size)
                {

                    //dense
                    for (int i = 0; i < output_size; i++)
                    {
                        output[i] = 0;
                        for (size_t j = 0; j < input_size; j++)
                        {
                            output[i] += (kernel[y] * input[j])/16;
                            y++;



                        }

                        output[i] += bias[i];
                       if(output[i]<0)
                        output[i]=0;


                    }



                    // softmax(probability)
                    double sum = 0;
                    for (int i = 0; i < output_size; i++)
                    {
                        sum += exp(output[i]);
                    }
                    for (int j = 0; j < output_size; j++)
                    {
                        output[j] = exp(output[j]) / sum;
                    }
                }


                void conv(float input[], float output[], int num_of_filters, int input_height, int input_width, int layers, int f_height, int f_width, float bias[], float weights[]) {
                                float *p = (float *)malloc(layers*(input_width + 2)*(input_width + 2)*sizeof(float));
                                int temp_index = 0;
                                float temp = 0;
                                int idx = 0;

                                for(int q = 0; q < layers; q++) {
                                                for(int o = 0; o < (input_width + 2)*(input_width + 2); o++) {
                                                                if(o < input_width + 2 || o > (input_width + 2)*(input_width + 1)) {
                                                                                p[o + q*(input_width + 2)*(input_width + 2)] = 0;
                                                                } else if(o % (input_width + 2) == 0 || (o+1) % (input_width + 2) == 0) {
                                                                                p[o + q*(input_width + 2)*(input_width + 2)] = 0;
                                                                } else {
                                                                                p[o + q*(input_width + 2)*(input_width + 2)] = input[temp_index];
                                                                                temp_index++;
                                                                }
                                                }
                                }


                                for(int i = 0; i < num_of_filters; i++) {
                                                for(int j = 0; j < input_height; j++) {
                                                                for(int k = 0; k < input_width; k++) {
                                                                                for (int l = 0; l < layers; l++) {
                                                                                                for (int m = 0, n = 0; m < f_height*f_width; m++, n++) {
                                                                                                                if (m % f_width == 0 && m != 0) {
                                                                                                                                n = n + (input_width+2) - f_width;
                                                                                                                }
                                                                                                                temp = temp + weights[l*f_height*f_width + m + i * layers*f_height*f_width]*p[k + n + j *(input_width+2) + l*(input_width + 2)*(input_width + 2)];
                                                                                                }
                                                                                }
                                                                                if(temp+bias[i]>0)
                                                                                output[idx] = temp + bias[i];
                                                                                else
                                                                               output[idx] = 0;
                                                                                idx++;
                                                                                temp = 0;
                                                                }
                                                }
                                }
                                free(p);
                }

void nn_pool(float input[], float output[], int height, int width, int channel) {
                float val1;
                float val2;
                int l = 0;
                for(int k = 0; k < channel; k++) {
                                for(int i = 0; i < height; i = i + 2) {
                                                for(int j = 0; j < width; j = j + 2) {

                                                                                                                if(input[j + i*width + k*height*width] < input[j + 1 + i*width + k*height*width]) { // compare first and second values
                                                                                                                                val1 = input[j + 1 + i*width + k*height*width];
                                                                                                                } else {
                                                                                                                                val1 = input[j + i*width + k*height*width];
                                                                                                                }
                                                                                                                if(input[j + width + i*width + k*height*width] < input[j + 1 + width + i*width + k*height*width]) { // compare third and fourth values
                                                                                                                                val2 = input[j + 1 + width + i*width + k*height*width];
                                                                                                                } else {
                                                                                                                                val2 = input[j + width + i*width + k*height*width];
                                                                                                                }
                                                                                                                if(val1 < val2) {
                                                                                                                                output[l] = val2;
                                                                                                                                l++;
                                                                                                                } else {
                                                                                                                                output[l] = val1;
                                                                                                                                l++;
                                                                                                                }
                                                }
                                }
                }
}


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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 // while (1)
 // {
    /* USER CODE END WHILE */
  /*  MX_USB_HOST_Process();

    /* USER CODE BEGIN 3
                sprintf(name,"num3.bmp");
    file_ready = 0;
    read_bmp(name);
    if(file_ready == 1)
    {
                out_img = ProcessBmp(rtext);
                    break;
    }
  }
  printf("Hello World\n");
   for(int i = 0; i < 28; i++){
 	for(int j = 0; j < 28; j++){
 		printf("%.0f ", out_img[i*28 + j]);
 	}
 	printf("\n ");
   }

 sprintf(name,"b1.txt");
float * b1 = read_txt(name, 4);


  sprintf(name,"w1.txt");
  float * w1= read_txt(name, 36);

  sprintf(name,"b2.txt");
  float * b2 = read_txt(name, 8);

  sprintf(name,"w2.txt");
  float * w2 = read_txt(name, 288);

  sprintf(name,"b3.txt");
  float * b3 = read_txt(name, 16);

  sprintf(name,"w3.txt");
  float * w3 = read_txt(name, 1152);

  sprintf(name,"bc.txt");
  float * bc = read_txt(name, 10);

  sprintf(name,"fc.txt");

  float * fc = read_txt(name, 160);



  float* output1 = (float *)malloc(3136*sizeof(float));
  conv(out_img,output1,4,28,28,1,3,3,b1,w1);
  free(w1);
  free(b1);


  float* output2 = (float *)malloc(784*sizeof(float));
  nn_pool(output1,output2,28,28,4);
  free(output1);


  float* output3 = (float *)malloc(1568*sizeof(float));
  conv(output2,output3,8,14,14,4,3,3,b2,w2);
  free(b2);
   free(w2);
  free(output2);

  float* output4 = (float *)malloc(392*sizeof(float));
  nn_pool(output3,output4,14,14,8);

  free(output3);

  float* output5 = (float *)malloc(784*sizeof(float));
  conv(output4,output5,16,7,7,8,3,3,b3,w3);
  free(b3);
   free(w3);
   free(output4);



  float* output6 = (float *)malloc(144*sizeof(float));
  nn_pool(output5,output6,7,7,16);

  free(output5);




  float* output7 = (float *)malloc(16*sizeof(float));
  nn_pool(output6,output7,3,3,16);
  free(output6);




  float* output8 = (float *)malloc(10*sizeof(float));
  dense(output7,fc,bc,output8,16,10);
  free(output7);
  for(int l = 0; l < 10; l++) {
  printf("prediction of number %d is %.5f\n", l, output8[l]);
  HAL_Delay(10);
  }
    free(output8);
    printf("Hello World\n");




*/











  // Test Conv function 1


  float input[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
  float weights[] ={1,2,3,4,5,6,7,8,9,3,3,3,3,3,3,3,3,3};
  float output[50];
  float bias[] = {0.5,1};
  conv(input, output, 2, 5,5,1,3,3,bias,weights);
  HAL_Delay(1000);
  int idx=0;
  for(int l = 0; l < 10; l++)
   {

	  for(int i=0;i<5;i++)
	  {
		  printf("%.2f ", output[idx]);
    idx++;
	  }

	  printf("\n");

    HAL_Delay(10);
   }



//test conv function 2

 /*float input[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5,2.5};
  float output[75];
  float bias[] = {0.5,1,2.5};
  float weights[] = {1,2,3,4,5,6,7,8,9,1,1,1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,2,2,2,2,2,2,2,2,2,5,5,5,5,5,5,5,5,5,0,0,0,0,0,0,0,0,0};
  conv(input, output, 3, 5, 5, 2, 3, 3, bias, weights);

  for(int l = 0; l < 75; l++)
  {

	  printf(" number %d is %.4f\n", l,output[l]);
	      HAL_Delay(10);

   }
  int idx=0;
    for(int l = 0; l < 15; l++)
     {
    	if(l%5==0)
    		 printf("\n");

  	  for(int i=0;i<5;i++)
  	  {
  		  printf("%.2f ", output[idx]);
      idx++;
  	  }

  	  printf("\n");

      HAL_Delay(10);
     }
// Test nn pool
 /*float input[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,4.5,3.5,2.5,1.5,8.5,7.5,6.5,5.5,12.5,11.5,10.5,9.5,16.5,15.5,14.5,13.5};
float output[8];
  nn_pool(input,output,4,4,2);
  for(int l = 0; l < 8; l++)
    {


     printf(" number %d is %.4f\n", l,output[l]);
     HAL_Delay(10);

     }
  while(1)
     {}
 //Test dense function

 /* float input[] = {2,2,3,2,4.5,3,3,2.2};
  float output[10];
  float bias[] = {0,1,2,3,4,5,6,7,8,9};
  float kernel[] = { 1. ,  1.5,  2. ,  2.5,  3. ,  3.5,  4. ,  4.5,  5. ,  5.5,
       2. ,  2.5,  3. ,  3.5,  4. ,  4.5,  5. ,  5.5,  6. ,  6.5,
       3. ,  3.5,  4. ,  4.5,  5. ,  5.5,  6. ,  6.5,  7. ,  7.5,
       4. ,  4.5,  5. ,  5.5,  6. ,  6.5,  7. ,  7.5,  8. ,  8.5,
       5. ,  5.5,  6. ,  6.5,  7. ,  7.5,  8. ,  8.5,  9. ,  9.5,
       6. ,  6.5,  7. ,  7.5,  8. ,  8.5,  9. ,  9.5, 10. , 10.5,
       7. ,  7.5,  8. ,  8.5,  9. ,  9.5, 10. , 10.5, 11. , 11.5,
       8. ,  8.5,  9. ,  9.5, 10. , 10.5, 11. , 11.5, 12. , 12.5};

  dense(input,kernel,bias,output,8,10);
  for(int l = 0; l < 10; l++)
      {


       printf("prediction of number %d is %.4f\n", l,output[l]);
       HAL_Delay(10);

       }
    while(1)
       {}









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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable USB Voltage detector
  */
  HAL_PWREx_EnableUSBVoltageDetector();
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
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
  HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
