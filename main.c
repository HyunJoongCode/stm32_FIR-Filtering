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
#include "arm_math.h"
#include "math_helper.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_SAMPLES 2000
#define BLOCK_SIZE 1
#define COEFF 300

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t adc_flag = 0;
uint8_t end_flag = 1;
float adc_value = 0.0;
float vol = 0.0;

uint32_t i = 0;
uint32_t j = 0;
uint32_t cnt = 0;

float32_t firStateF32[BLOCK_SIZE + COEFF - 1];

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = MAX_SAMPLES/BLOCK_SIZE;

float32_t input[MAX_SAMPLES];
float32_t output[MAX_SAMPLES];

float coeffs300[COEFF] = {
		0.000005340f,  0.000016128f,  0.000027093f,  0.000038279f,  0.000049726f,  0.000061472f,  0.000073554f,  0.000086004f,  0.000098849f,  0.000112109f,
		 0.000125798f,  0.000139922f,  0.000154479f,  0.000169457f,  0.000184833f,  0.000200575f,  0.000216638f,  0.000232966f,  0.000249490f,  0.000266129f,
		 0.000282791f,  0.000299368f,  0.000315741f,  0.000331781f,  0.000347343f,  0.000362273f,  0.000376406f,  0.000389565f,  0.000401565f,  0.000412212f,
		 0.000421307f,  0.000428641f,  0.000434004f,  0.000437181f,  0.000437955f,  0.000436110f,  0.000431432f,  0.000423709f,  0.000412735f,  0.000398313f,
		 0.000380254f,  0.000358379f,  0.000332526f,  0.000302544f,  0.000268304f,  0.000229692f,  0.000186620f,  0.000139019f,  0.000086849f,  0.000030094f,
		-0.000031232f, -0.000097085f, -0.000167389f, -0.000242035f, -0.000320880f, -0.000403747f, -0.000490422f, -0.000580653f, -0.000674153f, -0.000770598f,
		-0.000869625f, -0.000970834f, -0.001073789f, -0.001178017f, -0.001283010f, -0.001388226f, -0.001493089f, -0.001596993f, -0.001699300f, -0.001799347f,
		-0.001896444f, -0.001989877f, -0.002078912f, -0.002162799f, -0.002240771f, -0.002312050f, -0.002375851f, -0.002431382f, -0.002477852f, -0.002514472f,
		-0.002540457f, -0.002555036f, -0.002557450f, -0.002546958f, -0.002522843f, -0.002484412f, -0.002431004f, -0.002361991f, -0.002276784f, -0.002174835f,
		-0.002055643f, -0.001918755f, -0.001763771f, -0.001590347f, -0.001398197f, -0.001187099f, -0.000956893f, -0.000707487f, -0.000438859f, -0.000151057f,
		 0.000155800f,  0.000481516f,  0.000825824f,  0.001188379f,  0.001568762f,  0.001966475f,  0.002380950f,  0.002811540f,  0.003257527f,  0.003718119f,
		 0.004192455f,  0.004679605f,  0.005178574f,  0.005688304f,  0.006207674f,  0.006735511f,  0.007270586f,  0.007811621f,  0.008357295f,  0.008906244f,
		 0.009457071f,  0.010008347f,  0.010558618f,  0.011106410f,  0.011650232f,  0.012188586f,  0.012719969f,  0.013242882f,  0.013755832f,  0.014257338f,
		 0.014745943f,  0.015220211f,  0.015678738f,  0.016120157f,  0.016543143f,  0.016946415f,  0.017328750f,  0.017688977f,  0.018025989f,  0.018338747f,
		 0.018626280f,  0.018887695f,  0.019122175f,  0.019328986f,  0.019507479f,  0.019657092f,  0.019777353f,  0.019867884f,  0.019928397f,  0.019958702f,
		 0.019958702f,  0.019928397f,  0.019867884f,  0.019777353f,  0.019657092f,  0.019507479f,  0.019328986f,  0.019122175f,  0.018887695f,  0.018626280f,
		 0.018338747f,  0.018025989f,  0.017688977f,  0.017328750f,  0.016946415f,  0.016543143f,  0.016120157f,  0.015678738f,  0.015220211f,  0.014745943f,
		 0.014257338f,  0.013755832f,  0.013242882f,  0.012719969f,  0.012188586f,  0.011650232f,  0.011106410f,  0.010558618f,  0.010008347f,  0.009457071f,
		 0.008906244f,  0.008357295f,  0.007811621f,  0.007270586f,  0.006735511f,  0.006207674f,  0.005688304f,  0.005178574f,  0.004679605f,  0.004192455f,
		 0.003718119f,  0.003257527f,  0.002811540f,  0.002380950f,  0.001966475f,  0.001568762f,  0.001188379f,  0.000825824f,  0.000481516f,  0.000155800f,
		-0.000151057f, -0.000438859f, -0.000707487f, -0.000956893f, -0.001187099f, -0.001398197f, -0.001590347f, -0.001763771f, -0.001918755f, -0.002055643f,
		-0.002174835f, -0.002276784f, -0.002361991f, -0.002431004f, -0.002484412f, -0.002522843f, -0.002546958f, -0.002557450f, -0.002555036f, -0.002540457f,
		-0.002514472f, -0.002477852f, -0.002431382f, -0.002375851f, -0.002312050f, -0.002240771f, -0.002162799f, -0.002078912f, -0.001989877f, -0.001896444f,
		-0.001799347f, -0.001699300f, -0.001596993f, -0.001493089f, -0.001388226f, -0.001283010f, -0.001178017f, -0.001073789f, -0.000970834f, -0.000869625f,
		-0.000770598f, -0.000674153f, -0.000580653f, -0.000490422f, -0.000403747f, -0.000320880f, -0.000242035f, -0.000167389f, -0.000097085f, -0.000031232f,
		 0.000030094f,  0.000086849f,  0.000139019f,  0.000186620f,  0.000229692f,  0.000268304f,  0.000302544f,  0.000332526f,  0.000358379f,  0.000380254f,
		 0.000398313f,  0.000412735f,  0.000423709f,  0.000431432f,  0.000436110f,  0.000437955f,  0.000437181f,  0.000434004f,  0.000428641f,  0.000421307f,
		 0.000412212f,  0.000401565f,  0.000389565f,  0.000376406f,  0.000362273f,  0.000347343f,  0.000331781f,  0.000315741f,  0.000299368f,  0.000282791f,
		 0.000266129f,  0.000249490f,  0.000232966f,  0.000216638f,  0.000200575f,  0.000184833f,  0.000169457f,  0.000154479f,  0.000139922f,  0.000125798f,
		 0.000112109f,  0.000098849f,  0.000086004f,  0.000073554f,  0.000061472f,  0.000049726f,  0.000038279f,  0.000027093f,  0.000016128f,  0.000005340f
};
/*float coeffs200[COEFF] = {
		-0.000008008f, -0.000024304f, -0.000041165f, -0.000058819f, -0.000077497f, -0.000097426f, -0.000118827f, -0.000141908f, -0.000166865f, -0.000193872f,
		-0.000223082f, -0.000254620f, -0.000288581f, -0.000325025f, -0.000363976f, -0.000405416f, -0.000449284f, -0.000495472f, -0.000543824f, -0.000594135f,
		-0.000646145f, -0.000699544f, -0.000753966f, -0.000808992f, -0.000864151f, -0.000918916f, -0.000972709f, -0.001024903f, -0.001074823f, -0.001121746f,
		-0.001164911f, -0.001203513f, -0.001236717f, -0.001263656f, -0.001283436f, -0.001295144f, -0.001297854f, -0.001290629f, -0.001272529f, -0.001242619f,
		-0.001199974f, -0.001143687f, -0.001072872f, -0.000986677f, -0.000884287f, -0.000764929f, -0.000627886f, -0.000472494f, -0.000298157f, -0.000104349f,
		 0.000109379f,  0.000343396f,  0.000597981f,  0.000873321f,  0.001169507f,  0.001486529f,  0.001824275f,  0.002182523f,  0.002560946f,  0.002959105f,
		 0.003376450f,  0.003812320f,  0.004265944f,  0.004736441f,  0.005222822f,  0.005723995f,  0.006238764f,  0.006765836f,  0.007303825f,  0.007851257f,
		 0.008406574f,  0.008968145f,  0.009534268f,  0.010103180f,  0.010673067f,  0.011242065f,  0.011808279f,  0.012369783f,  0.012924634f,  0.013470881f,
		 0.014006575f,  0.014529778f,  0.015038571f,  0.015531070f,  0.016005427f,  0.016459849f,  0.016892601f,  0.017302015f,  0.017686504f,  0.018044567f,
		 0.018374796f,  0.018675887f,  0.018946644f,  0.019185987f,  0.019392958f,  0.019566725f,  0.019706589f,  0.019811985f,  0.019882488f,  0.019917810f,
		 0.019917810f,  0.019882488f,  0.019811985f,  0.019706589f,  0.019566725f,  0.019392958f,  0.019185987f,  0.018946644f,  0.018675887f,  0.018374796f,
		 0.018044567f,  0.017686504f,  0.017302015f,  0.016892601f,  0.016459849f,  0.016005427f,  0.015531070f,  0.015038571f,  0.014529778f,  0.014006575f,
		 0.013470881f,  0.012924634f,  0.012369783f,  0.011808279f,  0.011242065f,  0.010673067f,  0.010103180f,  0.009534268f,  0.008968145f,  0.008406574f,
		 0.007851257f,  0.007303825f,  0.006765836f,  0.006238764f,  0.005723995f,  0.005222822f,  0.004736441f,  0.004265944f,  0.003812320f,  0.003376450f,
		 0.002959105f,  0.002560946f,  0.002182523f,  0.001824275f,  0.001486529f,  0.001169507f,  0.000873321f,  0.000597981f,  0.000343396f,  0.000109379f,
		-0.000104349f, -0.000298157f, -0.000472494f, -0.000627886f, -0.000764929f, -0.000884287f, -0.000986677f, -0.001072872f, -0.001143687f, -0.001199974f,
		-0.001242619f, -0.001272529f, -0.001290629f, -0.001297854f, -0.001295144f, -0.001283436f, -0.001263656f, -0.001236717f, -0.001203513f, -0.001164911f,
		-0.001121746f, -0.001074823f, -0.001024903f, -0.000972709f, -0.000918916f, -0.000864151f, -0.000808992f, -0.000753966f, -0.000699544f, -0.000646145f,
		-0.000594135f, -0.000543824f, -0.000495472f, -0.000449284f, -0.000405416f, -0.000363976f, -0.000325025f, -0.000288581f, -0.000254620f, -0.000223082f,
		-0.000193872f, -0.000166865f, -0.000141908f, -0.000118827f, -0.000097426f, -0.000077497f, -0.000058819f, -0.000041165f, -0.000024304f, -0.000008008f
};*/
/*float coeffs100[COEFF] = {
		 0.000019244f,  0.000059528f,  0.000104499f,  0.000157056f,  0.000220228f,  0.000297130f,  0.000390927f,  0.000504785f,  0.000641829f,  0.000805094f,
		 0.000997480f,  0.001221707f,  0.001480267f,  0.001775387f,  0.002108983f,  0.002482625f,  0.002897502f,  0.003354395f,  0.003853644f,  0.004395134f,
		 0.004978278f,  0.005602001f,  0.006264744f,  0.006964458f,  0.007698616f,  0.008464220f,  0.009257828f,  0.010075569f,  0.010913182f,  0.011766044f,
		 0.012629212f,  0.013497471f,  0.014365375f,  0.015227305f,  0.016077518f,  0.016910207f,  0.017719558f,  0.018499807f,  0.019245301f,  0.019950555f,
		 0.020610309f,  0.021219583f,  0.021773732f,  0.022268489f,  0.022700017f,  0.023064946f,  0.023360412f,  0.023584084f,  0.023734192f,  0.023809545f,
		 0.023809545f,  0.023734192f,  0.023584084f,  0.023360412f,  0.023064946f,  0.022700017f,  0.022268489f,  0.021773732f,  0.021219583f,  0.020610309f,
		 0.019950555f,  0.019245301f,  0.018499807f,  0.017719558f,  0.016910207f,  0.016077518f,  0.015227305f,  0.014365375f,  0.013497471f,  0.012629212f,
		 0.011766044f,  0.010913182f,  0.010075569f,  0.009257828f,  0.008464220f,  0.007698616f,  0.006964458f,  0.006264744f,  0.005602001f,  0.004978278f,
		 0.004395134f,  0.003853644f,  0.003354395f,  0.002897502f,  0.002482625f,  0.002108983f,  0.001775387f,  0.001480267f,  0.001221707f,  0.000997480f,
		 0.000805094f,  0.000641829f,  0.000504785f,  0.000390927f,  0.000297130f,  0.000220228f,  0.000157056f,  0.000104499f,  0.000059528f,  0.000019244f
};*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __cplusplus
extern "C" int _write(int32_t file, uint8_t *ptr, int32_t len) {
#else
int _write(int32_t file, uint8_t *ptr, int32_t len) {
#endif
if( HAL_UART_Transmit(&huart1, ptr, len, len) == HAL_OK ) return len;
else return 0;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_IncTick(void){
	adc_flag = 1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	arm_fir_instance_f32 S;
	float32_t inputF32, outputF32;
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  arm_fir_init_f32(&S, COEFF, (float32_t *)&coeffs300[0], &firStateF32[0], blockSize);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (end_flag)
  {
	  if(adc_flag == 1){
	 		  HAL_GPIO_WritePin(check_GPIO_Port, check_Pin, 1);
	 		  adc_flag = 0;
	 		  HAL_ADC_Start(&hadc1);
	 		  HAL_ADC_PollForConversion(&hadc1, 10);
	 		  adc_value = HAL_ADC_GetValue(&hadc1);
	 		  vol = (3.295 * adc_value) / 4095.0;
	 		  inputF32 = vol;
	 		  //input[cnt]=vol;
	 		  arm_fir_f32(&S, &inputF32, &outputF32, blockSize);
	          //output[cnt]=outputF32;
	          //cnt++;
	 		  //if(cnt >= MAX_SAMPLES){
	 			//  end_flag = 0;
	 		  //}
	 		  HAL_GPIO_WritePin(check_GPIO_Port, check_Pin, 0);
	 	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  for(j=0; j < MAX_SAMPLES; j++){
	  printf("%f, %f\n\r",input[j], output[j]);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(check_GPIO_Port, check_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : check_Pin */
  GPIO_InitStruct.Pin = check_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(check_GPIO_Port, &GPIO_InitStruct);

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
