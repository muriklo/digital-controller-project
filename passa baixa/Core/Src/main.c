/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Implementação de Controlador Digital
 ******************************************************************************
 * @attention
 *
 * O código do controlador foi inserido neste arquivo. A lógica principal
 * é executada dentro da interrupção do TIM3, que ocorre a 10kHz. Um contador
 * interno faz com que o algoritmo de controle seja executado a cada 3ms (~333Hz).
 *
 * O código abaixo foi atualizado para refletir a lógica e os parâmetros do
 * exemplo em Mbed OS fornecido.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
// --- COEFICIENTES E ESTADOS DO CONTROLADOR (BASEADO NO CÓDIGO MBED) ---
// Período de amostragem em segundos (do código Mbed)
const float T_SAMPLE = 0.0001f;
const float VREF_VOLTS = 3.3f;

// Matrizes do observador (valores do código Mbed)
const float A[4] =
{ 0.0f, 1e7f, -1.7761e-3f, -1.1367e2f };

const float B[2] =
{ 0.0f, 1.7761e-3f };

const float Ke[2] =
{ 1.4719e2f, 2.9176e-3f };

const float Ki = 9045.3275f; // valor do integrador, calculado no MATLAB

const float Kc[2] =
{ 39.6464f, 1503575.2585f };

// ---- VARIÁVEIS GLOBAIS DO CONTROLADOR ----
volatile float g_control_u;

// Estados do Observador/Controlador
volatile float x1_obs;    // V_C1 estimado
volatile float x2_obs;    // i_C1 estimado
volatile float x_int;     // Estado do integrador
volatile float u_prev;    // Último sinal de controle (u[k-1])

// Derivadas dos estados (calculadas para a próxima iteração)
static float dx1;
static float dx2;

// Erro de rastreamento (usado no passo seguinte pelo integrador)
static float erro;

// Variáveis para leitura do ADC
uint16_t adc_raw_y;
float y_medido;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void Control_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t sample_count = 0;
float r_k = 1.0f;

int32_t pwm_duty;

/**
 * @brief Inicializa todas as variáveis de estado do controlador.
 */
void Control_Init(void)
{
	g_control_u = 0.0f;
	u_prev = 0.0f;

	x1_obs = 0.0f;
	x2_obs = 0.0f;
	x_int = 0.0f;

	dx1 = 0.0f;
	dx2 = 0.0f;
	erro = 0.0f;

	sample_count = 0;
	r_k = 1.0f; // Valor inicial da referência
}

/**
 * @brief  Callback da interrupção do Timer. Ocorre a cada 100us (10kHz).
 * @note   Esta função contém a lógica de controle em tempo real.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance != TIM3)
		return;
	// --- PASSO 1: LEITURA DO SENSOR (y_k) ---
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
	{
		adc_raw_y = HAL_ADC_GetValue(&hadc1);
		y_medido = adc_raw_y * (VREF_VOLTS / 4095.0f);
	}
	HAL_ADC_Stop(&hadc1);

	// --- PASSO 2: DEFINIÇÃO DA REFERÊNCIA (r_k) ---
	// Lógica para alternar a referência
	// Aprox. 200ms
	if (sample_count == 2000)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		r_k = 1.5f;
	}
	else if (sample_count >= 4000)
	{
		r_k = 1.0f;
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		sample_count = 0;
	}
	sample_count++;

	// --- PASSO 3: CÁLCULO DO CONTROLADOR ---

	// 1. Atualiza o estado do integrador com o erro da iteração ANTERIOR
	x_int += T_SAMPLE * erro;

	// 2. Calcula o erro de rastreamento ATUAL (será usado na próxima iteração)
	erro = r_k - y_medido;

	// 3. Atualiza os estados do observador com as derivadas da iteração ANTERIOR
	x1_obs += T_SAMPLE * dx1;
	x2_obs += T_SAMPLE * dx2;

	// 4. Calcula o erro do observador
	float y_obs = x1_obs;
	float obs_erro = y_medido - y_obs;

	// 5. Calcula o sinal de controle com os estados ESTIMADOS e o integrador
	g_control_u = -(Kc[0] * x1_obs + Kc[1] * x2_obs) + Ki * x_int;

	// 6. Armazena o controle atual para usar no cálculo das próximas derivadas
	u_prev = g_control_u;

	// 7. Calcula as derivadas dos estados para a PRÓXIMA iteração
	dx1 = A[0] * x1_obs + A[1] * x2_obs + B[0] * u_prev + Ke[0] * obs_erro;
	dx2 = A[2] * x1_obs + A[3] * x2_obs + B[1] * u_prev + Ke[1] * obs_erro;

	// --- PASSO 4: APLICAÇÃO DO SINAL DE CONTROLE ---
	float u_mapped = g_control_u;

	// 8. Saturação do sinal de controle
	if (u_mapped > VREF_VOLTS)
	{
		u_mapped = VREF_VOLTS;
	}
	if (u_mapped < 0.0f)
	{
		u_mapped = 0.0f;
	}

	// 9. Mapeia a tensão (0-3.3V) para o duty cycle do PWM (0-100)
	pwm_duty = (int32_t) ((u_mapped / VREF_VOLTS) * 100);

	// 10. Atualiza o duty cycle do PWM
	TIM3->CCR2 = pwm_duty;
}
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
	/* USER END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM3_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */

	// Inicializa as variáveis do controlador
	Control_Init();

	// Inicia o PWM e o Timer que dispara a interrupção de controle
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_ADC_Start(&hadc1); // Inicia o ADC

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		// O loop principal pode ser usado para tarefas de baixa prioridade.
		// A lógica de controle está na interrupção do TIM3.
		HAL_Delay(100);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

	ADC_ChannelConfTypeDef sConfig =
	{ 0 };

	/* USER CODE BEGIN ADC1_Init 1 */
	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */
	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */
	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig =
	{ 0 };
	TIM_MasterConfigTypeDef sMasterConfig =
	{ 0 };
	TIM_OC_InitTypeDef sConfigOC =
	{ 0 };

	/* USER CODE BEGIN TIM3_Init 1 */
	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 96 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 100 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0; // Inicia com 0% de duty cycle
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */
	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1 | GPIO_PIN_5 | GPIO_PIN_6,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : PA1 PA5 PA6 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
	while (1)
	{
		// Pisca um LED ou entra em loop infinito para indicar um erro fatal.
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
