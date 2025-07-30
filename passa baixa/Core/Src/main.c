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
// Define os limites do sinal de controle (PWM duty cycle)
#define PWM_MIN 	31
#define PWM_MAX 	46
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
// --- COEFICIENTES E ESTADOS DO CONTROLADOR ---
// Matrizes
const float R1 = 46000.0f;
const float R2 = 18000.0f;
const float C1 = 100e-9f;
const float C2 = 680e-9f;

const float A[4] =
{ 0.0f, 1e7f, -1.7761e-3f, -1.1367e2f };

const float B[2] =
{ 0.0f, 1.7761e-3f };

const float Ke[2] =
{ 1.4719e2f, 2.9176e-3f };

const float Ki = 9045.3275f; // valor do integrador, calculado no MATLAB

const float Kc[2] =
{ 39.6464f, 1503575.2585f };

static float erro_inst = 0.0f;

// --- VARI�?VEIS GLOBAIS DO CONTROLADOR ---
// 'volatile' para garantir que o compilador não otimize o acesso entre main e ISR
volatile float g_reference_r = 1.0f;
volatile float g_output_y = 0.0f;
volatile float g_control_u = 0.0f;
volatile float x_integrador = 0.0f;

// Período de amostragem em segundos (3ms = 0.003s)
const float T_SAMPLE = 0.0001f; //

// Variáveis de estado estimadas pelo observador
static float x1_obs_k = 0.0f;  // vC1 estimado no instante k
static float x2_obs_k = 0.0f;  // iC1 estimado no instante k

// Variáveis de estado estimadas no instante anterior
static float x1_obs_k_1 = 0.0f;
static float x2_obs_k_1 = 0.0f;

// Derivadas dos estados estimados (usadas na integração Euler)
static float x1_obs_pt = 0.0f;
static float x2_obs_pt = 0.0f;
static float x1_pt;
static float x2_pt;

// Saída estimada do observador
static float y_obs_k = 0.0f;
float erro_obs;
float zeta;
float zeta_pt;

// Valor da entrada (u) da planta no instante anterior (para o observador)
static float u_k_1 = 0.0f;

// Variáveis para leitura dos canais do ADC1
uint16_t adc_raw_y;

float y_medido = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void Control_Init(void);
//void Control_Update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t sample_count = 0;
static uint32_t control_counter = 0;
float r_k = 1.0f;

int32_t pwm_duty;

/**
 * @brief Inicializa todas as variáveis de estado do controlador.
 */
void Control_Init(void)
{
	// Variáveis de saída e controle (para garantir estado inicial conhecido)
	g_reference_r = 0.0f;
	g_output_y = 0.0f;
	g_control_u = 0.0f;
	u_k_1 = 0.0f; // Inicializa a ação de controle anterior
	zeta = 0.0f;
	zeta_pt = 0.0f;

	// Variáveis de estado estimadas do observador
	x1_pt = 0.0f;
	x2_pt = 0.0f;
	x1_obs_k = 0.0f;
	x2_obs_k = 0.0f;
	x1_obs_k_1 = 0.0f;
	x2_obs_k_1 = 0.0f;

	// Derivadas dos estados estimados
	x1_obs_pt = 0.0f;
	x2_obs_pt = B[1];

	// Saída estimada do observador
	y_obs_k = 0.0f;

	// Reinicia o contador de amostragem
	control_counter = 0;
	sample_count = 0;
	r_k = 1.0f; // Valor inicial da referência
}

/**
 * @brief  Callback da interrupção do Timer. Ocorre a cada 100us (10kHz).
 * @note   Esta função agora contém toda a lógica de controle em tempo real.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance != TIM3)
		return;

	// --- PASSO 1: LEITURA DO SENSOR (ENTRADA y_k) ---
	control_counter++;
	if (control_counter < 30)
	{ // Executa o controle a cada 3ms (333 Hz), próximo do projeto.
		return;
	}
	control_counter = 0; // Reseta o contador

	// Converte o valor do ADC (0-4095) para Volts (0-3.3V)
	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	{
		adc_raw_y = HAL_ADC_GetValue(&hadc1); // canal 0
		y_medido = adc_raw_y * (3.3f / 4095.0f); // Vc1 = y
	}
	HAL_ADC_Stop(&hadc1);

	// --- PASSO 2: DEFINIÇÃO DA REFERÊNCIA (ENTRADA r_k) ---

	if (sample_count == 33) // aprox. 100ms
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); // Atualiza pino A5 para HIGH (para trigger do osciloscópio
		r_k = 1.5f;
	}
	else if (sample_count >= 66)
	{
		r_k = 1.0f;
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		sample_count = 0;
	}
	sample_count++;

	// --- PASSO 3: C�?LCULO DO CONTROLADOR ---
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6); // pino para calcular tempo de execução dos cálculos

	zeta += T_SAMPLE * zeta_pt;

	x1_obs_k += T_SAMPLE * x1_pt;
	x2_obs_k += T_SAMPLE * x2_pt;

	y_obs_k = x1_obs_k;

	x1_pt = A[0] * x1_obs_k + A[1] * x2_obs_k + B[0] * g_control_u;
	x2_pt = A[2] * x1_obs_k + A[3] * x2_obs_k + B[1] * g_control_u;

	erro_inst = y_medido - y_obs_k;

	g_control_u = -(Kc[0] * x1_obs_k + Kc[1] * x2_obs_k) + Ki * zeta;

	x1_obs_pt = A[0] * x1_obs_k + A[1] * x2_obs_k + B[0] * g_control_u + Ke[0] * erro_inst;
	x2_obs_pt = A[2] * x1_obs_k + A[3] * x2_obs_k + B[1] * g_control_u + Ke[1] * erro_inst;


	zeta_pt = r_k - y_obs_k;


	u_k_1 = g_control_u;



	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6); // Finaliza medição de tempo de execução

	float u_mapped = g_control_u;
	// Saturação (se u_k puder exceder 3.3V ou ser menor que 0V)
	if (u_mapped >= 3.3f)
	{
		u_mapped = 3.3f; // Limite superior da tensão de entrada da planta
	}
	if (u_mapped <= 0.0f)
	{
		u_mapped = 0.0f; // Limite inferior (se a planta só aceita tensão positiva)
	}

	pwm_duty = (int32_t) ((u_mapped / 3.3f) * (100)); // Mapeia para o período do PWM (ARR)

	// Atualiza o duty cycle do PWM
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
	/* USER CODE END Init */

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

	// Inicia a conversão contínua do ADC

	// Inicia o PWM com interrupção. A interrupção irá guiar todo o controle.
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_ADC_Start(&hadc1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		//HAL_ADC_PollForConversion(&hadc1, 1);

		// O loop principal agora é usado para tarefas de baixa prioridade, como a comunicação.
		/*		if (send_data_flag)
		 {
		 send_data_flag = 0; // Reseta a flag

		 // Envia os dados do controlador via UART para depuração/plotagem
		 sprintf(msg, "R:%.2f, Y:%.2f, U:%.2f\r\n", g_reference_r, g_output_y, g_control_u);
		 HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
		 }
		 */
		// Pequeno delay para não sobrecarregar o processador com o loop while(1)
		//	HAL_Delay(10);
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
	sConfigOC.Pulse = 31;
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
  *         where the assert_param error has occurred.
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
