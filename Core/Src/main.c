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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_BUFFER_SIZE 128
#define UART_BUFFER_SIZE 64

#define STEPS_PER_TURN 2048
#define SHUTTER_TURNS 1

#define DELAY_PASO 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* Definitions for lightSensorTask */
osThreadId_t lightSensorTaskHandle;
const osThreadAttr_t lightSensorTask_attributes = {
    .name = "lightSensorTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for motorTask */
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
    .name = "motorTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for serialReadTask */
osThreadId_t serialReadTaskHandle;
const osThreadAttr_t serialReadTask_attributes = {
    .name = "serialReadTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for vacationModeTas */
osThreadId_t vacationModeTasHandle;
const osThreadAttr_t vacationModeTas_attributes = {
    .name = "vacationModeTas",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for lightQueue */
osMessageQueueId_t lightQueueHandle;
const osMessageQueueAttr_t lightQueue_attributes = {
    .name = "lightQueue"};
/* Definitions for serialQueue */
osMessageQueueId_t serialQueueHandle;
const osMessageQueueAttr_t serialQueue_attributes = {
    .name = "serialQueue"};
/* Definitions for openingPercentage */
osMutexId_t openingPercentageHandle;
const osMutexAttr_t openingPercentage_attributes = {
    .name = "openingPercentage"};
/* Definitions for operationMode */
osMutexId_t operationModeHandle;
const osMutexAttr_t operationMode_attributes = {
    .name = "operationMode"};
/* USER CODE BEGIN PV */
char rxByte;
volatile write_index = 0;
uint8_t word_index;
uint8_t opening_percentage_1 = 0;
uint8_t opening_percentage_2 = 0;

enum OperationMode
{
    MODE_AUTO,
    MODE_MANUAL,
    MODE_VACA
};

enum OperationMode operationMode = MODE_AUTO;

char word_buffer[UART_BUFFER_SIZE];
char uart_rx_buffer[UART_BUFFER_SIZE];

const int openingLevels[4] = {0, 30, 60, 100};

int vacationSequence[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_USB_Init(void);
void StartLightSensorTask(void *argument);
void StartMotorTask(void *argument);
void StartSerialReadTask(void *argument);
void StartVacationModeTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  PrintLn prints a  printf-style formatted string to the USART1 console and adds a new line.
 * @param  format: Format string (printf-style)
 * @param  ...: Arguments to be formatted
 * @retval None
 */
void PrintLn(const char *format, ...)
{
    char buffer[DEBUG_BUFFER_SIZE];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, DEBUG_BUFFER_SIZE - 3, format, args);
    va_end(args);

    strcat(buffer, "\r\n");
    // Transmit the formatted string via USART1
    HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

uint8_t stepSequence[4] = {
    0b1000,
    0b0100,
    0b0010,
    0b0001,
};

static uint32_t motorStartTime = 0;
static uint32_t motorNextStepTime = 0;
static uint16_t motorStepIndex1 = 0;
static uint8_t motorStepIndex2 = 0;
static bool motorRunningFlag = false;

void setStepper1Output(uint16_t step)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, (step & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, (step & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (step & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (step & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void setStepper2Output(uint16_t step)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, (step & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, (step & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (step & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, (step & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void runStepperStartNonBlocking(void)
{
    motorStartTime = osKernelGetTickCount();
    motorNextStepTime = motorStartTime;
    motorStepIndex1 = 0;
    motorStepIndex2 = 0;
    motorRunningFlag = true;
}

void runStepUpdate1(int16_t direction)
{
    if (!motorRunningFlag)
        return;

    if (direction > 0)
    {
        motorStepIndex1 = motorStepIndex1 + 1;
    }
    else if (direction < 0)
    {
        motorStepIndex1 = motorStepIndex1 - 1;
    }

    uint32_t currentTick = osKernelGetTickCount();
    if (currentTick >= motorNextStepTime)
    {
        setStepper1Output(stepSequence[motorStepIndex1 % 4]);
    }
    motorNextStepTime = currentTick + 1;
}

void runStepUpdate2(int16_t direction)
{
    if (!motorRunningFlag)
        return;

    if (direction > 0)
        motorStepIndex2 = motorStepIndex2 + 1;
    else if (direction < 0)
        motorStepIndex2 = motorStepIndex2 - 1;

    uint32_t currentTick = osKernelGetTickCount();
    if (currentTick >= motorNextStepTime)
    {
        setStepper2Output(stepSequence[motorStepIndex2 % 4]);
    }
    motorNextStepTime = currentTick + 1;
}

void InitSerialConsole(void)
{
    // ANSI escape sequence to clear screen and move cursor to top left.
    // Note: This works only if your serial console supports ANSI codes.
    PrintLn("\033[2J\033[H");
    PrintLn("=== New Program Start ===");
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        osStatus_t status = osMessageQueuePut(serialQueueHandle, &rxByte, 0, 0);
        if (status != osOK)
        {
            PrintLn("Queue overflow");
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&rxByte, 1);
    }
}

void GenerateVacationSequence()
{
    for (int i = 0; i < 4; i++)
    {
        vacationSequence[i] = openingLevels[rand() % 4];
    }
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

    /* Configure the peripherals common clocks */
    PeriphCommonClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_DFSDM1_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_OCTOSPI1_Init();
    MX_SPI3_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_USB_OTG_FS_USB_Init();
    /* USER CODE BEGIN 2 */
    InitSerialConsole();
    if (HAL_UART_Receive_IT(&huart1, &rxByte, 1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
    {
        // Calibration Error
        PrintLn("ADC Calibration failed!");
    }
    else
    {
        PrintLn("ADC Calibration successful.");
    }
    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();
    /* Create the mutex(es) */
    /* creation of openingPercentage */
    openingPercentageHandle = osMutexNew(&openingPercentage_attributes);

    /* creation of operationMode */
    operationModeHandle = osMutexNew(&operationMode_attributes);

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* creation of lightQueue */
    lightQueueHandle = osMessageQueueNew(16, sizeof(uint16_t), &lightQueue_attributes);

    /* creation of serialQueue */
    serialQueueHandle = osMessageQueueNew(16, sizeof(char), &serialQueue_attributes);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of lightSensorTask */
    lightSensorTaskHandle = osThreadNew(StartLightSensorTask, NULL, &lightSensorTask_attributes);

    /* creation of motorTask */
    motorTaskHandle = osThreadNew(StartMotorTask, NULL, &motorTask_attributes);

    /* creation of serialReadTask */
    serialReadTaskHandle = osThreadNew(StartSerialReadTask, NULL, &serialReadTask_attributes);

    /* creation of vacationModeTas */
    vacationModeTasHandle = osThreadNew(StartVacationModeTask, NULL, &vacationModeTas_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
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
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure LSE Drive Capability
     */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 60;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }

    /** Enable MSI Auto calibration
     */
    HAL_RCCEx_EnableMSIPLLMode();
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the peripherals clock
     */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
    PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
    PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
    PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
    PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK | RCC_PLLSAI1_ADC1CLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief DFSDM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DFSDM1_Init(void)
{

    /* USER CODE BEGIN DFSDM1_Init 0 */

    /* USER CODE END DFSDM1_Init 0 */

    /* USER CODE BEGIN DFSDM1_Init 1 */

    /* USER CODE END DFSDM1_Init 1 */
    hdfsdm1_channel2.Instance = DFSDM1_Channel2;
    hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
    hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
    hdfsdm1_channel2.Init.OutputClock.Divider = 2;
    hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
    hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
    hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
    hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
    hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
    hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
    hdfsdm1_channel2.Init.Awd.Oversampling = 1;
    hdfsdm1_channel2.Init.Offset = 0;
    hdfsdm1_channel2.Init.RightBitShift = 0x00;
    if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN DFSDM1_Init 2 */

    /* USER CODE END DFSDM1_Init 2 */
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
    hi2c1.Init.Timing = 0x30A175AB;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

    /* USER CODE BEGIN I2C2_Init 0 */

    /* USER CODE END I2C2_Init 0 */

    /* USER CODE BEGIN I2C2_Init 1 */

    /* USER CODE END I2C2_Init 1 */
    hi2c2.Instance = I2C2;
    hi2c2.Init.Timing = 0x30A175AB;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C2_Init 2 */

    /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief OCTOSPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_OCTOSPI1_Init(void)
{

    /* USER CODE BEGIN OCTOSPI1_Init 0 */

    /* USER CODE END OCTOSPI1_Init 0 */

    OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

    /* USER CODE BEGIN OCTOSPI1_Init 1 */

    /* USER CODE END OCTOSPI1_Init 1 */
    /* OCTOSPI1 parameter configuration*/
    hospi1.Instance = OCTOSPI1;
    hospi1.Init.FifoThreshold = 1;
    hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
    hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
    hospi1.Init.DeviceSize = 32;
    hospi1.Init.ChipSelectHighTime = 1;
    hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
    hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
    hospi1.Init.ClockPrescaler = 1;
    hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
    hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
    hospi1.Init.ChipSelectBoundary = 0;
    hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
    if (HAL_OSPI_Init(&hospi1) != HAL_OK)
    {
        Error_Handler();
    }
    OSPIM_Cfg_Struct.ClkPort = 1;
    OSPIM_Cfg_Struct.NCSPort = 1;
    OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
    if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN OCTOSPI1_Init 2 */

    /* USER CODE END OCTOSPI1_Init 2 */
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

    /* USER CODE BEGIN SPI3_Init 0 */

    /* USER CODE END SPI3_Init 0 */

    /* USER CODE BEGIN SPI3_Init 1 */

    /* USER CODE END SPI3_Init 1 */
    /* SPI3 parameter configuration*/
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 7;
    hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI3_Init 2 */

    /* USER CODE END SPI3_Init 2 */
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
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
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
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_USB_Init(void)
{

    /* USER CODE BEGIN USB_OTG_FS_Init 0 */

    /* USER CODE END USB_OTG_FS_Init 0 */

    /* USER CODE BEGIN USB_OTG_FS_Init 1 */

    /* USER CODE END USB_OTG_FS_Init 1 */
    /* USER CODE BEGIN USB_OTG_FS_Init 2 */

    /* USER CODE END USB_OTG_FS_Init 2 */
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
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, ST25DV04K_RF_DISABLE_Pin | ISM43362_RST_Pin | ISM43362_SPI3_CSN_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin | ARD_D4_Pin | ARD_D7_Pin | GPIO_PIN_6 | GPIO_PIN_7 | SPBTLE_RF_RST_Pin | ARD_D9_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 | ARD_D8_Pin | ISM43362_BOOT0_Pin | ISM43362_WAKEUP_Pin | LED2_Pin | SPSGRF_915_SDN_Pin | ARD_D5_Pin | SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, SPBTLE_RF_SPI3_CSN_Pin | PMOD_RESET_Pin | PMOD_SPI2_SCK_Pin | STSAFE_A110_RESET_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin | LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : ST25DV04K_RF_DISABLE_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
    GPIO_InitStruct.Pin = ST25DV04K_RF_DISABLE_Pin | ISM43362_RST_Pin | ISM43362_SPI3_CSN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin ST25DV04K_GPO_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin
                             ISM43362_DRDY_EXTI1_Pin */
    GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin | ST25DV04K_GPO_Pin | SPSGRF_915_GPIO3_EXTI5_Pin | SPBTLE_RF_IRQ_EXTI6_Pin | ISM43362_DRDY_EXTI1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : BUTTON_EXTI13_Pin VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
    GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin | VL53L0X_GPIO1_EXTI7_Pin | LSM3MDL_DRDY_EXTI8_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : ARD_D0_Pin */
    GPIO_InitStruct.Pin = ARD_D0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(ARD_D0_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : ARD_D10_Pin ARD_D4_Pin ARD_D7_Pin PA6
                             PA7 SPBTLE_RF_RST_Pin ARD_D9_Pin */
    GPIO_InitStruct.Pin = ARD_D10_Pin | ARD_D4_Pin | ARD_D7_Pin | GPIO_PIN_6 | GPIO_PIN_7 | SPBTLE_RF_RST_Pin | ARD_D9_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : ARD_D13_Pin */
    GPIO_InitStruct.Pin = ARD_D13_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(ARD_D13_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : ARD_D3_Pin */
    GPIO_InitStruct.Pin = ARD_D3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PB1 ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin
                             LED2_Pin SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | ARD_D8_Pin | ISM43362_BOOT0_Pin | ISM43362_WAKEUP_Pin | LED2_Pin | SPSGRF_915_SDN_Pin | ARD_D5_Pin | SPSGRF_915_SPI3_CSN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI10_Pin LSM6DSL_INT1_EXTI11_Pin USB_OTG_FS_PWR_EN_Pin ARD_D2_Pin
                             HTS221_DRDY_EXTI15_Pin PMOD_IRQ_EXTI2_Pin */
    GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI10_Pin | LSM6DSL_INT1_EXTI11_Pin | USB_OTG_FS_PWR_EN_Pin | ARD_D2_Pin | HTS221_DRDY_EXTI15_Pin | PMOD_IRQ_EXTI2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin PMOD_SPI2_SCK_Pin STSAFE_A110_RESET_Pin */
    GPIO_InitStruct.Pin = SPBTLE_RF_SPI3_CSN_Pin | PMOD_RESET_Pin | PMOD_SPI2_SCK_Pin | STSAFE_A110_RESET_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
    GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin | LED3_WIFI__LED4_BLE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : USB_OTG_FS_VBUS_Pin */
    GPIO_InitStruct.Pin = USB_OTG_FS_VBUS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USB_OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : USB_OTG_FS_ID_Pin USB_OTG_FS_DM_Pin USB_OTG_FS_DP_Pin */
    GPIO_InitStruct.Pin = USB_OTG_FS_ID_Pin | USB_OTG_FS_DM_Pin | USB_OTG_FS_DP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLightSensorTask */
/**
 * @brief  Function implementing the lightSensorTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLightSensorTask */
void StartLightSensorTask(void *argument)
{
    /* USER CODE BEGIN 5 */
    uint16_t lightPercentage;
    uint8_t counter = 0;
    uint8_t next_opening_percentage;
    /* Infinite loop */
    for (;;)
    {
        if (osMutexAcquire(operationModeHandle, 10) == osOK)
        {
            if (operationMode == MODE_AUTO)
            {
                HAL_ADC_Start(&hadc1);
                if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
                {
                    uint16_t currentReading = HAL_ADC_GetValue(&hadc1);

                    lightPercentage = currentReading * 100 / 4096;
                };

                HAL_ADC_Stop(&hadc1);

                if (lightPercentage > 90)
                {
                    if (next_opening_percentage == 50)
                        counter++;
                    else
                    {
                        next_opening_percentage = 50;
                        counter = 1;
                    }
                }
                else if (lightPercentage > 35)
                {
                    if (next_opening_percentage == 100)
                        counter++;
                    else
                    {
                        next_opening_percentage = 100;
                        counter = 1;
                    }
                }
                else
                {
                    if (next_opening_percentage == 0)
                        counter++;
                    else
                    {
                        next_opening_percentage = 0;
                        counter = 1;
                    }
                }
                if (counter == 10)
                {
                    if (osMutexAcquire(openingPercentageHandle, 1) == osOK)
                    {
                        PrintLn("modify opening percentage light task");
                        opening_percentage_1 = next_opening_percentage;
                        opening_percentage_2 = next_opening_percentage;
                        osMutexRelease(openingPercentageHandle);
                    }
                }
            }
        }
        osMutexRelease(operationModeHandle);
        HAL_Delay(500);
    }
    /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
 * @brief Function implementing the motorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
    /* USER CODE BEGIN StartMotorTask */
    bool motorRunning = false;
    runStepperStartNonBlocking();

    for (;;)
    {
        if (osMutexAcquire(operationModeHandle, 20) == osOK)
        {
            if (operationMode != MODE_VACA)
            {
                osMutexRelease(operationModeHandle);
                if (osMutexAcquire(openingPercentageHandle, 10) == osOK)
                {
                    uint16_t objective_steps = (opening_percentage_1 * STEPS_PER_TURN) / 100;
                    osMutexRelease(openingPercentageHandle);
                    int16_t difference_steps = objective_steps - motorStepIndex1;
                    if (difference_steps != 0)
                    {
                        while (motorStepIndex1 != objective_steps)
                        {
                            if (osMutexAcquire(operationModeHandle, 5) == osOK)
                            {
                                if (operationMode == MODE_VACA)
                                {
                                    osMutexRelease(operationModeHandle);
                                    break;
                                }
                                osMutexRelease(operationModeHandle);
                            }
                            runStepUpdate1(difference_steps);
                            osDelay(10);
                        }
                    }
                }
                else
                {
                    PrintLn("Motor: No new sensor value received");
                }
            }
            else
                osMutexRelease(operationModeHandle);
        }

        osDelay(50);
    }
    /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartSerialReadTask */
/**
 * @brief Function implementing the serialReadTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSerialReadTask */
void StartSerialReadTask(void *argument)
{
    /* USER CODE BEGIN StartSerialReadTask */
    PrintLn("Start serial read task");
    char received;
    char word_buffer[UART_BUFFER_SIZE];
    uint8_t word_index = 0;
    uint8_t selected_shutter = 0;
    /* Infinite loop */
    for (;;)
    {
        if (osMessageQueueGet(serialQueueHandle, &received, NULL, 5) == osOK)
        {
            PrintLn("Received: %c", received);
            if (received == '\n' || received == ';')
            {
                if (osMutexAcquire(operationModeHandle, 5) == osOK)
                {
                    word_buffer[word_index] = '\0';
                    word_index = 0;
                    PrintLn("Word buffer %s", word_buffer);

                    if (word_buffer[0] == 'M' && (word_buffer[1] == '1' || word_buffer[1] == '2'))
                    {
                        operationMode = MODE_MANUAL;
                        PrintLn("mode manual");
                        PrintLn("%s", word_buffer);
                        int shutter = word_buffer[1] - '0';
                        if (shutter == 1)
                            opening_percentage_1 = atoi(&word_buffer[2]);
                        if (shutter == 2)
                            opening_percentage_2 = atoi(&word_buffer[2]);
                    }
                    if (word_buffer[0] == 'A')
                        operationMode = MODE_AUTO;
                    if (word_buffer[0] == 'V')
                        operationMode = MODE_VACA;
                    osMutexRelease(operationModeHandle);
                }
            }
            else
            {
                if (word_index < UART_BUFFER_SIZE - 1)
                {
                    word_buffer[word_index++] = received;
                }
            }
        }
        osDelay(500);
    }
    /* USER CODE END StartSerialReadTask */
}

/* USER CODE BEGIN Header_StartVacationModeTask */
/**
 * @brief Function implementing the vacationModeTas thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartVacationModeTask */
void StartVacationModeTask(void *argument)
{
    /* USER CODE BEGIN StartVacationModeTask */
    /* Infinite loop */
    for (;;)
    {
        if (osMutexAcquire(operationModeHandle, 10) == osOK)
        {
            if (operationMode == MODE_VACA)
            {
                osMutexRelease(operationModeHandle);
                PrintLn("Modo vaca");
                GenerateVacationSequence();
                PrintLn("First step: %d, Second step: %d, Third step: %d, Fourth step: %d", vacationSequence[0], vacationSequence[1], vacationSequence[2], vacationSequence[3]);

                for (int i = 0; i < 4; i++)
                {

                    if (osMutexAcquire(openingPercentageHandle, 5) == osOK)
                    {
                        opening_percentage_1 = vacationSequence[i];

                        uint16_t objective_steps = (opening_percentage_1 * STEPS_PER_TURN) / 100;
                        osMutexRelease(openingPercentageHandle);
                        int16_t difference_steps = objective_steps - motorStepIndex1;
                        if (difference_steps != 0)
                        {
                            while (motorStepIndex1 != objective_steps)
                            {
                                if (osMutexAcquire(operationModeHandle, 5) == osOK)
                                {
                                    if (operationMode != MODE_VACA)
                                    {
                                        osMutexRelease(operationModeHandle);
                                        break;
                                    }
                                    osMutexRelease(operationModeHandle);
                                }

                                runStepUpdate1(difference_steps);
                                osDelay(10);
                            }
                        }
                        osDelay(3000);
                    }
                }
            }
            else
                osMutexRelease(operationModeHandle);
        }
        osDelay(50);
    }
    /* USER CODE END StartVacationModeTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

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
