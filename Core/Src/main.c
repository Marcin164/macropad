/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_KEYS 6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;

const uint8_t matrix[5][5] = {
    {0x29, 0x1E, 0x1F, 0x20, 0x21}, // ESC, 1, 2, 3, 4
    {0x2B, 0x14, 0x1A, 0x08, 0x15}, // Tab, Q, W, E, R
    {0x39, 0x04, 0x16, 0x07, 0x09}, // Caps Lock, A, S, D, F
    {0xE1, 0x1D, 0x1B, 0x06, 0x19}, // Left Shift, Z, X, C, V
    {0xE0, 0xE3, 0xE2, 0x2C, 0x00}  // Left Ctrl, Left GUI, Left Alt, Space, None
};

const uint16_t inputPins[5] = {ROW1_Pin, ROW2_Pin, ROW3_Pin, ROW4_Pin, ROW5_Pin};
const uint16_t outputPins[5] = {COL1_Pin, COL2_Pin, COL3_Pin, COL4_Pin, COL5_Pin};
uint8_t prevKeyPressed[5][5] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  void Keypad_Scan_And_Send(void);
  void Send_Key_To_USB(uint8_t *keycodes, uint8_t numKeys);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  while (1) {
        Keypad_Scan_And_Send();
        HAL_Delay(50); // Debounce delay
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, COL1_Pin|COL2_Pin|COL3_Pin|COL4_Pin
                          |COL5_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : ROW1_Pin ROW2_Pin ROW3_Pin ROW4_Pin
                           ROW5_Pin */
  GPIO_InitStruct.Pin = ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin
                          |ROW5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP ;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : COL1_Pin COL2_Pin COL3_Pin COL4_Pin
                           COL5_Pin */
  GPIO_InitStruct.Pin = COL1_Pin|COL2_Pin|COL3_Pin|COL4_Pin
                          |COL5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Send_Key_To_USB(uint8_t *keycodes, uint8_t numKeys) {
    uint8_t HID_Buffer[8] = {0};

    // Wypełnij bufor klawiszami, ale nie więcej niż MAX_KEYS
    for (uint8_t i = 0; i < numKeys && i < MAX_KEYS; i++) {
        HID_Buffer[2 + i] = keycodes[i];
    }

    // Wyślij raport HID
    USBD_HID_SendReport(&hUsbDeviceFS, HID_Buffer, sizeof(HID_Buffer));
    HAL_Delay(10);
}

void Keypad_Scan_And_Send(void) {
    uint8_t activeKeys[MAX_KEYS] = {0}; // Tablica na aktywne klawisze
    uint8_t keyCount = 0;              // Licznik wciśniętych klawiszy
    static uint8_t lastActiveKeys[MAX_KEYS] = {0}; // Poprzedni stan klawiszy
    static uint8_t lastKeyCount = 0;   // Liczba klawiszy w poprzednim raporcie
    uint8_t isStateChanged = 0;        // Flaga zmiany stanu

    for (int col = 0; col < 5; col++) {
        HAL_GPIO_WritePin(GPIOB, outputPins[col], GPIO_PIN_RESET);
        HAL_Delay(1);

        for (int row = 0; row < 5; row++) {
            GPIO_PinState pinState = HAL_GPIO_ReadPin(GPIOA, inputPins[row]);

            if (pinState == GPIO_PIN_RESET) { // Klawisz wciśnięty
                if (prevKeyPressed[row][col] == 0) {
                    HAL_Delay(10); // Debouncing
                    if (HAL_GPIO_ReadPin(GPIOA, inputPins[row]) == GPIO_PIN_RESET) {
                        prevKeyPressed[row][col] = 1;

                        if (keyCount < MAX_KEYS) {
                            activeKeys[keyCount++] = matrix[row][col];
                        }
                        isStateChanged = 1; // Zmieniono stan
                    }
                } else { // Klawisz przytrzymany
                    if (keyCount < MAX_KEYS) {
                        activeKeys[keyCount++] = matrix[row][col];
                    }
                }
            } else { // Klawisz zwolniony
                if (prevKeyPressed[row][col] == 1) {
                    prevKeyPressed[row][col] = 0;
                    isStateChanged = 1; // Zmieniono stan
                }
            }
        }

        HAL_GPIO_WritePin(GPIOB, outputPins[col], GPIO_PIN_SET);
    }

    // Porównaj aktualny stan z poprzednim
    if (keyCount != lastKeyCount || isStateChanged) {
        Send_Key_To_USB(activeKeys, keyCount);
        memcpy(lastActiveKeys, activeKeys, sizeof(activeKeys));
        lastKeyCount = keyCount;
    } else {
        // Jeśli przytrzymanie, wyślij ponownie ostatni raport
        Send_Key_To_USB(lastActiveKeys, lastKeyCount);
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
