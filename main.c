/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Определение адресов датчиков
#define BMP085_ADDRESS   0x77 << 1  // Адрес на шине I2C Барометра
#define ITG3205_ADDRESS  0x68 << 1  // Адрес на шине I2C Гироскоп
#define HMC5883L_ADDRESS 0x1E << 1  // Адрес на шине I2C Магнитометр
#define BMA180_ADDRESS   0x38 << 1  // Адрес на шине I2C Акселерометр
#define Buffer_size 128             // Константа размера буфера
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;


/* USER CODE BEGIN PV */
// Значения для калибровки гироскопа
uint16_t g_offx = 0;
uint16_t g_offy = -56;
uint16_t g_offz = 0;
// Значения для магнитрона
float m_scale;
int m_error = 0;
// Значения для калибровки акселерометра
int16_t a_offx = 0;
int16_t a_offy = 0;
int16_t a_offz = 0;
uint8_t a_resolution = 0x04;
float a_scale;
// Буфер для UART
char uart_buffer[Buffer_size];      // Буфер для UART(используется датчиками)
// Переменные для модема
char modem_rx_byte;                 // Единичный байт сообщения модема
char rx_modem_buffer[Buffer_size];  // Буфер для сохранения всего сообщения модема
uint16_t modem_index = 0;           // Счетчик для модема 
uint8_t modem_ready = 0;            // Флаг готовности сообщения модема
// Переменные для пк
char pc_rx_byte;                    // Единичный байт сообщения пк
char rx_pc_buffer[Buffer_size];     // Буфер для сохранения всего сообщения пк
uint16_t pc_index = 0;              // Счетчик для пк 
uint8_t pc_ready = 0;               // Флаг готовности сообщения пк
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Инициализация BMP085 Температура и давление
void BMP085_Init(void) {}
// Функция для получения температуры 
float BMP085_GetTemperature(void) {
    uint8_t data[2];
    long temp;

    // Начало измерения температуры
    uint8_t cmd = 0x2E;                                                          // Команда для измерения температуры
    HAL_I2C_Mem_Write(&hi2c1, BMP085_ADDRESS, 0xF4, 1, &cmd, 1, HAL_MAX_DELAY);  // Чтение по адресу через шину I2C
    HAL_Delay(5);                                                                // Ждем, пока датчик закончит измерение

    // Чтение данных температуры
    HAL_I2C_Mem_Read(&hi2c1, BMP085_ADDRESS, 0xF6, 1, data, 2, HAL_MAX_DELAY);   // Чтение по адресу через шину I2C
    temp = (data[0] << 8) | data[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP085_ADDRESS, 0xB2, 1, data, 2, HAL_MAX_DELAY);   // Чтение по адресу через шину I2C
    unsigned short ac5 = (data[0] << 8) | data[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP085_ADDRESS, 0xB4, 1, data, 2, HAL_MAX_DELAY);   // Чтение по адресу через шину I2C
    unsigned short ac6 = (data[0] << 8) | data[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP085_ADDRESS, 0xBC, 1, data, 2, HAL_MAX_DELAY);   // Чтение по адресу через шину I2C
    short MC = (data[0] << 8) | data[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP085_ADDRESS, 0xBE, 1, data, 2, HAL_MAX_DELAY);   // Чтение по адресу через шину I2C
    short MD = (data[0] << 8) | data[1];
    // Формулы для расчета температуры
    long x1, x2;
    x1 = ((temp - ac6) * ac5) >> 15;
    x2 = (MC << 11) / (x1 + MD);
    long b5 = x1 + x2;
    long t = (b5 + 8) >> 4;
    return t / 10;                                                               // Возвращаем температуру в градусах Цельсия
}
// Функция для получения давления
float BMP085_GetPressure(void) {
    uint8_t data[3];
    long pres;
    long temp;

    // Начало измерения температуры
    uint8_t cmd = 0x2E;                                                          // Команда для измерения температуры
    HAL_I2C_Mem_Write(&hi2c1, BMP085_ADDRESS, 0xF4, 1, &cmd, 1, HAL_MAX_DELAY);  // Чтение по адресу через шину I2C
    HAL_Delay(5);                                                                // Ждем, пока датчик закончит измерение

    // Чтение данных температуры
    HAL_I2C_Mem_Read(&hi2c1, BMP085_ADDRESS, 0xF6, 1, data, 2, HAL_MAX_DELAY);   // Чтение по адресу через шину I2C
    temp = (data[0] << 8) | data[1];

    // Начало измерения давления
    short oss = 0;
    uint8_t cmd1 = 0x34 + (oss << 6);                                            // Команда для измерения давления
    HAL_I2C_Mem_Write(&hi2c1, BMP085_ADDRESS, 0xF4, 1, &cmd1, 1, HAL_MAX_DELAY); // Чтение по адресу через шину I2C
    HAL_Delay(5);                                                                // Ждем, пока датчик закончит измерение

    // Чтение данных давления (3 байта)
    HAL_I2C_Mem_Read(&hi2c1, BMP085_ADDRESS, 0xF6, 1, data, 3, HAL_MAX_DELAY);   // Чтение по адресу через шину I2C
    pres = ((data[0] << 16) | (data[1] << 8) | data[2]) >> (8 - oss);

    // Чтение калибровочных коэффициентов
    HAL_I2C_Mem_Read(&hi2c1, BMP085_ADDRESS, 0xAA, 1, data, 2, HAL_MAX_DELAY);   // Чтение по адресу через шину I2C
    short ac1 = (data[0] << 8) | data[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP085_ADDRESS, 0xAC, 1, data, 2, HAL_MAX_DELAY);   // Чтение по адресу через шину I2C
    short ac2 = (data[0] << 8) | data[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP085_ADDRESS, 0xAE, 1, data, 2, HAL_MAX_DELAY);   // Чтение по адресу через шину I2C
    short ac3 = (data[0] << 8) | data[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP085_ADDRESS, 0xB0, 1, data, 2, HAL_MAX_DELAY);   // Чтение по адресу через шину I2C
    unsigned short ac4 = (data[0] << 8) | data[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP085_ADDRESS, 0xB2, 1, data, 2, HAL_MAX_DELAY);   // Чтение по адресу через шину I2C
    unsigned short ac5 = (data[0] << 8) | data[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP085_ADDRESS, 0xB4, 1, data, 2, HAL_MAX_DELAY);   // Чтение по адресу через шину I2C
    unsigned short ac6 = (data[0] << 8) | data[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP085_ADDRESS, 0xB6, 1, data, 2, HAL_MAX_DELAY);   // Чтение по адресу через шину I2C
    short b1 = (data[0] << 8) | data[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP085_ADDRESS, 0xB8, 1, data, 2, HAL_MAX_DELAY);   // Чтение по адресу через шину I2C
    short b2 = (data[0] << 8) | data[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP085_ADDRESS, 0xBC, 1, data, 2, HAL_MAX_DELAY);   // Чтение по адресу через шину I2C
    short MC = (data[0] << 8) | data[1];
    HAL_I2C_Mem_Read(&hi2c1, BMP085_ADDRESS, 0xBE, 1, data, 2, HAL_MAX_DELAY);   // Чтение по адресу через шину I2C
    short MD = (data[0] << 8) | data[1];

    // Расчет температуры и давления
    long x1, x2, x3, b3, b5, b6, t, p;
    unsigned long b4, b7;

    x1 = ((temp - ac6) * ac5) >> 15;
    x2 = (MC << 11) / (x1 + MD);
    b5 = x1 + x2;
    t = (b5 + 8) >> 4;

    b6 = b5 - 4000;
    x1 = (b2 * (b6 * b6 >> 12)) >> 11;
    x2 = (ac2 * b6) >> 11;
    x3 = x1 + x2;
    b3 = ((((long)ac1 * 4 + x3) << oss) + 2) >> 2;

    x1 = (ac3 * b6) >> 13;
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15;

    b7 = (unsigned long)(pres - b3) * (50000 >> oss);
    if (b7 < 0x80000000) {
        p = (b7 << 1) / b4;
    } else {
        p = (b7 / b4) << 1;
    }

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p += (x1 + x2 + 3791) >> 4;

    return p;
}
// Инициализация ITG3205 Гироскоп
void ITG3205_Init(void) {
  uint8_t data = 0x00; // Вывод из спящего режима
  HAL_I2C_Mem_Write(&hi2c1, ITG3205_ADDRESS, 0x3E, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY); // Запись по адресу через шину I2C
  data = 0x07; // Переменная для частоты дескритезации
  HAL_I2C_Mem_Write(&hi2c1, ITG3205_ADDRESS, 0x15, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY); // Запись по адресу через шину I2C
  data = 0x1E; // 0001 1110
  HAL_I2C_Mem_Write(&hi2c1, ITG3205_ADDRESS, 0x16, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY); // Запись по адресу через шину I2C
  data = 0x00; //  Выключения прерываний
  HAL_I2C_Mem_Write(&hi2c1, ITG3205_ADDRESS, 0x17, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY); // Запись по адресу через шину I2C
}
// Функция для получения углового ускорения
void  ITGGetGyroscopeData(int16_t * result){
  uint8_t buff[8];
  HAL_I2C_Mem_Read(&hi2c1, ITG3205_ADDRESS, 0x1B, I2C_MEMADD_SIZE_8BIT, buff, 8, HAL_MAX_DELAY); // Чтение по адресу через шину I2C
  result[0] = (buff[2]<<8 | buff[3]) + g_offx;
  result[1] = ((buff[4]<<8) | buff[5]) + g_offy;
  result[2] = ((buff[6]<<8) | buff[7]) + g_offz;
  result[3] = (buff[0]<<8) | buff[1];
}
// Функция для настройки режима работы магнетометра
void HMC5883LSetMeasurementMode(uint8_t mode);
// Функция для настройки масштаба
void HMC5883LSetScale(float gauss);
// Инициализация HMC5883L магнетометр
void HMC5883L_Init(float scale, uint16_t mode) {
    m_scale = 1;
    HMC5883LSetScale(scale);
    HMC5883LSetMeasurementMode(mode);
}
void HMC5883LSetScale(float gauss){
   uint8_t regValue = 0x00;
    if(gauss == 0.88){
        regValue = 0x00;
        m_scale = 0.73;
    }else if(gauss == 1.3){
        regValue = 0x01;
        m_scale = 0.92;
    }else if(gauss == 1.9){
        regValue = 0x02;
        m_scale = 1.22;
    }else if(gauss == 2.5){
        regValue = 0x03;
        m_scale = 1.52;
    }else if(gauss == 4.0){
        regValue = 0x04;
        m_scale = 2.27;
    }else if(gauss == 4.7){
        regValue = 0x05;
        m_scale = 2.56;
    }else if(gauss == 5.6){
        regValue = 0x06;
        m_scale = 3.03;
    }else if(gauss == 8.1){
        regValue = 0x07;
        m_scale = 4.35;
    }
    regValue = regValue << 5;
    HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDRESS, 0x01,I2C_MEMADD_SIZE_8BIT, &regValue, 1, HAL_MAX_DELAY);
}
void HMC5883LSetMeasurementMode(uint8_t mode){
  HAL_I2C_Mem_Write(&hi2c1, HMC5883L_ADDRESS, 0x02,I2C_MEMADD_SIZE_8BIT, &mode, 1, HAL_MAX_DELAY); // Запись по адресу через шину I2C
}
void HMC5883LGetMagnetometerData(int16_t *result){
  uint8_t buff[6];
  HAL_I2C_Mem_Read(&hi2c1,HMC5883L_ADDRESS,0x03,I2C_MEMADD_SIZE_8BIT,buff,6,HAL_MAX_DELAY); // Чтение по адресу через шину I2C
  result[0] = (buff[0]<<8 | buff[1]);
  result[1] = (buff[2]<<8 | buff[3]);
  result[2] = (buff[4]<<8 | buff[5]);
}
// Установка разрешения акселерометра
void bma180SetResolution(int gValue);
// Получение данных акселерометра
void bma180GetAccelerometerData(int16_t *result);
// Инициализация BMA180
void BMA180_Init(int resolution) {
    uint8_t temp[1];
    uint8_t temp1;
    uint8_t rst = 0xB6;   // 1100 0110
    uint8_t start = 0x10; // 0001 0000
    

    // Сброс устройства
    //HAL_I2C_Mem_Write(&hi2c1, BMA180_ADDRESS, 0x10, I2C_MEMADD_SIZE_8BIT, &rst, 1, HAL_MAX_DELAY);     // Запись по адресу через шину I2C    
    HAL_Delay(10);
    // Выход из режима сна
    HAL_I2C_Mem_Write(&hi2c1, BMA180_ADDRESS, 0x0D, I2C_MEMADD_SIZE_8BIT, &start, 1, HAL_MAX_DELAY);   // Запись по адресу через шину I2C
    // Установка разрешения
    bma180SetResolution(resolution);
    // Получение текущего значения низкочастотного фильтра
    HAL_I2C_Mem_Read(&hi2c1, BMA180_ADDRESS, 0x20, I2C_MEMADD_SIZE_8BIT, temp, 1, HAL_MAX_DELAY);      // Чтение по адресу через шину I2C
    temp1 = temp[0] & 0x0F;
    
    // Запись нового значения фильтра
    HAL_I2C_Mem_Write(&hi2c1, BMA180_ADDRESS, 0x20, I2C_MEMADD_SIZE_8BIT, &temp1, 1, HAL_MAX_DELAY);   // Запись по адресу через шину I2C
    
    // Установка диапазона
    HAL_I2C_Mem_Read(&hi2c1, BMA180_ADDRESS, 0x0B, I2C_MEMADD_SIZE_8BIT, temp, 1, HAL_MAX_DELAY);      // Чтение по адресу через шину I2C
    temp1 = (temp[0] & 0xF1) | a_resolution;
    HAL_I2C_Mem_Write(&hi2c1, BMA180_ADDRESS, 0x0B, I2C_MEMADD_SIZE_8BIT, &temp1, 1, HAL_MAX_DELAY);   // Запись по адресу через шину I2C
}
void bma180SetResolution(int gValue) {
    switch (gValue) {
        case 1:
            a_resolution = 0x00;
            a_scale = 4096.0 * 2;
            break;
        case 2:
            a_resolution = 0x04;
            a_scale = 4096.0;
            break; 
        case 3:
            a_resolution = 0x06;
            a_scale = 4096.0 * 2.0 / 3.0;
            break; 
        case 4:
            a_resolution = 0x08;
            a_scale = 4096.0 / 2.0;
            break;
        case 8:
            a_resolution = 0x0A;
            a_scale = 4096.0 / 4;
            break;
        case 16:
            a_resolution = 0x0C;
            a_scale = 4096.0 / 8;
            break;
        default:
            a_resolution = 0x04;
            a_scale = 4096.0; // Значение по умолчанию
            break;
    }
}
void bma180GetAccelerometerData(int16_t *result) {
    uint8_t buff[6];
    
    // Чтение данных
    HAL_I2C_Mem_Read(&hi2c1, BMA180_ADDRESS, 0x02, I2C_MEMADD_SIZE_8BIT, buff,6, HAL_MAX_DELAY);   // Чтение по адресу через шину I2C
    
    // Обработка данных с учетом смещений
    result[0] = (((int16_t)((buff[0] << 8) | buff[1])) >>2) + a_offx; // X
    result[1] = (((int16_t)((buff[2] << 8) | buff[3])) >>2) + a_offy; // Y
    result[2] = (((int16_t)((buff[4] << 8) | buff[5])) >>2) + a_offz; // Z
    
}
// Передача данных по UART
void SendUARTData(char *buffer) {
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}
// Поиск регистров датчиков на шине I2C
void ScanI2C(void) {
    uint8_t address;
    for (address = 1; address < 127; address++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, address << 1, 1, HAL_MAX_DELAY) == HAL_OK) {
            // Устройство найдено
            snprintf(uart_buffer, sizeof(uart_buffer), "Device found at address: 0x%02X\r\n", address);
            HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
        }
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
    //HMC5883L_Init;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  // Инициализация датчиков
  BMP085_Init();
  ITG3205_Init();
  HMC5883L_Init(1.3, 0x00);
  BMA180_Init(4);
  /* USER CODE BEGIN 2 */
  /*
  char atcommand[] = "AT+CGNSPWR=1\r\n";
  HAL_UART_Transmit(&huart1,(uint8_t*)atcommand,strlen(atcommand),1000);

  HAL_UART_Receive(&huart1,(uint8_t*)uart_buffer,sizeof(uart_buffer),1000);

  HAL_UART_Transmit(&huart2,(uint8_t*)uart_buffer,sizeof(uart_buffer),1000);
  */
  /* USER CODE END 2 */
  

 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Receive_IT(&huart2,(uint8_t*)&pc_rx_byte,1);                                  // Ожидание первого байта сообщния по UART2
  HAL_UART_Receive_IT(&huart1,(uint8_t*)&modem_rx_byte,1);                               // Ожидание первого байта сообщния по UART1
  while (1)
  {

    /*
    if(pc_ready){                                                                        // Условие готовности сообщения
      HAL_UART_Transmit(&huart1,(uint8_t*)&rx_pc_buffer,pc_index,HAL_MAX_DELAY);         // Отправка на UART1 сообщения от UART2
      pc_ready = 0;                                                                      // Обнуления флага готовности
      pc_index = 0;                                                                      // Обнуление счетчика длины сообщения
    }
    else if(modem_ready){                                                                // Условие готовности сообщения
      HAL_UART_Transmit(&huart2,(uint8_t*)&rx_modem_buffer,modem_index,HAL_MAX_DELAY);   // Отправка на UART2 сообщения от UART1
      modem_ready = 0;                                                                   // Обнуление флага готовности
      modem_index = 0;                                                                   // Обнуление счетчика длины сообщения
    }
    */
    long temperature = BMP085_GetTemperature();
    long pressure = BMP085_GetPressure();
    // BMP085 - Температура и давление
    snprintf(uart_buffer, sizeof(uart_buffer), "Temperature: %ld C,\r\n", temperature);
    SendUARTData(uart_buffer);
    snprintf(uart_buffer, sizeof(uart_buffer), "Pressure: %ld Pa,\r\n", pressure);
    SendUARTData(uart_buffer);
    
    // ITG3205 - Гироскоп
    int16_t gyro[4];
    ITGGetGyroscopeData(gyro);
    uint16_t gyroX = gyro[0] / 14.375;
    uint16_t gyroY = gyro[1] / 14.375;
    uint16_t gyroZ = gyro[2] / 14.375;
    snprintf(uart_buffer,sizeof(uart_buffer),"Angular velocity: X=%d,Y=%d,Z=%d\r\n", gyroX, gyroY, gyroZ);
    SendUARTData(uart_buffer);
      
    // HMC5883L - Магнетометр 
    int16_t magData[3];
    HMC5883LGetMagnetometerData(magData);
    int16_t mx = magData[0] * m_scale;
    int16_t my = magData[1] * m_scale;
    int16_t mz = magData[2] * m_scale;
    snprintf(uart_buffer,sizeof(uart_buffer),"Magnetometer: X=%d,Y=%d,Z=%d\r\n",mx,my,mz);
    SendUARTData(uart_buffer);

    // BMA180 - Акселерометр
    int16_t accel[3];
    bma180GetAccelerometerData(accel);
    int16_t x = accel[0] / a_scale;
    int16_t y = accel[1] / a_scale;
    int16_t z = accel[2] / a_scale;
    snprintf(uart_buffer,sizeof(uart_buffer),"Acceslerometer: X=%d,Y=%d,Z=%d\r\n",x,y,z);
    SendUARTData(uart_buffer);
  
    HAL_Delay(2000);  // Задержка перед следующим циклом
      
    
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  hi2c1.Init.ClockSpeed = 100000;  //Гц
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
  huart1.Init.BaudRate = 115200; // Бит/с
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
  huart2.Init.BaudRate = 115200;  // Бит/с
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Передача сообщений между пк и модемом
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {                                       // Данные от пк
      if(pc_index < Buffer_size){                                 // Счетчик объема сообщения
        rx_pc_buffer[pc_index++] = pc_rx_byte;                    // Запись побайтово в буфер
        if (pc_rx_byte == '\n') pc_ready = 1;                     // Условие проверки на окончание сообщения 
      }
      HAL_UART_Receive_IT(&huart2, (uint8_t*)&pc_rx_byte, 1);     // Ожидание следующего байта сообщения
    }
    else if (huart == &huart1) {                                  // Данные от SIM868
        if(modem_index < Buffer_size){                            // Счетчик объема сообщения
          rx_modem_buffer[modem_index++] = modem_rx_byte;         // Запись побайтово в буфер
          if (modem_rx_byte == '\n') modem_ready = 1;             // Условие проверки на окончание сообщения 
      }
      HAL_UART_Receive_IT(&huart1, (uint8_t*)&modem_rx_byte, 1);  // Ожидание следующего байта сообщения
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
