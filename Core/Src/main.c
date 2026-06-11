/* USER CODE BEGIN Header */
/**
 * HICH TI FILE
  ******************************************************************************
  * @file           : main.c
  * @brief          : HAB-01 Flight Computer — Full Sensor Suite + LoRa + Nano SD Logging
  *
  * STM32F303RETx (Nucleo-F303RE)
  *
  * =========================================================================
  * ROOT CAUSE FIX — "Nano doesn't receive"
  * =========================================================================
  * The old code assigned the Nano to USART2 (PA2 / PA3).
  * On EVERY Nucleo-F303RE board, PA2 is HARDWIRED to the ST-Link chip for
  * the Virtual COM Port bridge.  The ST-Link fights every byte you send,
  * corrupting the line.  That is why the Nano never received data.
  *
  * FIX: Nano → USART1 (PA9 TX).  PA9 is on the Arduino header (D1) and has
  * NO ST-Link connection.  USART2 is removed entirely.
  *
  * UART ASSIGNMENT
  * ---------------
  *   USART1  PA9 (TX) / PA10 (RX)  →  Arduino Nano D2 SoftwareSerial (SD log)
  *   USART3  PB10(TX) / PB11(RX)   →  LoRa E32-433T30D
  *   (USART2 NOT USED — PA2 belongs to ST-Link)
  *
  * I2C1    PB8 (SCL) / PB9 (SDA)
  *   0x77  BMP180   — temperature + pressure
  *   0x53  ADXL345  — 3-axis accelerometer
  *   0x38  AHT10    — humidity + temperature
  *
  * OneWire  PA8  → DS18B20 external temperature
  * ADC1_IN1 PA0  → GUVA-S12SD UV sensor
  * TIM2         microsecond timer (prescaler=7 → 1 MHz @ 8 MHz SYSCLK)
  *
  * LoRa AUX busy pin → PB12 (GPIO input, no pull)
  * LoRa M0 → GND, M1 → GND  (transparent / broadcast mode)
  *
  * -------------------------------------------------------------------------
  * FreeRTOS HEAP
  * -------------------------------------------------------------------------
  *  Open  Core/Inc/FreeRTOSConfig.h
  *  Change:  #define configTOTAL_HEAP_SIZE  ((size_t)(3072))
  *  To:      #define configTOTAL_HEAP_SIZE  ((size_t)(10240))
  *  OR do it in CubeMX → Middleware → FREERTOS → TOTAL_HEAP_SIZE = 10240
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define TELEM_BUF_SIZE   256
#define LORA_BUF_SIZE    256

/* LoRa E32 fixed addressing (address 0x0000, channel 0x06).
   Pass use_header=0 to LoRa_Send() for transparent/broadcast mode. */
#define LORA_ADDR_H  0x00
#define LORA_ADDR_L  0x00
#define LORA_CHAN     0x06
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef  hadc1;
I2C_HandleTypeDef  hi2c1;
TIM_HandleTypeDef  htim2;

/* -----------------------------------------------------------------------
   UART handles
   huart1 = USART1  PA9(TX)/PA10(RX)  → Arduino Nano D2  (PRIMARY logger)
   huart3 = USART3  PB10(TX)/PB11(RX) → LoRa E32-433T30D (secondary RF)
   ----------------------------------------------------------------------- */
UART_HandleTypeDef huart1;   /* Nano SD logger */
UART_HandleTypeDef huart3;   /* LoRa E32       */

/* -----------------------------------------------------------------------
   RTOS task  —  2 KB stack is sufficient
   ----------------------------------------------------------------------- */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name       = "defaultTask",
    .stack_size = 512 * 4,        /* 2048 bytes */
    .priority   = (osPriority_t)osPriorityNormal,
};

/* USER CODE BEGIN PV */

/* -- BMP180 ---------------------------------------------------------------- */
int16_t  AC1, AC2, AC3;
uint16_t AC4, AC5, AC6;
int16_t  B1, B2, MB, MC, MD;
int32_t  bmp_b5;
int32_t  bmp_temp;       /* 0.1 °C units  e.g. 248 = 24.8 °C */
int32_t  bmp_pressure;   /* Pa                                 */
int32_t  bmp_altitude;   /* m                                  */

/* -- AHT10 ----------------------------------------------------------------- */
int aht_t_int;   /* temp  ×100  e.g. 2404 = 24.04 °C */
int aht_h_int;   /* humid ×100  e.g. 5081 = 50.81 %  */

/* -- ADXL345 --------------------------------------------------------------- */
int16_t ax, ay, az;   /* mg */

/* -- UV -------------------------------------------------------------------- */
uint32_t uv_adc;
float    uv_voltage;

/* -- DS18B20 --------------------------------------------------------------- */
float dsb_temp;

/* -- LoRa RX --------------------------------------------------------------- */
volatile uint8_t lora_rx_byte;
static   char    lora_rx_buf[LORA_BUF_SIZE];
static   uint8_t lora_rx_idx = 0;
volatile char    lora_line_buf[LORA_BUF_SIZE];
volatile uint8_t lora_line_ready = 0;

/* -- Telemetry ------------------------------------------------------------- */
uint32_t telem_seq = 0;

/* USER CODE END PV */

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN 0 */

/* ==========================================================================
   UTILITY
   ========================================================================== */

void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

/*
 * Nano_Send — transmit to Arduino Nano over USART1 (PA9 TX).
 * This is the PRIMARY data path; Nano logs every packet to SD card.
 */
void Nano_Send(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 2000);
}

/*
 * LoRa_Send — transmit over LoRa E32 on USART3 (PB10 TX).
 * use_header=0  → transparent / broadcast mode (M0=GND, M1=GND)
 * use_header=1  → fixed-address mode (3-byte header prepended)
 */
void LoRa_Send(const char *msg, uint8_t use_header)
{
    if (use_header) {
        uint8_t hdr[3] = { LORA_ADDR_H, LORA_ADDR_L, LORA_CHAN };
        HAL_UART_Transmit(&huart3, hdr, 3, 500);
    }
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 2000);
}

/* ==========================================================================
   I2C SCAN  (runs once at boot; output goes to Nano → SD card)
   ========================================================================== */

void I2C_Scan(void)
{
    char buf[24];
    Nano_Send("# I2C Scan\r\n");
    for (uint8_t addr = 1; addr < 128; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 1, 50) == HAL_OK) {
            snprintf(buf, sizeof(buf), "# I2C 0x%02X\r\n", addr);
            Nano_Send(buf);
        }
    }
    Nano_Send("# Scan end\r\n");
}

/* ==========================================================================
   BMP180
   ========================================================================== */

uint8_t BMP180_ReadID(void)
{
    uint8_t id = 0;
    HAL_I2C_Mem_Read(&hi2c1, 0x77 << 1, 0xD0,
                     I2C_MEMADD_SIZE_8BIT, &id, 1, 100);
    return id;
}

void BMP180_ReadCalibration(void)
{
    uint8_t data[22];
    HAL_I2C_Mem_Read(&hi2c1, 0x77 << 1, 0xAA,
                     I2C_MEMADD_SIZE_8BIT, data, 22, 100);
    AC1 = (int16_t) ((data[0]  << 8) | data[1]);
    AC2 = (int16_t) ((data[2]  << 8) | data[3]);
    AC3 = (int16_t) ((data[4]  << 8) | data[5]);
    AC4 = (uint16_t)((data[6]  << 8) | data[7]);
    AC5 = (uint16_t)((data[8]  << 8) | data[9]);
    AC6 = (uint16_t)((data[10] << 8) | data[11]);
    B1  = (int16_t) ((data[12] << 8) | data[13]);
    B2  = (int16_t) ((data[14] << 8) | data[15]);
    MB  = (int16_t) ((data[16] << 8) | data[17]);
    MC  = (int16_t) ((data[18] << 8) | data[19]);
    MD  = (int16_t) ((data[20] << 8) | data[21]);
}

/* Returns temperature in 0.1 °C; must be called BEFORE BMP180_ReadPressure */
int32_t BMP180_ReadTemperature(void)
{
    uint8_t cmd = 0x2E, data[2];
    HAL_I2C_Mem_Write(&hi2c1, 0x77 << 1, 0xF4,
                      I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);
    HAL_Delay(5);
    HAL_I2C_Mem_Read(&hi2c1, 0x77 << 1, 0xF6,
                     I2C_MEMADD_SIZE_8BIT, data, 2, 100);
    int32_t UT = ((int32_t)data[0] << 8) | data[1];
    int32_t X1 = ((UT - (int32_t)AC6) * (int32_t)AC5) >> 15;
    int32_t X2 = ((int32_t)MC << 11) / (X1 + (int32_t)MD);
    bmp_b5 = X1 + X2;
    return (bmp_b5 + 8) >> 4;
}

/* Must call BMP180_ReadTemperature() first to populate bmp_b5 */
int32_t BMP180_ReadPressure(void)
{
    uint8_t cmd = 0x34, data[3];
    HAL_I2C_Mem_Write(&hi2c1, 0x77 << 1, 0xF4,
                      I2C_MEMADD_SIZE_8BIT, &cmd, 1, 100);
    HAL_Delay(5);
    HAL_I2C_Mem_Read(&hi2c1, 0x77 << 1, 0xF6,
                     I2C_MEMADD_SIZE_8BIT, data, 3, 100);
    int32_t  UP = (((int32_t)data[0] << 16) |
                   ((int32_t)data[1] <<  8) |
                    (int32_t)data[2]) >> 8;
    int32_t  B6 = bmp_b5 - 4000;
    int32_t  X1 = (B2 * ((B6 * B6) >> 12)) >> 11;
    int32_t  X2 = (AC2 * B6) >> 11;
    int32_t  X3 = X1 + X2;
    int32_t  B3 = (((int32_t)AC1 * 4 + X3) + 2) >> 2;
    X1 = ((int32_t)AC3 * B6) >> 13;
    X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    uint32_t B4 = ((uint32_t)AC4 * (uint32_t)(X3 + 32768)) >> 15;
    uint32_t B7 = ((uint32_t)UP - (uint32_t)B3) * 50000UL;
    int32_t  p  = (B7 < 0x80000000UL)
                  ? (int32_t)((B7 * 2UL) / B4)
                  : (int32_t)((B7 / B4) * 2UL);
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    return p + ((X1 + X2 + 3791) >> 4);
}

int32_t BMP180_CalculateAltitude(int32_t pressure)
{
    return (int32_t)(44330.0f *
           (1.0f - powf((float)pressure / 101325.0f, 0.1903f)));
}

/* ==========================================================================
   AHT10
   ========================================================================== */

void AHT10_Init(void)
{
    uint8_t cmd[3] = { 0xE1, 0x08, 0x00 };
    HAL_I2C_Master_Transmit(&hi2c1, 0x38 << 1, cmd, 3, 100);
    osDelay(50);
}

void AHT10_ReadAll(void)
{
    uint8_t cmd[3] = { 0xAC, 0x33, 0x00 };
    uint8_t data[6] = { 0 };
    HAL_I2C_Master_Transmit(&hi2c1, 0x38 << 1, cmd, 3, 100);
    osDelay(80);
    HAL_I2C_Master_Receive(&hi2c1, 0x38 << 1, data, 6, 100);
    uint32_t raw_h = ((uint32_t)data[1] << 12) |
                     ((uint32_t)data[2] <<  4) |
                      (data[3] >> 4);
    uint32_t raw_t = ((uint32_t)(data[3] & 0x0F) << 16) |
                     ((uint32_t)data[4] <<  8) |
                      data[5];
    aht_t_int = (int)(((float)raw_t * 200.0f / 1048576.0f - 50.0f) * 100.0f);
    aht_h_int = (int)(((float)raw_h * 100.0f / 1048576.0f) * 100.0f);
}

/* ==========================================================================
   ADXL345
   ========================================================================== */

#define ADXL345_ADDR  (0x53 << 1)

void ADXL345_Init(void)
{
    uint8_t d = 0x08;   /* POWER_CTL: measurement mode */
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR, 0x2D,
                      I2C_MEMADD_SIZE_8BIT, &d, 1, 100);
    d = 0x0B;           /* DATA_FORMAT: full-res, ±16 g */
    HAL_I2C_Mem_Write(&hi2c1, ADXL345_ADDR, 0x31,
                      I2C_MEMADD_SIZE_8BIT, &d, 1, 100);
}

void ADXL345_ReadAccel(void)
{
    uint8_t data[6];
    if (HAL_I2C_Mem_Read(&hi2c1, ADXL345_ADDR, 0x32,
                          I2C_MEMADD_SIZE_8BIT, data, 6, 100) == HAL_OK) {
        ax = (int16_t)(((int16_t)((data[1] << 8) | data[0])) * 4);
        ay = (int16_t)(((int16_t)((data[3] << 8) | data[2])) * 4);
        az = (int16_t)(((int16_t)((data[5] << 8) | data[4])) * 4);
    } else {
        ax = ay = az = 0;
    }
}

/* ==========================================================================
   DS18B20 (OneWire on PA8)
   ========================================================================== */

uint8_t OneWire_Reset(void)
{
    GPIO_InitTypeDef g = { 0 };
    g.Pin   = OneWire_Pin_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OneWire_Pin_GPIO_Port, &g);

    HAL_GPIO_WritePin(OneWire_Pin_GPIO_Port, OneWire_Pin_Pin, GPIO_PIN_RESET);
    delay_us(480);
    HAL_GPIO_WritePin(OneWire_Pin_GPIO_Port, OneWire_Pin_Pin, GPIO_PIN_SET);
    delay_us(70);

    g.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(OneWire_Pin_GPIO_Port, &g);
    uint8_t present = !HAL_GPIO_ReadPin(OneWire_Pin_GPIO_Port, OneWire_Pin_Pin);
    delay_us(410);
    return present;
}

void OneWire_WriteBit(uint8_t bit)
{
    GPIO_InitTypeDef g = { 0 };
    g.Pin   = OneWire_Pin_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OneWire_Pin_GPIO_Port, &g);
    portENTER_CRITICAL();
    HAL_GPIO_WritePin(OneWire_Pin_GPIO_Port, OneWire_Pin_Pin, GPIO_PIN_RESET);
    if (bit) {
        delay_us(2);
        HAL_GPIO_WritePin(OneWire_Pin_GPIO_Port, OneWire_Pin_Pin, GPIO_PIN_SET);
        delay_us(58);
    } else {
        delay_us(60);
        HAL_GPIO_WritePin(OneWire_Pin_GPIO_Port, OneWire_Pin_Pin, GPIO_PIN_SET);
        delay_us(2);
    }
    portEXIT_CRITICAL();
}

uint8_t OneWire_ReadBit(void)
{
    GPIO_InitTypeDef g = { 0 };
    g.Pin   = OneWire_Pin_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OneWire_Pin_GPIO_Port, &g);
    uint8_t bit;
    portENTER_CRITICAL();
    HAL_GPIO_WritePin(OneWire_Pin_GPIO_Port, OneWire_Pin_Pin, GPIO_PIN_RESET);
    delay_us(3);
    HAL_GPIO_WritePin(OneWire_Pin_GPIO_Port, OneWire_Pin_Pin, GPIO_PIN_SET);
    delay_us(10);
    bit = HAL_GPIO_ReadPin(OneWire_Pin_GPIO_Port, OneWire_Pin_Pin);
    delay_us(53);
    portEXIT_CRITICAL();
    return bit;
}

void OneWire_WriteByte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++) {
        OneWire_WriteBit(data & 0x01);
        data >>= 1;
    }
}

uint8_t OneWire_ReadByte(void)
{
    uint8_t d = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (OneWire_ReadBit()) d |= (uint8_t)(1 << i);
    }
    return d;
}

float DS18B20_ReadTemp(void)
{
    if (!OneWire_Reset()) return -999.0f;
    OneWire_WriteByte(0xCC);
    OneWire_WriteByte(0x44);
    osDelay(800);
    if (!OneWire_Reset()) return -999.0f;
    OneWire_WriteByte(0xCC);
    OneWire_WriteByte(0xBE);
    uint8_t lsb = OneWire_ReadByte();
    uint8_t msb = OneWire_ReadByte();
    return (float)(int16_t)((msb << 8) | lsb) / 16.0f;
}

/* ==========================================================================
   UV  (GUVA-S12SD on PA0 / ADC1_IN1)
   ========================================================================== */

void UV_Read(void)
{
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 50);
        sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        osDelay(2);
    }
    uv_adc     = sum / 16U;
    uv_voltage = ((float)uv_adc * 3.3f) / 4095.0f;
}

/* ==========================================================================
   TELEMETRY — CSV row sent to Nano (primary) then LoRa (secondary)
   ========================================================================== */

void Build_Telemetry(char *buf, size_t buf_size)
{
    int     dsb_int  = (int)dsb_temp;
    int     dsb_frac = (int)(fabsf(dsb_temp - (float)dsb_int) * 100.0f);
    int     uv_mv    = (int)(uv_voltage * 1000.0f);

    snprintf(buf, buf_size,
             "%lu,"
             "%ld,%ld,%ld,"
             "%d,%d,"
             "%d.%02d,%d,"
             "%d,%d,%d"
             "\r\n",
             (unsigned long)telem_seq,
             (long)bmp_temp, (long)bmp_pressure, (long)bmp_altitude,
             aht_t_int, aht_h_int,
             dsb_int, dsb_frac, uv_mv,
             (int)ax, (int)ay, (int)az);
}

/* USER CODE END 0 */

/* ==========================================================================
   main()
   ========================================================================== */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM2_Init();
    HAL_TIM_Base_Start(&htim2);     /* microsecond timer needed early */

    /*
     * IMPORTANT: Init USART1 (Nano) FIRST, before anything else,
     * so all boot messages are captured to the SD card log.
     */
    MX_USART1_UART_Init();          /* Nano — PRIMARY logger on PA9    */
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_USART3_UART_Init();          /* LoRa — secondary RF on PB10     */

    Nano_Send("# HAB-01 PRE-RTOS\r\n");
    Nano_Send("# USART1=Nano(PA9) USART3=LoRa(PB10)\r\n");

    {
        char m[40];
        snprintf(m, sizeof(m), "# BMP ID: 0x%02X\r\n", BMP180_ReadID());
        Nano_Send(m);
        BMP180_ReadCalibration();
        Nano_Send("# BMP cal done\r\n");
    }

    /* RTOS ---------------------------------------------------------------- */
    osKernelInitialize();

    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
                                    &defaultTask_attributes);

    if (!defaultTaskHandle) {
        /* Heap too small — see FreeRTOSConfig.h comment at top of file */
        Nano_Send("# THREAD_FAIL -- increase configTOTAL_HEAP_SIZE to 10240\r\n");
        while (1) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            HAL_Delay(200);
        }
    }

    Nano_Send("# RTOS START\r\n");
    osKernelStart();
    while (1) {}
}

/* ==========================================================================
   RTOS TASK
   ========================================================================== */

void StartDefaultTask(void *argument)
{
    (void)argument;

    /* Sensor init that needs osDelay */
    ADXL345_Init();
    AHT10_Init();
    I2C_Scan();
    osDelay(500);

    /* Start LoRa RX interrupt — Nano TX is receive-only from STM32 side */
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&lora_rx_byte, 1);

    /* CSV header — first line written to SD card by Nano */
    Nano_Send("seq,"
              "bmp_t_01C,bmp_p_Pa,bmp_alt_m,"
              "aht_t_01C,aht_h_01pct,"
              "dsb_t_C,uv_mV,"
              "ax_mg,ay_mg,az_mg"
              "\r\n");

    Nano_Send("# LOOP START\r\n");

    for (;;)
    {
        telem_seq++;

        /* ---- Read all sensors ----------------------------------------- */
        ADXL345_ReadAccel();
        bmp_temp     = BMP180_ReadTemperature();  /* populates bmp_b5 */
        bmp_pressure = BMP180_ReadPressure();
        bmp_altitude = BMP180_CalculateAltitude(bmp_pressure);
        AHT10_ReadAll();
        dsb_temp     = DS18B20_ReadTemp();
        UV_Read();

        /* ---- Process LoRa uplink (commands from ground) --------------- */
        if (lora_line_ready) {
            lora_line_ready = 0;
            char ack[64];
            snprintf(ack, sizeof(ack), "ACK:%s\r\n",
                     (const char *)lora_line_buf);
            LoRa_Send(ack, 0);
        }

        /* ---- Build telemetry CSV row ---------------------------------- */
        static char telem[TELEM_BUF_SIZE];
        Build_Telemetry(telem, sizeof(telem));

        /*
         * PRIMARY:  Nano → SD card  (USART1 / PA9)
         * Always sent first, unconditionally, every cycle.
         */
        Nano_Send(telem);

        /*
         * SECONDARY: LoRa → ground station  (USART3 / PB10)
         * Sent after Nano so Nano never blocks on LoRa timing.
         */
        LoRa_Send(telem, 0);

        osDelay(5000);
    }
}

/* ==========================================================================
   UART RX CALLBACK
   Only LoRa (USART3) — Nano is transmit-only from this side.
   ========================================================================== */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        uint8_t ch = lora_rx_byte;
        if (ch == '\n' || ch == '\r') {
            if (lora_rx_idx > 0) {
                if (!lora_line_ready) {
                    lora_rx_buf[lora_rx_idx] = '\0';
                    memcpy((char *)lora_line_buf, lora_rx_buf,
                           lora_rx_idx + 1);
                    lora_line_ready = 1;
                }
                lora_rx_idx = 0;
            }
        } else if (lora_rx_idx < LORA_BUF_SIZE - 1) {
            lora_rx_buf[lora_rx_idx++] = (char)ch;
        } else {
            lora_rx_idx = 0;
        }
        HAL_UART_Receive_IT(&huart3, (uint8_t *)&lora_rx_byte, 1);
    }
}

/* ==========================================================================
   PERIPHERAL INIT
   ========================================================================== */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       o = { 0 };
    RCC_ClkInitTypeDef       c = { 0 };
    RCC_PeriphCLKInitTypeDef p = { 0 };

    o.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    o.HSIState            = RCC_HSI_ON;
    o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    o.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();

    c.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    c.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV1;
    c.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_0) != HAL_OK) Error_Handler();

    /* USART2 removed — PA2 is ST-Link VCP, do NOT claim it */
    p.PeriphClockSelection = RCC_PERIPHCLK_USART1 |
                             RCC_PERIPHCLK_USART3 |
                             RCC_PERIPHCLK_I2C1   |
                             RCC_PERIPHCLK_ADC12;
    p.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    p.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    p.Adc12ClockSelection  = RCC_ADC12PLLCLK_DIV1;
    p.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&p) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
    ADC_MultiModeTypeDef   mm = { 0 };
    ADC_ChannelConfTypeDef sc = { 0 };

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    mm.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &mm) != HAL_OK)
        Error_Handler();

    sc.Channel      = ADC_CHANNEL_1;
    sc.Rank         = ADC_REGULAR_RANK_1;
    sc.SingleDiff   = ADC_SINGLE_ENDED;
    sc.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
    sc.OffsetNumber = ADC_OFFSET_NONE;
    sc.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sc) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x00201D2B;
    hi2c1.Init.OwnAddress1      = 0;
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2      = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
}

/*
 * USART1 — Arduino Nano SoftwareSerial D2  (PA9 TX, PA10 RX)
 * THIS is the Nano logger UART.  PA9 is on Arduino header D1.
 * Do NOT use USART2 (PA2) — it is wired to ST-Link on the Nucleo board.
 */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance                    = USART1;
    huart1.Init.BaudRate               = 9600;
    huart1.Init.WordLength             = UART_WORDLENGTH_8B;
    huart1.Init.StopBits               = UART_STOPBITS_1;
    huart1.Init.Parity                 = UART_PARITY_NONE;
    huart1.Init.Mode                   = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

/* USART3 — LoRa E32-433T30D  (PB10 TX, PB11 RX) */
static void MX_USART3_UART_Init(void)
{
    huart3.Instance                    = USART3;
    huart3.Init.BaudRate               = 9600;
    huart3.Init.WordLength             = UART_WORDLENGTH_8B;
    huart3.Init.StopBits               = UART_STOPBITS_1;
    huart3.Init.Parity                 = UART_PARITY_NONE;
    huart3.Init.Mode                   = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef  sc = { 0 };
    TIM_MasterConfigTypeDef mc = { 0 };

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 7;        /* 8 MHz / 8 = 1 MHz → 1 µs/tick */
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 65535;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

    sc.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sc) != HAL_OK) Error_Handler();

    mc.MasterOutputTrigger = TIM_TRGO_RESET;
    mc.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &mc) != HAL_OK)
        Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = { 0 };

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PA5 — Nucleo LD2 user LED */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    g.Pin   = GPIO_PIN_5;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);

    /* PA8 — OneWire DS18B20, open-drain, idle HIGH */
    HAL_GPIO_WritePin(OneWire_Pin_GPIO_Port, OneWire_Pin_Pin, GPIO_PIN_SET);
    g.Pin   = OneWire_Pin_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OneWire_Pin_GPIO_Port, &g);

    /* Buzzer pin — push-pull, idle LOW */
    HAL_GPIO_WritePin(GPIOB, Buzzer_Pin_Pin, GPIO_PIN_RESET);
    g.Pin   = Buzzer_Pin_Pin;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    /* PB12 — LoRa AUX busy, input, no pull */
    g.Pin   = GPIO_PIN_12;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) HAL_IncTick();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
