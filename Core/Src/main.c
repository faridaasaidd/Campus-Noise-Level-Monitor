/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Campus Noise Monitor STM32F303K8 + Pmod CLP LCD
  *
  * Pmod CLP wiring (UNCHANGED):
  *   DB0-DB7 : PA0,PA1,PA3,PA4,PA5,PA6,PA7,PA8
  *

  *   DB0     : PA0
  *   DB1     : PA1
  *   DB2     : PB1   
  *   DB3     : PA4
  *   DB4     : PA5
  *   DB5     : PA6
  *   DB6     : PA7
  *   DB7     : PA8
  *   RS      : PF0
  *   R/W     : PF1
  *   E       : PA9
  *   Mic ADC : PB0 (ADC1_IN11)
  *   Green   : PB4
  *   Red     : PB5
  *   Buzzer  : PB6  (deprecated)
  *   Button  : PA10
  *   UART2   : TX=PA2  RX=PA15
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/
typedef enum { ST_QUIET = 0u, ST_WARN = 1u, ST_ALARM = 2u } NoiseState;

typedef struct {
    const char *name;
    uint16_t    warn_thr;
    uint16_t    alarm_thr;
} Preset;

/* Private define ------------------------------------------------------------*/

#define LCD_DB0_PORT    GPIOA
#define LCD_DB0_PIN     GPIO_PIN_0
#define LCD_DB1_PORT    GPIOA
#define LCD_DB1_PIN     GPIO_PIN_1
#define LCD_DB2_PORT    GPIOB
#define LCD_DB2_PIN     GPIO_PIN_1
#define LCD_DB3_PORT    GPIOA
#define LCD_DB3_PIN     GPIO_PIN_4
#define LCD_DB4_PORT    GPIOA
#define LCD_DB4_PIN     GPIO_PIN_5
#define LCD_DB5_PORT    GPIOA
#define LCD_DB5_PIN     GPIO_PIN_6
#define LCD_DB6_PORT    GPIOA
#define LCD_DB6_PIN     GPIO_PIN_7
#define LCD_DB7_PORT    GPIOA
#define LCD_DB7_PIN     GPIO_PIN_8

/* Pmod CLP control */
#define LCD_RS_PORT     GPIOF
#define LCD_RS_PIN      GPIO_PIN_0
#define LCD_RW_PORT     GPIOF
#define LCD_RW_PIN      GPIO_PIN_1
#define LCD_E_PORT      GPIOA
#define LCD_E_PIN       GPIO_PIN_9

/* KS0066 commands */
#define LCD_CMD_CLEAR       0x01u
#define LCD_CMD_ENTRY_MODE  0x06u
#define LCD_CMD_DISP_ON     0x0Cu
#define LCD_CMD_FUNC_SET    0x38u
#define LCD_DDRAM_ROW0      0x80u
#define LCD_DDRAM_ROW1      0xC0u
#define LCD_DELAY_US(us)    lcd_delay_us(us)

/* Buzzer */
#define BUZZER_PORT     GPIOB
#define BUZZER_PIN      GPIO_PIN_6

/* Button */
#define BTN_PORT        GPIOA
#define BTN_PIN         GPIO_PIN_10
#define BTN_DEBOUNCE    3u

/* Tuning */
#define BURST_SAMPLES   64u
#define FILT_LEN        8u
#define ALARM_TICKS     5u
#define CALM_TICKS      30u
#define BLINK_TICKS     2u
#define BEEP_TICKS      1u
#define UART_TX_TIMEOUT 50u
#define UART_RX_BUF_LEN 64u

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

static Preset g_presets[3] = {
    { "Library   ", 200u, 500u },
    { "Study Room", 100u, 200u },
    { "Lab       ", 450u, 900u }
};
static uint8_t  g_preset_idx = 1u;
static uint16_t g_warn_thr   = 100u;
static uint16_t g_alarm_thr  = 200u;

static uint16_t filt_buf[FILT_LEN];
static uint8_t  filt_head = 0u;
static uint32_t filt_sum  = 0u;

static NoiseState g_state   = ST_QUIET;
static uint32_t   alarm_cnt = 0u;
static uint32_t   calm_cnt  = 0u;
static uint32_t   blink_cnt = 0u;
static uint32_t   beep_cnt  = 0u;

static volatile uint8_t g_tick = 0u;

static uint8_t  rx_buf[UART_RX_BUF_LEN];
static uint8_t  rx_head     = 0u;
static uint8_t  rx_tail     = 0u;
static uint8_t  rx_line[UART_RX_BUF_LEN];
static uint8_t  rx_line_len = 0u;
static uint8_t  rx_byte;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);

static uint16_t adc_peak_to_peak(void);
static uint16_t moving_avg(uint16_t sample);
static void     update_state(uint16_t avg);
static void     uart_log(uint16_t raw_pp, uint16_t avg);
static void     button_init(void);
static void     button_poll(void);
static void     apply_preset(uint8_t idx);
static void     uart_process_rx(void);
static void     uart_parse_line(const char *line);

static void lcd_delay_us(uint32_t us);
static void lcd_write_bus(uint8_t byte, uint8_t rs);
static void lcd_cmd(uint8_t cmd);
static void lcd_data(uint8_t ch);
static void lcd_init(void);
static void lcd_set_cursor(uint8_t row, uint8_t col);
static void lcd_print(const char *str);
static void lcd_update(uint16_t avg);

/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) { g_tick = 1u; }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uint8_t next = (uint8_t)((rx_head + 1u) % UART_RX_BUF_LEN);
        if (next != rx_tail) { rx_buf[rx_head] = rx_byte; rx_head = next; }
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1u);
    }
}

//ADC
static uint16_t adc_peak_to_peak(void)
{
    uint16_t mn = 4095u, mx = 0u;
    uint32_t i;
    for (i = 0u; i < BURST_SAMPLES; i++)
    {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 2u) == HAL_OK)
        {
            uint16_t v = (uint16_t)HAL_ADC_GetValue(&hadc1);
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }
    }
    return (mx >= mn) ? (mx - mn) : 0u;
}

//Filter
static uint16_t moving_avg(uint16_t sample)
{
    filt_sum           -= (uint32_t)filt_buf[filt_head];
    filt_buf[filt_head] = sample;
    filt_sum           += (uint32_t)sample;
    filt_head           = (uint8_t)((filt_head + 1u) % FILT_LEN);
    return (uint16_t)(filt_sum / FILT_LEN);
}

//Preset
static void apply_preset(uint8_t idx)
{
    char buf[56];
    int  len;
    if (idx >= 3u) { idx = 0u; }
    g_preset_idx = idx;
    g_warn_thr   = g_presets[idx].warn_thr;
    g_alarm_thr  = g_presets[idx].alarm_thr;
    len = snprintf(buf, sizeof(buf), "[PRESET] %s  WARN=%u ALARM=%u\r\n",
                   g_presets[idx].name, (unsigned)g_warn_thr, (unsigned)g_alarm_thr);
    if (len > 0 && (size_t)len < sizeof(buf))
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, UART_TX_TIMEOUT);
    g_state = ST_QUIET; alarm_cnt = 0u; calm_cnt = 0u;
}

//Button
static void button_init(void)
{
    GPIO_InitTypeDef gpio;
    memset(&gpio, 0, sizeof(gpio));
    gpio.Pin  = BTN_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BTN_PORT, &gpio);
}

static void button_poll(void)
{
    static uint8_t low_count  = 0u;
    static uint8_t last_state = 1u;
    uint8_t cur = (HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) == GPIO_PIN_RESET) ? 0u : 1u;
    if (cur == 0u) { low_count++; }
    else
    {
        if (low_count >= BTN_DEBOUNCE && last_state == 0u)
            apply_preset((uint8_t)((g_preset_idx + 1u) % 3u));
        low_count = 0u;
    }
    last_state = cur;
}

//fsm
static void update_state(uint16_t avg)
{
    switch (g_state)
    {
        case ST_QUIET:
            alarm_cnt = 0u; calm_cnt = 0u;
            if (avg >= g_warn_thr) { g_state = ST_WARN; }
            break;

        case ST_WARN:
            calm_cnt = 0u;
            if (avg < g_warn_thr) { g_state = ST_QUIET; alarm_cnt = 0u; }
            else if (avg >= g_alarm_thr)
            {
                alarm_cnt++;
                if (alarm_cnt >= ALARM_TICKS)
                { g_state = ST_ALARM; alarm_cnt = 0u; blink_cnt = 0u; beep_cnt = 0u; }
            }
            else { alarm_cnt = 0u; }
            break;

        case ST_ALARM:
            alarm_cnt = 0u;
            if (avg < g_warn_thr)
            { calm_cnt++; if (calm_cnt >= CALM_TICKS) { g_state = ST_QUIET; calm_cnt = 0u; } }
            else { calm_cnt = 0u; }
            break;

        default: g_state = ST_QUIET; break;
    }

    switch (g_state)
    {
        case ST_QUIET:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
            blink_cnt = 0u; beep_cnt = 0u;
            break;

        case ST_WARN:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
            beep_cnt++;
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN,
                              (beep_cnt <= BEEP_TICKS) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            if (beep_cnt >= 10u) { beep_cnt = 0u; }
            break;

        case ST_ALARM:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
            if (++blink_cnt >= BLINK_TICKS) { HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5); blink_cnt = 0u; }
            break;

        default: break;
    }
}

//	UART
static void uart_log(uint16_t raw_pp, uint16_t avg)
{
    static const char * const sn[] = { "QUIET", "WARN ", "ALARM" };
    char buf[72]; int len;
    len = snprintf(buf, sizeof(buf), "[%s] pp=%4u avg=%4u WARN=%u ALARM=%u preset=%s\r\n",
                   sn[(uint8_t)g_state], (unsigned)raw_pp, (unsigned)avg,
                   (unsigned)g_warn_thr, (unsigned)g_alarm_thr,
                   g_presets[g_preset_idx].name);
    if (len > 0 && (size_t)len < sizeof(buf))
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, UART_TX_TIMEOUT);
}

static void uart_parse_line(const char *line)
{
    static const uint8_t ok[]  = "OK\r\n";
    static const uint8_t err[] = "ERR\r\n";
    int val;
    if (strncmp(line, "SET THR WARN ", 13) == 0)
    {
        val = atoi(line + 13);
        if (val >= 100 && val <= 4000 && (uint16_t)val < g_alarm_thr)
        { g_warn_thr = (uint16_t)val; HAL_UART_Transmit(&huart2,(uint8_t*)ok, 4u, UART_TX_TIMEOUT); }
        else HAL_UART_Transmit(&huart2,(uint8_t*)err, 5u, UART_TX_TIMEOUT);
    }
    else if (strncmp(line, "SET THR ALARM ", 14) == 0)
    {
        val = atoi(line + 14);
        if (val >= 100 && val <= 4000 && (uint16_t)val > g_warn_thr)
        { g_alarm_thr = (uint16_t)val; HAL_UART_Transmit(&huart2,(uint8_t*)ok, 4u, UART_TX_TIMEOUT); }
        else HAL_UART_Transmit(&huart2,(uint8_t*)err, 5u, UART_TX_TIMEOUT);
    }
    else HAL_UART_Transmit(&huart2,(uint8_t*)err, 5u, UART_TX_TIMEOUT);
}

static void uart_process_rx(void)
{
    while (rx_tail != rx_head)
    {
        uint8_t ch = rx_buf[rx_tail];
        rx_tail = (uint8_t)((rx_tail + 1u) % UART_RX_BUF_LEN);
        if (ch == '\r') {  ch = '\n'; }
        if (ch == '\n' || rx_line_len >= (UART_RX_BUF_LEN - 1u))
        { rx_line[rx_line_len] = '\0'; if (rx_line_len > 0u) uart_parse_line((char*)rx_line); rx_line_len = 0u; }
        else { rx_line[rx_line_len++] = ch; }
    }
}

//LCD driver
static void lcd_delay_us(uint32_t us)
{
    /* At 48 MHz, ~12 NOPs per microsecond */
    volatile uint32_t c = us * 12u;
    while (c--) { __NOP(); }
}

static void lcd_write_bus(uint8_t byte, uint8_t rs)
{
    HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, rs ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_RW_PORT, LCD_RW_PIN, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(LCD_DB0_PORT, LCD_DB0_PIN, (byte & 0x01u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DB1_PORT, LCD_DB1_PIN, (byte & 0x02u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DB2_PORT, LCD_DB2_PIN, (byte & 0x04u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DB3_PORT, LCD_DB3_PIN, (byte & 0x08u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DB4_PORT, LCD_DB4_PIN, (byte & 0x10u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DB5_PORT, LCD_DB5_PIN, (byte & 0x20u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DB6_PORT, LCD_DB6_PIN, (byte & 0x40u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DB7_PORT, LCD_DB7_PIN, (byte & 0x80u) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_SET);
    LCD_DELAY_US(1u);
    HAL_GPIO_WritePin(LCD_E_PORT, LCD_E_PIN, GPIO_PIN_RESET);
    LCD_DELAY_US(1u);
}

static void lcd_cmd(uint8_t cmd) { lcd_write_bus(cmd, 0u); }
static void lcd_data(uint8_t ch) { lcd_write_bus(ch,  1u); }

static void lcd_init(void)
{
    HAL_Delay(30u);
    lcd_cmd(LCD_CMD_FUNC_SET);   LCD_DELAY_US(50u);
    lcd_cmd(LCD_CMD_DISP_ON);    LCD_DELAY_US(50u);
    lcd_cmd(LCD_CMD_CLEAR);      HAL_Delay(2u);
    lcd_cmd(LCD_CMD_ENTRY_MODE); LCD_DELAY_US(50u);
}

static void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t addr = (row == 0u) ? (uint8_t)(LCD_DDRAM_ROW0 | col)
                               : (uint8_t)(LCD_DDRAM_ROW1 | col);
    lcd_cmd(addr);
    LCD_DELAY_US(50u);
}

static void lcd_print(const char *str)
{
    while (*str) { lcd_data((uint8_t)*str++); LCD_DELAY_US(50u); }
}

static void lcd_update(uint16_t avg)
{
    static const char * const ss[3] = { "QUIET ", "WARN  ", "ALARM " };
    char row0[17], row1[17];
    uint8_t bar_len, i; uint16_t v;

    bar_len = (uint8_t)(((uint32_t)avg * 6u) / (uint32_t)g_alarm_thr);
    if (bar_len > 6u) { bar_len = 6u; }

    row0[0]='I'; row0[1]='d'; row0[2]='x'; row0[3]=':';
    v = avg;
    row0[4]=(char)('0'+(v/1000u)); v=(uint16_t)(v%1000u);
    row0[5]=(char)('0'+(v/100u));  v=(uint16_t)(v%100u);
    row0[6]=(char)('0'+(v/10u));   v=(uint16_t)(v%10u);
    row0[7]=(char)('0'+v);
    row0[8]=' '; row0[9]='[';
    for (i=0u; i<6u; i++) row0[10u+i] = (i < bar_len) ? (char)0xFF : ' ';
    row0[15]=']'; row0[16]='\0';

    memset(row1, ' ', 16u); row1[16]='\0';
    memcpy(row1,      g_presets[g_preset_idx].name, 10u);
    memcpy(row1 + 10, ss[(uint8_t)g_state], 6u);

    lcd_set_cursor(0u, 0u); lcd_print(row0);
    lcd_set_cursor(1u, 0u); lcd_print(row1);
}
/* USER CODE END 0 */

int main(void)
{
    /* ---- Changed: F3 ADC channel config struct has no SingleDiff/OffsetNumber ---- */
    ADC_ChannelConfTypeDef sChanCfg;
    uint16_t pp, avg;
    memset(&sChanCfg, 0, sizeof(sChanCfg));

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_USART2_UART_Init();

    /* Override ADC channel to IN11 (PB0 on F303K8) */
    sChanCfg.Channel      = ADC_CHANNEL_11;         /* << was CHANNEL_15 on L432 */
    sChanCfg.Rank         = ADC_REGULAR_RANK_1;
    sChanCfg.SamplingTime = ADC_SAMPLETIME_61CYCLES_5; /* closest to 47.5 on F3 */
    /* No SingleDiff / OffsetNumber on STM32F3 ADC */
    if (HAL_ADC_ConfigChannel(&hadc1, &sChanCfg) != HAL_OK) { Error_Handler(); }

    /* ---- Changed: F3 calibration takes only one argument ---- */
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) { Error_Handler(); }

    /* Filter buffer */
    memset(filt_buf, 0, sizeof(filt_buf));
    filt_sum = 0u; filt_head = 0u;

    /* LEDs and buzzer */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);   /* green ON  */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); /* red  OFF  */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); /* buzz OFF  */

    button_init();
    apply_preset(1u);

    /* LCD init */
    HAL_GPIO_WritePin(LCD_E_PORT,  LCD_E_PIN,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_RW_PORT, LCD_RW_PIN, GPIO_PIN_RESET);
    lcd_init();

    /* TIM2 interrupt � 10 Hz tick */
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) { Error_Handler(); }

    /* UART RX interrupt */
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1u);

    /* Startup banner */
    {
        static const char banner[] =
            "\r\n=== Campus Noise Monitor (F303K8 port) ===\r\n"
            "Presets: 0=Library  1=StudyRoom  2=Lab\r\n"
            "Commands: SET THR WARN <n>  /  SET THR ALARM <n>\r\n"
            "UART2: TX=PA2  RX=PA3  115200 8N1\r\n"
            "--------------------------------------------\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)banner,
                          (uint16_t)(sizeof(banner) - 1u), 200u);
    }

    lcd_update(0u);

    while (1)
    {
        uart_process_rx();
        if (g_tick == 0u) { continue; }
        g_tick = 0u;

        button_poll();
        pp  = adc_peak_to_peak();
        avg = moving_avg(pp);
        update_state(avg);
        lcd_update(avg);
        uart_log(pp, avg);
    }
}


void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* HSI on, PLL source = HSI/2 = 4 MHz, x12 = 48 MHz */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;  /* HSI/2 = 4 MHz */
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL12;      /* 4 x 12 = 48 MHz */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    /* SYSCLK = PLL, AHB/APB1/APB2 = /1 */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   /* APB1 max 36 MHz on F3 */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
        { Error_Handler(); }
}

/**
  * @brief  ADC1 init for STM32F303K8
  *         PB0 = ADC1_IN11 (channel 11, not 15)
  *         F3 ADC struct has no OversamplingMode, no SingleDiff
  */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    /* No OversamplingMode on F3 */
    if (HAL_ADC_Init(&hadc1) != HAL_OK) { Error_Handler(); }

    sConfig.Channel      = ADC_CHANNEL_11;            /* PB0 on F303K8 */
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
    /* No SingleDiff / OffsetNumber on F3 */
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) { Error_Handler(); }
}

//TIM2  10 Hz tick ,SYSCLK = 48 MHz, APB1 timer clock = 48 MHz (APB1 prescaler=2, so TIM clock x2), Prescaler = 47999 ? 48MHz/48000 = 1 kHz, Period    = 99    ? 1kHz/100    = 10 Hz
static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 47999u;  /* 48MHz / 48000 = 1kHz */
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 99u;     /* 1kHz  / 100   = 10Hz */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { Error_Handler(); }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
        { Error_Handler(); }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
        { Error_Handler(); }
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance                    = USART2;
    huart2.Init.BaudRate               = 115200;
    huart2.Init.WordLength             = UART_WORDLENGTH_8B;
    huart2.Init.StopBits               = UART_STOPBITS_1;
    huart2.Init.Parity                 = UART_PARITY_NONE;
    huart2.Init.Mode                   = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
}

/**
  * @brief  GPIO init for STM32F303K8
  *         DB2 moved to PB1 (PA3 is now UART2 RX).
  *         PB0 is ADC input � do NOT configure as GPIO output.
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    // Default output levels
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
    /* PA0, PA1, PA4-PA9 � LCD bus (PA2=UART TX, PA3=UART RX, PA10=button) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5
                           | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
                     GPIO_PIN_RESET);
    /* PB1=LCD DB2, PB3=LD3, PB4=green, PB5=red*/
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4
                           | GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);

    /* PF0, PF1 LCD RS, R/W */
   __HAL_RCC_GPIOF_CLK_ENABLE();

		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

		GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
		GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull  = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* PA0, PA1, PA4-PA9 � LCD data bits (DB0, DB1, DB3-DB7) + E */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5
                        | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA10 � button input, pull-up */
    GPIO_InitStruct.Pin  = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB1=LCD DB2, PB3=LD3, PB4=green, PB5=red outputs */
    GPIO_InitStruct.Pin   = GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_4
                          | GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB0 analog input for ADC */
    GPIO_InitStruct.Pin  = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3); /* LD3 on F303K8 Nucleo = PB3 */
        { volatile uint32_t d = 200000u; while (d--) { __NOP(); } }
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif
