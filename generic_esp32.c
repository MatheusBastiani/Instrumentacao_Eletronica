#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

static const char *TAG = "EEL7300_FULL";

/* ============================================================================
 * 1. CONFIGURACAO DE PINOS E BARRAMENTOS (ESP32 WROOM-32)
 * ============================================================================ */
#define I2C_PORT_NUM           I2C_NUM_0
#define I2C_SDA_GPIO           GPIO_NUM_21     // D21 (SDA)
#define I2C_SCL_GPIO           GPIO_NUM_22     // D22 (SCL)
#define I2C_FREQ_HZ            100000

#define DS18B20_GPIO           GPIO_NUM_4      // D4 (1-Wire com pull-up 4k7 em 3V3)

/* Endereco I2C do ADS1115 (ADDR -> GND = 0x48) */
#define ADS1115_ADDR           0x48

/* Registradores do ADS1115 */
#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG     0x01

/* Bits do Config Register */
#define ADS_OS_SINGLE          (1U << 15)

/* MUX (bits 14:12) */
#define ADS_MUX_AIN0_GND       (0x4U << 12)  // pH (AIN0 -> PH_DIV)
#define ADS_MUX_AIN2_AIN3      (0x3U << 12)  // CE diferencial (AIN2 [CE_FILT] - AIN3 [VREF])
#define ADS_MUX_AIN3_GND       (0x7U << 12)  // Leitura direta de VREF

/* PGA (bits 11:9) */
#define ADS_PGA_4_096V         (0x1U << 9)   // +/- 4.096 V -> LSB = 125.0 uV

#define ADS_MODE_SINGLE        (1U << 8)
#define ADS_DR_128SPS          (0x4U << 5)
#define ADS_COMP_DISABLE       0x0003U

/* ============================================================================
 * 2. PARAMETROS DE CALIBRACAO E MODELO
 * ============================================================================ */
typedef enum {
    CE_RANGE_LOW = 0,   // JP1 fechado: ganho ~5,02 (0,10 - 1,00 mS/cm)
    CE_RANGE_HIGH = 1   // JP1 aberto:  ganho 1,00 (1,00 - 5,00 mS/cm)
} ce_range_t;

typedef struct {
    /* Condutividade Eletrica (CE) */
    double k_cell_eff;         // Constante efetiva da celula K_ef (cm^-1)
    double alpha_ce_per_c;     // Coeficiente termico alfa (1/C) para 25 C
    double ce_a_low;           // Sensibilidade faixa LOW (V/mS)
    double ce_b_low;           // Offset residual no ar faixa LOW (V)
    double ce_a_high;          // Sensibilidade faixa HIGH (V/mS)
    double ce_b_high;          // Offset residual no ar faixa HIGH (V)

    /* pH */
    double ph_divider_gain;    // Divisor R12=10k, R13=10k -> 2.0
    bool   ph_invert_slope;    // true para PH-4502C
    double ph_ref_val;         // Tampao de referencia (ex.: 7.00)
    double ph_ref_v_mod;       // Tensao medida no tampao (ajuste com 'c' no serial)

    /* Temperatura */
    double temp_offset_c;
} system_cal_t;

static system_cal_t g_cal = {
    .k_cell_eff               = 1.000,
    .alpha_ce_per_c           = 0.019,

    .ce_a_low                 = 0.753,
    .ce_b_low                 = 0.000,
    .ce_a_high                = 0.150,
    .ce_b_high                = 0.000,

    .ph_divider_gain          = 2.000,
    .ph_invert_slope          = true,
    .ph_ref_val               = 7.00,
    .ph_ref_v_mod             = 3.272,

    .temp_offset_c            = 0.0
};

static ce_range_t g_ce_range = CE_RANGE_HIGH; // Inicia em ganho 1 (JP1 aberto)
static double g_last_ph_v_mod = 3.272;
static double g_last_ce_v_abs = 0.000;
static double g_last_temp_c   = 25.0;

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_ads_dev = NULL;

/* ============================================================================
 * 3. DRIVER I2C MASTER (ESP-IDF v6.0) E ADS1115
 * ============================================================================ */
static esp_err_t i2c_master_init_bus(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_NUM,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_io_num = I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (err != ESP_OK) return err;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADS1115_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    return i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_ads_dev);
}

static esp_err_t ads1115_write_reg(uint8_t reg, uint16_t val)
{
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (uint8_t)((val >> 8) & 0xFFU);
    buf[2] = (uint8_t)(val & 0xFFU);
    return i2c_master_transmit(s_ads_dev, buf, 3, 100);
}

static esp_err_t ads1115_read_reg(uint8_t reg, uint16_t *out_val)
{
    uint8_t data[2] = {0, 0};
    esp_err_t err = i2c_master_transmit_receive(s_ads_dev, &reg, 1, data, 2, 100);
    if (err != ESP_OK) return err;
    *out_val = (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
    return ESP_OK;
}

static esp_err_t ads1115_read_single_shot(uint16_t mux_bits,
                                          int16_t *raw_out, double *volts_out,
                                          bool *saturated_out)
{
    uint16_t cfg = ADS_OS_SINGLE | mux_bits | ADS_PGA_4_096V |
                   ADS_MODE_SINGLE | ADS_DR_128SPS | ADS_COMP_DISABLE;

    esp_err_t err = ads1115_write_reg(ADS1115_REG_CONFIG, cfg);
    if (err != ESP_OK) return err;

    bool ready = false;
    for (int i = 0; i < 25; i++) {
        vTaskDelay(pdMS_TO_TICKS(1));
        uint16_t status = 0;
        if (ads1115_read_reg(ADS1115_REG_CONFIG, &status) == ESP_OK) {
            if (status & ADS_OS_SINGLE) {
                ready = true;
                break;
            }
        }
    }
    if (!ready) {
        return ESP_ERR_TIMEOUT;
    }

    uint16_t raw_u16 = 0;
    err = ads1115_read_reg(ADS1115_REG_CONVERSION, &raw_u16);
    if (err != ESP_OK) return err;

    int16_t raw = (int16_t)raw_u16;
    *raw_out = raw;
    *volts_out = (double)raw * (4.096 / 32768.0); // 125 uV/bit (+/- 4.096 V)
    *saturated_out = (raw == INT16_MAX || raw == INT16_MIN);
    return ESP_OK;
}

/* ============================================================================
 * 4. DRIVER 1-WIRE / DS18B20 (12 BITS = 0.0625 C)
 * ============================================================================ */
static portMUX_TYPE s_ow_spinlock = portMUX_INITIALIZER_UNLOCKED;

static void ow_init_gpio(void)
{
    gpio_reset_pin(DS18B20_GPIO);
    gpio_set_pull_mode(DS18B20_GPIO, GPIO_FLOATING);
    gpio_set_direction(DS18B20_GPIO, GPIO_MODE_INPUT);
}

static inline void ow_line_low(void)
{
    gpio_set_direction(DS18B20_GPIO, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(DS18B20_GPIO, 0);
}

static inline void ow_line_release(void)
{
    gpio_set_level(DS18B20_GPIO, 1);
    gpio_set_direction(DS18B20_GPIO, GPIO_MODE_INPUT);
}

static inline int ow_line_read(void)
{
    return gpio_get_level(DS18B20_GPIO);
}

static bool ow_reset_and_presence(void)
{
    int presence = 1;
    taskENTER_CRITICAL(&s_ow_spinlock);
    ow_line_low();
    esp_rom_delay_us(480);
    ow_line_release();
    esp_rom_delay_us(70);
    presence = ow_line_read();
    taskEXIT_CRITICAL(&s_ow_spinlock);

    esp_rom_delay_us(410);
    return (presence == 0);
}

static void ow_write_bit(int bit)
{
    taskENTER_CRITICAL(&s_ow_spinlock);
    ow_line_low();
    if (bit) {
        esp_rom_delay_us(6);
        ow_line_release();
        esp_rom_delay_us(64);
    } else {
        esp_rom_delay_us(60);
        ow_line_release();
        esp_rom_delay_us(10);
    }
    taskEXIT_CRITICAL(&s_ow_spinlock);
}

static int ow_read_bit(void)
{
    int bit = 0;
    taskENTER_CRITICAL(&s_ow_spinlock);
    ow_line_low();
    esp_rom_delay_us(3);
    ow_line_release();
    esp_rom_delay_us(10);
    bit = ow_line_read();
    taskEXIT_CRITICAL(&s_ow_spinlock);

    esp_rom_delay_us(50);
    return bit;
}

static void ow_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        ow_write_bit((byte >> i) & 0x01U);
    }
}

static uint8_t ow_read_byte(void)
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        if (ow_read_bit()) {
            byte |= (uint8_t)(1U << i);
        }
    }
    return byte;
}

static uint8_t ow_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (int j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01U;
            crc >>= 1;
            if (mix) crc ^= 0x8CU;
            inbyte >>= 1;
        }
    }
    return crc;
}

static bool ds18b20_read_temp_c(double *temp_out_c)
{
    if (!ow_reset_and_presence()) {
        return false;
    }
    ow_write_byte(0xCC);
    ow_write_byte(0x44);

    vTaskDelay(pdMS_TO_TICKS(760));

    if (!ow_reset_and_presence()) {
        return false;
    }
    ow_write_byte(0xCC);
    ow_write_byte(0xBE);

    uint8_t scratch[9];
    for (int i = 0; i < 9; i++) {
        scratch[i] = ow_read_byte();
    }

    if (ow_crc8(scratch, 8) != scratch[8]) {
        return false;
    }

    int16_t raw = (int16_t)(((uint16_t)scratch[1] << 8) | (uint16_t)scratch[0]);
    *temp_out_c = ((double)raw * 0.0625) + g_cal.temp_offset_c;
    return true;
}

/* ============================================================================
 * 5. CALCULOS DE pH, CONDUTIVIDADE (CE) E COMANDOS SERIAIS
 * ============================================================================ */
static double compute_ph(double v_mod, double temp_c)
{
    double t_kelvin = temp_c + 273.15;
    double slope_v_per_ph = 0.0001984 * t_kelvin;
    double delta_v = g_cal.ph_invert_slope ? (g_cal.ph_ref_v_mod - v_mod)
                                           : (v_mod - g_cal.ph_ref_v_mod);
    return g_cal.ph_ref_val + (delta_v / slope_v_per_ph);
}

static void compute_ce(double v_diff_volts, double temp_c,
                       double *g_ms_out, double *ce_t_out, double *ce_25_out)
{
    double v_abs = fabs(v_diff_volts);
    g_last_ce_v_abs = v_abs;

    double a = (g_ce_range == CE_RANGE_LOW) ? g_cal.ce_a_low : g_cal.ce_a_high;
    double b = (g_ce_range == CE_RANGE_LOW) ? g_cal.ce_b_low : g_cal.ce_b_high;

    // A tensao DIMINUI quando a condutividade AUMENTA (b - v_abs)
    double g_ms = (b - v_abs) / a;
    if (g_ms < 0.0) g_ms = 0.0;

    double ce_t = g_ms * g_cal.k_cell_eff;
    double denom = 1.0 + g_cal.alpha_ce_per_c * (temp_c - 25.0);
    double ce_25 = (denom > 0.1) ? (ce_t / denom) : ce_t;

    *g_ms_out = g_ms;
    *ce_t_out = ce_t;
    *ce_25_out = ce_25;

}
static void check_serial_commands(void)
{
int ch = getchar();
if (ch == EOF) return;
if (ch == 'c' || ch == 'C') {
g_cal.ph_ref_v_mod = g_last_ph_v_mod;
ESP_LOGI(TAG, ">>> CALIBRADO pH: %.2f definido em V_mod = %.4f V (Temp = %.2f C)",
g_cal.ph_ref_val, g_cal.ph_ref_v_mod, g_last_temp_c);
} else if (ch == '7') {
g_cal.ph_ref_val = 7.00;
ESP_LOGI(TAG, ">>> Tampao pH 7.00 selecionado (envie 'c' para gravar)");
} else if (ch == '4') {
g_cal.ph_ref_val = 4.00;
ESP_LOGI(TAG, ">>> Tampao pH 4.00 selecionado (envie 'c' para gravar)");
} else if (ch == '9') {
g_cal.ph_ref_val = 9.18;
ESP_LOGI(TAG, ">>> Tampao pH 9.18 selecionado (envie 'c' para gravar)");
} else if (ch == 'r' || ch == 'R') {
g_ce_range = (g_ce_range == CE_RANGE_HIGH) ? CE_RANGE_LOW : CE_RANGE_HIGH;
ESP_LOGI(TAG, ">>> Faixa CE alterada para: %s",
(g_ce_range == CE_RANGE_LOW) ? "LOW (JP1 fechado, ganho ~5)" : "HIGH (JP1 aberto, ganho 1)");
} else if (ch == 'z' || ch == 'Z') {
if (g_ce_range == CE_RANGE_LOW) {
g_cal.ce_b_low = g_last_ce_v_abs;
} else {
g_cal.ce_b_high = g_last_ce_v_abs;
}
ESP_LOGI(TAG, ">>> ZERO CE (%s) gravado em |V_diff| = %.4f V",
(g_ce_range == CE_RANGE_LOW) ? "LOW" : "HIGH", g_last_ce_v_abs);
}
}
/* ============================================================================
•
	6. MAIN
• ============================================================================ */
void app_main(void)
{
ow_init_gpio();
ESP_ERROR_CHECK(i2c_master_init_bus());ESP_LOGI(TAG, "Sistema iniciado (Temp + pH + CE + VREF).");
ESP_LOGI(TAG, "Comandos: '7'/'4'/'9' tampao | 'c' calibra pH | 'r' faixa CE | 'z' zera CE no ar.");while (1) {
check_serial_commands();/* 1. Temperatura (DS18B20 no GPIO 4) */
double temp_c = 25.0;
bool temp_ok = ds18b20_read_temp_c(&temp_c);
if (temp_ok) {
g_last_temp_c = temp_c;
} else {
temp_c = g_last_temp_c;
}/* 2. pH (ADS1115 AIN0) */
int16_t raw_ph = 0;
double v_adc_ph = 0.0;
bool sat_ph = false;
esp_err_t err_ph = ads1115_read_single_shot(ADS_MUX_AIN0_GND, &raw_ph, &v_adc_ph, &sat_ph);/* 3. Leitura direta de VREF (ADS1115 AIN3) */
int16_t raw_vref = 0;
double v_ref = 0.0;
bool sat_vref = false;
esp_err_t err_vref = ads1115_read_single_shot(ADS_MUX_AIN3_GND, &raw_vref, &v_ref, &sat_vref);/* 4. Condutividade Eletrica (ADS1115 diferencial AIN2 - AIN3) */
int16_t raw_ce = 0;
double v_diff_ce = 0.0;
bool sat_ce = false;
esp_err_t err_ce = ads1115_read_single_shot(ADS_MUX_AIN2_AIN3, &raw_ce, &v_diff_ce, &sat_ce);if (err_ph == ESP_OK && err_vref == ESP_OK && err_ce == ESP_OK) {
double v_mod = v_adc_ph * g_cal.ph_divider_gain;
g_last_ph_v_mod = v_mod;
double ph_val = compute_ph(v_mod, temp_c);double g_ms = 0.0, ce_t = 0.0, ce_25 = 0.0;
compute_ce(v_diff_ce, temp_c, &g_ms, &ce_t, &ce_25);printf("TEMP: %s (%.2f C) | pH: %.2f (V_mod=%.3f V) | V_ref: %.3f V | CE_25: %.3f mS/cm (V_diff=%.4f V%s) [%s]\n",
temp_ok ? "OK" : "ERR",
temp_c,
ph_val,
v_mod,
v_ref,
ce_25,
v_diff_ce,
sat_ce ? " SAT" : "",
(g_ce_range == CE_RANGE_LOW) ? "LOW" : "HIGH");
} else {
printf("TEMP: %s (%.2f C) | ERRO I2C (pH=%s, VREF=%s, CE=%s)\n",
temp_ok ? "OK" : "ERR",
temp_c,
esp_err_to_name(err_ph),
esp_err_to_name(err_vref),
esp_err_to_name(err_ce));
}vTaskDelay(pdMS_TO_TICKS(1000));
}
}
