/*
 * ============================================================================
 * ESP32 Rehabilitation Arm Digital Twin - Master Firmware
 * ============================================================================
 *
 * Role:
 *   Wearable sensor node and ESP-NOW transmitter.
 *
 * Summary:
 *   This firmware runs on the ESP32 mounted on the user/wearable sensor system.
 *   It samples an MPU6050 IMU through I2C and an analog potentiometer through
 *   the ESP32 ADC, estimates shoulder and elbow motion, maps those values to
 *   servo PWM duty cycles, and transmits the resulting command packet to the
 *   robotic arm controller using ESP-NOW.
 *
 * Engineering notes:
 *   - FreeRTOS separates acquisition, processing, wireless transmission, and
 *     serial monitoring into independent periodic tasks.
 *   - The transmitted packet contains servo duty cycles rather than raw sensor
 *     data, keeping the receiver simple and deterministic.
 *   - Calibration constants are intentionally grouped near the top of the file
 *     so the prototype can be tuned for different users, linkages, and servo
 *     travel limits.
 *
 * Hardware:
 *   - ESP32 development board
 *   - MPU6050 / MPU6000 IMU
 *   - 10 kOhm potentiometer
 *   - Second ESP32 receiver controlling the robotic arm servos
 *
 * Author:
 *   Academic embedded systems and rehabilitation engineering prototype.
 * ============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"

/* ---------------------------------------------------------------------------
 * ESP-NOW peer configuration
 * ---------------------------------------------------------------------------
 * Replace this address with the station MAC address printed by the slave
 * firmware at boot. Both boards must use the same ESP-NOW packet structure.
 */
static uint8_t MAC_ESCLAVO[] = {0x6C, 0xC8, 0x40, 0x56, 0x4E, 0xD0};

/* ---------------------------------------------------------------------------
 * Calibration constants
 * ---------------------------------------------------------------------------
 * Tune these values during bench testing. Start with conservative servo limits
 * to avoid mechanical interference, then expand the range gradually.
 */

#define MPU_ADDR            0x68
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define I2C_FREQ_HZ         400000

#define POT_ADC_UNIT        ADC_UNIT_1
#define POT_ADC_CHANNEL     ADC_CHANNEL_6   // GPIO34

#define SERVO_DUTY_MIN      1638
#define SERVO_DUTY_MAX      8192

#define HOMBRO_H_ANGULO_MIN -60.0
#define HOMBRO_H_ANGULO_MAX  60.0
#define HOMBRO_V_ANGULO_MIN -60.0
#define HOMBRO_V_ANGULO_MAX  60.0

#define POT_MIN             600
#define POT_MAX             4095

#define CODO_ANGULO_MIN     0.0
#define CODO_ANGULO_MAX     135.0

#define ALFA_FILTRO         0.75

#define INVERTIR_HOMBRO_H   0
#define INVERTIR_HOMBRO_V   0
#define INVERTIR_CODO       0

#define PERIODO_ADQUISICION    20
#define PERIODO_PROCESAMIENTO  20
#define PERIODO_ENVIO          20
#define PERIODO_MONITOR       500

/* ---------------------------------------------------------------------------
 * Wireless command packet
 * ---------------------------------------------------------------------------
 * This structure must match the slave firmware exactly. It uses three 32-bit
 * duty values, producing a compact 12-byte ESP-NOW payload.
 */
typedef struct {
    uint32_t duty_hombro_h;
    uint32_t duty_hombro_v;
    uint32_t duty_codo;
} paquete_servos_t;

/* ---------------------------------------------------------------------------
 * Shared state
 * ---------------------------------------------------------------------------
 * The prototype uses simple global variables shared between periodic tasks.
 * For a production system, protect shared data with queues, mutexes, or double
 * buffering when timing analysis requires stricter consistency guarantees.
 */
int16_t g_accel_x = 0;
int16_t g_accel_y = 0;
int16_t g_accel_z = 0;
int g_pot_raw = 0;

float g_pitch = 0.0;
float g_roll = 0.0;
float g_pitch_filtrado = 0.0;
float g_roll_filtrado = 0.0;

paquete_servos_t g_paquete = {0, 0, 0};
volatile bool g_envio_exitoso = false;

adc_oneshot_unit_handle_t adc_handle;

static void init_i2c(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    printf("[INIT] I2C initialized on SDA=%d, SCL=%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
}

static void mpu_write_byte(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | 0, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
}

static void mpu_read_bytes(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | 0, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU_ADDR << 1) | 1, true);

    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }

    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
}

static void init_mpu(void)
{
    mpu_write_byte(0x6B, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));
    mpu_write_byte(0x1C, 0x00);
    printf("[INIT] MPU6050 initialized in +/-2g accelerometer range\n");
}

static void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = POT_ADC_UNIT,
    };
    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    adc_oneshot_config_channel(adc_handle, POT_ADC_CHANNEL, &chan_config);
    printf("[INIT] ADC configured on channel %d (GPIO34)\n", POT_ADC_CHANNEL);
}

static void espnow_envio_cb(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;
    g_envio_exitoso = (status == ESP_NOW_SEND_SUCCESS);
}

static void init_espnow(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_netif_init();
    esp_event_loop_create_default();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    uint8_t mac_propia[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac_propia);
    printf("[INIT] Master MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           mac_propia[0], mac_propia[1], mac_propia[2],
           mac_propia[3], mac_propia[4], mac_propia[5]);

    esp_now_init();
    esp_now_register_send_cb(espnow_envio_cb);

    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, MAC_ESCLAVO, 6);
    peer_info.channel = 0;
    peer_info.encrypt = false;
    esp_now_add_peer(&peer_info);

    printf("[INIT] ESP-NOW initialized\n");
    printf("[INIT] Slave target MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
           MAC_ESCLAVO[0], MAC_ESCLAVO[1], MAC_ESCLAVO[2],
           MAC_ESCLAVO[3], MAC_ESCLAVO[4], MAC_ESCLAVO[5]);
}

static float mapear(float valor, float in_min, float in_max, float out_min, float out_max)
{
    return (valor - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static float limitar(float valor, float minimo, float maximo)
{
    if (valor < minimo) return minimo;
    if (valor > maximo) return maximo;
    return valor;
}

static uint32_t angulo_a_duty(float angulo, float ang_min, float ang_max)
{
    float duty_f = mapear(angulo, ang_min, ang_max, (float)SERVO_DUTY_MIN, (float)SERVO_DUTY_MAX);
    return (uint32_t)limitar(duty_f, (float)SERVO_DUTY_MIN, (float)SERVO_DUTY_MAX);
}

void tarea_adquisicion(void *parametro)
{
    (void)parametro;
    printf("[TASK] Acquisition task started\n");
    uint8_t buffer[6];

    for (;;) {
        mpu_read_bytes(0x3B, buffer, 6);

        g_accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
        g_accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
        g_accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);

        adc_oneshot_read(adc_handle, POT_ADC_CHANNEL, &g_pot_raw);

        vTaskDelay(pdMS_TO_TICKS(PERIODO_ADQUISICION));
    }
}

void tarea_procesamiento(void *parametro)
{
    (void)parametro;
    printf("[TASK] Processing task started\n");

    for (;;) {
        float ax = (float)g_accel_x;
        float ay = (float)g_accel_y;
        float az = (float)g_accel_z;

        float pitch_nuevo = atan2(ax, sqrt(ay * ay + az * az)) * (180.0 / M_PI);
        float roll_nuevo = atan2(ay, sqrt(ax * ax + az * az)) * (180.0 / M_PI);

        g_pitch_filtrado = ALFA_FILTRO * g_pitch_filtrado + (1.0 - ALFA_FILTRO) * pitch_nuevo;
        g_roll_filtrado = ALFA_FILTRO * g_roll_filtrado + (1.0 - ALFA_FILTRO) * roll_nuevo;

        g_pitch = pitch_nuevo;
        g_roll = roll_nuevo;

        float hombro_h_ang = limitar(g_roll_filtrado, HOMBRO_H_ANGULO_MIN, HOMBRO_H_ANGULO_MAX);
        float hombro_v_ang = limitar(g_pitch_filtrado, HOMBRO_V_ANGULO_MIN, HOMBRO_V_ANGULO_MAX);

        if (INVERTIR_HOMBRO_H) hombro_h_ang = HOMBRO_H_ANGULO_MIN + HOMBRO_H_ANGULO_MAX - hombro_h_ang;
        if (INVERTIR_HOMBRO_V) hombro_v_ang = HOMBRO_V_ANGULO_MIN + HOMBRO_V_ANGULO_MAX - hombro_v_ang;

        g_paquete.duty_hombro_h = angulo_a_duty(hombro_h_ang, HOMBRO_H_ANGULO_MIN, HOMBRO_H_ANGULO_MAX);
        g_paquete.duty_hombro_v = angulo_a_duty(hombro_v_ang, HOMBRO_V_ANGULO_MIN, HOMBRO_V_ANGULO_MAX);

        float pot_limitado = limitar((float)g_pot_raw, (float)POT_MIN, (float)POT_MAX);
        float codo_ang = mapear(pot_limitado, (float)POT_MIN, (float)POT_MAX,
                                CODO_ANGULO_MIN, CODO_ANGULO_MAX);
        if (INVERTIR_CODO) codo_ang = CODO_ANGULO_MIN + CODO_ANGULO_MAX - codo_ang;

        g_paquete.duty_codo = angulo_a_duty(codo_ang, CODO_ANGULO_MIN, CODO_ANGULO_MAX);

        vTaskDelay(pdMS_TO_TICKS(PERIODO_PROCESAMIENTO));
    }
}

void tarea_envio(void *parametro)
{
    (void)parametro;
    printf("[TASK] ESP-NOW transmission task started\n");

    for (;;) {
        esp_err_t resultado = esp_now_send(MAC_ESCLAVO,
                                           (uint8_t *)&g_paquete,
                                           sizeof(paquete_servos_t));
        if (resultado != ESP_OK) {
            printf("[ERROR] ESP-NOW packet transmission failed\n");
        }

        vTaskDelay(pdMS_TO_TICKS(PERIODO_ENVIO));
    }
}

void tarea_monitor(void *parametro)
{
    (void)parametro;
    printf("[TASK] Serial monitor task started\n");

    for (;;) {
        printf("PITCH:%6.1f (f:%6.1f) | ROLL:%6.1f (f:%6.1f) | POT:%4d | "
               "Duty[Hh:%5lu Hv:%5lu Co:%5lu] | TX:%s\n",
               g_pitch, g_pitch_filtrado,
               g_roll, g_roll_filtrado,
               g_pot_raw,
               (unsigned long)g_paquete.duty_hombro_h,
               (unsigned long)g_paquete.duty_hombro_v,
               (unsigned long)g_paquete.duty_codo,
               g_envio_exitoso ? "OK" : "FAIL");

        vTaskDelay(pdMS_TO_TICKS(PERIODO_MONITOR));
    }
}

void app_main(void)
{
    printf("\n==============================================\n");
    printf(" ESP32 Rehabilitation Arm - Master Sensor Node\n");
    printf("==============================================\n\n");

    init_i2c();
    init_mpu();
    init_adc();
    init_espnow();

    printf("\nCreating FreeRTOS tasks...\n");

    xTaskCreate(tarea_adquisicion, "Acquisition", 4096, NULL, 4, NULL);
    xTaskCreate(tarea_procesamiento, "Processing", 4096, NULL, 3, NULL);
    xTaskCreate(tarea_envio, "ESP_NOW_TX", 4096, NULL, 2, NULL);
    xTaskCreate(tarea_monitor, "Monitor", 4096, NULL, 1, NULL);

    printf("Master started. Streaming servo commands to slave...\n\n");
}
