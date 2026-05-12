/*
 * ============================================================================
 * ESP32 Rehabilitation Arm Digital Twin - Slave Firmware
 * ============================================================================
 *
 * Role:
 *   Robotic arm actuator node and ESP-NOW receiver.
 *
 * Summary:
 *   This firmware runs on the ESP32 connected to the robotic arm servos. It
 *   receives compact ESP-NOW command packets from the master wearable node and
 *   applies the received duty cycles to three PWM servo channels. A connection
 *   timeout places the servos in a central fallback position if the wireless
 *   link is interrupted.
 *
 * Engineering notes:
 *   - PWM generation uses the ESP32 LEDC peripheral at 50 Hz.
 *   - The receiver validates packet size before copying data into shared state.
 *   - Safety behavior is implemented locally on the actuator node so the arm
 *     can enter a known fallback state even if the transmitter stops sending.
 *
 * Hardware:
 *   - ESP32 development board
 *   - 2x MG996R servos for shoulder motion
 *   - 1x MG90S servo for elbow motion
 *   - External servo power supply recommended
 *
 * Author:
 *   Academic embedded systems and rehabilitation engineering prototype.
 * ============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_timer.h"

/* ---------------------------------------------------------------------------
 * Servo and timing configuration
 * ---------------------------------------------------------------------------
 * Use conservative duty limits during first bring-up. Confirm servo direction,
 * mechanical stops, and current draw before expanding the motion range.
 */
#define SERVO_HOMBRO_H_PIN  25
#define SERVO_HOMBRO_V_PIN  26
#define SERVO_CODO_PIN      27

#define SERVO_FRECUENCIA    50
#define SERVO_DUTY_RES      LEDC_TIMER_16_BIT

#define SERVO_DUTY_MIN      2300
#define SERVO_DUTY_MAX      8192

#define TIMEOUT_SIN_DATOS_MS 2000

#define PERIODO_SALIDA       20
#define PERIODO_MONITOR     500

/* ---------------------------------------------------------------------------
 * Wireless command packet
 * ---------------------------------------------------------------------------
 * This structure must match the master firmware exactly.
 */
typedef struct {
    uint32_t duty_hombro_h;
    uint32_t duty_hombro_v;
    uint32_t duty_codo;
} paquete_servos_t;

paquete_servos_t g_paquete_recibido = {0, 0, 0};
volatile uint64_t g_ultimo_paquete_ms = 0;
volatile uint32_t g_paquetes_recibidos = 0;
volatile bool g_conexion_activa = false;

static uint64_t get_time_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static void espnow_recepcion_cb(const esp_now_recv_info_t *info,
                                const uint8_t *data,
                                int data_len)
{
    (void)info;

    if (data_len == sizeof(paquete_servos_t)) {
        memcpy(&g_paquete_recibido, data, sizeof(paquete_servos_t));
        g_ultimo_paquete_ms = get_time_ms();
        g_paquetes_recibidos++;
        g_conexion_activa = true;
    }
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
    printf("\nSlave MAC address\n");
    printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
           mac_propia[0], mac_propia[1], mac_propia[2],
           mac_propia[3], mac_propia[4], mac_propia[5]);
    printf("{0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X}\n\n",
           mac_propia[0], mac_propia[1], mac_propia[2],
           mac_propia[3], mac_propia[4], mac_propia[5]);

    esp_now_init();
    esp_now_register_recv_cb(espnow_recepcion_cb);

    printf("[INIT] ESP-NOW initialized. Waiting for master packets...\n");
}

static void configurar_servo_pwm(int gpio, ledc_channel_t canal, ledc_timer_t timer)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = timer,
        .duty_resolution = SERVO_DUTY_RES,
        .freq_hz = SERVO_FRECUENCIA,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ch_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = canal,
        .timer_sel = timer,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = gpio,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ch_conf);
}

static void init_servos(void)
{
    configurar_servo_pwm(SERVO_HOMBRO_H_PIN, LEDC_CHANNEL_0, LEDC_TIMER_0);
    configurar_servo_pwm(SERVO_HOMBRO_V_PIN, LEDC_CHANNEL_1, LEDC_TIMER_0);
    configurar_servo_pwm(SERVO_CODO_PIN, LEDC_CHANNEL_2, LEDC_TIMER_0);

    printf("[INIT] Servo PWM configured: Hh=GPIO%d, Hv=GPIO%d, Elbow=GPIO%d\n",
           SERVO_HOMBRO_H_PIN, SERVO_HOMBRO_V_PIN, SERVO_CODO_PIN);
}

static uint32_t limitar_duty(uint32_t duty)
{
    if (duty < SERVO_DUTY_MIN) return SERVO_DUTY_MIN;
    if (duty > SERVO_DUTY_MAX) return SERVO_DUTY_MAX;
    return duty;
}

static uint32_t duty_central(void)
{
    return (SERVO_DUTY_MIN + SERVO_DUTY_MAX) / 2;
}

static void escribir_servo(ledc_channel_t canal, uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, canal, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, canal);
}

void tarea_salida(void *parametro)
{
    (void)parametro;
    printf("[TASK] Actuator output task started\n");

    for (;;) {
        uint64_t ahora = get_time_ms();
        uint64_t tiempo_sin_datos = ahora - g_ultimo_paquete_ms;

        if (g_ultimo_paquete_ms == 0 || tiempo_sin_datos > TIMEOUT_SIN_DATOS_MS) {
            g_conexion_activa = false;
            uint32_t centro = duty_central();
            escribir_servo(LEDC_CHANNEL_0, centro);
            escribir_servo(LEDC_CHANNEL_1, centro);
            escribir_servo(LEDC_CHANNEL_2, centro);
        } else {
            escribir_servo(LEDC_CHANNEL_0, limitar_duty(g_paquete_recibido.duty_hombro_h));
            escribir_servo(LEDC_CHANNEL_1, limitar_duty(g_paquete_recibido.duty_hombro_v));
            escribir_servo(LEDC_CHANNEL_2, limitar_duty(g_paquete_recibido.duty_codo));
        }

        vTaskDelay(pdMS_TO_TICKS(PERIODO_SALIDA));
    }
}

void tarea_monitor(void *parametro)
{
    (void)parametro;
    printf("[TASK] Serial monitor task started\n");
    uint32_t paquetes_anterior = 0;

    for (;;) {
        uint32_t paq_actual = g_paquetes_recibidos;
        uint32_t paq_por_seg = (paq_actual - paquetes_anterior) * (1000 / PERIODO_MONITOR);
        paquetes_anterior = paq_actual;

        if (g_conexion_activa) {
            printf("RX OK | Duty[Hh:%5lu Hv:%5lu Co:%5lu] | %lu pkt/s | Total:%lu\n",
                   (unsigned long)g_paquete_recibido.duty_hombro_h,
                   (unsigned long)g_paquete_recibido.duty_hombro_v,
                   (unsigned long)g_paquete_recibido.duty_codo,
                   (unsigned long)paq_por_seg,
                   (unsigned long)paq_actual);
        } else {
            printf("NO LINK | Servos in center fallback position | Waiting for master...\n");
        }

        vTaskDelay(pdMS_TO_TICKS(PERIODO_MONITOR));
    }
}

void app_main(void)
{
    printf("\n===============================================\n");
    printf(" ESP32 Rehabilitation Arm - Slave Actuator Node\n");
    printf("===============================================\n\n");

    init_espnow();
    init_servos();

    uint32_t centro = duty_central();
    escribir_servo(LEDC_CHANNEL_0, centro);
    escribir_servo(LEDC_CHANNEL_1, centro);
    escribir_servo(LEDC_CHANNEL_2, centro);
    printf("[INIT] Servos moved to initial center position\n");

    printf("\nCreating FreeRTOS tasks...\n");

    xTaskCreate(tarea_salida, "Servo_Output", 2048, NULL, 3, NULL);
    xTaskCreate(tarea_monitor, "Monitor", 4096, NULL, 1, NULL);

    printf("Slave started. Waiting for master commands...\n\n");
}
