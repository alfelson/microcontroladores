#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#define PIN_LSA     GPIO_NUM_12   // Final carrera abierto (activo LOW)
#define PIN_LSC     GPIO_NUM_13   // Final carrera cerrado (activo HIGH)
#define PIN_FTC     GPIO_NUM_14   // Fotocelda (activo HIGH)
#define PIN_PP      GPIO_NUM_27   // Pulsador (activo LOW)
#define PIN_CA      GPIO_NUM_26   // Comando abrir (activo HIGH)

#define PIN_MA      GPIO_NUM_25   // Motor abrir
#define PIN_MC      GPIO_NUM_33   // Motor cerrar
#define PIN_LAMP    GPIO_NUM_32   // Lámpara

#define TIMER_PERIOD_US   (50 * 1000)
#define MAX_TICKS_3MIN    3600   // 3 minutos = 3*60*1000/50

static const char *TAG = "FSM_PORTON";

// Estados
typedef enum {
    EST_INIT_CONFIG = 0,
    EST_CERRANDO,
    EST_ABRIENDO,
    EST_ABIERTO,
    EST_CERRADO,
    EST_PARADO,
    EST_ERROR
} estado_t;

// Errores
typedef enum {
    ERR_OK = 0,
    ERR_DLSA,
    ERR_TIMEOUT
} error_t;

// Variables globales
static estado_t estado_actual = EST_INIT_CONFIG;
static error_t error_code = ERR_OK;
static uint32_t tick50ms;
static esp_timer_handle_t timer_handle;

// Estructura de IO
typedef struct {
    uint8_t lsa  : 1;
    uint8_t lsc  : 1;
    uint8_t ftc  : 1;
    uint8_t pp   : 1;
    uint8_t ca   : 1;
    uint8_t ma   : 1;
    uint8_t mc   : 1;
    uint8_t lamp : 3;
} io_t;

static io_t io;

// --- Callback del temporizador de 50 ms ---
static void IRAM_ATTR timer_cb(void* arg) {
    tick50ms++;
    io.lsa = (gpio_get_level(PIN_LSA) == 0);  // activo LOW
    io.lsc = (gpio_get_level(PIN_LSC) == 1);  // activo HIGH
    io.ftc = (gpio_get_level(PIN_FTC) == 1);
    io.pp  = (gpio_get_level(PIN_PP)  == 0);
    io.ca  = (gpio_get_level(PIN_CA)  == 1);
}

// --- Configuración de hardware ---
static void setup_hardware(void) {
    gpio_config_t in_conf = {
        .pin_bit_mask = (1ULL<<PIN_LSA)|(1ULL<<PIN_LSC)|(1ULL<<PIN_FTC)|(1ULL<<PIN_PP)|(1ULL<<PIN_CA),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&in_conf);

    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL<<PIN_MA)|(1ULL<<PIN_MC)|(1ULL<<PIN_LAMP),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&out_conf);

    esp_timer_create_args_t tcfg = { .callback = timer_cb, .name = "tick50ms" };
    esp_timer_create(&tcfg, &timer_handle);
    esp_timer_start_periodic(timer_handle, TIMER_PERIOD_US);

    ESP_LOGI(TAG, "Hardware y timer configurados");
}

// --- Control del motor y lámpara ---
static void motor_parar()  { gpio_set_level(PIN_MA, 0); gpio_set_level(PIN_MC, 0); }
static void motor_abrir()  { gpio_set_level(PIN_MA, 1); gpio_set_level(PIN_MC, 0); }
static void motor_cerrar() { gpio_set_level(PIN_MA, 0); gpio_set_level(PIN_MC, 1); }
static void lampara(uint8_t e) { io.lamp = e; gpio_set_level(PIN_LAMP, e ? 1 : 0); }

// --- Estados ---
static estado_t f_init_config(void) {
    ESP_LOGI(TAG, "INIT_CONFIG");
    motor_parar();
    lampara(0);
    tick50ms = 0;
    error_code = ERR_OK;

    if (io.lsa && io.lsc) {
        error_code = ERR_DLSA;
        return EST_ERROR;
    }

    // Si ambos apagados → CERRANDO
    if (!io.lsa && !io.lsc) return EST_CERRANDO;

    if (io.lsc) return EST_CERRADO;
    if (io.lsa) return EST_ABIERTO;
    return EST_CERRADO;
}

static estado_t f_cerrando(void) {
    ESP_LOGI(TAG, "CERRANDO");
    motor_cerrar();
    lampara(2);
    tick50ms = 0;

    while (true) {
        if (io.lsa && io.lsc) { error_code = ERR_DLSA; return EST_ERROR; }
        if (io.ftc)           return EST_ABRIENDO;  // Fotocelda → abrir
        if (io.lsc)           return EST_CERRADO;   // Límite cerrado → stop
        if (tick50ms > MAX_TICKS_3MIN) { error_code = ERR_TIMEOUT; return EST_ERROR; }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static estado_t f_abriendo(void) {
    ESP_LOGI(TAG, "ABRIENDO");
    motor_abrir();
    lampara(2);
    tick50ms = 0;

    while (true) {
        if (io.lsa && io.lsc) { error_code = ERR_DLSA; return EST_ERROR; }
        if (io.lsa)           return EST_ABIERTO;  // Llegó al tope abierto
        if (io.pp)            return EST_PARADO;
        if (tick50ms > MAX_TICKS_3MIN) { error_code = ERR_TIMEOUT; return EST_ERROR; }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static estado_t f_abierto(void) {
    ESP_LOGI(TAG, "ABIERTO");
    motor_parar();
    lampara(0);
    tick50ms = 0;

    while (true) {
        // Mantenerse en ABIERTO mientras FTC esté activa
        if (io.ftc) { tick50ms = 0; vTaskDelay(pdMS_TO_TICKS(100)); continue; }

        // Si FTC no está activa y se presiona PP o CA → CERRANDO
        if ((io.pp || io.ca) && !io.ftc) return EST_CERRANDO;

        if (io.lsa && io.lsc) { error_code = ERR_DLSA; return EST_ERROR; }
        if (tick50ms > MAX_TICKS_3MIN) { error_code = ERR_TIMEOUT; return EST_ERROR; }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static estado_t f_cerrado(void) {
    ESP_LOGI(TAG, "CERRADO");
    motor_parar();
    lampara(0);

    while (true) {
        if (io.pp || io.ca) return EST_ABRIENDO;
        if (!io.lsa && !io.lsc) return EST_CERRANDO;
        if (io.lsa && io.lsc) { error_code = ERR_DLSA; return EST_ERROR; }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static estado_t f_parado(void) {
    ESP_LOGI(TAG, "PARADO");
    motor_parar();
    lampara(3);

    while (true) {
        if (io.pp && !io.lsc) return EST_CERRANDO;
        if (io.lsa && !io.ftc) return EST_ABRIENDO;
        if (io.lsa && io.lsc) { error_code = ERR_DLSA; return EST_ERROR; }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static estado_t f_error(void) {
    ESP_LOGE(TAG, "ERROR %d", error_code);
    motor_parar();
    lampara(1);

    if (error_code == ERR_DLSA) {
        while (io.lsa && io.lsc) vTaskDelay(pdMS_TO_TICKS(100));
        return EST_INIT_CONFIG;
    }

    while (1) vTaskDelay(pdMS_TO_TICKS(1000)); // Error grave
}

// --- Bucle principal ---
void app_main(void) {
    setup_hardware();

    while (true) {
        switch (estado_actual) {
            case EST_INIT_CONFIG: estado_actual = f_init_config(); break;
            case EST_CERRANDO:    estado_actual = f_cerrando();    break;
            case EST_ABRIENDO:    estado_actual = f_abriendo();    break;
            case EST_ABIERTO:     estado_actual = f_abierto();     break;
            case EST_CERRADO:     estado_actual = f_cerrado();     break;
            case EST_PARADO:      estado_actual = f_parado();      break;
            case EST_ERROR:       estado_actual = f_error();       break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
