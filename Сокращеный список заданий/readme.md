# Список л/р

Все лабораторные работы можно выполнить в эмуляторе wokwi.com

## 1 Вывод информации в консоль и реализация алгоритма - Светофор
Вывод в консоль
```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    int i = 0;
    printf("Hello world!\n");
    while (1)
    {
        printf("This program runs since %d seconds.\n", i++); 
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
```

В некоторых случаях одного светодиода на плате может оказаться мало. Допустим, мы хотим реализовать простой индикатор, имитирующий цвета светофора:  

- **Зелёный** – всё в порядке.  
- **Жёлтый** – какой-то параметр (например, измеряемая температура) приблизился к критическому значению и нужно предпринять шаги по нейтрализации надвигающейся угрозы.  
- **Красный** – критическая ситуация, требующая немедленного реагирования.  

В этом случае понадобится **три порта GPIO**, работающих на выход, к каждому из которых будет подключен светодиод.  

> [!WARNING]  
> ⚠️ Каждый светодиод должен подключаться через **токоограничивающий резистор**.  
> Необходимо учитывать максимальный ток выхода GPIO. Для большинства микроконтроллеров типовое максимальное значение тока составляет 20 мА (у ESP32 реальное значение выше).  
> Ток светодиода (`Iled`) обычно находится в диапазоне 2…20 мА.  

---
Если же мы используем какой-либо модуль, то они уже содержат токоограничивающий резистор (и дополнительный транзисторный ключ), так что их можно подключать напрямую.
Программа, в которой будем управлять несколькими светодиодами будет следующая:

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// пины подключения светодиодов
#define     LED_RED     12
#define     LED_YELLOW  13
#define     LED_GREEN   11
#define     GPIO_PINS   ((1ULL << LED_RED) | (1ULL << LED_YELLOW) | (1ULL << LED_GREEN))

void app_main(void)
{
    // объявление структуры конфигурации
    gpio_config_t io_conf = {};

    // задание необходимых свойств
    io_conf.pin_bit_mask = GPIO_PINS;             // порты
    io_conf.mode = GPIO_MODE_OUTPUT;              // режим работы - выход
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;     // нет подтягивающего резистора
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // нет стягивающего резистора
    io_conf.intr_type = GPIO_INTR_DISABLE;        // прерывания отключены

    // установка конфигурации портов
    gpio_config(&io_conf);

    // счетчик итераций
    uint8_t ticks = 0;

    while (1)
    {
        switch(++ticks % 8)
        {
            case 0:
                gpio_set_level(LED_RED, 0);
                gpio_set_level(LED_YELLOW, 0);
                gpio_set_level(LED_GREEN, 0);
                break;
            case 1:
                gpio_set_level(LED_RED, 1);
                break;
            case 2:
                gpio_set_level(LED_YELLOW, 1);
                break;
            case 3:
                gpio_set_level(LED_RED, 0);
                break;
            case 4:
                gpio_set_level(LED_GREEN, 1);
                break;
            case 5:
                gpio_set_level(LED_YELLOW, 0);
                break;
            case 6:
                gpio_set_level(LED_RED, 1);
                break;
            case 7:
                gpio_set_level(LED_GREEN, 0);
                break;
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}
```
> [!TIP]
> **Задания**
>
> 1. Объеденить программы в одну (вывод сообщения в консоль и светофор)
> 2. В представленной программе НЕ реализован алгоритм Светофора, необходимо это исправить
>    👉 Алгоритм по ГОСТ Р 52289-2019:   
>    🔴🟡🟢 Красный → Красный+Жёлтый (≤2 с) → Зелёный → (опционально мигание зелёного ~3 с) → Жёлтый (3 с) → Красный  
>    Для упрощения временем можно принебречь  

## 2 Подключение сенсора/датчика

Вывести данные с любого сенсора - некоторые реальные приеры можно посмотреть в 2.1 sensors 

# 3 Mqtt
Выполнить задание 4.1 и 4.2

# 4 Отправка данных с микроконтроллера в облако
Необходимо реализовать программу, получающую данные с датчика и отправляющую их в облако (например в Rightech) 

```c
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "mqtt_client.h"

/* ================== CONFIG ================== */

#define WIFI_SSID "Wokwi-GUEST"

#define CLIENT_ID ""

// Для примера приведена эмитация передачи температуры и влажности в соответствующие топики
#define TOPIC_TEMP "base/state/temperature"
#define TOPIC_HUM  "base/state/humidity"

static const char *TAG = "RIGHTECH";

/* ================== EVENTS ================== */

static EventGroupHandle_t sys_event_group;
#define WIFI_OK_BIT  BIT0
#define MQTT_OK_BIT  BIT1

/* ================== WIFI ================== */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, retrying...");
        esp_wifi_connect();
        xEventGroupClearBits(sys_event_group, WIFI_OK_BIT);
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "WiFi connected (IP ready)");
        xEventGroupSetBits(sys_event_group, WIFI_OK_BIT);
    }
}

static void wifi_init(void)
{
    sys_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

/* ================== MQTT ================== */

static esp_mqtt_client_handle_t mqtt_client;

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    if (event_id == MQTT_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "MQTT connected");
        xEventGroupSetBits(sys_event_group, MQTT_OK_BIT);
    }

    if (event_id == MQTT_EVENT_DISCONNECTED) {
        ESP_LOGW(TAG, "MQTT disconnected");
        xEventGroupClearBits(sys_event_group, MQTT_OK_BIT);
    }
}

static void mqtt_start(void)
{
    esp_mqtt_client_config_t cfg = {
        .broker = {
            .address = {
                .hostname  = ,
                .port      = ,
                .transport = MQTT_TRANSPORT_OVER_TCP,
            },
        },
        .credentials = {
            .client_id = CLIENT_ID,
        },
    };

    mqtt_client = esp_mqtt_client_init(&cfg);

    esp_mqtt_client_register_event(
        mqtt_client,
        ESP_EVENT_ANY_ID,
        mqtt_event_handler,
        NULL
    );

    esp_mqtt_client_start(mqtt_client);
}


/* ================== TELEMETRY ================== */
// Заменить на получение данных с датчика
static void telemetry_task(void *arg)
{
    char buf[16];
    int counter = 0;

    /* 🔥 ЖДЁМ WIFI + MQTT */
    xEventGroupWaitBits(
        sys_event_group,
        WIFI_OK_BIT | MQTT_OK_BIT,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY
    );

    ESP_LOGI(TAG, "Telemetry started");

    while (1) {
        float t = 20.0 + (counter % 10);
        float h = 40.0 + (counter % 20);

        snprintf(buf, sizeof(buf), "%.1f", t);
        esp_mqtt_client_publish(mqtt_client, TOPIC_TEMP, buf, 0, 1, true);

        snprintf(buf, sizeof(buf), "%.1f", h);
        esp_mqtt_client_publish(mqtt_client, TOPIC_HUM, buf, 0, 1, true);

        ESP_LOGI(TAG, "Sent → %.1f °C | %.1f %%", t, h);

        counter++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ================== MAIN ================== */

void app_main(void)
{
    nvs_flash_init();
    wifi_init();

    /* ЖДЁМ IP */
    xEventGroupWaitBits(
        sys_event_group,
        WIFI_OK_BIT,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY
    );

    mqtt_start();

    xTaskCreate(
        telemetry_task,
        "telemetry",
        4096,
        NULL,
        5,
        NULL
    );
}

```

# РГР
Тема: Изучение требований и выбор сенсора для мониторинга температуры и влажности на фармацевтическом складе.
Задание содержит работу с изучением требований и компонентов. В качестве требования выступает вариант задания. Ряд требований можно найти в Приказе Минздравсоцразвития РФ от 23 августа 2010 г. N 706н "Об утверждении Правил хранения лекарственных средств".
Выписать подробности ТЗ, несколько основных моментов, как например (могут быть и другие):
1. Минимальная частота снятия показаний датчика
2. Срок хранения логов
3. Минимальная площадь склада
4. Допуски к точности снятия показаний

Затем строится сравнительный анализ датчиков и таблица характеристик. Чтобы выбрать подходящий датчик для задачи, потребуется прочитать документацию на ряд датчиков подходящих к задаче. У разных датчиков - разные возможности и ограничения. Необходимо составить сравнительную таблицу характеристик датчиков. (Минимум 4 датчика) Далее написать минимум 3 закономерности, которые можно заметить в характеристиках сходных датчиков. На основании таблицы выбирать те датчики, которые подходят к решению поставленной задачи

Минимальный объем 25стр. и 5 источников