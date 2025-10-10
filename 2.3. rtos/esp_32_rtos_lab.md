## Лабораторная работа 2.3. RTOS на примере FreeRTOS (ESP32)

### Цель работы
Познакомиться с принципами работы **операционной системы реального времени (RTOS)** на микроконтроллере **ESP32**, используя **FreeRTOS**.

---

### 2.3.1 Введение

**RTOS (Real-Time Operating System)** — это операционная система, которая позволяет выполнять несколько задач **параллельно** и с гарантией **предсказуемого времени реакции**.

Основные особенности RTOS:
- Планирование задач с учетом приоритетов.
- Механизмы синхронизации (мьютексы, семафоры, очереди).
- Поддержка таймеров и событий.
- Обработка прерываний с минимальной задержкой.

**ESP32** использует встроенную RTOS — **FreeRTOS**, которая запускается сразу при старте программы. Все задачи и обработка аппаратных событий управляются планировщиком.

#### Основные компоненты FreeRTOS
| Компонент | Описание |
|-----------|----------|
| Task (Задача) | Поток выполнения, который выполняет свою функцию параллельно с другими задачами. |
| Queue (Очередь) | Передача данных между задачами без конфликта доступа. |
| Semaphore (Семафор) | Ограничивает доступ к общему ресурсу между задачами. |
| Mutex (Мьютекс) | Вид семафора для защиты ресурсов от одновременного доступа. |
| Timer (Таймер) | Позволяет вызывать функцию через заданный интервал времени. |
| Event Group | Набор флагов для сигнализации между задачами. |
| ISR (Interrupt Service Routine) | Обработчик аппаратного события, например, нажатия кнопки. |

Документация: [FreeRTOS API Reference (Espressif)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html)

---

### 2.3.2 Таймеры
Таймеры позволяют запускать функцию через определенный интервал времени без блокировки других задач.

```c
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "driver/gpio.h"

#define LED_PIN 2

TimerHandle_t led_timer;

void toggle_led(TimerHandle_t xTimer) {
    int level = gpio_get_level(LED_PIN);
    gpio_set_level(LED_PIN, !level);
}

void app_main(void) {
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    led_timer = xTimerCreate("LED Timer", pdMS_TO_TICKS(100), pdTRUE, 0, toggle_led);
    xTimerStart(led_timer, 0);
}
```

---

### 2.3.3 Потоки (Tasks)
Задачи в FreeRTOS работают как потоки. Планировщик выделяет каждой задаче время для выполнения в зависимости от приоритета.

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdio.h>

#define LED_PIN 2

void led_task(void *pvParameters) {
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    while (1) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void print_task(void *pvParameters) {
    while (1) {
        printf("*");
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void) {
    xTaskCreate(led_task, "LED Task", 2048, NULL, 1, NULL);
    xTaskCreate(print_task, "Print Task", 2048, NULL, 1, NULL);
}
```

---

### 2.3.4 Прерывания (Interrupts)
Прерывания позволяют мгновенно реагировать на события, например, нажатие кнопки.

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_intr_alloc.h"

#define LED_PIN 2
#define BUTTON_PIN 0

volatile int delay_ms = 500;

void IRAM_ATTR button_isr_handler(void* arg) {
    delay_ms = 100;
}

void led_task(void *pvParameters) {
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    while (1) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

void app_main(void) {
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_en(BUTTON_PIN);
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);

    xTaskCreate(led_task, "LED Task", 2048, NULL, 1, NULL);
}
```

---

### 2.3.5 Дребезг контактов
Механическая кнопка может многократно замыкать и размыкать контакты при нажатии — это **дребезг контактов**. Его можно устранить программно с помощью задержки.

```c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define LED_PIN 2
#define BUTTON_PIN 0

volatile bool button_pressed = false;
int64_t last_press_time = 0;

void IRAM_ATTR button_isr_handler(void* arg) {
    int64_t now = esp_timer_get_time();
    if (now - last_press_time > 300000) {
        button_pressed = true;
        last_press_time = now;
    }
}

void led_task(void *pvParameters) {
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    while (1) {
        if (button_pressed) {
            button_pressed = false;
            gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void) {
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_en(BUTTON_PIN);
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);

    xTaskCreate(led_task, "LED Task", 2048, NULL, 1, NULL);
}
```

---

### Теория
1. **FreeRTOS планировщик**: использует алгоритм Round-Robin с учетом приоритетов. Задачи с более высоким приоритетом выполняются раньше.
2. **Задачи**: минимальная единица работы, каждая задача имеет стек и приоритет.
3. **Семафоры и мьютексы**: защищают разделяемые ресурсы, предотвращая гонки.
4. **Очереди (Queues)**: безопасная передача данных между задачами.
5. **Прерывания**: используются для мгновенной реакции на события, обычно вызывают короткие функции, которые сигнализируют задаче через семафор или очередь.
6. **Таймеры**: позволяют выполнять периодические действия без блокирования основного потока.
7. **Дребезг контактов**: устраняется с помощью программного дебаунса, т.е. игнорирования повторных нажатий за короткий промежуток времени.

---

Теперь лабораторная полностью адаптирована под ESP32 и FreeRTOS с теорией, примерами и объяснениями.

