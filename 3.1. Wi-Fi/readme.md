>Примечание: код не проверен преподавателем
**# Wi-Fi и передача данных на ESP32-S3 UNO**

**## Wi-Fi**

Интернет вещей невозможно представить без сетевого взаимодействия. Даже простое IoT-устройство становится «умным», когда может передавать данные наружу или принимать команды извне. Выбор технологии связи влияет на архитектуру системы, её надёжность, энергопотребление и масштабируемость.

В IoT используются разные технологии связи:

  Показатель            LoRa   NB-IoT   BLE    Wi-Fi

**  -------------------- ------ -------- ------ -------**

  Работа без шлюза       --      --      --     \\+

  Высокая скорость       --      --      --     \\+

  Mesh-топология         \\+      --      \\+     \\+

  Низкая стоимость      +/--     --     +/--    \\+

  Безопасность           --      \\+      \\+     \\+

  Дальность связи        \\+      \\+      --    +/--

  Распространённость     --      --      \\+     \\+

Wi-Fi имеет развитую инфраструктуру и доступен на ESP32 «из коробки», поэтому удобен для обучения и прототипирования. При этом его дальность ограничена, а энергопотребление сравнительно велико, поэтому для автономных IoT-устройств Wi-Fi подходит не всегда.

Wi-Fi --- технология беспроводной передачи данных по радиоканалу, основанная на стандартах семейства IEEE 802.11. Сам Wi-Fi не является Интернетом: он обеспечивает подключение устройства к локальной сети, а поверх него работают протоколы стека TCP/IP.

Типовая схема:

\`\`\` text

ESP32-S3 → Wi-Fi → роутер → локальная сеть / Интернет

![Типовая схема подключения ESP32 к Интернету через Wi-Fi](img/wifi_network.png)

\`\`\`

Наиболее распространены диапазоны 2.4 и 5 ГГц. 2.4 ГГц обеспечивает большую дальность и лучше проходит через препятствия, поэтому часто используется в IoT. ESP32-S3 работает с Wi-Fi в диапазоне 2.4 ГГц.

Каждое Wi-Fi-устройство имеет MAC-адрес. После подключения к сети устройство обычно автоматически получает IP-адрес по DHCP. Этот IP-адрес может изменяться при повторных подключениях.

ESP32 поддерживает несколько режимов Wi-Fi:

1\.  **\*\*STA (Station)\*\*** --- подключение к существующей Wi-Fi-сети.

    Основной режим для этой работы.

2\.  **\*\*AP (Access Point)\*\*** --- ESP32 создаёт собственную точку доступа.

3\.  **\*\*STA + AP\*\*** --- одновременная работа в обоих режимах.

В большинстве IoT-систем устройство само инициирует соединение с сервером.

**------------------------------------------------------------------------**

**# Практическая часть 1. Подключение к Wi-Fi**

В рамках работы подключим ESP32-S3 UNO к существующей точке доступа Wi-Fi. Точка доступа предоставит устройству IP-адрес по DHCP и обеспечит подключение к сети.

**## Необходимо**

\-   ESP32-S3 UNO;

\-   Wi-Fi-роутер или точка доступа смартфона;

\-   SSID сети;

\-   пароль Wi-Fi.

Использование смартфона в качестве точки доступа удобно для проверки обрывов связи и восстановления соединения.

Общая схема работы сетевого устройства:

1\.  инициализация Wi-Fi;

2\.  сканирование доступных сетей;

3\.  подключение к выбранной сети;

4\.  получение IP от DHCP;

5\.  инициализация DNS;

6\.  работа через TCP/UDP;

7\.  переподключение при обрыве.

В ESP-IDF работа с Wi-Fi строится на событийной модели. Основные события:

\-   запуск Wi-Fi в режиме STA;

\-   успешное подключение к точке доступа;

\-   потеря соединения;

\-   получение IP-адреса по DHCP.

Нельзя считать соединение постоянным: приложение должно уметь ждать подключения и восстанавливать его после обрыва.

Wi-Fi-стек ESP-IDF использует NVS, поэтому перед запуском Wi-Fi необходимо вызвать \`nvs\_flash\_init()\`.

В исходном варианте практики состояние подключения отображалось светодиодом на \`GPIO\_NUM\_2\`. Для ESP32-S3 UNO индикация светодиодом здесь опущена, так как она не относится непосредственно к подключению Wi-Fi.

**## Код подключения**

\`\`\` c

\#include \<stdio.h>

\#include "freertos/FreeRTOS.h"

\#include "freertos/task.h"

\#include "freertos/event\_groups.h"

\#include "esp\_system.h"

\#include "esp\_wifi.h"

\#include "esp\_event.h"

\#include "esp\_netif.h"

\#include "esp\_log.h"

\#include "nvs\_flash.h"

\#define WIFI\_SSID "MY\_WIFI\_SSID"

\#define WIFI\_PASS "MY\_WIFI\_PASSWORD"

static EventGroupHandle\_t wifi\_event\_group;

\#define WIFI\_CONNECTED\_BIT BIT0

static void wifi\_event\_handler(

    void \*arg,

    esp\_event\_base\_t event\_base,

    int32\_t event\_id,

    void \*event\_data

) {

    if (event\_base == WIFI\_EVENT &&

        event\_id == WIFI\_EVENT\_STA\_START) {

        esp\_wifi\_connect();

    } else if (event\_base == WIFI\_EVENT &&

               event\_id == WIFI\_EVENT\_STA\_DISCONNECTED) {

        xEventGroupClearBits(

            wifi\_event\_group,

            WIFI\_CONNECTED\_BIT

        );

        esp\_wifi\_connect();

    } else if (event\_base == IP\_EVENT &&

               event\_id == IP\_EVENT\_STA\_GOT\_IP) {

        xEventGroupSetBits(

            wifi\_event\_group,

            WIFI\_CONNECTED\_BIT

        );

    }

}

static void wifi\_task(void \*pvParameter)

{

    esp\_netif\_ip\_info\_t ip\_info;

    printf("Waiting for connection to the Wi-Fi network...\n");

    xEventGroupWaitBits(

        wifi\_event\_group,

        WIFI\_CONNECTED\_BIT,

        false,

        true,

        portMAX\_DELAY

    );

    printf("Connected!\n");

    esp\_netif\_get\_ip\_info(

        esp\_netif\_get\_handle\_from\_ifkey("WIFI\_STA\_DEF"),

        &ip\_info

    );

    printf("IP Address:  " IPSTR "\n", IP2STR(&ip\_info.ip));

    printf("Subnet mask: " IPSTR "\n", IP2STR(&ip\_info.netmask));

    printf("Gateway:     " IPSTR "\n", IP2STR(&ip\_info.gw));

    printf("You can connect now to any web servers!\n");

    while (1) {

        vTaskDelay(1000 / portTICK\_PERIOD\_MS);

    }

}

void app\_main(void)

{

    printf("\nESP-IDF version used: %s\n", IDF\_VER);

    esp\_err\_t ret = nvs\_flash\_init();

    if (ret == ESP\_ERR\_NVS\_NO\_FREE\_PAGES ||

        ret == ESP\_ERR\_NVS\_NEW\_VERSION\_FOUND) {

        ESP\_ERROR\_CHECK(nvs\_flash\_erase());

        ret = nvs\_flash\_init();

    }

    ESP\_ERROR\_CHECK(ret);

    ESP\_ERROR\_CHECK(esp\_netif\_init());

    ESP\_ERROR\_CHECK(esp\_event\_loop\_create\_default());

    esp\_netif\_create\_default\_wifi\_sta();

    wifi\_event\_group = xEventGroupCreate();

    wifi\_init\_config\_t wifi\_init\_config =

        WIFI\_INIT\_CONFIG\_DEFAULT();

    ESP\_ERROR\_CHECK(

        esp\_wifi\_init(&wifi\_init\_config)

    );

    ESP\_ERROR\_CHECK(

        esp\_event\_handler\_instance\_register(

            WIFI\_EVENT,

            ESP\_EVENT\_ANY\_ID,

            &wifi\_event\_handler,

            NULL,

            NULL

        )

    );

    ESP\_ERROR\_CHECK(

        esp\_event\_handler\_instance\_register(

            IP\_EVENT,

            IP\_EVENT\_STA\_GOT\_IP,

            &wifi\_event\_handler,

            NULL,

            NULL

        )

    );

    wifi\_config\_t wifi\_config = {

        .sta = {

            .ssid = WIFI\_SSID,

            .password = WIFI\_PASS,

        },

    };

    ESP\_ERROR\_CHECK(

        esp\_wifi\_set\_mode(WIFI\_MODE\_STA)

    );

    ESP\_ERROR\_CHECK(

        esp\_wifi\_set\_config(

            WIFI\_IF\_STA,

            &wifi\_config

        )

    );

    ESP\_ERROR\_CHECK(esp\_wifi\_start());

    printf("Connecting to %s\n", WIFI\_SSID);

    xTaskCreate(

        &wifi\_task,

        "wifi\_task",

        2048,

        NULL,

        5,

        NULL

    );

}

\`\`\`

**## Выполнение**

1\.  Создайте новый проект ESP-IDF, например \`wifi\_connect\`.

2\.  Скопируйте листинг в \`main.c\`.

3\.  Укажите свои \`WIFI\_SSID\` и \`WIFI\_PASS\`.

4\.  Сохраните проект.

5\.  Выполните компиляцию и загрузите прошивку.

6\.  Убедитесь, что ESP32-S3 UNO подключается к сети и получает IP-адрес.

\>[!TIP] ## Задание

\>

\>1.  Проверьте, что при обрыве связи устройство предпринимает попытки переподключения.

\>2.  Добавьте счётчик числа подключений в обработчик событий.

\>

**------------------------------------------------------------------------**

**# Практическая часть 2. Передача данных**

Предыдущий пример только подключает ESP32-S3 UNO к точке доступа. Теперь добавим передачу данных.

Wi-Fi обеспечивает подключение устройства к сети, а непосредственная доставка данных осуществляется протоколами транспортного уровня. Основные из них:

\-   **\*\*TCP (Transmission Control Protocol)\*\***;

\-   **\*\*UDP (User Datagram Protocol)\*\***.

Для сетевого взаимодействия ESP-IDF использует **\*\*LwIP (Lightweight IP)\*\*** --- стек TCP/IP для встраиваемых систем. LwIP предоставляет BSD-совместимый API сокетов.

Для TCP-клиента нам понадобятся:

\`\`\` c

socket()

connect()

send()

recv()

\`\`\`

Сокет в TCP/IP связан с сетевым адресом и номером порта. Клиент инициирует подключение к серверу с помощью \`connect()\`, \`send()\` отвечает за отправку данных, а \`recv()\` --- за приём.

**## Параметры сервера**

Добавьте:

\`\`\` c

\#include "lwip/sockets.h"

\#include "lwip/netdb.h"

\#include "esp\_log.h"

\#define SERVER\_HOST "192.168.1.100"

\#define SERVER\_PORT 12345

\`\`\`

Замените \`192.168.1.100\` на IP-адрес компьютера или другого устройства, на котором будет запущен TCP-сервер.

ESP32-S3 UNO и сервер должны находиться в одной сети либо между ними должна быть настроена маршрутизация.

**## TCP-клиент**

Добавьте к программе предыдущей части задачу:

\`\`\` c

static void socket\_task(void \*arg)

{

    char tx\_buf[64];

    char rx\_buf[128];

    for (;;) {

        xEventGroupWaitBits(

            wifi\_event\_group,

            WIFI\_CONNECTED\_BIT,

            pdFALSE,

            pdTRUE,

            portMAX\_DELAY

        );

        ESP\_LOGI("sock", "WiFi ready, creating socket");

        int sock = socket(

            AF\_INET,

            SOCK\_STREAM,

            IPPROTO\_IP

        );

        if (sock < 0) {

            ESP\_LOGE("sock", "Unable to create socket");

            vTaskDelay(pdMS\_TO\_TICKS(2000));

            continue;

        }

        struct sockaddr\_in dest\_addr = {

            .sin\_family = AF\_INET,

            .sin\_port = htons(SERVER\_PORT),

        };

        inet\_pton(

            AF\_INET,

            SERVER\_HOST,

            &dest\_addr.sin\_addr

        );

        if (connect(

                sock,

                (struct sockaddr \*)&dest\_addr,

                sizeof(dest\_addr)

            ) != 0) {

            ESP\_LOGE("sock", "Socket connect failed");

            close(sock);

            vTaskDelay(pdMS\_TO\_TICKS(2000));

            continue;

        }

        ESP\_LOGI("sock", "Socket connected");

        while (

            xEventGroupGetBits(wifi\_event\_group) &

            WIFI\_CONNECTED\_BIT

        ) {

            snprintf(

                tx\_buf,

                sizeof(tx\_buf),

                "Hello from ESP32\n"

            );

            int err = send(

                sock,

                tx\_buf,

                strlen(tx\_buf),

                0

            );

            if (err < 0) {

                ESP\_LOGE("sock", "Send failed");

                break;

            }

            int len = recv(

                sock,

                rx\_buf,

                sizeof(rx\_buf) - 1,

                0

            );

            if (len < 0) {

                ESP\_LOGE("sock", "Recv failed");

                break;

            } else if (len > 0) {

                rx\_buf[len] = 0;

                ESP\_LOGI(

                    "sock",

                    "Received: %s",

                    rx\_buf

                );

            }

            vTaskDelay(pdMS\_TO\_TICKS(2000));

        }

        ESP\_LOGW(

            "sock",

            "WiFi lost or socket error, closing socket"

        );

        close(sock);

    }

}

\`\`\`

В \`app\_main()\` добавьте создание задачи:

\`\`\` c

xTaskCreate(

    socket\_task,

    "socket\_task",

    4096,

    NULL,

    5,

    NULL

);

\`\`\`

**------------------------------------------------------------------------**

**## Узнаём IP-адрес компьютера**

**### Linux/macOS**

\`\`\` bash

ifconfig

\`\`\`

Для Linux также может быть доступна команда:

\`\`\` bash

hostname -I

\`\`\`

**### Windows**

\`\`\` bash

ipconfig

![Пример определения IPv4-адреса в Windows](img/windows_ipconfig.png)

\`\`\`

Если отображается несколько IP-адресов, нужен адрес сетевого интерфейса, через который компьютер находится в одной сети с ESP32-S3 UNO.

Укажите найденный адрес в \`SERVER\_HOST\`.

**------------------------------------------------------------------------**

**## Запускаем TCP-сервер**

Для проверки используем Ncat/Netcat.

В зависимости от установленной программы:

\`\`\` bash

nc -vlk 12345

\`\`\`

или:

\`\`\` bash

ncat -vlk 12345

![Запуск TCP-сервера Ncat в Windows](img/windows_ncat_listen.png)

\`\`\`

Будет запущен TCP-сервер, ожидающий подключения на порту \`12345\`.

После запуска ESP32-S3 UNO подключится к Wi-Fi, затем к TCP-серверу и передаст:

\`\`\` text

Hello from ESP32

![Получение сообщения от ESP32 в Ncat](img/windows_ncat_hello.png)

\`\`\`

Если в консоли сервера ввести короткий текст и нажать Enter, он будет передан ESP32-S3 UNO и отображён в терминале платы.

### Вариант со смартфоном Android

В оригинальном материале также показаны варианты запуска TCP-сервера на Android.

**Netcat for Android**

![Запуск Ncat на Android](img/android_ncat.png)

**NetPal**

![Настройка TCP-сервера в NetPal](img/android_netpal.png)

**------------------------------------------------------------------------**

\>[!TIP] ## Задание

\>

\>1.  Модифицируйте пример предыдущей части по предложенному алгоритму.

\>2.  Проверьте передачу данных от IoT-устройства к серверу и обратно.

\>3.  Подумайте, почему в примере упомянут «короткий текст» и насколько длинным он может быть.
