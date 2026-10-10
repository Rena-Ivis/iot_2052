# Форматы данных, JSON и веб-визуализация IoT

В предыдущих работах были рассмотрены Wi‑Fi и MQTT. Теперь добавим ещё две части IoT-системы: **структурированный формат данных** и **пользовательский веб-интерфейс**.

В конце раздела находится самостоятельная итоговая задача **«Станция экомониторинга»**.

---

## 1. Форматы обмена данными IoT-устройств

MQTT определяет способ доставки сообщений, но не формат их `payload`. Устройство и приложение должны заранее договориться, как интерпретировать передаваемые данные.

В MQTTX, например, можно работать с Plaintext, JSON, Base64, Hex, CBOR и MsgPack. Для IoT важны объём сообщения, сложность обработки на микроконтроллере, наличие библиотек и возможность расширения структуры.

### XML и бинарные форматы

XML позволяет описывать структурированные данные с помощью вложенных тегов:

```xml
<message>
  <device_id>esp32_01</device_id>
  <temperature>24.3</temperature>
  <humidity>42.7</humidity>
</message>
```

Для микроконтроллеров его недостатком является высокая избыточность.

Base64 и Hex позволяют представить бинарные данные текстом. CBOR и MessagePack используются для более компактного бинарного представления данных.

### JSON

**JSON (JavaScript Object Notation)** — текстовый формат структурированных данных. Он состоит из пар «ключ–значение», объектов и массивов.

```json
{
  "device_id": "esp32_01",
  "temperature": 24.3,
  "humidity": 42.7,
  "battery_mv": 3680
}
```

Значением может быть число, строка, `true`/`false`, объект, массив или `null`.

JSON не отвечает за доставку сообщения — этим занимается MQTT. JSON задаёт **структуру полезной нагрузки**.

По сравнению с XML он обычно проще для чтения и разбора и создаёт менее объёмные сообщения.

Данные можно группировать:

```json
{
  "data": {
    "temperature": 24.3,
    "humidity": 42.7
  },
  "status": {
    "battery_mv": 3680,
    "rssi": -72
  }
}
```

При этом чрезмерная вложенность усложняет обработку на микроконтроллере.

---

## 2. Работа с JSON в ESP-IDF

Для простого фиксированного сообщения JSON можно сформировать через `sprintf()`:

```c
float temperature, humidity;

char json_data[64];
sprintf(
    json_data,
    "{\"temperature\":%.1f,\"humidity\":%.1f}",
    temperature,
    humidity
);
```

Но при усложнении структуры и особенно при разборе входящих сообщений такой подход становится неудобным.

В ESP-IDF для работы с JSON используется **cJSON**:

```c
#include "cJSON.h"
```

### Создание JSON

```c
cJSON *root = cJSON_CreateObject();

cJSON_AddStringToObject(root, "id", "esp32_01");
cJSON_AddNumberToObject(root, "ts", 1768904153);
```

Для вложенного объекта:

```c
cJSON *data = cJSON_CreateObject();

cJSON_AddNumberToObject(data, "temperature", 23.7);
cJSON_AddNumberToObject(data, "humidity", 54);

cJSON_AddItemToObject(root, "data", data);
```

После `cJSON_AddItemToObject()` вложенный объект становится частью `root` и отдельно удалять его не нужно.

### Сериализация

Для удобного человеку форматированного вывода:

```c
char *formatted_json_str = cJSON_Print(root);
```

Для передачи по сети предпочтительнее компактная строка:

```c
char *json_str = cJSON_PrintUnformatted(root);
```

> [!WARNING]
>
> `cJSON_Print()` и `cJSON_PrintUnformatted()` выделяют память динамически. После использования строку необходимо освободить.

```c
free(json_str);
cJSON_Delete(root);
```

Удаление `root` освобождает и вложенные в него объекты.

### Разбор JSON

Входящая строка разбирается функцией:

```c
cJSON *root = cJSON_Parse(json_data);

if (root == NULL) {
    ESP_LOGE(TAG, "Ошибка разбора JSON");
    return;
}
```

Получение поля:

```c
cJSON *temperature = cJSON_GetObjectItem(root, "temperature");

if (cJSON_IsNumber(temperature)) {
    ESP_LOGI(TAG, "Температура: %.1f", temperature->valuedouble);
}
```

Нельзя предполагать, что нужный ключ всегда существует и имеет ожидаемый тип. После обработки:

```c
cJSON_Delete(root);
```

> [!TIP]
> ### Задание
> 1. Реализуйте обработчик MQTT-сообщения с JSON.
> 2. Разберите сообщение через `cJSON`.
> 3. Проверьте наличие `data`, `status`, `temperature`, `humidity`, `battery_mv` и их типы.
> 4. При ошибке формата выведите диагностическое сообщение, не завершая программу.
> 5. Переименуйте `temperature` в `temp`, а `humidity` в `humid` и убедитесь, что старая версия обработчика обнаруживает изменение формата.
> 6. Добавьте новый параметр, например идентификатор устройства.
> 7. Реализуйте поддержку двух версий формата.
> 8. Сравните объём получившихся сообщений.

---

# 3. Практическая работа: веб-приложение с Яндекс.Картой

В этой работе создадим веб-приложение постепенно:

1. базовая HTML-страница;
2. Яндекс.Карта;
3. конфигурация приложения;
4. MQTT-клиент;
5. тестовые метки;
6. получение реальных данных по MQTT.

Практикум основан на **JavaScript API Яндекс.Карт 2.1**. В исходных материалах API 3.0 приведён как более современная, но более сложная альтернатива.

## Этап 1. Рабочий каркас приложения

### Шаг 1. Создание проекта

Создайте структуру:

```text
meteo-monitoring/
├── index.html
├── css/
│   └── style.css
└── js/
    ├── config.js
    ├── main.js
    └── paho-mqtt-min.js
```

`index.html`:

```html
<!DOCTYPE html>
<html lang="ru">
<head>
    <meta charset="UTF-8">
    <title>Метеомониторинг</title>
    <link rel="stylesheet" href="css/style.css">
</head>
<body>
    <h1>Метеомониторинг</h1>
    <div id="map"></div>
</body>
</html>
```

`css/style.css`:

```css
body {
    font-family: Arial, sans-serif;
    margin: 0;
    padding: 20px;
    background-color: #f5f5f5;
}

h1 {
    text-align: center;
    color: #333;
}

#map {
    width: 100%;
    height: 500px;
    border: 2px solid #ddd;
    border-radius: 8px;
    margin-top: 20px;
}
```

Пока приложение содержит только контейнер для будущей карты.

### Шаг 2. Добавление Яндекс.Карты

Получите ключ **JavaScript API и HTTP Геокодера Яндекс.Карт** и подключите API:

```html
<script src="https://api-maps.yandex.ru/2.1/?lang=ru_RU&apikey=ВАШ_API_КЛЮЧ"></script>
```

> [!WARNING]
>
> Замените `ВАШ_API_КЛЮЧ` на полученный ключ. В исходном практикуме указано, что после получения активация ключа может занять до 15 минут.

Подключите `main.js`:

```html
<script src="js/main.js"></script>
```

`js/main.js`:

```javascript
ymaps.ready(function () {
    const map = new ymaps.Map('map', {
        center: [55.76, 37.64],
        zoom: 10
    });

    console.log('Карта загружена!');
});
```

После этого должна открываться интерактивная карта.

---

## Этап 2. Карта с данными

### Шаг 3. Конфигурация приложения

Создайте `js/config.js`:

```javascript
APP_CONFIG = {
    map: {
        center: {
            lon: 37.64,
            lat: 55.76
        },
        zoom: 10
    }
};
```

Подключите его **до** `main.js`:

```html
<script src="js/config.js"></script>
<script src="js/main.js"></script>
```

Используйте конфигурацию в `main.js`:

```javascript
const config = APP_CONFIG;

ymaps.ready(function () {
    const map = new ymaps.Map('map', {
        center: [config.map.center.lat, config.map.center.lon],
        zoom: config.map.zoom
    });
});
```

> [!TIP]
> ### Задание
>
> Измените координаты в `config.js`, чтобы при запуске карта центрировалась на вашем населённом пункте.

### Шаг 4. Подключение MQTT-библиотеки

Для работы с MQTT в браузере используется JavaScript-клиент **Paho MQTT**. Поместите `paho-mqtt-min.js` в папку `js/`.

Подключите скрипты:

```html
<script src="js/paho-mqtt-min.js"></script>
<script src="js/config.js"></script>
<script src="js/main.js"></script>
```

В предыдущей работе локальный Mosquitto уже был настроен так:

```text
listener 1883
allow_anonymous true

listener 1884
protocol websockets
```

`1883` используется обычными MQTT-клиентами, а браузер подключается через WebSocket на `1884`.

Добавьте MQTT в `config.js`:

```javascript
APP_CONFIG = {
    map: {
        center: {
            lon: 37.64,
            lat: 55.76
        },
        zoom: 10
    },

    mqtt: {
        host: 'localhost',
        port: 1884,
        useSSL: false,
        topic: 'iot_practice/meteo',
        clientIdPrefix: 'web_meteo_'
    }
};
```

Логин и пароль для локального брокера не используются.

### Шаг 5. Добавление тестовых меток

Создадим хранилище меток и функцию добавления станции:

```javascript
const mapConfig = APP_CONFIG.map;

let map;
const markers = {};

ymaps.ready(function () {
    map = new ymaps.Map('map', {
        center: [mapConfig.center.lat, mapConfig.center.lon],
        zoom: mapConfig.zoom
    });
});

function addMeteoMarker(stationId, lat, lon, temperature, pressure) {
    const coords = [parseFloat(lat), parseFloat(lon)];

    markers[stationId] = new ymaps.Placemark(
        coords,
        {
            iconContent: stationId,
            hintContent: `Станция ${stationId}`
        },
        {
            preset: 'islands#blueDotIcon'
        }
    );

    map.geoObjects.add(markers[stationId]);
}
```

Проверим без MQTT:

```javascript
setTimeout(() => {
    addMeteoMarker('test1', 55.7558, 37.6173, 22.5, 1013);
    addMeteoMarker('test2', 55.7400, 37.4172, 21.8, 1012);
}, 1000);
```

На карте должны появиться две метки.

### Шаг 6. Подключение к реальным данным

Вместо тестовых данных подключим браузер к локальному Mosquitto.

```javascript
const mqttConfig = APP_CONFIG.mqtt;

const client = new Paho.MQTT.Client(
    mqttConfig.host,
    mqttConfig.port,
    `${mqttConfig.clientIdPrefix}${Math.random().toString(36).slice(2)}`
);

client.onConnectionLost = responseObject => {
    console.log('Соединение потеряно:', responseObject.errorMessage);
};

client.onMessageArrived = message => {
    try {
        const payload = JSON.parse(message.payloadString);
        processMeteoData(payload);
    } catch (e) {
        console.error('Ошибка обработки сообщения:', e);
    }
};

function connectAndSubscribe() {
    client.connect({
        useSSL: mqttConfig.useSSL,
        onSuccess: () => {
            console.log('Подключено к MQTT');
            client.subscribe(mqttConfig.topic);
        },
        onFailure: e => console.error('Ошибка подключения:', e)
    });
}

window.onload = connectAndSubscribe;
```

Пример тестового сообщения:

```json
{
  "id": "meteo_01",
  "lon": 37.832,
  "lat": 55.82,
  "temperature": 20.7,
  "pressure": 1011.7
}
```

Его можно опубликовать в:

```text
iot_practice/meteo
```

Обработка:

```javascript
function processMeteoData(data) {
    const { id, lat, lon, temperature, pressure } = data;

    if (!id || !lat || !lon) {
        console.warn('Некорректные данные станции:', data);
        return;
    }

    const coords = [parseFloat(lat), parseFloat(lon)];

    if (!markers[id]) {
        markers[id] = new ymaps.Placemark(
            coords,
            {
                iconContent: id,
                balloonContent: `
                    <b>Станция: ${id}</b><br>
                    Температура: ${temperature} °C<br>
                    Давление: ${pressure} мбар
                `
            },
            {
                preset: 'islands#blueDotIcon'
            }
        );

        map.geoObjects.add(markers[id]);
    } else {
        markers[id].geometry.setCoordinates(coords);
        markers[id].properties.set(
            'balloonContent',
            `
                <b>Станция: ${id}</b><br>
                Температура: ${temperature} °C<br>
                Давление: ${pressure} мбар
            `
        );
    }
}
```

> [!TIP]
> ### Задание
> 1. Изучите код приложения и документацию Яндекс.Карт.
> 2. Измените стиль метки.
> 3. Сделайте цвет метки зависимым от температуры: например, синий — холодно, зелёный — тепло, красный — жарко.

Практика строится инкрементно: после добавления карты получается минимально работоспособное приложение, после тестовых меток — функциональный прототип, после подключения MQTT — приложение с реальными данными.

---

# 4. Задача: Станция экомониторинга

Теперь необходимо **самостоятельно объединить изученные ранее части** в одну IoT-систему.

## Задача

В рамках городской программы «Умный экологический мониторинг» требуется создать прототип распределённой системы для оперативного контроля параметров окружающей среды.

Система должна состоять из:

- автономного сенсорного устройства;
- MQTT-брокера;
- веб-приложения визуализации.

### Станция

Каждые **60 секунд** устройство должно измерять:

- температуру воздуха;
- относительную влажность;
- атмосферное давление;
- концентрацию PM2.5.

После каждого цикла измерений устройство передаёт данные по Wi‑Fi на MQTT-брокер. Пакет должен быть сформирован в формате **JSON**.

Каждая станция должна иметь уникальный идентификатор, передаваемый вместе с данными.

При временной потере Wi‑Fi устройство должно периодически пытаться восстановить соединение и продолжить работу.

### MQTT

Вместо общего брокера используйте **локальный Mosquitto на компьютере студента**.

ESP32 подключается к IPv4-адресу компьютера на порту `1883`.

Используйте топик:

```text
iot_practice/eco
```

Браузерное приложение подключается к тому же Mosquitto через WebSocket на порту `1884`.

### Веб-приложение

Приложение должно:

- отображать карту;
- отображать все активные станции маркерами по их статическим координатам;
- менять цвет маркера в зависимости от уровня PM2.5: зелёный, жёлтый или красный;
- при клике на маркер показывать температуру, влажность, давление, PM2.5 и время последнего обновления.

### Серверная часть

Необходимо обеспечить:

- приём данных от станции через MQTT;
- контроль пороговых значений;
- формирование события `alert`.

Хранение в SQL/NoSQL, обработку истории и агрегацию данных реализовывать **не требуется**.

> [!NOTE]
>
> Конкретные числовые границы уровней PM2.5 в исходном задании не указаны. При реализации необходимо самостоятельно определить используемые пороги и явно указать их в проекте.

## Методические указания

1. Подключите к ESP32 метеосенсор и датчик запылённости, использованные в предыдущих работах.
2. Если датчика давления нет, давление можно исключить.
3. Координаты станции являются статическими и задаются при конфигурации устройства.
4. Идентификатор станции должен передаваться в JSON. Например:

```json
{
  "id": "Eco-01",
  "lat": 54.3182,
  "lon": 48.3831,
  "temperature": 23.7,
  "humidity": 54.0,
  "pressure": 1013.2,
  "pm25": 18.4
}
```

5. Для веб-приложения используйте API Яндекс.Карт.
6. Используйте знания из предыдущих работ: подключение датчиков, Wi‑Fi, восстановление соединения, MQTT, JSON и отображение данных на карте.

## Результат

В результате должна работать цепочка:

```text
датчики
   ↓
ESP32
   ↓ Wi‑Fi
JSON / MQTT
   ↓
локальный Mosquitto
   ↓ WebSocket
веб-приложение
   ↓
Яндекс.Карта
```

Финальная задача не содержит готового решения: программное обеспечение ESP32 и веб-приложение необходимо разработать самостоятельно.
