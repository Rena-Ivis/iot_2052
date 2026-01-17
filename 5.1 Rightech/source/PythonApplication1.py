# Подключаем библиотеки
import json
import time
import paho.mqtt.client as mqtt

# Указываем идентификатор клиента, где hiber_device - идентификатор устройства (deviceId)
client = mqtt.Client('d:quickstart:my_test:hiber_device')

client.username_pw_set('use-token-auth', 'TOKEN')
# Установка параметров подключения. В одинарных кавычках указывается адрес подключения, далее через запятую порт, в нашем случае 1883
client.connect('quickstart.messaging.internetofthings.ibmcloud.com', 1883, 60) 

# Функция loop() должна вызываться регулярно, чтобы клиент мог обрабатывать входящие сообщения для отправки данных публикации и обновлять соединение.  
client.loop()
# Переменная temp отвечает за хранение значения температуры, присваиваем ей изначально значение 20
temp = 20
# Цикл будет выполняться пока не остановить принудительно программу
while (True):
    # Связываем в payload ключ со значением, в нашем случае ключ это temperature, значение хранится в temp
    payload = {'temperature': temp}
    # Переменную temp увеличиваем на 1
    temp += 1
    # Публикуем в топик iot-2/evt/my_event/fmt/json(важно корректно указать эти параметры сообщения) и значение payload, связанное с ключом
    client.publish('iot-2/evt/my_event/fmt/json', json.dumps(payload))
    # Выводим надпись на экран о значении опубликованной в топик температуры
    print('published temperature {}'.format(temp))
    # Ждем 1 секунду
    time.sleep(1)
# Функция для разрыва соединения. 
client.disconnect()
