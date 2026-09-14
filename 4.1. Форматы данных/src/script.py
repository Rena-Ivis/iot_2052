# подключаем библиотеку
import paho.mqtt.client as mqtt

# данная функция вызывается при удачном подключении к серверу
def on_connect(client, userdata, flags, rc):
    # уведомление о подключении
    print("Connected is successfull")
    # подписываемся на топик
    client.subscribe("iot_topic2") #ОБРАТИТЕ ВНИМАНИЕ. Вместо "$SYS/#" используейте название своего топика

# данная функция вызывается при поступлении сообщений 
# в топик на который подписались
def on_message(client, userdata, msg):
    # вывод текста полученного сообщения
    print("Message: "+str(msg.payload) )

# Создаем подключение
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("127.0.0.1", 1883, 60) #ОБРАТИТЕ ВНИМАНИЕ. В отличии от учебного пособия, вместо "iot.eclipse.org" мы ввели ip-адресс нашего пк. Порт спользуется тот, на котором запускался mosquitto
# отправляем одно сообщение для теста 
# работаспособности функции отправки сообщений 
#client.publish(topic="iot_topic", payload="hello from python", qos=0, retain=False)
# вызов функции бесконечного цикла для постоянной прослушки порта
client.loop_forever()