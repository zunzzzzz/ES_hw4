import paho.mqtt.client as paho
import time
import numpy as np
import matplotlib.pyplot as plt
import sys
# https://os.mbed.com/teams/mqtt/wiki/Using-MQTT#python-client

# MQTT broker hosted on local machine
mqttc = paho.Client()

# Settings for connection
# TODO: revise host to your ip
host = "172.20.10.2"
topic = "Mbed"
list_x = []
list_y = []
list_z = []
list_time = []
time_acc = 0
# Callbacks
def on_connect(self, mosq, obj, rc):
      print("Connected rc: " + str(rc))

def on_message(mosq, obj, msg):
      global time_acc
      global list_x
      global list_y
      global list_z
      data = str(msg.payload)
      print("[Received] Topic: " + msg.topic + ", Message: " + data)
      split_data = data.split()
      print(split_data)
      if(len(split_data) > 3):
            x = 5
            x = float((split_data[0].split("b'"))[1])
            y = float(split_data[1])
            z = float(split_data[2])
            time_interval = float((split_data[3].split("\\"))[0])
            # time_interval = 0.5
            # print(x)
            # print(y)
            # print(z)
            # print(time_interval)
            # print(list_x)
            # print(time_acc)
            list_x.append(x)
            list_y.append(y)
            list_z.append(z)
            list_time.append(time_acc)
            time_acc = time_acc + time_interval
            if time_acc > 20:
                  plt.figure()
                  plt.plot(list_time, list_x, 'g')
                  plt.plot(list_time, list_y, 'r')
                  plt.plot(list_time, list_z, 'b')
                  plt.title("Acceleration Plot")
                  plt.xlabel("timestamp")
                  plt.ylabel("acc value")
                  plt.show()
                  sys.exit()



def on_subscribe(mosq, obj, mid, granted_qos):
      print("Subscribed OK")

def on_unsubscribe(mosq, obj, mid, granted_qos):
      print("Unsubscribed OK")

# Set callbacks
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_unsubscribe = on_unsubscribe

# Connect and subscribe
print("Connecting to " + host + "/" + topic)
mqttc.connect(host, port=1883, keepalive=60)
mqttc.subscribe(topic, 0)
# Publish messages from Python
# num = 0
# while num != 5:
#       ret = mqttc.publish(topic, "Message from Python!\n", qos=0)
#       if (ret[0] != 0):
#             print("Publish failed")
#       mqttc.loop()
#       time.sleep(1.5)
#       num += 1
# Loop forever, receiving messages
mqttc.loop_forever()