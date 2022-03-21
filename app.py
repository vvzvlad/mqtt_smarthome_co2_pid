#!/usr/bin/env -S python3 -u

#
#############################################################################
#
# helper to mqtt for smarthome
# /usr/local/opt/python/bin/python3.9 -m pip install simple-pid
#############################################################################
#

import paho.mqtt.client as mqtt
from simple_pid import PID

import time

import os
import json
import sys
import random

sensor_co2 = 0
period = 30
ahu_value = 0
min_power = 20

def on_connect(client, userdata, flags, rc):
  print("Connected with result code " + str(rc))
  client.subscribe("zigbee/sensor_CO2_a0d8")
  client.subscribe("mqtt_co2_pid/kp/set")
  client.subscribe("mqtt_co2_pid/ki/set")
  client.subscribe("mqtt_co2_pid/kd/set")
  client.subscribe("mqtt_co2_pid/target_value/set")
  client.subscribe("mqtt_co2_pid/power")
  client.subscribe("mqtt_co2_pid/min_power")
  client.publish("mqtt_co2_pid/status", payload="daemon started", qos=0, retain=False)
  client.publish("ahu/power/set", 10, qos=0, retain=False)



def on_message(client, userdata, msg):
  #print("Received MQTT message:" + msg.topic + ": " + str(msg.payload))
  global sensor_co2, pid, ahu_value, min_power

  if msg.topic == "zigbee/sensor_CO2_a0d8":
    json_data = json.loads(msg.payload)
    sensor_co2 = json_data.get('co2')
    print("CO2 sensor message received, co2: " + str(sensor_co2) + " ppm")
    ahu_value = pid(sensor_co2)
    print("New PID value: " + str(ahu_value) + " %")
    client.publish("mqtt_co2_pid/value", ahu_value, qos=0, retain=False)
    client.publish("ahu/power/set", int(ahu_value), qos=0, retain=False)

  if msg.topic == "mqtt_co2_pid/kp/set":
    pid.Kp = float(msg.payload)
    print("PID Kp value: " + str(pid.Kp))
    client.publish("mqtt_co2_pid/kp", str(pid.Kp), qos=0, retain=False)
  if msg.topic == "mqtt_co2_pid/ki/set":
    pid.Ki = float(msg.payload)
    print("PID Ki value: " + str(pid.Ki))
    client.publish("mqtt_co2_pid/ki", str(pid.Ki), qos=0, retain=False)
  if msg.topic == "mqtt_co2_pid/kd/set":
    pid.Kd = float(msg.payload)
    print("PID Kd value: " + str(pid.Kd))
    client.publish("mqtt_co2_pid/kd", str(pid.Kd), qos=0, retain=False)
  if msg.topic == "mqtt_co2_pid/target_value/set":
    pid.setpoint = float(msg.payload)
    print("PID setpoint value: " + str(pid.setpoint))
    client.publish("mqtt_co2_pid/target_value", str(pid.setpoint), qos=0, retain=False)
  if msg.topic == "mqtt_co2_pid/min_power/set":
    min_power = float(msg.payload)
    print("PID min power: " + str(min_power))
    client.publish("mqtt_co2_pid/min_power", str(min_power), qos=0, retain=False)
    pid.output_limits = (min_power, 100)
  if msg.topic == "mqtt_co2_pid/power":
    if msg.payload == "true":
      pid.auto_mode = True
    if msg.payload == "false":
      pid.auto_mode = False
    print("PID power value: " + str(pid.auto_mode))



def main():
  global period, pid, client

  pid = PID(-0.30, -0.0, -0.0, setpoint=600)
  pid.sample_time = 60*2
  pid.output_limits = (20, 100)

  counter = 0

  client = mqtt.Client()
  client.on_connect = on_connect
  client.on_message = on_message
  client.connect("192.168.88.111", 1883, 60)
  time.sleep(5)
  client.loop_start()
  print("MQTT co2 helper daemon started")
  client.publish("mqtt_co2_pid/status/uptime", str(0), qos=0, retain=False)



  while True:
    counter = counter + 1
    time.sleep(period)
    uptime = counter * period

    print("\n\nPublish uptime: " + str(uptime) + "s. ki:" + str(pid.Ki) + ", kp:" + str(pid.Kp) + ", kd:" + str(pid.Kd) + ", setpoint:" + str(pid.setpoint) + ", power:" + str(pid.auto_mode) + ", value:" + str(ahu_value), ", min_power:" + str(pid.min_power))
    client.publish("mqtt_co2_pid/status/uptime", str(uptime), qos=0, retain=False)

    client.publish("mqtt_co2_pid/kp", str(pid.Kp), qos=0, retain=False)
    client.publish("mqtt_co2_pid/ki", str(pid.Ki), qos=0, retain=False)
    client.publish("mqtt_co2_pid/kd", str(pid.Kd), qos=0, retain=False)
    client.publish("mqtt_co2_pid/target_value", str(pid.setpoint), qos=0, retain=False)
    client.publish("mqtt_co2_pid/min_power", str(pid.setpoint), qos=0, retain=False)

  client.loop_stop()



if __name__ == "__main__":
  main()
