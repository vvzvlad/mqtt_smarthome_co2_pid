#!/usr/bin/env -S python3 -u

#
#############################################################################
#
# helper to mqtt for smarthome
#
#############################################################################
#

import paho.mqtt.client as mqtt
import time

import os
import json
import sys
import random

period = 60

sensor_temperature = 0
valve_sensor_temperature = 0
valve_sensor_calibration = 0

def on_connect(client, userdata, flags, rc):
  print("Connected with result code " + str(rc))
  client.subscribe("zigbee/sensor_082a")
  client.subscribe("zigbee/thermostat_0403")
  client.subscribe("mqtt_thermo_helper/period/set")
  client.publish("mqtt_thermo_helper/status", payload="mqtt thermp helper daemon started", qos=0, retain=False)
  client.publish("zigbee/thermostat_0403/set", str("{\"eco_temperature\": 25}"), qos=0, retain=False)


def on_message(client, userdata, msg):
  #print("Received MQTT message:" + msg.topic + ": " + str(msg.payload))
  global sensor_temperature, valve_sensor_temperature, valve_sensor_calibration

  if msg.topic == "mqtt_thermo_helper/period/set":
    period = atof(msg.payload)
    print("New period: " + str(period))

  if msg.topic == "zigbee/sensor_082a":
    json_data = json.loads(msg.payload)
    sensor_temperature = json_data.get('temperature')
    print("Room sensor message received, temperature: " + str(sensor_temperature) + "°C")
  if msg.topic == "zigbee/thermostat_0403":
    json_data = json.loads(msg.payload)
    valve_sensor_temperature = json_data.get('local_temperature')
    valve_sensor_calibration = json_data.get('local_temperature_calibration')
    print("Thermostat message received, thermostat local temperature: " + str(valve_sensor_temperature) + "°C (with calibration), calibration: " + str(valve_sensor_calibration) + "°C" + ", without calibration: " + str(valve_sensor_temperature-valve_sensor_calibration) + "°C")
    client.publish("zigbee/thermostat_0403/calculated", str("{\"temperature_raw\": " + str(valve_sensor_temperature-valve_sensor_calibration) + "}"), qos=0, retain=False)


def main():
  global period
  counter = 0
  client = mqtt.Client()
  client.on_connect = on_connect
  client.on_message = on_message
  client.connect("192.168.88.111", 1883, 60)
  time.sleep(5)
  client.loop_start()
  print("MQTT thermo helper daemon started")
  client.publish("mqtt_thermo_helper/status/uptime", str(0), qos=0, retain=False)

  while True:
    counter = counter + 1
    time.sleep(period)
    print("\n\nCycle start")
    uptime = counter * period

    print("Publish uptime: " + str(uptime) + "s")
    client.publish("mqtt_thermo_helper/status/uptime", str(uptime), qos=0, retain=False)

    if sensor_temperature != 0 and valve_sensor_temperature != 0:
      new_calibration_diff = round(sensor_temperature-valve_sensor_temperature, 1)
      new_full_calibration_value = new_calibration_diff + valve_sensor_calibration

      print("Run calibration procedure. \n\tRoom temp: {} \n\tValve temp: {} \n\tValve old calibration: {} \n\tNew calibration diff: {} \n\tFull calibration value: {}".format(str(sensor_temperature), str(valve_sensor_temperature), str(valve_sensor_calibration), str(new_calibration_diff), str(new_full_calibration_value)))

      client.publish("zigbee/thermostat_0403/set", str("{\"local_temperature_calibration\": " + str(new_full_calibration_value) + "}"), qos=0, retain=False)

  client.loop_stop()



if __name__ == "__main__":
  main()
