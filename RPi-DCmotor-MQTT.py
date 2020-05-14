#!/usr/bin/env python3
from MpythDCmotorGPIO import DCMotor
from SpeedSensor import motorEncoder
import RPi.GPIO as GPIO
import sys, re, json
from time import time
from typing import NamedTuple
import paho.mqtt.client as mqtt

#to get input during executing python command
#import sys
#arg1 = sys.argv[1]
#arg2 = sys.argv[2]

# Some useful functions
def valmap(value, istart, istop, ostart, ostop):
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

#=======   SETUP MQTT =================#
MQTT_ADDRESS = '10.0.0.22'
#MQTT_USER = 'chawn1272'
#MQTT_PASSWORD = 'dewberry2233'
MQTT_TOPIC = 'cocoacrisp/buggy/+'  # + means one or more occurrence
MQTT_REGEX = 'cocoacrisp/([^/]+)/([^/]+)'  #regular expression.  ^ means start with
MQTT_CLIENT_ID = 'cocoacrisp'
topic_pub = 'cocoacrisp/buggy/all'

# create call back functions and then link them to the mqtt callback below in main program
def on_connect(client, userdata, flags, rc):
    """ The callback for when the client receives a CONNACK response from the server."""
    print('Connected with result code ' + str(rc))  #str() returns the nicely printable representation of a given object.
    client.subscribe(MQTT_TOPIC)

#on message will receive data from client 
def on_message(client, userdata, msg):
    """The callback for when a PUBLISH message is received from the server."""
    print(msg.topic + ' ' + str(msg.payload))

#on publish will send data to client
def on_publish(client, userdata, mid):
    print("mid: "+str(mid))

#=======   SETUP GPIO =================#
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
frequency = 2000

enApin = 12   # assign pins
in1 = 24
in2 = 23
in3 = 8
in4 = 25
enBpin = 18

GPIO.setup(enApin, GPIO.OUT)
GPIO.setup(in1, GPIO.OUT)
GPIO.setup(in2, GPIO.OUT)
GPIO.setup(in3, GPIO.OUT)
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(enBpin, GPIO.OUT)

enA = GPIO.PWM(enApin, frequency)  # setup PWM pins and frequency
enB = GPIO.PWM(enBpin, frequency)

dc_motor1 = DCMotor(in2, in1, enA) # assign motors
dc_motor2 = DCMotor(in4, in3, enB)
enA.start(0)                       # start PWM at 0 duty cycle
enB.start(0)

#=======   Speed Sensor Setup =================#
speedSensorON = False  # turn speed sensor on/off
if speedSensorON:
  enc1 = motorEncoder(17)
  enc1.setup()
  enc2 = motorEncoder(27)
  enc2.setup()

#==== variables used for testing loop ======#
timeFaster = 10            # interval to increase speed
timeStop = timeFaster * 2  # interval to stop test
okToTest = True            # flag to stop test loop

#==== variables ====================#
timeReportIntv = 1       # interval for RPM checks
time0 = time()           # beginnning time used for reporting check
timeR = time()           # timer used for reporting check
speed1, speed2 = 30, 30  # initial speed
buggyD = {}              # dictionary to send data as JSON thru MQTT

#==== start mqtt functions ===========#
mqtt_client = mqtt.Client(MQTT_CLIENT_ID)
# mqtt_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
mqtt_client.on_connect = on_connect  #bind call back function
mqtt_client.on_message = on_message  #bind function to be used when PUBLISH messages are found
mqtt_client.on_publish = on_publish  #bind function for publishing
mqtt_client.connect(MQTT_ADDRESS, 1883)  # connect to the mqtt
mqtt_client.loop_start()   # other option is client.loop_forever() but it is blocking

while okToTest:
  dc_motor1.forward(speed1)
  dc_motor2.forward(speed2)
  if speedSensorON:         # start monitoring rpm.. can be more frequent than the reportRPM below
    enc1.monitorRPM()
    enc2.monitorRPM()
  if (time() - timeR) > timeReportIntv:
    timeR = time()   # reset initial reporting timer
    if speedSensorON:
      enc1.rpm, enc1.freq = enc1.reportRPM()
      enc2.rpm, enc2.freq = enc2.reportRPM()
      rpmDelta = enc1.rpm - enc2.rpm
      if rpmDelta < 100 and rpmDelta > -100:
        rpmAdj = int(valmap(rpmDelta, -100, 100, -10, 10))
        speed2 = speed2 + rpmAdj
        speed1 = speed1 - rpmAdj
      #print("rpm1:{0:3.0f} rpm2:{1:3.0f} freq1: {2:3.0f} freq2: {3:3.0f}".format(enc1.rpm, enc2.rpm, enc1.freq, enc2.freq))
      print("freq1: {0} freq2: {1} speed1: {2} speed2: {3}".format(enc1.freq, enc2.freq, speed1 + 100, speed2 + 100))
      buggyD = {"irpm1":str(enc1.rpm), "irpm2":str(enc2.rpm), "ispeed1":str(speed1), "ispeed2":str(speed2)}
    else:
      buggyD = {"ispeed1":str(speed1), "ispeed2":str(speed2)}
    buggyMQTT=json.dumps(buggyD)
    mqtt_client.publish(topic_pub, buggyMQTT)
  if (time() - time0) > timeFaster:
    speed1, speed2 = 90, 90
  if (time() - time0) > timeStop:
    okToTest = False
print('stopping test')
enA.stop()
enB.stop()
GPIO.cleanup()