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
MQTT_TOPIC = 'cocoacrisp/buggy/motor'  # + means one or more occurrence
MQTT_REGEX = 'cocoacrisp/([^/]+)/([^/]+)'  #regular expression.  ^ means start with
MQTT_CLIENT_ID = 'cocoacrisp'
topic_pub = 'cocoacrisp/buggy/speedSensor'

class motorObj(NamedTuple):
    location: str
    device: str
    motor: str
    value: float

# create call back functions and then link them to the mqtt callback below in main program
def on_connect(client, userdata, flags, rc):
    """ The callback for when the client receives a CONNACK response from the server."""
    print('Connected with result code ' + str(rc))  #str() returns the nicely printable representation of a given object.
    client.subscribe(MQTT_TOPIC)

#on message will receive data from client 
def on_message(client, userdata, msg):
    global speed1, speed2
    """The callback for when a PUBLISH message is received from the server."""
    #print(msg.topic + ' ' + str(msg.payload))
    sensor_data = str(msg.payload.decode("utf-8", "ignore"))     #set sensor_data to Class SensorData return
    #sensor_data = parse_mqtt_message(msg.topic, msg.payload.decode('utf-8'))
    dataD = json.loads(sensor_data) # decode json data
    speed1 = dataD[0]['right']
    speed2 = dataD[0]['left']
    moving = dataD[0]['moving']
    print(moving)
    
    # not using the parse-mqtt-message right now
def parse_mqtt_message(topic, payload):
    match = re.match(MQTT_REGEX, topic)      # check if topic matches the test/sensor/temperature format
    if match.group(2) == 'motor':
        location = match.group(0)
        device = match.group(1)
        motor = match.group(2)
        if device == 'status':
            return None
        return motorObj(location, device, motor, payload)  # returns the data from Class ServoObj
    
#on publish will send data to client
def on_publish(client, userdata, mid):
    pass
    #print("mid: "+str(mid))

#=======   SETUP GPIO =================#
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
frequency = 1500

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
speedSensorON = True  # turn speed sensor on/off
if speedSensorON:
  enc1 = motorEncoder(17)
  enc1.setup()
  enc2 = motorEncoder(27)
  enc2.setup()

#==== variables used for testing loop ======#
timeStop = 600  # interval to stop test
okToTest = True            # flag to stop test loop

#==== variables ====================#
moving = 'start-up'
timeReportIntv = 1       # interval for RPM checks
time0 = time()           # beginnning time used for reporting check
timeR = time()           # timer used for reporting check
speed1, speed2 = 0, 0  # initial speed 
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
  speed1 = min(100, max(-100, speed1))
  speed2 = min(100, max(-100, speed2))
  dc_motor1.forward(speed1) if speed1 > 0 else dc_motor1.backwards(abs(speed1)) if speed1 < 0 else dc_motor1.stop()
  dc_motor2.forward(speed2) if speed2 > 0 else dc_motor2.backwards(abs(speed2)) if speed2 < 0 else dc_motor2.stop()
  if speedSensorON:         # start monitoring rpm.. can be more frequent than the reportRPM below
    enc1.monitorRPM()
    enc2.monitorRPM()
  if (time() - timeR) > timeReportIntv:
    timeR = time()   # reset initial reporting timer
    enc1.rpm, enc1.freq = enc1.reportRPM()
    enc2.rpm, enc2.freq = enc2.reportRPM()
    buggyD = {"irpm1":str(enc1.rpm), "irpm2":str(enc2.rpm), "ispeed1":str(speed1), "ispeed2":str(speed2)}
    buggyMQTT=json.dumps(buggyD)
    mqtt_client.publish(topic_pub, buggyMQTT)
    if speedSensorON and moving == 'straight1':
      enc1.rpm, enc1.freq = enc1.reportRPM()
      enc2.rpm, enc2.freq = enc2.reportRPM()
      rpmDelta = enc1.rpm - enc2.rpm
      if rpmDelta < 100 and rpmDelta > -100:
        rpmAdj = int(valmap(rpmDelta, -100, 100, -10, 10))
        speed2 = speed2 + rpmAdj
        speed1 = speed1 - rpmAdj
      #buggyD = {"irpm1":str(enc1.rpm), "irpm2":str(enc2.rpm), "ispeed1":str(speed1), "ispeed2":str(speed2)}
    #else:
      #buggyD = {"ispeed1":str(speed1), "ispeed2":str(speed2)}
    #buggyMQTT=json.dumps(buggyD)
    #mqtt_client.publish(topic_pub, buggyMQTT)
  if (time() - time0) > timeStop:
    okToTest = False
print('stopping test')
enA.stop()
enB.stop()
GPIO.cleanup()
