#!/usr/bin/env python3
# See RPi-DCmotor-MQTT for detailed comments
# This version does not have MQTT
from MpythDCmotorGPIO import DCMotor
from SpeedSensor import motorEncoder
import RPi.GPIO as GPIO
import time

def valmap(value, istart, istop, ostart, ostop):
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart))

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
frequency = 2000

enApin = 12
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

enA = GPIO.PWM(enApin, frequency)
enB = GPIO.PWM(enBpin, frequency)

dc_motor1 = DCMotor(in2, in1, enA)
dc_motor2 = DCMotor(in4, in3, enB)
enA.start(0)
enB.start(0)
enc1 = motorEncoder(17)
enc1.setup()
enc2 = motorEncoder(27)
enc2.setup()

timeFaster = 20  # interval to increase speed
timeStop = timeFaster * 2   # interval to stop test
timeRPM = 1     # interval for RPM checks
time0 = time.time() # beginnning time used for speed and stop timers
timeR = time.time() # timer used for RPM checks
speed1, speed2 = 30, 30       # initial speed
okToTest = True  # flag to stop test loop
while okToTest:
  dc_motor1.forward(speed1)
  dc_motor2.forward(speed2)
  enc1.monitorRPM()
  enc2.monitorRPM()
  if (time.time() - timeR) > timeRPM:
    timeR = time.time()
    enc1.rpm, enc1.freq = enc1.reportRPM()
    enc2.rpm, enc2.freq = enc2.reportRPM()
    rpmDelta = enc1.rpm - enc2.rpm
    if rpmDelta < 100 and rpmDelta > -100:
      rpmAdj = valmap(rpmDelta, -100, 100, -10, 10)
      speed2 = speed2 + rpmAdj
      speed1 = speed1 - rpmAdj
    #print("rpm1:{0:3.0f} rpm2:{1:3.0f} freq1: {2:3.0f} freq2: {3:3.0f}".format(enc1.rpm, enc2.rpm, enc1.freq, enc2.freq))
    print("freq1: {0:3.0f} freq2: {1:3.0f} speed1: {2:3.0f} speed2: {3:3.0f}".format(enc1.freq, enc2.freq, speed1 + 100, speed2 + 100))
  if (time.time() - time0) > timeFaster:
    speed1, speed2 = 90, 90
  if (time.time() - time0) > timeStop:
    okToTest = False
print('stopping test')
enA.stop()
enB.stop()
GPIO.cleanup()