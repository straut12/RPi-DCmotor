#!/usr/bin/env python3
# LM393 encoder.  Can use 3.3V or 5V power
import RPi.GPIO as GPIO
from time import time

class motorEncoder:
  def __init__(self, pinE):    
    self.pinE = pinE          # data pin on RPi
    self.intervalCheck = 0.5  # how frequently to calculate the rpm
    self.time0 = time()
    #self.time1 = self.time0
    self.counterR = 0
    self.rpm = 0
    self.freq = 0
    self.SLOTS = 40  # of interrupts in one revolution

  def handle_interrupt(self, channel):
    self.counterR +=1

  def setup(self):         # setup GPIO and interrupt event
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(self.pinE, GPIO.IN)
    GPIO.add_event_detect(self.pinE, GPIO.RISING, callback=self.handle_interrupt) 

  def monitorRPM(self):  # if time interval met, based on IRQ, will calculate freq/rpm
    if (time() - self.time0) > self.intervalCheck:
      self.freq = int(self.counterR/(time() - self.time0))
      self.rpm = int(self.freq/self.SLOTS*60)
      self.counterR = 0          # reset the counter
      self.time0 = time()        # reset the time 0

  def reportRPM(self):
    return self.rpm, self.freq   # report the rpm and freq
 

if __name__ == "__main__":
  encoder1 = motorEncoder(27)  # pin 27 and 17 used for testing
  encoder1.setup()
  encoder2 = motorEncoder(17)
  encoder2.setup()
  while True:
    encoder1.rpm, encoder1.freq = encoder1.reportRPM()
    encoder2.rpm, encoder2.freq = encoder2.reportRPM()
    print("rpm1:{0} rpm2:{1} freq1: {2} freq2: {3}".format(encoder1.rpm, encoder2.rpm, encoder1.freq, encoder2.freq))
