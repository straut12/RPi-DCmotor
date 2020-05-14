# Created by https://RandomNerdTutorials.com

#This file includes a class to control DC motors
import RPi.GPIO as GPIO

class DCMotor:
  #the min_duty and max_duty are defined for 15000Hz frequency
  #you can pass as arguments
  def __init__(self, pin1, pin2, enable_pin, min_duty=0, max_duty=100):
    self.pin1 = pin1
    self.pin2= pin2
    self.enable_pin = enable_pin
    self.min_duty = min_duty
    self.max_duty = max_duty
  
  #speed value can be between 0 and 100
  def forward(self, speed):
    self.speed = speed
    self.enable_pin.ChangeDutyCycle(self.duty_cycle(self.speed))
    GPIO.output(self.pin1, GPIO.HIGH)
    GPIO.output(self.pin2, GPIO.LOW)

  def backwards(self, speed):
    self.speed = speed
    self.enable_pin.ChangeDutyCycle(self.duty_cycle(self.speed))
    GPIO.output(self.pin1, GPIO.LOW)
    GPIO.output(self.pin2, GPIO.HIGH)

  def stop(self):
    self.enable_pin.ChangeDutyCycle(0)
    GPIO.output(self.pin1, GPIO.LOW)
    GPIO.output(self.pin2, GPIO.LOW)
        
  def duty_cycle(self, speed):
    if self.speed <= 0 or self.speed > 100:
      duty_cycle = 0
    else:
      duty_cycle = self.speed
    return duty_cycle