""" 
This module is designed to be used in the AQUA management
project task with reading sensors, limit switches and controlling
LEDs and Lasers through MOSFET driven circuits while also providing
error handling and status feedback 

"""

import RPi.GPIO as GPIO # type: ignore
import time

# ============================================================

# Global variables
LED_PIN = 10
LASER_PIN = [9, 25]
SWITCH_PINS = {8:'X', 7:'Y', 0:'Z', 1:'Theta', 5:'Z_cam'}
GPIO.setmode(GPIO.BCM)   # to use the GPIO pin numbers


# Pins always in output mode
for i in LASER_PIN:
    pwm_laser = GPIO.setup(i, GPIO.OUT)
    pwm_laser = GPIO.PWM(i, 1000)   # 1000 is the frequency pwm, CHANGE !!!!!!!!

pwm_led = GPIO.setup(LED_PIN, GPIO.OUT)
pwm_led = GPIO.PWM(LED_PIN, 1000)   # 1000 is the frequency pwm, CHANGE !!!!!!!!

# ============================================================

# To turn the LED on/off and control the intensity
# Default intensity is 100% if no value is provided 
def led_on(state: bool, intensity: int = 100):
    if state:
        pwm_laser.ChangeDutyCycle(intensity)   # Intenisty is the percentage %
        print("LED turned on with intensity:", intensity, "%")
    else:
        pwm_led.ChangeDutyCycle(0)
        print("LED turned off")

# ============================================================

# To turn the laser on/off and control its intensity
# If no PWM intensity is mentioned, it puts it at 100%
def laser_on(state: bool, intensity: int = 100):
    if state:
        pwm_laser.ChangeDutyCycle(intensity)   # Intenisty is the percentage %
        print("Laser turned on with intensity:", intensity, "%")
    else:
        pwm_laser.ChangeDutyCycle(0)   # Puts it off
        print("Laser turned off")

# ============================================================

"""Final output of this function still to be determined"""
# To read in limit switches 
def readLimit(i):
    print("Limit switch of", SWITCH_PINS[i], "axis reached")


# ============================================================

"""Main code body to be executed"""

# If limit switch which is normally off (LOW) is pulled on (HIGH)
for pin in SWITCH_PINS:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.add_event_detect(pin, GPIO.RISING, callback=readLimit, bouncetime=200)

# Makes code run continuously until keyboard is pressed
try:
    while True:
        time.sleep(0.01)
except KeyboardInterrupt:
    GPIO.cleanup()