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
LED_PINS = [1, 2, 3, 4, 5, 6, 7] # CHANGE !!!!!!!!!!!!!!!!!!!!
LASER_PIN = 10 # CHANGE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
GPIO.setmode(GPIO.BCM)   # to use the GPIO pin numbers


# Pins always in output mode
pwm_laser = GPIO.setup(LASER_PIN, GPIO.OUT)
pwm_laser = GPIO.PWM(LASER_PIN, 1000) # 1000 is the frequency pwm, CHANGE !!!!!!!!

for i in LED_PINS:
    GPIO.setup(i, GPIO.OUT)

# pins always in input mode
SWITCH_PINS = [8, 9, 10]
for j in SWITCH_PINS:
    GPIO.setup(j, GPIO.IN)

# ============================================================

# To turn the provided pin ON
def toggle_on(PIN_no: int):
    GPIO.output(PIN_no, GPIO.HIGH)   # ON
    print("LED turned on")

# ============================================================

# To turn the provided pin OFF  
def toggle_off(PIN_no: int):
    GPIO.output(PIN_no, GPIO.LOW)   # OFF
    print("LED turned off")

# ============================================================

# To turn on the laser
# If no PWM intensity is mentioned, it puts it at 100%
def laser_on(state: bool, intensity: int = 100):
    if state:
        pwm_laser.ChangeDutyCycle(intensity)   # Intenisty is the percentage %
        print("Duty cycle changed to " + intensity + "%")
    else:
        pwm_laser.ChangeDutyCycle(0)   # Puts it off
        print("Laser turned off")

# ============================================================

# Event listener will be added to capture when a limit switch is pushed

# To read in limit switches 
def readLimit(channel):
    print("Limit reached")



# ============================================================
# ============================================================

# Main code body to be executed

# If limit switch is pushed
GPIO.add_event_detect(SWITCH_PINS, GPIO.RISING, 
                      callback=readLimit, bouncetime=50)

# Makes code run continuously until keyboard is pressed
try:
    while True:
        time.sleep(0.01)
except KeyboardInterrupt:
    GPIO.cleanup()