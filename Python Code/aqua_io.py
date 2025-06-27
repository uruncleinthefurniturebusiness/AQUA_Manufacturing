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
LED_status = False; LASER_status = False
X_limit = False; Y_limit = False; Z_limit = False
Theta_limit = False; Z_cam_limit = False


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
    previous = LED_status
    if state:
        pwm_laser.ChangeDutyCycle(intensity)   # Intenisty is the percentage %
        print("LED turned on with intensity:", intensity, "%")
    else:
        pwm_led.ChangeDutyCycle(0)
        print("LED turned off")

    if state != previous:
        writeStatuses()   # This means the status changed

# ============================================================

# To turn the laser on/off and control its intensity
# If no PWM intensity is mentioned, it puts it at 100%
def laser_on(state: bool, intensity: int = 100):
    previous = LASER_status
    if state:
        pwm_laser.ChangeDutyCycle(intensity)   # Intenisty is the percentage %
        print("Laser turned on with intensity:", intensity, "%")
    else:
        pwm_laser.ChangeDutyCycle(0)   # Puts it off
        print("Laser turned off")

    if state != previous:
        writeStatuses()   # This means the status changed

# ============================================================

# To read state of limit switches and write them to a file
def readLimit(i):
    if GPIO.input(i):
        print("Limit switch of", SWITCH_PINS[i], "axis reached") 

    global X_limit, Y_limit, Z_limit, Theta_limit, Z_cam_limit  # allow writing to global vars
    state = GPIO.input(i)   # 1 or 0
    
    match i:
        case 8:
            X_limit = bool(state)   # bool converts the 1 or 0 to True or False
        case 7:
            Y_limit = bool(state)
        case 0: 
            Z_limit = bool(state)
        case 1:
            Theta_limit = bool(state)
        case 5:
            Z_cam_limit = bool(state)

    writeStatuses()   # Write new statuses to the file

# ============================================================

# Update the file used to read statuses of peripherals
def writeStatuses():
    try:
        with open("Status.txt", 'w') as file:
            file.write("led_on =", LED_status, "\n"
                    "laser_on =", LASER_status, "\n"
                    "X_limit =", X_limit, "\n"
                    "Y_limit =", Y_limit, "\n"
                    "Z_limit =", Z_limit, "\n"
                    "Theta_limit =", Theta_limit, "\n"
                    "Z_cam_limit =", Z_cam_limit)
        print("Successfully updated Status.txt")
    except IOError as e:
        print(f"Error writing to file. Error given as {e}")

# ============================================================

"""Main code body to be executed"""

writeStatuses()   # Write initial states to file

# If limit switch which is normally off (LOW) is pulled on (HIGH)
for pin in SWITCH_PINS:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.add_event_detect(pin, GPIO.BOTH, callback=readLimit, bouncetime=200)

# Makes code run continuously until keyboard is pressed
try:
    while True:
        time.sleep(0.01)
except KeyboardInterrupt:
    GPIO.cleanup()