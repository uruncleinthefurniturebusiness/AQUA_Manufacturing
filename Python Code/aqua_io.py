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
LED_PIN = [10, 9]
LASER_PIN = [25, 11]
SWITCH_PINS = {8:'X', 7:'Y', 0:'Z', 5:'Z_cam'}
HALL_PIN = 1
GPIO.setmode(GPIO.BCM)   # to use the GPIO pin numbers
LED_1_status = False; LASER_1_status = False
LED_2_status = False; LASER_2_status = False
X_limit; Y_limit; Z_limit   # Read in from Mahir's section
Theta_limit = False; Z_cam_limit = False


# Pins always in output mode
# laser 1
pwm_laser1 = GPIO.setup(25, GPIO.OUT)
pwm_laser1 = GPIO.PWM(25, 1000)   # 1000 is the frequency pwm, CHANGE !!!!!!!!

# Laser 2
pwm_laser2 = GPIO.setup(11, GPIO.OUT)
pwm_laser2 = GPIO.PWM(11, 1000)   # 1000 is the frequency pwm, CHANGE !!!!!!!!

# LED 1
pwm_led1 = GPIO.setup(10, GPIO.OUT)
pwm_led1 = GPIO.PWM(10, 1000)   # 1000 is the frequency pwm, CHANGE !!!!!!!!

# LED 2
pwm_led2 = GPIO.setup(9, GPIO.OUT)
pwm_led2 = GPIO.PWM(9, 1000)   # 1000 is the frequency pwm, CHANGE !!!!!!!!

# ============================================================

# To turn the LED on/off and control the intensity
# Default intensity is 100% if no value is provided 
def led_on(num: int, state: bool, intensity: int = 100):

    match num:
        case 1:
            previous = LED_1_status
            if state:
                pwm_led1.ChangeDutyCycle(intensity)   # Intenisty is the percentage %
                print("LED 1 turned on with intensity:", intensity, "%")
            else:
                pwm_led1.ChangeDutyCycle(0)
                print("LED 1 turned off")

        case 2:
            previous = LED_2_status
            if state:
                pwm_led2.ChangeDutyCycle(intensity)   # Intenisty is the percentage %
                print("LED 2 turned on with intensity:", intensity, "%")
            else:
                pwm_led2.ChangeDutyCycle(0)
                print("LED 2 turned off")

    if state != previous:
        writeStatus()   # This means the status changed

# ============================================================

# To turn the laser on/off and control its intensity
# If no PWM intensity is mentioned, it puts it at 100%
def laser_on(num: int, state: bool, intensity: int = 100):

    match num:
        case 1:
            previous = LASER_1_status
            if state:
                pwm_laser1.ChangeDutyCycle(intensity)   # Intenisty is the percentage %
                print("Laser 1 turned on with intensity:", intensity, "%")
            else:
                pwm_laser1.ChangeDutyCycle(0)
                print("Laser 1 turned off")

        case 2:
            previous = LASER_2_status
            if state:
                pwm_laser2.ChangeDutyCycle(intensity)   # Intenisty is the percentage %
                print("Laser 2 turned on with intensity:", intensity, "%")
            else:
                pwm_laser2.ChangeDutyCycle(0)
                print("Laser 2 turned off")

    if state != previous:
        writeStatus()   # This means the status changed

# ============================================================

# To read state of limit switches and write them to a file
def readLimit(i):
    if GPIO.input(i):
        print("Limit switch of", SWITCH_PINS[i], "axis reached") 

    global X_limit, Y_limit, Z_limit, Z_cam_limit  # allow writing to global vars
    state = GPIO.input(i)   # 1 or 0
    
    match i:
        case 8:
            X_limit = bool(state)   # bool converts the 1 or 0 to True or False
        case 7:
            Y_limit = bool(state)
        case 0: 
            Z_limit = bool(state)
        case 5:
            Z_cam_limit = bool(state)

    writeStatus()   # Write new statuses to the file

# ============================================================

# When the proximity sensor senses the magnet at the preset distance 
# the Hall sensor reads a voltage (other times reads 0V). This means 
# the HALL_PIN goes HIGH and we can work with this state change
def readHall():
    previous = Theta_limit
    state = GPIO.input(HALL_PIN)
    global Theta_limit

    if state:
        Theta_limit = True
        print("Rotating bed position reset")
    else:
        Theta_limit = False

    if state != previous:
        writeStatus()

# ============================================================

# Update the file used to read statuses of peripherals
def writeStatus():
    status_file = "Status.txt"
    status_dict = {}

    # Read existing values (if file exists)
    try:
        with open(status_file, 'r') as file:
            for line in file:
                if '=' in line:
                    key, value = line.strip().split('=')
                    status_dict[key.strip()] = value.strip() == 'True'
    except FileNotFoundError:
        print("Status.txt not found, creating a new one.")

    # Update only the values your code controls
    status_dict["led_1_on"] = LED_1_status
    status_dict["led_2_on"] = LED_2_status
    status_dict["laser_1_on"] = LASER_1_status
    status_dict["laser_2_on"] = LASER_2_status
    status_dict["Theta_limit"] = Theta_limit
    status_dict["Z_cam_limit"] = Z_cam_limit
    # Don't touch X_limit, Y_limit, Z_limit — keep their existing values

    # Write updated values back to file
    try:
        with open(status_file, 'w') as file:
            for key in sorted(status_dict.keys()):
                file.write(f"{key} = {status_dict[key]}\n")
        print("Successfully updated Status.txt")
    except IOError as e:
        print(f"Error writing to file. Error given as {e}")

# ============================================================
# ============================================================

"""Main code body to be executed"""

writeStatus()   # Write initial states to file

# If limit switch which is normally off (LOW) is pulled on (HIGH)
for pin in SWITCH_PINS:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.add_event_detect(pin, GPIO.BOTH, callback=readLimit, bouncetime=200)

# Hall sensor which is normally LOW is pulled HIGH
GPIO.setup(HALL_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(HALL_PIN, GPIO.BOTH, callback=readHall, bouncetime=200)

# Makes code run continuously until keyboard is pressed
try:
    while True:
        time.sleep(0.01)
except KeyboardInterrupt:
    GPIO.cleanup()