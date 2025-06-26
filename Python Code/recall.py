i = 9000
SWITCH_PINS = {8:'X', 7:'Y', 0:'Z'}

def readLimit(i):
    print("Limit switch of ", SWITCH_PINS[i], " axis reached")

readLimit(8)