# 2017-07-12 by scharette modified by sssynk
# Basic 2D IK example of AL5 + Python

# Obtain required libraries
import serial
import time
from al5dpy import al5_2D_IK, al5_moveMotors

# Constants - Speed in µs/s, 4000 is roughly equal to 360°/s or 60 RPM
#           - A lower speed will most likely be more useful in real use, such as 100 µs/s (~9°/s)
CST_SPEED_MAX = 4000
CST_SPEED_DEFAULT = 800

# Create and open a serial port
sp = serial.Serial('/dev/serial0', 115200)

# Set default values
AL5_DefaultPos = 1500;
cont = True
defaultTargetX = 4
defaultTargetY = 4
defaultTargetZ = 90
defaultTargetG = 90
defaultTargetWA = 0
defaultTargetWR = 90
defaultTargetShoulder = 90
defaultTargetElbow = 90
targetX = defaultTargetX
targetY = defaultTargetY
targetZ = defaultTargetZ
targetG = defaultTargetG
targetWA = defaultTargetWA
targetWR = defaultTargetWR
index_X = 0
index_Y = 1
index_Z = 2
index_G = 3
index_WA = 4
index_WR = 5
targetXYZGWAWR = (targetX, targetY, targetZ, targetG, targetWA, targetWR)
targetQ = "y"
motors_SEWBZWrG = (90, 90, 90, 90, 90, 90)
speed_SEWBZWrG = (CST_SPEED_DEFAULT, CST_SPEED_DEFAULT, CST_SPEED_DEFAULT, CST_SPEED_DEFAULT, CST_SPEED_DEFAULT, CST_SPEED_DEFAULT)

# Set the arm to default centered position (careful of sudden movement)
print("Default position is " + str(AL5_DefaultPos) + ".")
for i in range(0,6):
    print(("#" + str(i) + " P" + str(AL5_DefaultPos) + "\r").encode())
    sp.write(("#" + str(i) + " P" + str(AL5_DefaultPos) + "\r").encode())

targets = [[4,4], [4,10], [6,10], [6,4], [4,4]]

while cont:
    
    # loop through targets
    for target in targets:
        targetXYZGWAWR = (target[0], target[1], defaultTargetZ, defaultTargetG, defaultTargetWA, defaultTargetWR)
        errorValue = al5_2D_IK(targetXYZGWAWR)
        if isinstance(errorValue, tuple):
            motors_SEWBZWrG = errorValue
        else:
            print(errorValue)
            motors_SEWBZWrG = (defaultTargetShoulder, defaultTargetElbow, defualtTargetWA, defaultTargetZ, defaultTargetG, defaultTargetWR)
        time.sleep(1)
        errorValue = al5_moveMotors(motors_SEWBZWrG, speed_SEWBZWrG, sp)


    
    #targetQ = str(input("Quit ? (Y/N) "))
    #if targetQ == "y":
    #    cont = False

# Set all motors to idle/unpowered (pulse = 0)
print("< Idling motors... >");
for i in range(0,6):
    print(("#" + str(i) + " P" + str(0) + "\r").encode())
    sp.write(("#" + str(i) + " P" + str(0) + "\r").encode())
print("< Done >")