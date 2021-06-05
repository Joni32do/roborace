#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile, Font
import math as m
import time

                                       

###### Initialising the Hardware       

ev3 = EV3Brick()

#Motors
motorA = Motor(Port.A)
motorD = Motor(Port.D)
steerMotor = Motor(Port.B)

#Sensors
distanceSensor = UltrasonicSensor(Port.S3)
lightSensor = ColorSensor(Port.S4)


#Defining some expected constants
normalBrightness = lightSensor.reflection() #Either by meassuring in a nother helper function or just hard code this magic numbers
normalDistance = 30

maxRot = 70
maxLight = 70
maxDistance = 100




###### Defining some helper functions
       
def driveForward(motorA, motorD, speed = 100):
    motorA.run(speed)
    motorD.run(-speed)

def getAngleInPosition(steerMotor):
    ##########
    #To Do ###
    ##########
    steerMotor.run_target(90,0)
    return 0.0
    ##########


#Function to evaluate how much to steer according to which part of the track the bot is
def getSteeringValue(lightSensor, distanceSensor, oldLight, oldDistance, normalBrightness, normalDistance):
    light = lightSensor.reflection()
    distance = distanceSensor.distance()

    ######################
    ###   T O    D O   ###
    ######################
    lightToSteering = 1
    distanceToSteering = 1

    ######################

    #Checking for what the current sensor in the map is
    #Alternatively you could also just let one sensor for example the light sensor be dominant (easy implementation)
    #Maybe this is better done seperatly - Try it!
    #mode: 0 - light, 1 - distance, 2 - both
    if distance > maxDistance:
        mode = 0
    elif light > maxLight & distance > 20: #TODO Change -20- : Add the appropriate value for which distance the light should be the color of the tunnel
        mode = 1
    else:
        mode = 2 
    
    
    
    #Could be saved but I'm thinking about changing this somewhere else
    if mode == 0:
        steering = lightToSteering * (normalBrightness - light)
    elif mode == 1:
        steering = distanceToSteering * (normalDistance - distance)
    elif mode == 2:
        steering = 1/2 *( lightToSteering * (normalBrightness - light) +  distanceToSteering * (normalDistance - distance))

    return steering, light, distance
    

    #We could also use a lightToSteering etc. function like:
    #steering = 150*m.asin((0.008*(normalBrightness - currentBrightness))) 
    #Again this is best testet with the robot

    #Another thing to implement is to use old data to calculate the derivative
    
    



#####################################################################
#####     M A I N     P R O G R A M     #############################
#####################################################################

driveForward(motorA, motorD)
light = lightSensor.reflection()
distance = distanceSensor.distance()
angleError = getAngleInPosition()

while True:
    
    rot = steerMotor.angle() - angleError

    steering, light, distance = getSteeringValue(lightSensor, distanceSensor, light, distance, normalBrightness, normalDistance)

    
    if (-maxRot < rot & rot < maxRot):
        steerMotor.run_angle(90, steering)
        #try steerMotor.run_target and change getSteeringValue
    
