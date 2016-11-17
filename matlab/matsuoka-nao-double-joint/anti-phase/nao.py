# -*- encoding: UTF-8 -*-

import sys
import time
from naoqi import ALProxy
import almath

def setJointAngle(jointName_l, jointName_r, jointAngle_l, jointAngle_r, timeToReach):

    robotIp = "127.0.0.1"
    robotPort = 54321

    try:
        motionProxy = ALProxy("ALMotion", robotIp, robotPort)
    except Exception,e:
        print "Could not create proxy to ALMotion"
        print "Error was: ",e
        sys.exit(1)

    motionProxy.setStiffnesses("Body", 1.0)
    names      = [jointName_l, jointName_r]
    angleLists = [jointAngle_l*almath.TO_RAD, jointAngle_r*almath.TO_RAD]
    timeLists  = timeToReach
    isAbsolute = True
    fractionMaxSpeed = 1.0
    motionProxy.setAngles(names, angleLists,fractionMaxSpeed)

def getJointAngle(jointName):

    robotIp = "127.0.0.1"
    robotPort = 54321

    try:
        motionProxy = ALProxy("ALMotion", robotIp, robotPort)
    except Exception,e:
        print "Could not create proxy to ALMotion"
        print "Error was: ",e
        sys.exit(1)

    useSensors  = True
    sensorAngles = motionProxy.getAngles(jointName, useSensors)
    return sensorAngles

