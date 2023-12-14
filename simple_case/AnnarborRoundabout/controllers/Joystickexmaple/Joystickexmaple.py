"""Joystickexmaple controller."""

import re
from controller import Joystick
from controller import Robot, Supervisor
from vehicle import Driver



driver=Driver()
timestep=50

gpad = driver.getJoystick()
gpad.enable(timestep)




minimum= -32768
maximum= 32767
gear = 1
driver.setGear(gear)

def convertfeedback(raw,minimum,maximum):
    a=(raw+minimum)/(maximum-minimum)     
    return min(1, -a)

def conversteeringwheelfeedback(raw,minimum,maximum):
    a=(raw-minimum)/(maximum-minimum)
    return max(0,a)
 
  


while driver.step() != -1:

    steeringwheelfeedback=gpad.getAxisValue(0)
    steeringAngle=conversteeringwheelfeedback(steeringwheelfeedback,minimum,maximum)
    driver.setSteeringAngle(steeringAngle-0.5)
    throttlefeedback=gpad.getAxisValue(2)
    if throttlefeedback==0:
        throttle=0
    else:
        throttle=convertfeedback(throttlefeedback,minimum,maximum)
    driver.setThrottle(throttle)    
   
    
    brakefeedback=gpad.getAxisValue(1)
    if brakefeedback==0:
        brake=0
    else:
        brake=convertfeedback(brakefeedback,minimum,maximum) 
          
    driver.setBrakeIntensity(brake)
    gpadbutton = gpad.getPressedButton()         

    

            



