from vpython import *
from time import *
import numpy as np
import math
import serial
Arduino_COM = 'COM3'
Arduino_Baudrate = 38400
Arduino_Serial = serial.Serial( Arduino_COM, Arduino_Baudrate )
sleep(1)

scene.range=10

scene.forward = vector(-1,-1,-1)

scene.width = 800
scene.height = 800

good_to_read = False

xarrow=arrow(lenght=2, shaftwidth=.1, color=color.red,axis=vector(1,0,0))
yarrow=arrow(lenght=2, shaftwidth=.1, color=color.green,axis=vector(0,1,0))
zarrow=arrow(lenght=4, shaftwidth=.1, color=color.blue,axis=vector(0,0,1))

frontArrow=arrow(length=4,shaftwidth=.1,color=color.purple,axis=vector(1,0,0))
upArrow=arrow(length=1,shaftwidth=.1,color=color.magenta,axis=vector(0,1,0))
sideArrow=arrow(length=2,shaftwidth=.1,color=color.orange,axis=vector(0,0,1))

box=box(length=4,width=3,height=2,opacity=.8,pos=vector(0,0,0))

customobject = compound([box])
while (True):
    while (Arduino_Serial.inWaiting()==0):
        pass
    # Assumption is data is in the following format:
    # "uptime, roll, pitch, yaw, gyrotemperatureC \n\r"
    data_raw = Arduino_Serial.readline()
    data_str = str(data,'utf-8')
    if( False == good_to_read ):
        print(data_str)
    if( False == good_to_read and data_str == "Begin Outputting Data!\r\n"):
        print("Found Begin Outputting Data!")
        good_to_read = True
        continue
    if( False == good_to_read and 5 == data_str.count(',') and 0 ==  data_str.count("(") and 0 == data_str.count(")")):
        good_to_read = True
        continue
    if( good_to_read ):
        data_split = data_str.split(",")
        uptime = float(data_split[0])
        roll = math.radians( float(data_split[1]) )
        pitch = math.radians( float(data_split[2]) )
        yaw = math.radians( float(data_split[3]) )
        tempC = float(data_split[4])
        print("Uptime= ",uptime, "Roll= ",roll*toDeg," Pitch= ",pitch*toDeg,"Yaw= ",yaw*toDeg, "TempC= ", tempC)
        rate(75)
        k = vector(cos(yaw)*cos(pitch), sin(pitch),sin(yaw)*cos(pitch))
        y = vector(0,1,0)
        s = cross(k,y)
        v = cross(s,k)
        vrot = v*cos(roll) + cross(k,v) * sin(roll)
    
        frontArrow.axis = k
        sideArrow.axis = cross(k,vrot)
        upArrow.axis = vrot
        customobject.axis = k
        customobject.up = vrot
        sideArrow.length = 2
        frontArrow.length = 4
        upArrow.length = 1