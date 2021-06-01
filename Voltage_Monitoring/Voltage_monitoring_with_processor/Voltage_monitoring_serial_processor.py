# ----------------------------------------------------------------------------
# Voltage Monitoring Serial Processing Program
# HPS Autopilot 2021: https://github.com/UW-BAH-HPS-Autopilot-2021/Arduino-MCU
# Reference: https://toptechboy.com/python-with-arduino-lesson-11-plotting-and-graphing-live-data-from-arduino-with-matplotlib/
# Original program created 5/31/2021 by James Lee
# Last Modified 5/31/2021 by James Lee
#
# PRIOR TO RUNNING THIS PROGRAM:
# 1. UPLOAD "Voltage_monitoring_with_processor.ino" TO ARDUINO
# OR
# 2. RESET ARDUINO IF PROGRAM ALREADY UPLOADED
# ---------------------------------------------------------------------------

#####################
# Importing Libraries
#####################
import serial                       # serial communication
import time                         # runtime verification
import matplotlib.pyplot as plt     # plotting
import drawnow as drw               # updating plots in realtime
import sympy as sp                  # symbolic math and pretty print with LaTeX
sp.init_printing()                  # enable LaTeX when plotting
import datetime                     # timestamps
import os                           # filepath configuration
import pandas as pd                 # .csv export

############################
# Global Timers (in seconds)
############################
failtimer = 10          # Fail timer for startup process
logTimer = 5*60         # Set time for data logging
plotwindow = 60         # Set range of time for rollowing plot window

##############
# Serial Setup
##############
# Verify Arduino port connection and update accordingly!
# Verify com port in Arduino IDE: Tools -> Port
arduino_port = 'COM5'   # <---- VERIFY THIS PORT
arduino_baudrate = 115200   # Arduino baudrate (MATCH TO ARDUINO!)
arduino = serial.Serial(port=arduino_port, baudrate=arduino_baudrate, timeout=.01)
logCycle = 300              # milliseconds between each log point (MATCH TO ARDUINO!)

def write_read_Arduino(x):
    """ Write given data to serial, wait 0.05s, read serial"""
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline().decode('utf-8')
    return data

# Read Arduino and return String
def read_Arduino():
    """ Read Arduino and return String"""
    data = arduino.readline().decode('utf-8')
    return data

###################
# Data Export Setup
###################
timestamp = str(datetime.datetime.now())        # Current date and time (unneccessary precision)
timestamp = timestamp[0:timestamp.find('.')-3].replace(':','')    # format time to 24hr, no colons
fileName = "Voltage Monitoring Data - " + timestamp
filePath = os.path.dirname(__file__) + "\\" + fileName # directory of this .py file
csvPath = filePath + ".csv"
headers = ['Time [s]', 'V_BATT [V]', 'V_5V [V]', 'V_3V3 [V]']
df = pd.DataFrame(data=None,columns=headers)    # Empty dataframe with column headers
df.to_csv(csvPath,index=False)                 # Create new .csv file with headers

################
# Plotting setup
################
plt.ion()                   # Turn on interactive plotting
plotdata = [[],[],[],[]]    # empty list for [t,V_BATT,V_5V,V_3V3] (full data)
finalplotdata = [[],[],[],[]] # empty list for [t,V_BATT,V_5V,V_3V3] (rolling window)
plotlabels = [r'$Time [s]$',r'$V_{BATT} [V]$',r'$V_{5V} [V]$',r'$V_{3.3V} [V]$']
linecolors = ['','go-','mo-','ro-'] # change line styles according to visual preference
rollingTitle = fileName + "(Zoomed)"    # Title for rolling window figure
# Setup pyplot figure parameters
fig = plt.figure(figsize=(15,8))

def setplotparams(title):
    """ Set pyplot figure parameters """
    plt.xlabel(plotlabels[0],fontsize=14)
    plt.ylabel(r'$Voltages \ [V]$',fontsize=14)
    plt.title(title,fontsize=18)
    plt.tick_params(labelsize=10)
    plt.grid(True)

def updatedata(data):
    """ Updates plot data
    
    data = incoming data from serial"""
    global plotdata
    global finalplotdata
    for n in range(4):
        plotdata[n].append(data[n])
        finalplotdata[n].append(data[n])

def setplotwindow(t):
    """ Removes old data to set a rolling window

    t = time in seconds"""
    global plotdata
    # solve for number of data points in window
    datasize = int(t*1000/logCycle)
    if len(plotdata[0]) > datasize:
        for n in plotdata:
            n.pop(0) # removes data at 0 index
        # update x-axis window
        plt.xlim(plotdata[0][0],plotdata[0][len(plotdata[0])-1])

def plotVoltages():
    """ Plot voltages in rolling window after data updates"""
    for n in range(1,4): # plot new data points
        plt.plot(plotdata[0],plotdata[n],linecolors[n],label=plotlabels[n])
    global rollingTitle
    setplotparams(rollingTitle)
    global plotwindow
    setplotwindow(plotwindow)  # comment out to plot entire test
    plt.legend(fontsize=12)

def plotAll():
    """ Plot voltages of entire test"""
    for n in range(1,4): # plot new data points
        plt.plot(finalplotdata[0],finalplotdata[n],linecolors[n],label=plotlabels[n])
    global fileName
    setplotparams(fileName)
    plt.legend(fontsize=12)
    plt.xlim(finalplotdata[0][0],finalplotdata[0][len(finalplotdata[0])-1])

#########
# Startup
#########
arduinoReady = False        # Check if ready flag received from Arduino
arduinoPass = "Startup pass. Starting Voltage Monitor..."   # Arduino ready flag
startupTime = time.perf_counter()   # mark startup time
runTime = 0                 # Initialize startup runtime to 0
# Send start flag to Arduino in loop until it responds
while arduinoReady != True and runTime < failtimer:
    arduinoCheck = "Ready"
    readData = write_read_Arduino(arduinoCheck) # send ready flag to arduino and read startup flag
    if readData == arduinoPass: # update
        arduinoReady = True
        print(readData)
    runTime = time.perf_counter() - startupTime

##########
# Data log
##########
logStart = time.perf_counter()  # Start time for data logging
runTime = 0                     # initialize data runtime to 0
# logs data in loop until 
while arduinoReady != False and runTime < logTimer:
    while not arduino.in_waiting and runTime < logTimer:
        runTime = time.perf_counter() - logStart    # Check timer and wait for data
    if runTime < logTimer:
        readData = read_Arduino()
        print(readData)
        datapoint = [float(x) for x in readData.split(",")]
        df = pd.DataFrame(data=[datapoint],columns=headers)
        df.to_csv(csvPath,mode='a',header=False,index=False)
        updatedata(datapoint)
        plotVoltages()
        drw.drawnow(plotVoltages)   # Update plotted figure
        plt.pause(0.000001)         # Allows for drawing to update
    runTime = time.perf_counter() - logStart

##############
# Program Exit
##############
if arduinoReady != True:
    print("Startup Failed!") # Startup fail message
else:
    # Save figures as .svg file for best resolution (open with Inkscape or web browser)
    figPath = filePath + "(Zoomed).svg"
    fig.savefig(figPath,dpi=1200)   # save final rolling window
    plotAll()
    drw.drawnow(plotAll)   # Update plotted figure
    plt.pause(0.000001)    # Allows for drawing to update
    figPath = filePath + ".svg"
    fig.savefig(figPath,dpi=1200)