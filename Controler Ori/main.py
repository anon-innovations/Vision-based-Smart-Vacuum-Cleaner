import serial
from time import sleep
import threading
import controler

#To test module.
#controler.printo()

#Initialize controler variables.
controler.dVariables()

#Open COM port to tether the bot.
ser = serial.Serial('COM7', 9600)

#/////////////////// controler Module Settings //////////////////////

#Frames passed before object is considered tracked.
#controler.trackingFidelityLim = 150

#Frames to wait before moving.
#Defaults to 160
controler.waitedFrames = 160

#How close to does the robot need to be? Greater is less accurate.
#Defaults to 5.
#controler.targetProximity = 5

#GUI X, Y 
#Defaults to 0, 0
controler.guiX = 440
controler.guiY = 320

#/////////////////// controler Module Settings //////////////////////

def OpenCV():
    #Execute the controler.
    controler.otracker()

def rx():
    while(True):
        # Read the newest output from the Arduino
        if ser.readline() != "":
            rx = ser.readline()
            #This is supposed to take only the first three digits.
            rx = rx[:3]
                
            #This removes any EOL characters
            rx = rx.strip()
                
            #If the number is less than 3 digits, then it will be included
            #we get rid of it so we can have a clean str to int conversion.
            rx = rx.replace(".", "")
        
            #Here, you pass controler your raw compass data.  The very first reading it gets
            #it stores and uses to offset every other reading.  The off set amount depends on
            #which direction you have the bot facing when it's initialized.  In short, the
            #direction the robot is facing at the beginning is what it will call "North."
            controler.compass(int(rx))

def motorTimer():
        
    while(1):
        #This is for threading out the motor timer.  Allowing for control
        #over the motor burst duration.  There has to be both, something to write and
        #the motors can't be busy.
        if controler.tranx_ready == True and controler.motorBusy == False:
            ser.write(controler.tranx)
            ser.flushOutput() #Clear the buffer?s
            controler.motorBusy = True
            controler.tranx_ready = False
        if controler.motorBusy == True:
            sleep(.2) #Sets the motor burst duration.
            ser.write(controler.stop)
            sleep(.3) #Sets time inbetween motor bursts.
            controler.motorBusy = False

#Threads OpenCV stuff.        
OpenCV = threading.Thread(target=OpenCV)
OpenCV.start()

#Threads the serial functions.
rx = threading.Thread(target=rx)
rx.start()

#Threads the motor functions.
motorTimer = threading.Thread(target=motorTimer)
motorTimer.start()
