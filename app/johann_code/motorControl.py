import odroid_wiringpi as wiringpi
import time
from threading import Thread
import csv

#Test git

# One of the following MUST be called before using IO functions:
wiringpi.wiringPiSetup()      # For sequential pin numbering
# OR
#wiringpi.wiringPiSetupSys()   # For /sys/class/gpio with GPIO pin numbering
# OR
#wiringpi.wiringPiSetupGpio()  # For GPIO pin numbering

PI = 3.14159265358979
LMot_Pin = 22
RMot_Pin = 26
LSS_Pin = 3
RSS_Pin = 4
testPin = 3
testPin2 = 4

R = 14.25 #Distance between wheels [cm]
r = 3.25 #radius of wheel [cm]

#Theta angle in radians, distance in cm
def motorControl(theta,distance):
    # fullvelocity = 100 #This is the max velocity of motor
    # fullTime = 1

    LNoRot=0
    RNoRot=0

    #Rotation Movement
    #Left Turn
    if(theta>0):
        NoRotations = (R*theta)/(2*PI*r)

        print("NoRotation = ",NoRotations)
        W1_Ticks = NoRotations*20
        print("NoTicks = ",W1_Ticks)

        wiringpi.digitalWrite(RMot_Pin, 0)  # Write 1 ( HIGH ) to pin 6
        LNoRot,RNoRot = speedSensor(W1_Ticks)
        wiringpi.digitalWrite(RMot_Pin, 1)  # Write 0 ( LOW ) to pin 6
    #Right Turn
    elif(theta<0):
        NoRotations = (R*abs(theta))/(2*PI*r)
        W2_Ticks = NoRotations*20

        wiringpi.digitalWrite(LMot_Pin, 0)  # Write 1 ( HIGH ) to pin 7
        LNoRot,RNoRot = speedSensor(W2_Ticks)
        wiringpi.digitalWrite(LMot_Pin, 1)  # Write 0 ( LOW ) to pin 7

    print("LNoRot = ",LNoRot)
    print("RNoRot = ",RNoRot)
    getAngle(LNoRot,RNoRot)

    #Forward Movement
    NoRotations = distance/(2*PI*r)
    W12_Ticks = NoRotations*20

    wiringpi.digitalWrite(RMot_Pin, 0)  # Write 1 ( HIGH ) to pin 6
    wiringpi.digitalWrite(LMot_Pin, 0)  # Write 1 ( HIGH ) to pin 7
    LNoRot,RNoRot = speedSensor(W12_Ticks)
    wiringpi.digitalWrite(RMot_Pin, 1)  # Write 0 ( LOW ) to pin 6
    wiringpi.digitalWrite(LMot_Pin, 1)  # Write 0 ( LOW ) to pin 7

    getDist(LNoRot,RNoRot)
    
    return

def speedSensor(NoTicks):
    left_old = 0
    left_new = 0
    left_count = 0

    right_old = 0
    right_new = 0
    right_count = 0

    while(left_count<=NoTicks and right_count<=NoTicks):
        left_new = wiringpi.digitalRead(LSS_Pin)
        right_new = wiringpi.digitalRead(RSS_Pin)

        if(left_old == 0 and left_new == 1):
            left_count += 1
            #print("Left = ",left_count)
        
        if(right_old == 0 and right_new == 1):
            right_count += 1
            #print("Right = ",right_count)

        left_old = left_new
        right_old = right_new
    
    left = left_count/20
    right = right_count/20

    

    return left,right

def getAngle(LNoRot,RNoRot):
    #Determine actual angle
    thetaL = (LNoRot*2*PI*r/R) * 180/(2*PI)
    thetaR = (RNoRot*2*PI*r/R) * 180/(2*PI)
    print("Angle = left - right = ",thetaR," - ",thetaL," = ",thetaR-thetaL)

def getDist(LNoRot,RNoRot):
    print("LNoRot = ",LNoRot)
    print("RNoRot = ",RNoRot)
    #Determine actual distance
    distL = LNoRot*2*PI*r
    distR = RNoRot*2*PI*r
    print("DistCheck = left - right = ",distL," - ",distR," = ",distL-distR," (should be 0)")

def test():
    print("TEST")
    wiringpi.pinMode(testPin, 1)       # Set pin 6 to 1 ( OUTPUT )
    while(True):
        wiringpi.digitalWrite(testPin, 0)  # Write 1 ( Low ) to pin 6
        print("LOW = ",wiringpi.digitalRead(testPin))
        time.sleep(0.6)
        wiringpi.digitalWrite(testPin, 1)  # Write 1 ( HIGH ) to pin 6
        wiringpi.digitalRead(testPin)      # Read pin 6
        print("HIGH = ",wiringpi.digitalRead(testPin),"\n")
        time.sleep(0.1)

def test2():
    print("TEST")
    wiringpi.pinMode(testPin, 1)       # Set pin 6 to 1 ( OUTPUT )
    wiringpi.pinMode(testPin2, 1)       # Set pin 6 to 1 ( OUTPUT )
    
    for i in range(0,5):
        wiringpi.digitalWrite(testPin, 0)  # Write 1 ( Low ) to pin 6
        wiringpi.digitalWrite(testPin2, 0)  # Write 1 ( Low ) to pin 6
        print("LOW = ",wiringpi.digitalRead(testPin))
        time.sleep(0.6)
        wiringpi.digitalWrite(testPin, 1)  # Write 1 ( HIGH ) to pin 6
        wiringpi.digitalWrite(testPin2, 0)  # Write 1 ( Low ) to pin 6
        wiringpi.digitalRead(testPin)      # Read pin 6
        print("HIGH = ",wiringpi.digitalRead(testPin),"\n")
        time.sleep(0.1)

    while(True):
        wiringpi.digitalWrite(testPin, 1)  # Write 1 ( Low ) to pin 6
        wiringpi.digitalWrite(testPin2, 1)  # Write 1 ( Low ) to pin 6
        print("LOW = ",wiringpi.digitalRead(testPin))
        print("LOW = ",wiringpi.digitalRead(testPin2))
        time.sleep(0.6)
        wiringpi.digitalWrite(testPin, 1)  # Write 1 ( Low ) to pin 6
        wiringpi.digitalWrite(testPin2, 1)  # Write 1 ( Low ) to pin 6
        time.sleep(0.6)

def test3():
    print("TEST")
    wiringpi.pinMode(testPin, 1)       # Set pin 6 to 1 ( OUTPUT )
    wiringpi.pinMode(testPin2, 1)       # Set pin 6 to 1 ( OUTPUT )





    # wiringpi.digitalWrite(testPin, 0)  # Write 1 ( Low ) to pin 6
    # wiringpi.digitalWrite(testPin2, 0)  # Write 1 ( Low ) to pin 6

    wiringpi.digitalWrite(testPin, 0)  # Write 1 ( Low ) to pin 6
    #wiringpi.digitalWrite(testPin2, 0)  # Write 1 ( Low ) to pin 6
    left_old = 0
    left_new = 0
    left_count = 0

    right_old = 0
    right_new = 0
    right_count = 0

    while(left_count<60):
        left_new = wiringpi.digitalRead(LSS_Pin)
        right_new = wiringpi.digitalRead(RSS_Pin)

        if(left_old == 0 and left_new == 1):
            left_count += 1
            print("Left = ",left_count)
        
        if(right_old == 0 and right_new == 1):
            right_count += 1
            print("Right = ",right_count)

        left_old = left_new
        right_old = right_new
    wiringpi.digitalWrite(testPin, 1)  # Write 1 ( Low ) to pin 6
    wiringpi.digitalWrite(testPin2, 1)  # Write 1 ( Low ) to pin 6
            
    print("Left Sensor = ",left_count/20," rotations")
    print("Right Sensor = ",right_count/20," rotations")
        
    
    time.sleep(0.5)
    
def test4():
    left_old = 0
    left_new = 0
    left_count = 0
    right = 0

    checked = False
    while(True):
        left_new = wiringpi.digitalRead(LSS_Pin )

        if(left_old == 0 and left_new == 1):
            left_count += 1
            print("Left = ",left_count)

        left_old = left_new

def readInstructions():
    x_coord = []
    y_coord = []
    with open('motorCSV.csv','r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x_coord.append(float(row[0]))
            y_coord.append(float(row[1]))

def writeOdometry():
    x_coord = []
    y_coord = []
    with open('motorCSV.csv','r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x_coord.append(float(row[0]))
            y_coord.append(float(row[1]))



    
wiringpi.pinMode(LSS_Pin, 0)       # Set pin to 0 ( INPUT )
wiringpi.pinMode(RSS_Pin, 0)       # Set pin to 0 ( INPUT )
wiringpi.pinMode(RMot_Pin, 1)       # Set pin 6 to 1 ( OUTPUT )
wiringpi.pinMode(LMot_Pin, 1)       # Set pin 7 to 1 ( OUTPUT )\
#while True:
#    wiringpi.digitalWrite(testPin, 0)  # Write 1 ( HIGH ) to pin 6
#test3()
#motorControl(PI/2,0)
test2()






