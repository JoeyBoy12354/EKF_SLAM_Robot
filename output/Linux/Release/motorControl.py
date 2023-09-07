import odroid_wiringpi as wiringpi
import time
from threading import Thread
import csv

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
testPin = 22
testPin2 = 26

R = 142.5 #Distance between wheels [mm]
r = 32.5 #radius of wheel [mm]
clkID = 0

#Theta angle in radians, distance in mm
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
    angle = getAngle(LNoRot,RNoRot)

    #Forward Movement
    NoRotations = distance/(2*PI*r)
    W12_Ticks = NoRotations*20

    wiringpi.digitalWrite(RMot_Pin, 0)  # Write 1 ( HIGH ) to pin 6
    wiringpi.digitalWrite(LMot_Pin, 0)  # Write 1 ( HIGH ) to pin 7
    old_time = time.clock_gettime(clkID)
    LNoRot,RNoRot = speedSensor(W12_Ticks)
    t = time.clock_gettime(clkID)-old_time
    wiringpi.digitalWrite(RMot_Pin, 1)  # Write 0 ( LOW ) to pin 6
    wiringpi.digitalWrite(LMot_Pin, 1)  # Write 0 ( LOW ) to pin 7

    dist = getDist(LNoRot,RNoRot)
    print("time = ",t)
    
    return angle,dist,t

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
    thetaL = (LNoRot*2*PI*r/R)
    thetaR = (RNoRot*2*PI*r/R)
    angle = thetaR-thetaL
    print("Angle = left - right = ",thetaR* 180/(PI)," - ",thetaL* 180/(PI)," = ",(thetaR-thetaL)* 180/(PI))

    return angle

def getDist(LNoRot,RNoRot):
    print("LNoRot = ",LNoRot)
    print("RNoRot = ",RNoRot)
    #Determine actual distance
    distL = LNoRot*2*PI*r
    distR = RNoRot*2*PI*r
    print("DistCheck = left - right = ",distL," - ",distR," = ",distL-distR," (should be 0)")

    return distL

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
    print("TEST2")
    wiringpi.pinMode(testPin, 1)       # Set pin 6 to 1 ( OUTPUT )
    wiringpi.pinMode(testPin2, 1)       # Set pin 6 to 1 ( OUTPUT )
    wiringpi.digitalWrite(testPin, 1)  # Write 1 ( Low ) to pin 6
    wiringpi.digitalWrite(testPin2, 1)  # Write 1 ( Low ) to pin 6
    
    for i in range(0,4):
        wiringpi.digitalWrite(testPin, 0)  # Write 1 ( Low ) to pin 6
        #wiringpi.digitalWrite(testPin2, 0)  # Write 1 ( Low ) to pin 6
        print("LOW = ",wiringpi.digitalRead(testPin))
        time.sleep(0.6)
        wiringpi.digitalWrite(testPin, 1)  # Write 1 ( HIGH ) to pin 6
        #wiringpi.digitalWrite(testPin2, 0)  # Write 1 ( Low ) to pin 6
        wiringpi.digitalRead(testPin)      # Read pin 6
        print("HIGH = ",wiringpi.digitalRead(testPin),"\n")
        time.sleep(0.1)

    while(True):
        wiringpi.digitalWrite(testPin, 1)  # Write 1 ( Low ) to pin 6
        #wiringpi.digitalWrite(testPin2, 1)  # Write 1 ( Low ) to pin 6
        print("LOW = ",wiringpi.digitalRead(testPin))
        print("LOW = ",wiringpi.digitalRead(testPin2))
        time.sleep(0.6)
        wiringpi.digitalWrite(testPin, 1)  # Write 1 ( Low ) to pin 6
        #wiringpi.digitalWrite(testPin2, 1)  # Write 1 ( Low ) to pin 6
        time.sleep(0.6)

def test3():
    print("TEST")
    wiringpi.pinMode(testPin, 1)       # Set pin 6 to 1 ( OUTPUT )
    wiringpi.pinMode(testPin2, 1)       # Set pin 6 to 1 ( OUTPUT )





    # wiringpi.digitalWrite(testPin, 0)  # Write 1 ( Low ) to pin 6
    # wiringpi.digitalWrite(testPin2, 0)  # Write 1 ( Low ) to pin 6

    #wiringpi.digitalWrite(testPin, 0)  # Write 1 ( Low ) to pin 6
    wiringpi.digitalWrite(testPin2, 0)  # Write 1 ( Low ) to pin 6
    left_old = 0
    left_new = 0
    left_count = 0

    right_old = 0
    right_new = 0
    right_count = 0

    while(left_count<60 and right_count<60):
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
    values = []
    with open('motorCSV.csv','r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            values.append(float(row[0]))
    
    return values[0],values[1]

def writeOdometry(angle, distance,t):
    existingData = []
    with open('motorCSV.csv','r',newline='') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            existingData.append(row[0])

    if(len(existingData)<=2):
        existingData.append(angle)
        existingData.append(distance)
        existingData.append(t)
    else:
        existingData[2] = angle
        existingData[3] = distance
        existingData[4] = t

    with open('motorCSV.csv','w') as file:
        csv_writer = csv.writer(file)
        for row in existingData:
            csv_writer.writerow([str(row)])

    return


        



    
wiringpi.pinMode(LSS_Pin, 0)       # Set pin to 0 ( INPUT )
wiringpi.pinMode(RSS_Pin, 0)       # Set pin to 0 ( INPUT )
wiringpi.pinMode(RMot_Pin, 1)       # Set pin 6 to 1 ( OUTPUT )
wiringpi.pinMode(LMot_Pin, 1)       # Set pin 7 to 1 ( OUTPUT )\
#while True:
#    wiringpi.digitalWrite(testPin, 0)  # Write 1 ( HIGH ) to pin 6
#test3()

# angle,distance = readInstructions()
# angle = PI/2
# distance = 0
# angle,distance,t = motorControl(angle,distance)
# writeOdometry(angle,distance,t)
test3()






