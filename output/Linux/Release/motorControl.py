import odroid_wiringpi as wiringpi
import time
from threading import Thread
import csv
import sonarControl

# One of the following MUST be called before using IO functions:
wiringpi.wiringPiSetup()      # For sequential pin numbering
# OR
#wiringpi.wiringPiSetupSys()   # For /sys/class/gpio with GPIO pin numbering
# OR
#wiringpi.wiringPiSetupGpio()  # For GPIO pin numbering

PI = 3.14159265358979
RMot_Pin = 22
LMot_Pin = 26
LSS_Pin = 3
RSS_Pin = 4
testPin = 22
testPin2 = 26

R = 142.5 #Distance between wheels [mm]
r = 32.5 #radius of wheel [mm]

#Avoidance
clockAngleInit = 0.959931 #55 Degree inital rotation
clockAngleStep = 0.261799 #15 Degree step rotation

#Theta angle in radians, distance in mm
def motorControl(theta,distance):
    LNoRot=0
    RNoRot=0
    
    #Rotation Movement
    #Left Turn
    if(theta>0):
        LNoRot,RNoRot = turnLeft(theta)
    #Right Turn
    elif(theta<0):
        LNoRot,RNoRot = turnRight(theta)

    angle = getAngle(LNoRot,RNoRot)


    #Check for obstacles ahead

    #Check distance to obstacle
    if(checkAvoidance(distance)):
        avoidedAngle = clockAvoidance(distance)
        angle = avoidedAngle + angle
    

    dist,elapsed = forward(distance)

    return angle,dist,elapsed


def turnLeft(theta):
    NoRotations = (R*theta)/(2*PI*r)
    W1_Ticks = NoRotations*20

    wiringpi.digitalWrite(LMot_Pin, 0)  # Write 1 ( HIGH ) to pin 6
    LNoRot,RNoRot = speedSensor(W1_Ticks)
    wiringpi.digitalWrite(LMot_Pin, 1)  # Write 0 ( LOW ) to pin 6

    return LNoRot,RNoRot

def turnRight(theta):
    NoRotations = (R*abs(theta))/(2*PI*r)
    W2_Ticks = NoRotations*20

    wiringpi.digitalWrite(RMot_Pin, 0)  # Write 1 ( HIGH ) to pin 7
    LNoRot,RNoRot = speedSensor(W2_Ticks)
    wiringpi.digitalWrite(RMot_Pin, 1)  # Write 0 ( LOW ) to pin 7

    return LNoRot,RNoRot

def forward(distance):
    #Forward Movement
    NoRotations = distance/(2*PI*r)
    W12_Ticks = NoRotations*20

    wiringpi.digitalWrite(LMot_Pin, 0)  # Write 1 ( HIGH ) to pin 6
    wiringpi.digitalWrite(RMot_Pin, 0)  # Write 1 ( HIGH ) to pin 7
    old_time = time.time()
    LNoRot,RNoRot = speedSensor(W12_Ticks)
    elapsed = time.time() - old_time
    wiringpi.digitalWrite(LMot_Pin, 1)  # Write 0 ( LOW ) to pin 6
    wiringpi.digitalWrite(RMot_Pin, 1)  # Write 0 ( LOW ) to pin 7

    dist = getDist(LNoRot,RNoRot)

    return dist,elapsed




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
        
        if(right_old == 0 and right_new == 1):
            right_count += 1
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
    #Determine actual distance
    distL = LNoRot*2*PI*r
    distR = RNoRot*2*PI*r
    print("DistCheck = left - right = ",distL," - ",distR," = ",distL-distR," (should be 0)")

    return distL




#OLD AND NOT USED
def Avoidance(avoidDistL,avoidDistR):
    #print("\nOBSTACLE DETECTED Turn left!")
    turnLeft(PI/2)

    #Check left
    obs_distance = sonarControl.runSonar()
    if(obs_distance<avoidDistL):
        #print("OBSTACLE DETECTED IN AVOID PATH Turn right!")
        turnRight(PI/2)

        #Check Right
        obs_distance = sonarControl.runSonar()
        if(obs_distance<avoidDistR):
            print("OBSTACLE DETECTED AVOID, I AM BOXED IN !")
        
        # GO FORWARD AND REPOSITION
        else:
            forward(avoidDistR)
            turnLeft(PI/2)#Face Back (Turn Left)
            avoidDistL = avoidDistL + avoidDistR

    # GO FORWARD AND REPOSITION
    else:
        forward(avoidDistR)
        turnRight(PI/2)#Face Back (Turn Right)
        avoidDistR = avoidDistR + avoidDistL

    return avoidDistL,avoidDistR

def checkAvoidance(distance):


    totalAngle = 0

    while(totalAngle<clockAngleInit):
        #Turn Left
        turnLeft(clockAngleStep)
        totalAngle += clockAngleStep
        if(sonarControl.runSonar() < distance):
            return True
        
    turnRight(totalAngle)
    totalAngle = 0

    while(totalAngle<clockAngleInit):
        #Turn Left
        turnRight(clockAngleStep)
        totalAngle += clockAngleStep
        if(sonarControl.runSonar() < distance):
            return True
        
    return False


def clockAvoidance(distance):

    totalAngle = 0
    #Turn Left
    turnLeft(clockAngleInit)
    totalAngle = clockAngleInit

    if(sonarControl.runSonar() < distance):
        #Turn Right
        turnRight(clockAngleInit*2)
        totalAngle = -clockAngleInit

        if(sonarControl.runSonar() < distance):
            #Turn Right
            turnRight(clockAngleInit)
            totalAngle = -clockAngleInit*2
            
            if(sonarControl.runSonar() < distance):
                #Turn Left
                turnLeft(clockAngleInit*3)
                totalAngle = clockAngleInit*2
        



    
    return totalAngle

    









    




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
wiringpi.pinMode(LMot_Pin, 1)       # Set pin 6 to 1 ( OUTPUT )
wiringpi.pinMode(RMot_Pin, 1)       # Set pin 7 to 1 ( OUTPUT )\
#while True:
#    wiringpi.digitalWrite(testPin, 0)  # Write 1 ( HIGH ) to pin 6
#test3()

#angle,distance = readInstructions()
angle = PI/2
# distance = 0
#angle = 0
distance = 200
angle,distance,t = motorControl(angle,distance)
writeOdometry(angle,distance,t)
# test3()






