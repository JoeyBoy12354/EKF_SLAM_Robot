import odroid_wiringpi as wiringpi
import time
import threading
import csv
import sonarControl
import math

# One of the following MUST be called before using IO functions:
wiringpi.wiringPiSetup()      # For sequential pin numbering
# OR
#wiringpi.wiringPiSetupSys()   # For /sys/class/gpio with GPIO pin numbering
# OR
#wiringpi.wiringPiSetupGpio()  # For GPIO pin numbering

LMot_Pin = 22
RMot_Pin = 7
LMotR_Pin = 2
RMotR_Pin = 26

RSS_Pin = 3
LSS_Pin = 4

testPin = 22
testPin2 = 26

R = 142.5 #Distance between wheels [mm]
r = 32.5 #radius of wheel [mm]

#Avoidance
clockAngleInit = 0.959931 #55 Degree inital rotation
clockAngleInit = 0.513599 #? Degree inital rotation
clockAngleStep = 0.261799 #15 Degree step rotation

runDone = False
sonarFlag = False
sonarOn = False
wait = 0.05

#Calibration
timeOn = 0.009
timeOff = 0.001

timeOnL = 0.009
timeOnR = 0.009
timeOffL = 0.001
timeOffR = 0.001

#Avoidance
# turnableDistance = 57 #Total distance from object required to make a turn
# sensorDistace = 95 #Distance between sensors

turnableDistance = 70 #Total distance from object required to make a turn
sensorDistace = 95 #Distance between sensors


def avoidanceTurn(angle):
    rightSonarDist = sonarControl.runSonar(False)
    leftSonarDist = sonarControl.runSonar(True)

    reverseTotal = 0
    dist=0
    

    

    #Determine if we want to turn in the direction of smaller distance

    if(rightSonarDist<turnableDistance or leftSonarDist<turnableDistance):
        print("AVOIDANCE TURN: left or right or both are too close to obstacle; LEFT = ",leftSonarDist," RIGHT = ",rightSonarDist)
        time.sleep(5)

        #Right Side is closer than left
        if(rightSonarDist<leftSonarDist):
            reverseTotal = turnableDistance - rightSonarDist

            #if we want to turn in the right direction (determine if left will make the turn)
            if(angle>0):
                leftSonarDist = leftSonarDist+reverseTotal
                rightSonarDist = rightSonarDist+reverseTotal

                theta = math.atan(sensorDistace/(leftSonarDist-rightSonarDist))

                #will post reverse left be enough
                r = leftSonarDist*math.sin(theta)

                if(r<turnableDistance):
                    reverseTotal += turnableDistance - r

        #Left Side is closer than right
        elif(leftSonarDist<rightSonarDist):
            reverseTotal = turnableDistance - leftSonarDist

            #if we want to turn in the left direction (determine if right will make the turn)
            if(angle<0):
                leftSonarDist = leftSonarDist+reverseTotal
                rightSonarDist = rightSonarDist+reverseTotal
                theta = math.atan(sensorDistace/(rightSonarDist-leftSonarDist))
                
                #will post reverse right be enough
                r = rightSonarDist*math.sin(theta)

                if(r<turnableDistance):
                    reverseTotal += turnableDistance - r
        #Left Side and Right side are equally close
        else:
            reverseTotal = turnableDistance-leftSonarDist

        print("For safety add 30 to reverseTotal")
        reverseTotal = reverseTotal +30

        left,right = speedControl(0,reverseTotal,False)
        dist = getDist(left,right)
        print("Avoidance reverse for: ",reverseTotal,"mm True value = ",dist,"mm")

    return dist

def avoidanceForward(distance):
    rightSonarDist = sonarControl.runSonar(False)
    leftSonarDist = sonarControl.runSonar(True)
    shortenedDist = distance

    

    if((rightSonarDist - distance)<turnableDistance or (leftSonarDist - distance)<turnableDistance):
        print("AVOIDANCE FORWARD left or right or both are too close to obstacle; LEFT = ",leftSonarDist," RIGHT = ",rightSonarDist)
        time.sleep(5)
        if(rightSonarDist>leftSonarDist):
            shortenedDist = distance - turnableDistance + (leftSonarDist - distance) - 30
        
        elif(leftSonarDist>rightSonarDist):
            shortenedDist = distance - turnableDistance + (rightSonarDist - distance) - 30
        
        else:
            shortenedDist = distance - turnableDistance + (rightSonarDist - distance) - 30

        print("Avoidance forward changed from ",distance," to ",shortenedDist)

    return shortenedDist

#MOTOR THREADS
def left_thread(timeOn,timeOff):
    #print("MC: LEFT_Thread")
    global runDone
    #Forward Movement

    while(runDone == False):
        wiringpi.digitalWrite(RMot_Pin, 0)  # Write 1 ( HIGH ) to pin 6
        time.sleep(timeOn)
        wiringpi.digitalWrite(RMot_Pin, 1)  # Write 0 ( LOW ) to pin 6
        time.sleep(timeOff)

    return

def leftR_thread(timeOn,timeOff):
    #print("MC: LEFT_R_Thread")
    global runDone
    #Forward Movement

    while(runDone == False):
        wiringpi.digitalWrite(RMotR_Pin, 0)  # Write 1 ( HIGH ) to pin 6
        time.sleep(timeOn)
        wiringpi.digitalWrite(RMotR_Pin, 1)  # Write 0 ( LOW ) to pin 6
        time.sleep(timeOff)

    return

def right_thread(timeOn,timeOff):
    #print("MC: RIGHT_Thread")
    global runDone
    #Forward Movement

    while(runDone == False):
        wiringpi.digitalWrite(LMot_Pin, 0)  # Write 1 ( HIGH ) to pin 6
        time.sleep(timeOn)
        wiringpi.digitalWrite(LMot_Pin, 1)  # Write 0 ( LOW ) to pin 6
        time.sleep(timeOff)

    #print("leaving")
    return

def rightR_thread(timeOn,timeOff):
    #print("MC: RIGHT_R_Thread")
    global runDone
    #Forward Movement

    while(runDone == False):
        wiringpi.digitalWrite(LMotR_Pin, 0)  # Write 1 ( HIGH ) to pin 6
        time.sleep(timeOn)
        wiringpi.digitalWrite(LMotR_Pin, 1)  # Write 0 ( LOW ) to pin 6
        time.sleep(timeOff)

    return


#MOTOR CONTROL
def motorControl_wThread(theta,distance):
    LNoRot=0
    RNoRot=0

    #Check for turn avoidance
    reverseDistance = avoidanceTurn(theta)

    #Do turn
    print("MC:TURN: ",theta*180/math.pi," deg")
    LNoRot,RNoRot  = speedControl(theta,0,True)

    # theta = -math.pi/2
    # LNoRot,RNoRot  = speedControl(theta,0,True)

    angle = getAngle(LNoRot,RNoRot,theta)

    time.sleep(0.6)


    distance = avoidanceForward(distance)
    print("MC GO FORWARD FOR: ",distance)
    left,right = speedControl(0,distance,True)
    dist = getDist(left,right)    
    
    # print("MC: Old angle = ",angle*180/(math.pi))
    #angle = angle + angle_diff #Only sum angle diff cause the angleDiff function will return positive or negative

    if(abs(angle) > abs(theta*2) ):
        print("\nMC: !! ANGLE IS WACKED !! Measured = ",angle*180/math.pi,"\n")
        angle = theta


    #print("MC measured dist = ",dist)
    print("MC measured angle x = ",angle*180/math.pi)
    print("MC: Possible angle = ", getAngleDifference(left,right,angle)* 180/(math.pi))
    angle = getAngleDifference(left,right,angle)

    return angle,dist

def speedControl(theta,distance,direction):
    global runDone
    global sonarOn
    global sonarFlag
    global timeOnL
    global timeOnR
    global timeOffL
    global timeOffR
    runDone = False
    sonarOn = True
    

    sonar_dist = 50
    #sonar_thread = threading.Thread(target=sonarScan, args=(sonar_dist,))
    
    #SET THREAD AND DIRECTION
    if(theta == 0 and distance > 0):
        NoRotations = distance/(2*math.pi*r) #STRAIGHT
        if(direction == True):
            threadL = threading.Thread(target=left_thread, args=(timeOnL,timeOffL,))
            threadR = threading.Thread(target=right_thread, args=(timeOnR,timeOffR,))
        else:
            print("Set Reverse Threads")
            threadL = threading.Thread(target=leftR_thread, args=(timeOnL,timeOffL,))
            threadR = threading.Thread(target=rightR_thread, args=(timeOnR,timeOffR,))
    elif(theta != 0 and distance == 0):
        NoRotations = (R*abs(theta))/(2*math.pi*r) # ROTATE
        
        if(theta<0):
            if(direction == True):
                thread = threading.Thread(target=left_thread, args=(timeOnL,timeOffL,))
            else:
                thread = threading.Thread(target=leftR_thread, args=(timeOnL,timeOffL,))
        else:
            NoRotations = abs(NoRotations)
            if(direction == True):
                thread = threading.Thread(target=right_thread, args=(timeOnR,timeOffR,))
            else:
                thread = threading.Thread(target=rightR_thread, args=(timeOnR,timeOffR,))
    else:
        #print("MC:DO NOTHING")
        return 0,0
                
    NoTicks = NoRotations*20
    
    left_old = 0
    left_new = 0
    left_count = 0

    right_old = 0
    right_new = 0
    right_count = 0

    #sonar_thread.start() #turrned off for testing
    startT = time.time()
    # if(theta == 0 and distance > 0):
    #     threadR.start()
    #     threadL.start()
    # else:
    #     thread.start()

    if(theta == 0 and distance > 0):
        threadR.start()
        threadL.start()
        #Straight
        while(left_count<=NoTicks and right_count<=NoTicks and sonarFlag == False):
            left_new = wiringpi.digitalRead(LSS_Pin)
            right_new = wiringpi.digitalRead(RSS_Pin)

            if(left_old == 0 and left_new == 1):
                left_count += 1
            
            if(right_old == 0 and right_new == 1):
                right_count += 1
            left_old = left_new
            right_old = right_new
    elif(theta<0):
        thread.start()
        #Rotate Left
        while(left_count<=NoTicks and sonarFlag == False):
            left_new = wiringpi.digitalRead(LSS_Pin)
            right_new = wiringpi.digitalRead(RSS_Pin)

            if(left_old == 0 and left_new == 1):
                left_count += 1
            
            if(right_old == 0 and right_new == 1):
                right_count += 1
            left_old = left_new
            right_old = right_new
    else:
        thread.start()
        #Rotate Right
        while(right_count<=NoTicks and sonarFlag == False):
            left_new = wiringpi.digitalRead(LSS_Pin)
            right_new = wiringpi.digitalRead(RSS_Pin)

            if(left_old == 0 and left_new == 1):
                left_count += 1
            
            if(right_old == 0 and right_new == 1):
                right_count += 1
            left_old = left_new
            right_old = right_new


    # while(left_count<=NoTicks and right_count<=NoTicks and sonarFlag == False):
    #     left_new = wiringpi.digitalRead(LSS_Pin)
    #     right_new = wiringpi.digitalRead(RSS_Pin)

    #     if(left_old == 0 and left_new == 1):
    #         left_count += 1
        
    #     if(right_old == 0 and right_new == 1):
    #         right_count += 1
    #     left_old = left_new
    #     right_old = right_new

    runDone = True
    sonarOn = False
    #print("runDone = ",runDone," sonarFlag = ",sonarFlag, " Waiting to join")

    if(theta == 0 and distance > 0):
        threadR.join()
        threadL.join()
    else:
        thread.join()

    delta_time = time.time() - startT
    #sonar_thread.join()
    
    left = left_count/20
    right = right_count/20

    print("TIME = ",delta_time,"s")
    print("LEFT SPEED = ",left/delta_time," rotations/s, ROTATIONS = ",left,", TICKS = ",left_count)
    print("RIGHT SPEED = ",right/delta_time," rotations/s, ROTATIONS = ",right,", TICKS = ",right_count)


    time.sleep(wait)

    return left,right

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

def getAngle(LNoRot,RNoRot,theta):
    #Determine actual angle
    thetaL = (LNoRot*2*math.pi*r/R)
    thetaR = (RNoRot*2*math.pi*r/R)

    print("ThetaL = ",-1*thetaL*180/math.pi,"ThetaR = ",thetaR*180/math.pi)
    #Assumme the other wheel picking up roations is shaking
    if(theta<0):
        return -1*thetaL
    else:
        return thetaR
    
    # if(thetaL>thetaR):
    #     return -1*thetaL
    # else:
    #     return thetaR

def getAngleDifference(LNoRot,RNoRot,angle):
    print("Get diff for = ",abs(LNoRot - RNoRot))


    if abs(LNoRot - RNoRot) > 0.051 :
        print("Angle Diff exists")

        if(RNoRot>LNoRot): RNoRot = RNoRot-0.05
        else: LNoRot = LNoRot-0.05

        diff = (2*math.pi*r/R)*(RNoRot - LNoRot)
        print("MC: Straight angle_diff = ",diff* 180/(math.pi))

        angle = angle + diff
    # else:
    #     #diff = (2*math.pi*r/R)*(RNoRot - LNoRot)/2
    #     diff = (2*math.pi*r/R)*(RNoRot - LNoRot)*0.5
    #     print("MC: Straight angle_diff small = ",diff* 180/(math.pi))
    #     angle = angle + diff


    return angle

def getDist(LNoRot,RNoRot):
    #Determine actual distance
    distL = LNoRot*2*math.pi*r
    distR = RNoRot*2*math.pi*r
    #print("DistCheck = left - right = ",distL," - ",distR," = ",distL-distR," (should be 0)")

    return distL


#CALIBRATION CODE
def motorCorrection(distance,runs,left_sum,right_sum):
    time.sleep(0.6)
    for i in range(0,runs):
        left,right = speedControl(0,distance,True)
        left_sum+=left
        right_sum+=right
        time.sleep(0.6)
    speedControl(0,distance*runs,False)

    return left_sum,right_sum
    
def motorCalibrate():
    global timeOnL
    global timeOnR
    global timeOffL
    global timeOffR

    print("MCAL: IN motor calibration")
    #default times
    timeOn = 0.008
    timeOff = 0.002
    timeOnL = timeOn
    timeOnR = timeOn
    timeOffL = timeOff
    timeOffR = timeOff

    distance = 800
    runs = 1

    left_avg = 0
    right_avg = 0
    left_avg,right_avg = motorCorrection(distance,runs,left_avg,right_avg)
    left_avg,right_avg = motorCorrection(distance,runs,left_avg,right_avg)
    left_avg,right_avg = motorCorrection(distance,runs,left_avg,right_avg)
    left_avg,right_avg = motorCorrection(distance,runs,left_avg,right_avg)
    left_avg,right_avg = motorCorrection(distance,runs,left_avg,right_avg)
    left_avg,right_avg = motorCorrection(distance,runs,left_avg,right_avg)
    left_avg,right_avg = motorCorrection(distance,runs,left_avg,right_avg)
    left_avg,right_avg = motorCorrection(distance,runs,left_avg,right_avg)
    left_avg = left_avg/(runs*8)
    right_avg = right_avg/(runs*8)
    
    
    if(left_avg>right_avg):
        timeOnL = (right_avg/left_avg)*timeOn
        timeOffL = timeOn+timeOff-timeOnL
    elif(right_avg>left_avg):
        timeOnR = (left_avg/right_avg)*timeOn
        timeOffR = timeOn+timeOff-timeOnR
        

    writeCalibration(timeOnL,timeOnR,timeOffL,timeOffR)

    #Return to previous position attempt
    print("MCAL: time Left = ",timeOnL,"s ",timeOffL,"s")
    print("MCAL: time Right = ",timeOnR,"s ",timeOffR,"s")

    
    print("\n YOU HAVE 10s TO MOVE ME TO STARTING POSITION \n")
    time.sleep(10)

    return


#CSV CODE
def writeCalibration(timeOnL, timeOnR, timeOffL, timeOffR):
    existingData = [timeOnL, timeOnR, timeOffL, timeOffR]

    with open('motorCalCSV.csv', 'w', newline='') as file:
        csv_writer = csv.writer(file)
        csv_writer.writerow(existingData)

def readInstructions():
    values = []
    with open('motorCSV.csv','r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            values.append(float(row[0]))
    
    return values[0],values[1]

def writeOdometry(angle, distance):
    existingData = []
    with open('motorCSV.csv','r',newline='') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            existingData.append(row[0])

    if(len(existingData)<=2):
        existingData.append(angle)
        existingData.append(distance)
    else:
        existingData[2] = angle
        existingData[3] = distance

    with open('motorCSV.csv','w') as file:
        csv_writer = csv.writer(file)
        for row in existingData:
            csv_writer.writerow([str(row)])

    return

#This function will read the cali values
def readCalibration():
    with open('motorCalCSV.csv', 'r', newline='') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            timeOnL, timeOnR, timeOffL, timeOffR = map(float, row)
            return timeOnL, timeOnR, timeOffL, timeOffR

#Thus function will read the state, either calibration or mapping
def readState():
    with open('motorStateCSV.csv', 'r', newline='') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            state = int(row[0])

            return state == 1


#TEST CODE 
def testAngles():
    print("MC: BEGIN TESTING ANGLES")
    print("Cali Left = ",timeOnL," ",timeOffL)
    print("Cali Right = ",timeOnR," ",timeOffR)

    angles = [180,180] #mm
    waitTime = 4 #In seconds

    
    for i in range(0,len(angles)):
        motorControl_wThread(angles[i]*math.pi/180,0)
        
        #Print Results
        #print("MC: Set:",round(distances[i])," Dist_F:",round(dist_F,2))
        print("Please give input")
        input()

def testDistances():
    print("MC: BEGIN TESTING DISTANCES")
    #timeOnL, timeOnR, timeOffL, timeOffR = readCalibration()
    print("Cali Left = ",timeOnL," ",timeOffL)
    print("Cali Right = ",timeOnR," ",timeOffR)

    distances = [300,300,300] #mm
    waitTime = 4 #In seconds

    
    for i in range(0,len(distances)):
        motorControl_wThread(0,distances[i])
        
        #Print Results
        #print("MC: Set:",round(distances[i])," Dist_F:",round(dist_F,2))
        print("Please give input")
        input()

def testThread(distance):
    global runDone

    NoRotations = distance/(2*math.pi*r)
    NoTicks = NoRotations*20
    
    left_old = 0
    left_new = 0
    left_count = 0

    right_old = 0
    right_new = 0
    right_count = 0

    runDone = False
    print("MC: Main: runDone = ",runDone," NoTicks = ",NoTicks)
    # Define the specific data you want to pass to the thread
    thread_data = 0.01
    # Create a thread and pass the data as an argument
    thread = threading.Thread(target=forward_thread, args=(thread_data,))
    thread.start()

    while(left_count<=NoTicks and right_count<=NoTicks):
        left_new = wiringpi.digitalRead(LSS_Pin)
        right_new = wiringpi.digitalRead(RSS_Pin)

        if(left_old == 0 and left_new == 1):
            left_count += 1
        
        if(right_old == 0 and right_new == 1):
            right_count += 1
        left_old = left_new
        right_old = right_new

    print("MC: set RunDone True")
    runDone = True
    thread.join()
    
    left = left_count/20
    right = right_count/20

    
    

    return left,right
      
def testSpeedControl(angle,distance):
    wait = 5
    backangle = 0.20944
    print("MC: Forward for ",distance)
    speedControl(0,distance,True)
    # # print("Reverse for ",distance)
    # # speedControl(0,distance,False)

    # time.sleep(wait)

    # print("Left for ",angle*180/math.pi)
    # speedControl(angle,0,True)
    # time.sleep(wait/2)
    # speedControl(-1*backangle,0,True)
    # time.sleep(wait)

    # print("Left_R for ",backangle*180/math.pi)
    # speedControl(angle,0,False)
    # time.sleep(wait/2)
    # speedControl(backangle,0,True)
    # time.sleep(wait)

    print("MC: Right for ",angle*180/math.pi)
    speedControl(-1*angle,0,True)
    time.sleep(wait/2)
    speedControl(backangle,0,True)
    time.sleep(wait)

    print("MC: Right_R for ",angle*180/math.pi)
    speedControl(-1*angle,0,False)
    time.sleep(wait/2)
    speedControl(-1*backangle,0,True)
    time.sleep(wait)


    # print("Forward for ",distance)
    # speedControl(0,distance,True)

def testSonar():
    print("MC In Sonar Test")
    while(True):
        avoidanceForward(100)
    
    
wiringpi.pinMode(LSS_Pin, 0)       # Set pin to 0 ( INPUT )
wiringpi.pinMode(RSS_Pin, 0)       # Set pin to 0 ( INPUT )
wiringpi.pinMode(LMot_Pin, 1)       # Set pin 6 to 1 ( OUTPUT )
wiringpi.pinMode(RMot_Pin, 1)       # Set pin 7 to 1 ( OUTPUT )
wiringpi.pinMode(LMotR_Pin, 1)       # Set pin 6 to 1 ( OUTPUT )
wiringpi.pinMode(RMotR_Pin, 1)       # Set pin 7 to 1 ( OUTPUT )

wiringpi.digitalWrite(RMotR_Pin, 1)
wiringpi.digitalWrite(RMot_Pin, 1) 
wiringpi.digitalWrite(LMotR_Pin, 1)
wiringpi.digitalWrite(LMot_Pin, 1)





print("MC started")
motorCalibrate()
#timeOnL, timeOnR, timeOffL, timeOffR = readCalibration()
# timeOnL=0.0071
# timeOnR=0.008
# timeOffL=0.0029
# timeOffR=0.002

# timeOnL=0.008
# timeOnR=0.00777
# timeOffL=0.002
# timeOffR=0.00223


timeOnL=0.008
timeOnR=0.007746434231378764
timeOffL=0.002
timeOffR=0.002253565768621236

# timeOnL=0.008
# timeOnR=0.007777
# timeOffL=0.002
# timeOffR=0.002223

# timeOnL=0.008
# timeOnR=0.006
# timeOffL=0.002
# timeOffR=0.004

#testDistances()
#testAngles()

#angle,distance = readInstructions()
# # timeOnL, timeOnR, timeOffL, timeOffR = readCalibration()
# # print("MC: time Left = ",timeOnL,"s ",timeOffL,"s")
# # print("MC: time Right = ",timeOnR,"s ",timeOffR,"s")

# angle = -1*math.pi/2
# distance = 0
#angle = 0
#distance = 400

# if(distance > 900):
#     print(" !! Resetting distance, ",distance," to 900mm")
#     distance = 900


# angle,distance = motorControl_wThread(angle,distance)
# print("MC: Angle turned = ",angle*180/math.pi)
# print("MC: distance moved = ",distance)
# writeOdometry(angle,distance)


# ##Actual code to do things
# if(readState() == True):
#     motorCalibrate()
# else:
#     angle,distance = readInstructions()
#     timeOnL, timeOnR, timeOffL, timeOffR = readCalibration()
#     # print("MC: time Left = ",timeOnL,"s ",timeOffL,"s")
#     # print("MC: time Right = ",timeOnR,"s ",timeOffR,"s")


#     angle,distance = motorControl_wThread(angle,distance)
#     print("MC: Angle turned = ",angle*180/math.pi)
#     print("MC: distance moved = ",distance)
#     writeOdometry(angle,distance)
















#AVOIDANCE CODE
#OLD AND NOT USED
# def Avoidance(avoidDistL,avoidDistR):
#     #print("\nOBSTACLE DETECTED Turn left!")
#     turnLeft(math.pi/2)

#     #Check left
#     obs_distance = sonarControl.runSonar()
#     if(obs_distance<avoidDistL):
#         #print("OBSTACLE DETECTED IN AVOID PATH Turn right!")
#         turnRight(math.pi/2)

#         #Check Right
#         obs_distance = sonarControl.runSonar()
#         if(obs_distance<avoidDistR):
#             print("MC: OBSTACLE DETECTED AVOID, I AM BOXED IN !")
        
#         # GO FORWARD AND REPOSITION
#         else:
#             forward(avoidDistR)
#             turnLeft(math.pi/2)#Face Back (Turn Left)
#             avoidDistL = avoidDistL + avoidDistR

#     # GO FORWARD AND REPOSITION
#     else:
#         forward(avoidDistR)
#         turnRight(math.pi/2)#Face Back (Turn Right)
#         avoidDistR = avoidDistR + avoidDistL

#     return avoidDistL,avoidDistR

# def checkAvoidance_wThread(distance):
#     print("\n MC: in checkAvoidance")

#     sonarError = 20 #how much error the sonar may have [mm]


#     totalAngle = 0

#     while(totalAngle<clockAngleInit):
        
#         #Turn Left
#         speedControl(clockAngleStep,0,True)

#         #print("checkA: turnLeft distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)), " | ",sonarControl.runSonar() )

#         totalAngle += clockAngleStep
#         if(sonarControl.runSonar() <abs((R/2)*math.cos(math.pi - totalAngle)) + sonarError):
#             print("MC: RETURN TRUE distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)))
#             return True
    
    
#     speedControl(totalAngle,0,False)
#     #print("checkA: revLeft distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)), " | ",sonarControl.runSonar() )
#     totalAngle = 0

#     while(totalAngle<clockAngleInit):
        
#         #Turn Right
#         speedControl(-1*clockAngleStep,0,True)
#         totalAngle += clockAngleStep

#         #print("checkA: turnRight distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)), " | ",sonarControl.runSonar() )

#         if(sonarControl.runSonar() < abs((R/2)*math.cos(math.pi - totalAngle)) + sonarError):
#             print("MC:  RETURN TRUE distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)))
#             return True
    
    
#     speedControl(-1*totalAngle,0,False)

#     #print("checkA: revRight distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)), " | ",sonarControl.runSonar() )

#     totalAngle = 0
    
#     return False

# def clockAvoidance_wThread(distance):
#     print("\nMC:  in clock avoidance")

    

#     sonarError = 20 #how much error the sonar may have [mm]
#     totalAngle = 0

#     speedControl(0,R/2+sonarError,False)

#     #Turn Left
#     print("MC: CA: turnL distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)), " | ",sonarControl.runSonar() )
#     speedControl(clockAngleInit,0,True)
#     totalAngle = clockAngleInit

#     if(sonarControl.runSonar() < abs((R/2)*math.cos(math.pi - totalAngle)) + sonarError):
#         print("MC:  CA: revL,turnR distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)), " | ",sonarControl.runSonar() )
#         #Turn Right

#         speedControl(clockAngleInit,0,False)
#         speedControl(-1*clockAngleInit,0,True)
#         totalAngle = -clockAngleInit

#         if(sonarControl.runSonar() < abs((R/2)*math.cos(math.pi - abs(totalAngle))) + sonarError):
#             print("MC: CA: turnR distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)), " | ",sonarControl.runSonar() )

#             #Turn Right

#             speedControl(-1*clockAngleInit,0,True)
#             totalAngle = -clockAngleInit*2
            
#             if(sonarControl.runSonar() < abs((R/2)*math.cos(math.pi - abs(totalAngle))) + sonarError):
#                 print("MC: CA: revR*2,turnL*2 distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)), " | ",sonarControl.runSonar() )
    
#                 #Turn Left

#                 speedControl(-2*clockAngleInit,0,False)
#                 speedControl(clockAngleInit,0,True)

#                 totalAngle = clockAngleInit*2

#                 if(sonarControl.runSonar()<abs((R/2)*math.cos(math.pi - totalAngle)) + sonarError):
#                     #total failure to find alternative route
#                     print("\nMC:  FAILURE TO FIND ROUTE!\n distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)), " | ",sonarControl.runSonar() )
#                     distance = 0

    
#     return totalAngle,distance

# def sonarScan(maxDist):
#     global sonarFlag
#     global sonarOn
#     while(sonarOn==True):
#         if(sonarControl.runSonar() < maxDist):
#             sonarFlag = True
#         else:
#             sonarFlag = False

#         print("MC: ",sonarControl.runSonar())






