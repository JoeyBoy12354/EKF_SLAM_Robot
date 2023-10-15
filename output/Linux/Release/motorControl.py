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



def motorControl_wThread(theta,distance):
    LNoRot=0
    RNoRot=0

    #print("CHECK MOVEMENT SPACE")
    # if(sonarControl.runSonar()<R/2):
    #     #print("REVERSE!")
    #     speedControl(0,100,False)
    #     #update odometry accordinglty NB!!!!!



    #Do turn
    print("MC:TURN: ",theta*180/math.pi," deg")
    #theta = math.pi/2
    LNoRot,RNoRot  = speedControl(theta,0,True)

    # theta = -math.pi/2
    # LNoRot,RNoRot  = speedControl(theta,0,True)

    angle = getAngle(LNoRot,RNoRot)

    time.sleep(1)


    #Check for obstacles ahead
    #print("Gonna check Avoidance")
    #Check distance to obstacle
    # if(checkAvoidance_wThread(distance)):
    #     print("\n CLOCK AVOIDANCE!!!!! \n")
    #     distance = distance/2
    #     avoidedAngle,distance = clockAvoidance_wThread(distance)
    #     angle = avoidedAngle + angle
    
    # if(sonarControl.runSonar()<distance+R/2+30):
    #     #I will hit a wall, last chance to check!
    #     #print("\nFORWARD CHECK FOUND I WILL HIT WALL")
    #     distance = distance - R/2 - 50
    #     #print("NEWDISTANCE = ",distance)
    #     if(distance<0):
    #         #print("negative DISTACE ERRRORRRR!!!")
    #         speedControl(0,100,False)

    print("MC GO FORWARD FOR: ",distance) 
    left,right = speedControl(0,distance,True)
    dist = getDist(left,right)

    

    return angle,dist



# def forward_thread(timeOn,timeOff):
#     print("MC: FORWARD_Thread")
#     global runDone
#     #Forward Movement

#     while(runDone == False):
#         wiringpi.digitalWrite(LMot_Pin, 0)  # Write 1 ( HIGH ) to pin 6
#         wiringpi.digitalWrite(RMot_Pin, 0)  # Write 1 ( HIGH ) to pin 7
#         time.sleep(timeOn)
#         wiringpi.digitalWrite(LMot_Pin, 1)  # Write 0 ( LOW ) to pin 6
#         wiringpi.digitalWrite(RMot_Pin, 1)  # Write 0 ( LOW ) to pin 7
#         time.sleep(timeOff)

#     return

# def reverse_thread(timeOn,timeOff):
#     print("MC: REVERSE_Thread")
#     global runDone
#     #Forward Movement

#     while(runDone == False):
#         wiringpi.digitalWrite(LMotR_Pin, 0)  # Write 1 ( HIGH ) to pin 6
#         wiringpi.digitalWrite(RMotR_Pin, 0)  # Write 1 ( HIGH ) to pin 7
#         time.sleep(timeOn)
#         wiringpi.digitalWrite(LMotR_Pin, 1)  # Write 0 ( LOW ) to pin 6
#         wiringpi.digitalWrite(RMotR_Pin, 1)  # Write 0 ( LOW ) to pin 7
#         time.sleep(timeOff)

#     return

def left_thread(timeOn,timeOff):
    print("MC: LEFT_Thread")
    global runDone
    #Forward Movement

    while(runDone == False):
        wiringpi.digitalWrite(RMot_Pin, 0)  # Write 1 ( HIGH ) to pin 6
        time.sleep(timeOn)
        wiringpi.digitalWrite(RMot_Pin, 1)  # Write 0 ( LOW ) to pin 6
        time.sleep(timeOff)

    return

def leftR_thread(timeOn,timeOff):
    print("MC: LEFT_R_Thread")
    global runDone
    #Forward Movement

    while(runDone == False):
        wiringpi.digitalWrite(RMotR_Pin, 0)  # Write 1 ( HIGH ) to pin 6
        time.sleep(timeOn)
        wiringpi.digitalWrite(RMotR_Pin, 1)  # Write 0 ( LOW ) to pin 6
        time.sleep(timeOff)

    return

def right_thread(timeOn,timeOff):
    print("MC: RIGHT_Thread")
    global runDone
    #Forward Movement

    while(runDone == False):
        wiringpi.digitalWrite(LMot_Pin, 0)  # Write 1 ( HIGH ) to pin 6
        time.sleep(timeOn)
        wiringpi.digitalWrite(LMot_Pin, 1)  # Write 0 ( LOW ) to pin 6
        time.sleep(timeOff)

    print("leaving")
    return

def rightR_thread(timeOn,timeOff):
    print("MC: RIGHT_R_Thread")
    global runDone
    #Forward Movement

    while(runDone == False):
        wiringpi.digitalWrite(LMotR_Pin, 0)  # Write 1 ( HIGH ) to pin 6
        time.sleep(timeOn)
        wiringpi.digitalWrite(LMotR_Pin, 1)  # Write 0 ( LOW ) to pin 6
        time.sleep(timeOff)

    return



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
            threadL = threading.Thread(target=leftR_thread, args=(timeOnL,timeOffL,))
            threadR = threading.Thread(target=rightR_thread, args=(timeOnR,timeOffR,))
    elif(theta != 0 and distance == 0):
        NoRotations = (R*abs(theta))/(2*math.pi*r) # ROTATE
        
        if(theta>0):
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
        print("MC:DO NOTHING")
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
    if(theta == 0 and distance > 0):
        threadR.start()
        threadL.start()
    else:
        thread.start()

    while(left_count<=NoTicks and right_count<=NoTicks and sonarFlag == False):
        left_new = wiringpi.digitalRead(LSS_Pin)
        right_new = wiringpi.digitalRead(RSS_Pin)

        if(left_old == 0 and left_new == 1):
            left_count += 1
        
        if(right_old == 0 and right_new == 1):
            right_count += 1
        left_old = left_new
        right_old = right_new

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
    print("LEFT SPEED = ",left/delta_time," rotations/s, ROTATIONS = ",left)
    print("RIGHT SPEED = ",right/delta_time," rotations/s, ROTATIONS = ",right)


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

def getAngle(LNoRot,RNoRot):
    #Determine actual angle
    thetaL = (LNoRot*2*math.pi*r/R)
    thetaR = (RNoRot*2*math.pi*r/R)


    #Assumme the other wheel picking up roations is shaking
    if(thetaL>thetaR):
        
        return thetaL
    else:
        return thetaR

    
    # angle = thetaL-thetaR
    # print("MC: Angle = left - right = ",thetaL* 180/(math.pi)," - ",thetaR* 180/(math.pi)," = ",(thetaL-thetaR)* 180/(math.pi))

    # return angle

def getDist(LNoRot,RNoRot):
    #Determine actual distance
    distL = LNoRot*2*math.pi*r
    distR = RNoRot*2*math.pi*r
    #print("DistCheck = left - right = ",distL," - ",distR," = ",distL-distR," (should be 0)")

    return distL




#OLD AND NOT USED
def Avoidance(avoidDistL,avoidDistR):
    #print("\nOBSTACLE DETECTED Turn left!")
    turnLeft(math.pi/2)

    #Check left
    obs_distance = sonarControl.runSonar()
    if(obs_distance<avoidDistL):
        #print("OBSTACLE DETECTED IN AVOID PATH Turn right!")
        turnRight(math.pi/2)

        #Check Right
        obs_distance = sonarControl.runSonar()
        if(obs_distance<avoidDistR):
            print("MC: OBSTACLE DETECTED AVOID, I AM BOXED IN !")
        
        # GO FORWARD AND REPOSITION
        else:
            forward(avoidDistR)
            turnLeft(math.pi/2)#Face Back (Turn Left)
            avoidDistL = avoidDistL + avoidDistR

    # GO FORWARD AND REPOSITION
    else:
        forward(avoidDistR)
        turnRight(math.pi/2)#Face Back (Turn Right)
        avoidDistR = avoidDistR + avoidDistL

    return avoidDistL,avoidDistR

def checkAvoidance_wThread(distance):
    print("\n MC: in checkAvoidance")

    sonarError = 20 #how much error the sonar may have [mm]


    totalAngle = 0

    while(totalAngle<clockAngleInit):
        
        #Turn Left
        speedControl(clockAngleStep,0,True)

        #print("checkA: turnLeft distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)), " | ",sonarControl.runSonar() )

        totalAngle += clockAngleStep
        if(sonarControl.runSonar() <abs((R/2)*math.cos(math.pi - totalAngle)) + sonarError):
            print("MC: RETURN TRUE distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)))
            return True
    
    
    speedControl(totalAngle,0,False)
    #print("checkA: revLeft distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)), " | ",sonarControl.runSonar() )
    totalAngle = 0

    while(totalAngle<clockAngleInit):
        
        #Turn Right
        speedControl(-1*clockAngleStep,0,True)
        totalAngle += clockAngleStep

        #print("checkA: turnRight distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)), " | ",sonarControl.runSonar() )

        if(sonarControl.runSonar() < abs((R/2)*math.cos(math.pi - totalAngle)) + sonarError):
            print("MC:  RETURN TRUE distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)))
            return True
    
    
    speedControl(-1*totalAngle,0,False)

    #print("checkA: revRight distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)), " | ",sonarControl.runSonar() )

    totalAngle = 0
    
    return False

def clockAvoidance_wThread(distance):
    print("\nMC:  in clock avoidance")

    

    sonarError = 20 #how much error the sonar may have [mm]
    totalAngle = 0

    speedControl(0,R/2+sonarError,False)

    #Turn Left
    print("MC: CA: turnL distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)), " | ",sonarControl.runSonar() )
    speedControl(clockAngleInit,0,True)
    totalAngle = clockAngleInit

    if(sonarControl.runSonar() < abs((R/2)*math.cos(math.pi - totalAngle)) + sonarError):
        print("MC:  CA: revL,turnR distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)), " | ",sonarControl.runSonar() )
        #Turn Right

        speedControl(clockAngleInit,0,False)
        speedControl(-1*clockAngleInit,0,True)
        totalAngle = -clockAngleInit

        if(sonarControl.runSonar() < abs((R/2)*math.cos(math.pi - abs(totalAngle))) + sonarError):
            print("MC: CA: turnR distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)), " | ",sonarControl.runSonar() )

            #Turn Right

            speedControl(-1*clockAngleInit,0,True)
            totalAngle = -clockAngleInit*2
            
            if(sonarControl.runSonar() < abs((R/2)*math.cos(math.pi - abs(totalAngle))) + sonarError):
                print("MC: CA: revR*2,turnL*2 distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)), " | ",sonarControl.runSonar() )
    
                #Turn Left

                speedControl(-2*clockAngleInit,0,False)
                speedControl(clockAngleInit,0,True)

                totalAngle = clockAngleInit*2

                if(sonarControl.runSonar()<abs((R/2)*math.cos(math.pi - totalAngle)) + sonarError):
                    #total failure to find alternative route
                    print("\nMC:  FAILURE TO FIND ROUTE!\n distCos = ",abs((R/2)*math.cos(math.pi - totalAngle)), " | ",sonarControl.runSonar() )
                    distance = 0

    
    return totalAngle,distance

def sonarScan(maxDist):
    global sonarFlag
    global sonarOn
    while(sonarOn==True):
        if(sonarControl.runSonar() < maxDist):
            sonarFlag = True
        else:
            sonarFlag = False

        print("MC: ",sonarControl.runSonar())



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

def readCalibration():
    with open('motorCalCSV.csv', 'r', newline='') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            timeOnL, timeOnR, timeOffL, timeOffR = map(float, row)
            return timeOnL, timeOnR, timeOffL, timeOffR


    
def testAngles():
    print("MC: BEGIN TESTING ANGLES")

    angles = [math.pi/8,math.pi/4,math.pi/2,math.pi]
    angles = angles + angles
    waitTime = 6 #In seconds

    LNoRot=0
    RNoRot=0

    measAngle_F = 0
    measAngle_R = 0
    
    for i in range(0,len(angles)):
        if(i>=len(angles)/2):
            theta = angles[i]
        else:
            theta = -1*angles[i]

        #Left Turn
        if(theta>0):
            print("MC: forward Turn Left")
            LNoRot,RNoRot = turnLeft(theta)
            measAngle_F = (RNoRot*2*math.pi*r/R)
    
        #Right Turn
        elif(theta<0):
            print("MC: forward Turn Right")
            LNoRot,RNoRot = turnRight(theta)
            measAngle_F = (LNoRot*2*math.pi*r/R)

        #measAngle_F = getAngle(LNoRot,RNoRot)
        print("MC: measAngle_F = ",measAngle_F)
        time.sleep(waitTime)
        

        #Return to position
        #Left Turn
        if(theta>0):
            print("MC: reverse turn Left")
            LNoRot,RNoRot = turnLeftR(theta)
            measAngle_R = (RNoRot*2*math.pi*r/R)
        #Right Turn
        elif(theta<0):
            print("MC: reverse turn Right")
            LNoRot,RNoRot = turnRightR(theta)
            measAngle_R = (LNoRot*2*math.pi*r/R)

        #measAngle_R = getAngle(LNoRot,RNoRot)
        print("MC: measAngle_R = ",measAngle_R)
        time.sleep(waitTime)

        #Print Results
        if(theta>0):
            print("MC: Left Turn: Set:",round(theta*180/math.pi,2)," Angle_F:",round(measAngle_F*180/math.pi,2),", Angle_R:",round(measAngle_R*180/math.pi,2),"\n")
        elif(theta<0):
            print("MC: Right Turn: Set:",round(theta*180/math.pi,2)," Angle_F:",round(measAngle_F*180/math.pi,2),", Angle_R:",round(measAngle_R*180/math.pi,2),"\n")

        time.sleep(waitTime)

def testDistances():
    print("MC: BEGIN TESTING DISTANCES")

    distances = [300,300,300,300,300] #cm
    waitTime = 4 #In seconds

    
    for i in range(0,len(distances)):
        dist_F,elapsed = forward(distances[i])
        print("MC: Dist_F = ",round(dist_F,2)," cm")
        time.sleep(waitTime)
        dist_R,elapsed = reverse(distances[i])
        print("MC: Dist_R = ",round(dist_R,2)," cm")

        #Print Results
        print("MC: Set:",round(distances[i],2)," Dist_F:",round(dist_F,2),", Dist_R:",round(dist_R,2),"\n")
        time.sleep(waitTime)

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

#while True:
#    wiringpi.digitalWrite(testPin, 0)  # Write 1 ( HIGH ) to pin 6
#test3()
print()
angle,distance = readInstructions()
timeOnL, timeOnR, timeOffL, timeOffR = readCalibration()
print("MC: time Left = ",timeOnL,"s ",timeOffL,"s")
print("MC: time Right = ",timeOnR,"s ",timeOffR,"s")
#angle = math.pi/2
#angle = 0
#distance = 300

angle,distance = motorControl_wThread(angle,distance)
print("MC: Angle turned = ",angle*180/math.pi)
print("MC: distance moved = ",distance)
writeOdometry(angle,distance)
print()
# testWheels()

#testAngles()
#testDistances()
#testThread(200)
#testSpeedControl(math.pi,200)

#angle = 0
# distance = 200
#motorControl_wThread(angle,distance)







