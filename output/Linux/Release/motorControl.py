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

runDone2 = False
runDone = False
sonarFlag = False
sonarOn = False
wait = 0.05

echoPin3 = 31
trigPin3 = 30
powerPin3 = 11

#Left
echoPin2 = 23
trigPin2 = 27

#Right
echoPin1 = 6
trigPin1 = 5

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

turnableDistance = 100 #Total distance from object required to make a turn (REMEMBER SAME AS IN NAVI C++)
sensorDistace = 95 #Distance between sensors

def fetchCoord(filename):
    x_coord = []
    y_coord = []
    try:
        with open(filename, 'r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                x_coord.append(float(row[0]))
                y_coord.append(float(row[1]))
    except FileNotFoundError:
        print(f"Error: The file '{filename}' was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")
    else:
        return x_coord, y_coord

#Calculate the distance between 2 points
def pointDistance(x1,y1,x2,y2):
    return math.sqrt(pow((x1-x2),2) + pow((y1-y2),2))

def avoidanceTurn(angle): 
    rightSonarDist = sonarControl.runSonar(False)
    leftSonarDist = sonarControl.runSonar(True)

    reverseTotal = 0
    dist=0
    

    

    #Determine if we want to turn in the direction of smaller distance

    if(rightSonarDist<turnableDistance or leftSonarDist<turnableDistance):
        print("AVOIDANCE TURN: left or right or both are too close to obstacle; LEFT = ",leftSonarDist," RIGHT = ",rightSonarDist)

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
        time.sleep(0.1)
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
    global runDone2
    #Forward Movement

    while(runDone2 == False):
        while(runDone == False):
            wiringpi.digitalWrite(RMot_Pin, 0)  # Write 1 ( HIGH ) to pin 6
            time.sleep(timeOn)
            wiringpi.digitalWrite(RMot_Pin, 1)  # Write 0 ( LOW ) to pin 6
            time.sleep(timeOff)

    return

def leftR_thread(timeOn,timeOff):
    #print("MC: LEFT_R_Thread")
    global runDone
    global runDone2
    #Forward Movement

    while(runDone2 == False):
        while(runDone == False):
            wiringpi.digitalWrite(RMotR_Pin, 0)  # Write 1 ( HIGH ) to pin 6
            time.sleep(timeOn)
            wiringpi.digitalWrite(RMotR_Pin, 1)  # Write 0 ( LOW ) to pin 6
            time.sleep(timeOff)

    return

def right_thread(timeOn,timeOff):
    #print("MC: RIGHT_Thread")
    global runDone
    global runDone2
    #Forward Movement

    while(runDone2 == False):
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
    global runDone2
    #Forward Movement

    while(runDone2 == False):
        while(runDone == False):
            wiringpi.digitalWrite(LMotR_Pin, 0)  # Write 1 ( HIGH ) to pin 6
            time.sleep(timeOn)
            wiringpi.digitalWrite(LMotR_Pin, 1)  # Write 0 ( LOW ) to pin 6
            time.sleep(timeOff)

    return


#MOTOR CONTROL
def motorControl_wThread(theta,distance):
    global sonarFlag
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

    if(sonarFlag == True):
        print("EDGE was detected, now doing avoidance")
        #An edge was detected
        edgeAvoid_angle = 0
        if(theta>0):
            edgeAvoid_angle = -1*(theta+math.pi/2)
        else:
            edgeAvoid_angle = (abs(theta)+math.pi/2)
        LNoRot,RNoRot  = speedControl(edgeAvoid_angle,0,True)

        if(edgeAvoid_angle>0):
            angle = getAngle(LNoRot,RNoRot,edgeAvoid_angle) - angle
        else:
            angle = getAngle(LNoRot,RNoRot,edgeAvoid_angle) + angle
    

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
    print("MC: Post Difference angle = ", getAngleDifference(left,right,angle)* 180/(math.pi))

    return angle,dist

def speedControl(theta,distance,direction):
    global runDone
    global runDone2
    global sonarOn
    global sonarFlag
    global timeOnL
    global timeOnR
    global timeOffL
    global timeOffR
    runDone = True
    runDone2 = False
    
    
    #sonar_thread = threading.Thread(target=runSonarEdge, args=())
    sonar_thread = threading.Thread(target=runSonarFront(), args=())
    
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
        #NoRotations = (R*abs(theta))/(2*math.pi*r) # ROTATE
        bias = 0.05

        #newRotation Scheme:
        NoRotations = (R*(abs(theta)))/(2*math.pi*r) + bias# ROTATE
        
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
    # print("Set noTicks = 20 (1 full rotation)")
    #NoTicks = 40

    NoTicks=NoTicks-1
    
    left_old = 0
    left_new = 0
    left_count = 0

    right_old = 0
    right_new = 0
    right_count = 0

    print("SonarThreadGonnaStart")
    sonarOn = True
    sonar_thread.start() #start edge detection thread
    print("SonarThreadStarted")
    startT = time.time()

    if(theta == 0 and distance > 0):
        threadR.start()
        threadL.start()
        runDone = False
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
        runDone = False

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
        runDone = False
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




    runDone = True
    runDone2 = True
    sonarOn = False

    left_new = wiringpi.digitalRead(LSS_Pin)
    right_new = wiringpi.digitalRead(RSS_Pin)

    if(left_old == 0 and left_new == 1):
        left_count += 1
    
    if(right_old == 0 and right_new == 1):
        right_count += 1
    left_old = left_new
    right_old = right_new

    #print("runDone = ",runDone," sonarFlag = ",sonarFlag, " Waiting to join")

    if(theta == 0 and distance > 0):
        threadR.join()
        threadL.join()
    else:
        thread.join()

    delta_time = time.time() - startT
    sonar_thread.join()
    
    
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
    #bias = 0.05*2*math.pi*r/R
    bias = 0
    #Determine actual angle
    thetaL = (LNoRot*2*math.pi*r/R)
    thetaR = (RNoRot*2*math.pi*r/R)

    print("ThetaL = ",-1*thetaL*180/math.pi,"ThetaR = ",thetaR*180/math.pi)
    print("SUBTRACT BIAS FROM ANGLE :",bias)

    #Assumme the other wheel picking up roations is shaking
    if(theta<0):
        return (-1*thetaL+bias)
    else:
        return (thetaR-bias)
    
    # if(thetaL>thetaR):
    #     return -1*thetaL
    # else:
    #     return thetaR

def getAngleDifference(LNoRot,RNoRot,angle):
    print("Get diff for = ",abs(LNoRot - RNoRot))

    #We also need to check for small data
    if(LNoRot<0.25 and RNoRot<0.25):
        print("BAD MOTOR DATA DETECTED! LNoRot & RNoRot < 0.25")
        return angle

    #We also need to check for bad data
    elif(abs(LNoRot/RNoRot) <=0.7 or abs(RNoRot/LNoRot) <=0.7 or abs(RNoRot - LNoRot) >= 0.4 ):
        print("BAD MOTOR DATA DETECTED! Difference >= 70% || Difference >= 0.4 rotations")


        
        if(RNoRot>LNoRot): diff = (2*math.pi*r/R)*(0.1)
        else: diff = (2*math.pi*r/R)*(-0.1)
        angle = angle + diff

    elif abs(LNoRot - RNoRot) > 0.051 :
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

def runSonarEdge():
    global sonarFlag
    global sonarOn
    echoPin = echoPin3
    trigPin = trigPin3
    sonarFlag = False

    while(sonarOn == True):
        print("RunSonarEdge")
        # Set trigger to False (Low)
        wiringpi.digitalWrite(trigPin, 1) #Set low

        # Allow module to settle
        #print("Wait for module to settle")
        time.sleep(1)#This can possibly be made to be 2us or 5us as commonly found on websites
        #time.sleep(1.8)

        # Send 10us pulse to trigger
        #print("Send Pulse")
        wiringpi.digitalWrite(trigPin, 0)#Set High
        #print("trig set H = ",wiringpi.digitalRead(trigPin))
        time.sleep(0.00001)
        wiringpi.digitalWrite(trigPin, 1)#Set Low
        #print("trig set L = ",wiringpi.digitalRead(trigPin))
        start = time.time()
        stop = time.time()

        print("T0")

        while (wiringpi.digitalRead(echoPin) == 0 and (start-stop) < 3):
            start = time.time()
            #print("start = ",start, "diff = ",start-stop)


        print("T1: echo = 0")

        while (wiringpi.digitalRead(echoPin) == 1 and (stop-start) < 3):
            stop = time.time()
            #print("stop = ",stop, "diff = ",start-stop)
        
        print("T2: echo = 1")
        # Calculate pulse length
        elapsed = stop-start

        # Distance pulse travelled in that time is time
        # multiplied by the speed of sound (cm/s) divided by 2. Then convert to mm
        distance = (elapsed * 17150) * 10
        
        #Rounding
        distance = round(distance,2)


        print("Edge Sonar Distance : %.1f" % distance," mm")

        #The HC-SR04 Ultrasonics have a max range of 4000mm (4m) if something is further than that they will bug out
        #And provide a distance greater than 12000 the same will occur for the minimum range of 20mm
        if(distance> 66):
            print("WARNING EDGE DETECTED at distance = ",distance)
            sonarFlag = True
            sonarOn = False
    
    return

def runSonarFront():
    global sonarFlag
    global sonarOn
    echoPin = echoPin2
    trigPin = trigPin2
    sonarFlag = False
    left = True

    return

    while(sonarOn == True):

        if(left == True):
            echoPin = echoPin2
            trigPin = trigPin2
        else:
            echoPin = echoPin1
            trigPin = trigPin1
        

        #print("RunSonarEdge")
        # Set trigger to False (Low)
        wiringpi.digitalWrite(trigPin, 1) #Set low

        # Allow module to settle
        #print("Wait for module to settle")
        time.sleep(1)#This can possibly be made to be 2us or 5us as commonly found on websites
        #time.sleep(1.8)

        # Send 10us pulse to trigger
        #print("Send Pulse")
        wiringpi.digitalWrite(trigPin, 0)#Set High
        #print("trig set H = ",wiringpi.digitalRead(trigPin))
        time.sleep(0.00001)
        wiringpi.digitalWrite(trigPin, 1)#Set Low
        #print("trig set L = ",wiringpi.digitalRead(trigPin))
        start = time.time()
        stop = time.time()

        #print("T0")

        while (wiringpi.digitalRead(echoPin) == 0 and (start-stop) < 3):
            start = time.time()
            #print("start = ",start, "diff = ",start-stop)


        #print("T1: echo = 0")

        while (wiringpi.digitalRead(echoPin) == 1 and (stop-start) < 3):
            stop = time.time()
            #print("stop = ",stop, "diff = ",start-stop)
        
        #print("T2: echo = 1")
        # Calculate pulse length
        elapsed = stop-start

        # Distance pulse travelled in that time is time
        # multiplied by the speed of sound (cm/s) divided by 2. Then convert to mm
        distance = (elapsed * 17150) * 10
        
        #Rounding
        distance = round(distance,2)


        #print("Front Sonar Distance : %.1f" % distance," mm")

        #The HC-SR04 Ultrasonics have a max range of 4000mm (4m) if something is further than that they will bug out
        #And provide a distance greater than 12000 the same will occur for the minimum range of 20mm
        if(distance < 130):
            print("WARNING OBSTACLE DETECTED at distance = ",distance)
            sonarFlag = True
            sonarOn = False
        
        left = False
    
    return



#CALIBRATION CODE
def motorCorrection(distance,runs,left_sum,right_sum,lefts,rights,error):
    time.sleep(0.6)
    for i in range(0,runs):
        left,right = speedControl(0,distance,True)
        # left,right = speedControl(math.pi/2,0,True)
        lefts.append(left)
        rights.append(right)
        error.append(right - left)

        left_sum+=left
        right_sum+=right
        time.sleep(0.6)

        left,right = speedControl(0,distance*runs,False)
        # lefts.append(left)
        # rights.append(right)
        # error.append(right - left)

        # left_sum+=left
        # right_sum+=right
    
    #left,right = speedControl(0,distance*runs,False)

    

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

    # timeOn = 0.006
    # timeOff = 0.004
    # timeOnL = timeOn
    # timeOnR = timeOn
    # timeOffL = timeOff
    # timeOffR = timeOff

    # timeOnL=0.008
    # timeOnR=0.007623678646934458
    # timeOffL=0.002
    # timeOffR=0.002376321353065542

    

    # timeOnL=0.008
    # timeOnR=0.007756670224119532
    # timeOffL=0.002
    # timeOffR=0.002243329775880468
					

    # timeOnL=0.008
    # timeOnR=0.007647594936708859
    # timeOffL=0.002
    # timeOffR=0.0023524050632911415 


    # timeOnL=0.008
    # timeOnR=0.00770213258
    # timeOffL=0.002
    # timeOffR=0.00229786742 

    
    # timeOnL=0.008
    # timeOnR=0.00767486375
    # timeOffL=0.002
    # timeOffR=0.00232513625

    
    #STANDING BEHIND BOT IT VEERED TO THE LEFT HEAVILY
    # timeOnL=0.008
    # timeOnR=0.00390635680109363
    # timeOffL=0.002
    # timeOffR=0.0061093643198906373						


    # timeOnL=0.007776536312849163
    # timeOnR=0.007890635680109363
    # timeOffL=0.0022234636871508373
    # timeOffR=0.0021093643198906373 


    #This cause super left turn
    # timeOnL=0.003
    # timeOnR=0.007890635680109363
    # timeOffL=0.002
    # timeOffR=0.0021093643198906373



    # timeOnL=0.008
    # timeOnR=0.007800635680109363
    # timeOffL=0.002
    # timeOffR=0.002199364319890637

    # timeOnL=0.008
    # timeOnR=0.007810635680109363
    # timeOffL=0.002
    # timeOffR=0.002189364319890637

    # timeOnL=0.008
    # timeOnR=0.007816635680109363
    # timeOffL=0.002
    # timeOffR=0.002183364319890637

    # timeOnL=0.008
    # timeOnR=0.008
    # timeOffL=0.002
    # timeOffR=0.002


    # timeOnL=0.007820635680109363
    # timeOnR=0.008
    # timeOffL=0.00217936431	
    # timeOffR=0.002	

    # timeOnL=0.008
    # timeOnR=0.007820635680109363
    # timeOffL=0.002
    # timeOffR=0.00217936431		


    # timeOnL=0.007855635680109363
    # timeOnR=0.008
    # timeOffL=0.002144364319890637
    # timeOffR=0.002	

    # timeOnL=0.007890635680109363
    # timeOnR=0.008
    # timeOffL=0.0021093643198906373 
    # timeOffL=0.002
    


    # timeOnL=0.007550561797752809
    # timeOnR=0.008
    # timeOffL=0.002449438202247191
    # timeOffR=0.002	

    # timeOnL=0.007400561797752809
    # timeOnR=0.008
    # timeOffL=0.002599438202247191
    # timeOffR=0.002	

    # timeOnL=0.008
    # timeOnR=0.007400561797752809
    # timeOffL=0.002
    # timeOffR=0.002599438202247191


    # timeOnL=0.008
    # timeOnR=0.007360561797752809
    # timeOffL=0.002
    # timeOffR=0.002639438202247191


    # timeOnL=0.008
    # timeOnR=0.007380561797752809
    # timeOffL=0.002
    # timeOffR=0.002619438202247191


    # timeOnL=0.008
    # timeOnR=0.007300561797752809
    # timeOffL=0.002
    # timeOffR=0.002699438202247191

    timeOnL=0.008
    timeOnR=0.007200561797752809
    timeOffL=0.002
    timeOffR=0.002799438202247191

    
    # timeOnL=0.008
    # timeOnR=0.007280561797752809
    # timeOffL=0.002
    # timeOffR=0.002719438202247191

    # timeOnL=0.008
    # timeOnR=0.007240561797752809
    # timeOffL=0.002
    # timeOffR=0.002759438202247191

    # timeOnL=0.008
    # timeOnR=0.007904328018223236
    # timeOffL=0.002
    # timeOffR=0.0020956719817767644 

    # timeOnL=0.007913093196112065
    # timeOnR=0.007904328018223236
    # timeOffL=0.002086906803887935
    # timeOffR=0.0020956719817767644 

    # timeOnL= 0.0078552036199095
    # timeOnR=0.008
    # timeOffL=0.0021447963800905
    # timeOffR=0.002	

    # timeOnL= 0.00790888382687927
    # timeOnR=0.008
    # timeOffL=0.00209111617312073
    # timeOffR=0.002	


    # timeOnL= 0.007869527896995708
    # timeOnR=0.008
    # timeOffL=0.002130472103004292
    # timeOffR=0.002	


    # timeOnL= 0.007869527896995708
    # timeOnR=0.007841379310344827
    # timeOffL=0.002130472103004292
    # timeOffR=0.0021586206896551732


    # timeOnL=0.008
    # timeOnR=0.007924137931034483
    # timeOffL=0.002
    # timeOffR=0.002075862068965517 



    # timeOnL=0.008
    # timeOnR=0.0076975945017182116
    # timeOffL=0.002
    # timeOffR=0.0023024054982817886



    
    # timeOnL=0.00785529715762274
    # timeOnR=0.0076975945017182116
    # timeOffL=0.00214470284237726
    # timeOffR=0.0023024054982817886


    # timeOnL=0.008
    # timeOnR=0.007875647668393783
    # timeOffL=0.002
    # timeOffR=0.0021243523316062177




    # timeOnL=0.00783673469387755
    # timeOnR=0.008
    # timeOffL=0.0021632653061224496
    # timeOffR=0.002


    # timeOnL=0.007200561797752809
    # timeOnR=0.008
    # timeOffL=.002799438202247191
    # timeOffR=0.002


    # timeOnL=0.007100561797752809
    # timeOnR=0.008
    # timeOffL=0.002899438202247191
    # timeOffR=0.002

    # timeOnL=0.008
    # timeOnR=0.007100561797752809
    # timeOffL=0.002
    # timeOffR=0.002899438202247191

    # timeOnL=0.007698744769874478
    # timeOnR=0.007100561797752809
    # timeOffL=0.0023012552301255223 
    # timeOffR=0.002899438202247191


    
    # timeOnL=0.007000561797752809
    # timeOnR=0.008
    # timeOffL=0.002999438202247191
    # timeOffR=0.002

    # timeOnL=0.008
    # timeOnR=0.007000561797752809
    # timeOffL=0.002
    # timeOffR=0.002999438202247191


    # timeOnL=0.006960561797752809
    # timeOnR=0.008
    # timeOffL=0.003039438202247191
    # timeOffR=0.002


    # timeOnL=0.006900561797752809
    # timeOnR=0.008
    # timeOffL=0.003099438202247191
    # timeOffR=0.002

    # timeOnL=0.008
    # timeOnR=0.006900561797752809
    # timeOffL=0.002
    # timeOffR=0.003099438202247191


    
    # timeOnL=0.006000561797752809
    # timeOnR=0.008
    # timeOffL=0.003999438202247191
    # timeOffR=0.002

    # timeOnL=0.008
    # timeOnR=0.006000561797752809
    # timeOffL=0.002
    # timeOffR=0.003999438202247191

    # timeOnL=0.008
    # timeOnR=0.006300561797752809
    # timeOffL=0.002
    # timeOffR=0.003699438202247191

    timeOnL=0.008
    timeOnR=0.006500561797752809
    timeOffL=0.002
    timeOffR=0.003499438202247191


    # timeOnL=0.007250561797752809
    # timeOnR=0.008
    # timeOffL=0.002749438202247191
    # timeOffR=0.002

    # timeOnL=0.008
    # timeOnR=0.007250561797752809
    # timeOffL=0.002
    # timeOffR=0.002749438202247191


    # timeOnL=0.007225561797752809
    # timeOnR=0.008
    # timeOffL=0.002774438202247191
    # timeOffR=0.002

    # timeOnL=0.007300561797752809
    # timeOnR=0.008
    # timeOffL=0.002699438202247191
    # timeOffR=0.002


    
    # timeOnL=0.007647594936708859
    # timeOnR=0.008
    # timeOffL=0.0023524050632911415 
    # timeOffR=0.002	


        
    # timeOnL=0.008
    # timeOnR=0.007647594936708859
    # timeOffL=0.002
    # timeOffR=0.0023524050632911415 


    
    
    # timeOnL=0.005
    # timeOnR=0.005
    # timeOffL=0.005
    # timeOffR=0.005


    # timeOnL=0.009
    # timeOnR=0.009
    # timeOffL=0.001
    # timeOffR=0.001




    # timeOnL=0.007659147869674185
    # timeOnR=0.008
    # timeOffL=0.0023408521303258153
    # timeOffR=0.002	





    # timeOnL=0.005659147869674185
    # timeOnR=0.006
    # timeOffL=0.0043408521303258153
    # timeOffR=0.004	



    # timeOnL=0.00740250626566416
    # timeOnR=0.008
    # timeOffL=0.0025974937343358406
    # timeOffR=0.002	


    # timeOnL=0.00745250626566416
    # timeOnR=0.008
    # timeOffL=0.00254749373433584
    # timeOffR=0.002	


    # timeOnL=0.008
    # timeOnR=0.00745250626566416
    # timeOffL=0.002
    # timeOffR=0.00254749373433584





    # timeOnL=0.007300561797752809
    # timeOnR=0.008
    # timeOffL=0.002699438202247191
    # timeOffR=0.002


    
    # timeOnL=0.008
    # timeOnR=0.007300561797752809
    # timeOffL=0.002
    # timeOffR=0.002699438202247191


    # timeOnL=0.003
    # timeOnR=0.003
    # timeOffL=0.007
    # timeOffR=0.007



    # timeOnL=0.007225561797752809
    # timeOnR=0.008
    # timeOffL=0.002774438202247191
    # timeOffR=0.002



    # timeOnL=0.007225561797752809
    # timeOnR=0.007724137931034481
    # timeOffL=0.002774438202247191
    # timeOffR=0.002275862068965519

    # timeOnL=0.005
    # timeOnR=0.005
    # timeOffL=0.005
    # timeOffR=0.005



    # timeOnL=0.007783274440518255 
    # timeOnR=0.008
    # timeOffL=0.002216725559481745
    # timeOffR=0.002


    # timeOnL=0.007330457290767905
    # timeOnR=0.008
    # timeOffL=0.0026695427092320954
    # timeOffR=0.002


    # timeOnL=0.006500561797752809
    # timeOnR=0.008
    # timeOffL=0.003499438202247191
    # timeOffR=0.002

    # timeOnL=0.006900561797752809
    # timeOnR=0.008
    # timeOffL=0.003099438202247191
    # timeOffR=0.002

    # timeOnL=0.0071310344827586235 
    # timeOnR=0.008
    # timeOffL=0.0028689655172413767
    # timeOffR=0.002
    
    # timeOnL=0.006000561797752809
    # timeOnR=0.008
    # timeOffL=0.003999438202247191
    # timeOffR=0.002

    # timeOnL=0.006250561797752809
    # timeOnR=0.008
    # timeOffL=0.003749438202247191
    # timeOffR=0.002


    # timeOnL=0.008
    # timeOnR=0.007709601873536302
    # timeOffL=0.002
    # timeOffR=0.002290398126463698 






    print("INIT settings")
    print("timeOnL = ",timeOnL)
    print("timeOnR = ",timeOnR)
    print("timeOffL = ",timeOffL)
    print("timeOffR = ",timeOffR)


    # timeOnL=0.008
    # timeOnR=0.007781196581196581
    # timeOffL=0.002
    # timeOffR=0.002218803418803419


    



    



#     MCAL: time Left =  0.008 s  0.002 s
# MCAL: time Right =  0.007845454545454546 s  0.002154545454545454 s

#     MCAL: time Left =  0.008 s  0.002 s
# MCAL: time Right =  0.007904328018223236 s  0.0020956719817767644 



    distance =1500
    runs = 1 #THIS CANNOT BE CHANGED

    lefts = []
    rights = []
    error = []

    left_avg = 0
    right_avg = 0

    num = 1

    for i in range(0,num):
        left_avg,right_avg = motorCorrection(distance,runs,left_avg,right_avg,lefts,rights,error)
    
    left_avg = left_avg/(len(lefts))
    right_avg = right_avg/(len(lefts))
    
    
    if(left_avg>right_avg):
        timeOnL = (right_avg/left_avg)*timeOn
        timeOffL = timeOn+timeOff-timeOnL
    elif(right_avg>left_avg):
        timeOnR = (left_avg/right_avg)*timeOn
        timeOffR = timeOn+timeOff-timeOnR
        

    writeCalibration(timeOnL,timeOnR,timeOffL,timeOffR)

    #Return to previous position attempt
    

    runs = list(range(0, len(lefts)))

    

    column_width = 10
    break_scaler = 14

    t1 = "Runs"
    t2 = "Left Rotations"
    t3 = "Right Rotations"
    print()
    print("{:<{width}} {:<{width}} {:<{width}}".format(t1, t2, t3, width=column_width))
    print("-" * (6))

    for i in range(len(runs)):
        print("{0:{width}} {1:{width}.4f} {2:{width}.4f} {3:{width}.4f}".format(runs[i], lefts[i], rights[i],error[i], width=column_width))

    print("-" * (6))

    print()

    era = 0
    lea = 0
    rea = 0
    for i in range(0,len(error)):
        era+=abs(error[i])
        lea+=abs(left_avg-lefts[i])
        rea+=abs(right_avg-rights[i])
    
    print("Average Left = ",left_avg )
    print("Average Right = ",right_avg )
    print("Average Error = ",era/len(error))
    print("Average Left Error = ",lea/len(error))
    print("Average Right Error = ",rea/len(error))

    print("MCAL: time Left = ",timeOnL,"s ",timeOffL,"s")
    print("MCAL: time Right = ",timeOnR,"s ",timeOffR,"s")


    
    
    # print("\n YOU HAVE 10s TO MOVE ME TO STARTING POSITION \n")
    # time.sleep(10)

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
    
# def testMotorSpeedSensor():



    
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


# timeOnL=0.008
# timeOnR=0.007746434231378764
# timeOffL=0.002
# timeOffR=0.002253565768621236

# timeOnL=0.008
# timeOnR=0.007430379746835442
# timeOffL=0.002
# timeOffR=0.002569620253164558

# timeOnL=0.008
# timeOnR=0.007647594936708859
# timeOffL=0.002
# timeOffR=0.0023524050632911415 

# timeOnL=0.007776536312849163
# timeOnR=0.007890635680109363
# timeOffL=0.0022234636871508373
# timeOffR=0.0021093643198906373 


# timeOnL=0.007746434231378764
# timeOnR=0.008
# timeOffL=0.002253565768621236
# timeOffR=0.002	

# timeOnL=0.007225561797752809
# timeOnR=0.008
# timeOffL=0.002774438202247191
# timeOffR=0.002


# timeOnL=0.008
# timeOnR=0.00767486375
# timeOffL=0.002
# timeOffR=0.00232513625

# timeOnL=0.008
# timeOnR=0.00767486375
# timeOffL=0.002
# timeOffR=0.00232513625


# timeOnL=0.007647594936708859
# timeOnR=0.008
# timeOffL=0.0023524050632911415 
# timeOffR=0.002	


# timeOnL=0.008
# timeOnR=0.007300561797752809
# timeOffL=0.002
# timeOffR=0.002699438202247191

# timeOnL=0.007855635680109363
# timeOnR=0.008
# timeOffL=0.002144364319890637
# timeOffR=0.002	

# timeOnL=0.008
# timeOnR=0.007300561797752809
# timeOffL=0.002
# timeOffR=0.002699438202247191

# timeOnL=0.007913093196112065
# timeOnR=0.007904328018223236
# timeOffL=0.002086906803887935
# timeOffR=0.0020956719817767644 



# timeOnL=0.007250561797752809
# timeOnR=0.008
# timeOffL=0.002749438202247191
# timeOffR=0.002



# timeOnL=0.008
# timeOnR=0.008
# timeOffL=0.002
# timeOffR=0.002




# timeOnL=0.008
# timeOnR=0.007623678646934458
# timeOffL=0.002
# timeOffR=0.002376321353065542

# timeOnL=0.008
# timeOnR=0.0076975945017182116
# timeOffL=0.002
# timeOffR=0.0023024054982817886


# timeOnL=0.007550561797752809
# timeOnR=0.008
# timeOffL=0.002449438202247191
# timeOffR=0.002	

# timeOnL=0.008
# timeOnR=0.0076975945017182116
# timeOffL=0.002
# timeOffR=0.0023024054982817886

# timeOnL=0.00740250626566416
# timeOnR=0.008
# timeOffL=0.0025974937343358406
# timeOffR=0.002	

# timeOnL=0.007100561797752809
# timeOnR=0.008
# timeOffL=0.002899438202247191
# timeOffR=0.002


# timeOnL=0.006960561797752809
# timeOnR=0.008
# timeOffL=0.003039438202247191
# timeOffR=0.002

# timeOnL=0.008
# timeOnR=0.007820635680109363
# timeOffL=0.002
# timeOffR=0.00217936431		

# timeOnL=0.008
# timeOnR=0.007647594936708859
# timeOffL=0.002
# timeOffR=0.0023524050632911415 

timeOnL=0.008
timeOnR=0.00745250626566416
timeOffL=0.002
timeOffR=0.00254749373433584


# # # #testDistances()
# # # #testAngles()

# # timeOnL, timeOnR, timeOffL, timeOffR = readCalibration()
# # print("MC: time Left = ",timeOnL,"s ",timeOffL,"s")
# # print("MC: time Right = ",timeOnR,"s ",timeOffR,"s")

# angle,distance = readInstructions()

# # angle = -1*math.pi/2
# # distance = 0
# # angle = math.pi/2
# # #angle = 0
# # distance = 200

# # angle =0
# # distance =0


# if(distance > 900):
#     print(" !! Resetting distance, ",distance," to 900mm")
#     distance = 900

# #Edge sonar
# wiringpi.pinMode(trigPin3, 1)       # Set pin to 1 ( OUTPUT )
# wiringpi.pinMode(echoPin3, 0)       # Set pin to 0 ( INPUT )


# angle,distance = motorControl_wThread(angle,distance)
# print("MC: Angle turned = ",angle*180/math.pi)
# print("MC: distance moved = ",distance)
# writeOdometry(angle,distance)







