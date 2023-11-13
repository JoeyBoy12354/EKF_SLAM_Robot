import odroid_wiringpi as wiringpi
import time
import csv

# One of the following MUST be called before using IO functions:
wiringpi.wiringPiSetup()      # For sequential pin numbering
# OR
#wiringpi.wiringPiSetupSys()   # For /sys/class/gpio with GPIO pin numbering
# OR
#wiringpi.wiringPiSetupGpio()  # For GPIO pin numbering

#Down
echoPin3 = 31
trigPin3 = 30
powerPin3 = 11

#Left
echoPin2 = 23
trigPin2 = 27

#Right
echoPin1 = 6
trigPin1 = 5



def readSonarCSV():
    try:
        dist= 4000
        # Calculate goal that we want to move
        with open('sonarPredictCSV.csv', 'r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                dist = float(row[0])
    except FileNotFoundError:
        print("Error: The file 'sonarPredictCSV.csv' was not found.")
        return None
    except Exception as e:
        print(f"An error occurred while reading 'sonarPredictCSV.csv': {e}")
        return None
    else:
        file.close()

    return dist

#returns distance to obstacle in cm
def runSonar(Left):
    echoPin = echoPin1
    trigPin = trigPin1
    if(Left == True):
        echoPin = echoPin2
        trigPin = trigPin2

    # Set trigger to False (Low)
    wiringpi.digitalWrite(trigPin, 1) #Set low

    # Allow module to settle
    #print("Wait for module to settle")
    time.sleep(1)#This can possibly be made to be 2us or 5us as commonly found on websites
    #time.sleep(1.8)

    # Send 10us pulse to trigger
    #print("Send Pulse")
    wiringpi.digitalWrite(trigPin, 0)#Set High
    time.sleep(0.00001)
    wiringpi.digitalWrite(trigPin, 1)#Set Low
    start = time.time()

    while wiringpi.digitalRead(echoPin) == 0:
        start = time.time()

    while wiringpi.digitalRead(echoPin) == 1:
        stop = time.time()

    # Calculate pulse length
    elapsed = stop-start

    # Distance pulse travelled in that time is time
    # multiplied by the speed of sound (cm/s) divided by 2. Then convert to mm
    distance = (elapsed * 17150) * 10
    
    #Rounding
    distance = round(distance,2)

    #print("Sonar Distance : %.1f" % distance," mm")

    #The HC-SR04 Ultrasonics have a max range of 4000mm (4m) if something is further than that they will bug out
    #And provide a distance greater than 12000 the same will occur for the minimum range of 20mm
    if(distance> 12000):
        print("12m <= DISTANCE is set to ",readSonarCSV()," || measured = ",distance)
        distance = readSonarCSV()
    else:
        #Always selected smallest
        print("Distance < 12000 Measured:",distance," Read = ",readSonarCSV())
        
        if(distance > readSonarCSV()):
            distance = readSonarCSV()
        print("selected = ",distance)

    writeOdometry(distance)
    return distance


def runSonarEdge():
    echoPin = echoPin3
    trigPin = trigPin3


    # Set trigger to False (Low)
    wiringpi.digitalWrite(trigPin, 1) #Set low

    # Allow module to settle
    #print("Wait for module to settle")
    time.sleep(1)#This can possibly be made to be 2us or 5us as commonly found on websites
    #time.sleep(1.8)

    # Send 10us pulse to trigger
    #print("Send Pulse")
    wiringpi.digitalWrite(trigPin, 0)#Set High
    print("trig set H = ",wiringpi.digitalRead(trigPin))
    time.sleep(0.00001)
    wiringpi.digitalWrite(trigPin, 1)#Set Low
    print("trig set L = ",wiringpi.digitalRead(trigPin))
    start = time.time()
    stop = time.time()

    while wiringpi.digitalRead(echoPin) == 0:
        start = time.time()

    while wiringpi.digitalRead(echoPin) == 1:
        stop = time.time()

    # Calculate pulse length
    elapsed = stop-start

    # Distance pulse travelled in that time is time
    # multiplied by the speed of sound (cm/s) divided by 2. Then convert to mm
    distance = (elapsed * 17150) * 10
    
    #Rounding
    distance = round(distance,2)


    print("Sonar Distance : %.1f" % distance," mm")

    #The HC-SR04 Ultrasonics have a max range of 4000mm (4m) if something is further than that they will bug out
    #And provide a distance greater than 12000 the same will occur for the minimum range of 20mm
    if(distance> 66):
        print("WARNING EDGE DETECTED at distance = ",distance)




    return distance


def writeOdometry(distance):
    with open('sonarCSV.csv','w') as file:
        csv_writer = csv.writer(file)
        csv_writer.writerow(str(distance))

    return

 


wiringpi.pinMode(trigPin1, 1)       # Set pin to 1 ( OUTPUT )
wiringpi.pinMode(echoPin1, 0)       # Set pin to 0 ( INPUT )
wiringpi.pinMode(trigPin2, 1)       # Set pin to 1 ( OUTPUT )
wiringpi.pinMode(echoPin2, 0)       # Set pin to 0 ( INPUT )

wiringpi.pinMode(powerPin3, 1)       # Set pin to 1 ( OUTPUT )
wiringpi.pinMode(trigPin3, 1)       # Set pin to 1 ( OUTPUT )
wiringpi.pinMode(echoPin3, 0)       # Set pin to 0 ( INPUT )
wiringpi.digitalWrite(powerPin3, 0) #Set high

while(True):
    print("\n New RUN")
    runSonarEdge()
    time.sleep(1)
    # print("Set High")
    # wiringpi.digitalWrite(powerPin3, 0) #Set high
    # time.sleep(3)
    # print("Set Low")
    # wiringpi.digitalWrite(powerPin3, 1) #Set high
    # time.sleep(3)
    
#runSonar()