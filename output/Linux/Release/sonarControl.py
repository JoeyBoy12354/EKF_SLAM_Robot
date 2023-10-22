import odroid_wiringpi as wiringpi
import time
import csv

# One of the following MUST be called before using IO functions:
wiringpi.wiringPiSetup()      # For sequential pin numbering
# OR
#wiringpi.wiringPiSetupSys()   # For /sys/class/gpio with GPIO pin numbering
# OR
#wiringpi.wiringPiSetupGpio()  # For GPIO pin numbering

#Left
echoPin2 = 23
trigPin2 = 27

#Right
echoPin1 = 6
trigPin1 = 5

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
    #time.sleep(1)
    time.sleep(1.8)

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
        distance = 4000
        print("12m <= DISTANCE ! set to 400")

    writeOdometry(distance)
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
#runSonar()