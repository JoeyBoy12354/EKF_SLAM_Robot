import odroid_wiringpi as wiringpi
import time
import csv

# One of the following MUST be called before using IO functions:
wiringpi.wiringPiSetup()      # For sequential pin numbering
# OR
#wiringpi.wiringPiSetupSys()   # For /sys/class/gpio with GPIO pin numbering
# OR
#wiringpi.wiringPiSetupGpio()  # For GPIO pin numbering

echoPin = 23
trigPin = 27

#returns distance to obstacle in cm
def runSonar():

    #print("Ultrasonic Measurement")


    # Set trigger to False (Low)
    wiringpi.digitalWrite(trigPin, 1) #Set low

    # Allow module to settle
    #print("Wait for module to settle")
    #time.sleep(1)
    time.sleep(1.5)

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

    # That was the distance there and back so halve the value
    #distance = distance / 2


    
    #Rounding
    distance = round(distance,2)

    print("Sonar Distance : %.1f" % distance," mm")

    if(distance> 12000):
        distance = 0
        print("12m =< DISTANCE ! ")

    writeOdometry(distance)
    return distance

def writeOdometry(distance):
    

    with open('sonarCSV.csv','w') as file:
        csv_writer = csv.writer(file)
        csv_writer.writerow(str(distance))

    return

 


wiringpi.pinMode(trigPin, 1)       # Set pin to 1 ( OUTPUT )
wiringpi.pinMode(echoPin, 0)       # Set pin to 0 ( INPUT )
runSonar()