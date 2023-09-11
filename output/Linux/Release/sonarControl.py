import odroid_wiringpi as wiringpi
import time

# One of the following MUST be called before using IO functions:
wiringpi.wiringPiSetup()      # For sequential pin numbering
# OR
#wiringpi.wiringPiSetupSys()   # For /sys/class/gpio with GPIO pin numbering
# OR
#wiringpi.wiringPiSetupGpio()  # For GPIO pin numbering

echoPin = 23
trigPin = 27
echoTime = 100
count=0


#all values used here I just made up except the 38 which has some merit
def readSonar():
    
    #send out an echo
    wiringpi.digitalWrite(trigPin, 1)
    time.sleep(echoTime)

    #Wait for reception
    wiringpi.digitalWrite(trigPin, 0)
    
    count=0
    curr_time = time.clock_gettime()
    while(time.clock_gettime() - curr_time <= 38):
        if(wiringpi.digitalRead(echoPin) == 1):
            count+=1

    #Use count to determine length
    distance = count/20

def test2():

    print("Ultrasonic Measurement")


    # Set trigger to False (Low)
    wiringpi.digitalWrite(trigPin, 1) #Set low

    # Allow module to settle
    print("Wait for module to settle")
    time.sleep(2)

    # Send 10us pulse to trigger
    print("Send Pulse")
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
    print("Esapsed Time = ",elapsed)

    # Distance pulse travelled in that time is time
    # multiplied by the speed of sound (cm/s) divided by 2
    distance = elapsed * 17150

    # That was the distance there and back so halve the value
    #distance = distance / 2
    
    #Rounding
    distance = round(distance,2)

    print("Distance : %.1f" % distance," cm")

 


wiringpi.pinMode(trigPin, 1)       # Set pin to 1 ( OUTPUT )
wiringpi.pinMode(echoPin, 0)       # Set pin to 0 ( INPUT )
test2()