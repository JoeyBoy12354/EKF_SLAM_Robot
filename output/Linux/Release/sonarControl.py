import odroid_wiringpi as wiringpi
import time

# One of the following MUST be called before using IO functions:
wiringpi.wiringPiSetup()      # For sequential pin numbering
# OR
#wiringpi.wiringPiSetupSys()   # For /sys/class/gpio with GPIO pin numbering
# OR
#wiringpi.wiringPiSetupGpio()  # For GPIO pin numbering

echoPin = 6
trigPin = 7
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


wiringpi.pinMode(trigPin, 1)       # Set pin to 1 ( OUTPUT )
wiringpi.pinMode(echoPin, 0)       # Set pin to 0 ( INPUT )