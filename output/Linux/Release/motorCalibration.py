import motorControl as mc
import time
import csv


def writeCalibration(timeOnL, timeOnR, timeOffL, timeOffR):
    existingData = [timeOnL, timeOnR, timeOffL, timeOffR]

    with open('motorCalCSV.csv', 'w', newline='') as file:
        csv_writer = csv.writer(file)
        csv_writer.writerow(existingData)

def motorCalibrate():

    print("MCAL: IN motor calibration")
    #default times
    timeOn = 0.009
    timeOff = 0.001

    distance = 100
    runs = 4

    left_avg = 0
    right_avg = 0
    for i in range(0,runs):
        left,right = mc.speedControl(0,distance,True)
        left_avg+=left
        right_avg+=right
        time.sleep(0.01)
    
    left_avg = left_avg/runs
    right_avg = right_avg/runs

    

    if(left_avg>right_avg):
        timeOnL = (right_avg/left_avg)*timeOn
        timeOffL = timeOn+timeOff-timeOnL
    elif(right_avg>left_avg):
        timeOnR = (left_avg/right_avg)*timeOn
        timeOffR = timeOn+timeOff-timeOnR
    else:
        timeOnL,timeOnR = timeOn
        timeOffL,timeOffR = timeOff

    writeCalibration(timeOnL,timeOnR,timeOffL,timeOffR)

    for i in range(0,runs):
        mc.speedControl(0,distance,False)

    print("MCAL: time Left = ",timeOnL,"s ",timeOffL,"s")
    print("MCAL: time Right = ",timeOnR,"s ",timeOffR,"s")

    

    return


motorCalibrate()
print("\n YOU HAVE 10s TO MOVE ME TO STARTING POSITION \n")
time.sleep(10)