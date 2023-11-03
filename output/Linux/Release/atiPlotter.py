import csv
import matplotlib.pyplot as plt
import numpy as np
import subprocess
import ati

def fetchState():
    print("Fetch STATE ekf")
    state = []
    with open('ekf_uCSV.csv', 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x = float(row[0])
            y = float(row[1])
            state.append([x,y])


    return state

def fetchLandmarks():
    landmarks = []
    current_group = []  # Initialize an empty list to store the current group of 3 landmarks
    print("Fetch lm ekf")
    with open('ekf_lmCSV.csv', 'r') as file:
        csv_reader = csv.reader(file, delimiter=',')  # Specify the delimiter as a comma
        for row in csv_reader:
            # Convert the values to floats
            row = [float(value) for value in row]

            current_group.append([row[0], row[1]])  # Store x and y as a list
            current_group.append([row[2], row[3]])
            current_group.append([row[4], row[5]])

            # Check if we have collected 3 landmarks in the current group
            if len(current_group) == 3:
                landmarks.append(current_group)
                current_group = []  # Reset the current group for the next 3 landmarks

    return landmarks

def fetchTrueState():
    state = []
    with open('atsi_pathCSV.csv', 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x = float(row[0])
            y = float(row[1])
            state.append([x,y])

    return state

def fetchTrueLM():
    state = []
    with open('atsi_lm_trueCSV.csv', 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x = float(row[0])
            y = float(row[1])
            state.append([x,y])

    return state

def fetchOdo():
    state = []
    with open('atsi_odoCSV.csv', 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x = float(row[0])
            y = float(row[1])
            state.append([x,y])

    return state

def newCustomPlot():
    myLM = fetchLandmarks()
    myX = fetchState()
    trueX = fetchTrueState()
    trueLM = fetchTrueLM()
    odo = fetchOdo()


    # plot for myX
    myX = list(zip(*myX))  # Transpose myX for plotting
    plt.plot(myX[0], myX[1], c='orange', label='myX')

    # Scatter plot for atiLM
    for lm in myLM:
        lm = list(zip(*lm))  # Transpose each set of landmarks
        plt.scatter(lm[0], lm[1],c='green' ,marker='x')


    # plot for trueX
    trueX = list(zip(*trueX))  # Transpose trueX for plotting
    plt.plot(trueX[0], trueX[1], c='blue', label='trueX')

    # Scatter plot for trueLM
    for lm in trueLM:
        plt.scatter(lm[0], lm[1], marker='*',c="k")


    # Scatter plot for trueX
    odo = list(zip(*odo))  # Transpose trueX for plotting
    plt.plot(odo[0], odo[1], c='k', label='odo')

    plt.legend()
    plt.title(f"Custom Plot")
    plt.xlabel("X-coordinate")
    plt.ylabel("Y-coordinate")
    plt.grid(True)
    plt.show()

def calculateRMSE():
    trueX = fetchTrueState()
    myX = fetchState()
    x_axis = []
    rmse_x = []
    rmse_y = []

    for i in range(0,len(myX)):
        rmse_x.append(np.sqrt((trueX[i+1][0] - myX[i][0])**2))
        rmse_y.append(np.sqrt((trueX[i+1][1] - myX[i][1])**2))
        x_axis.append(i)

    

    # # Create two subplots on the same figure
    # plt.figure(figsize=(10, 6))
    
    # plt.subplot(2, 1, 1)  # 2 rows, 1 column, first plot
    # plt.plot(x_axis, rmse_x, label='RMSE_x')
    # plt.xlabel('Time')
    # plt.ylabel('RMSE_x')
    # plt.legend()

    # plt.subplot(2, 1, 2)  # 2 rows, 1 column, second plot
    # plt.plot(x_axis, rmse_y, label='RMSE_y')
    # plt.xlabel('Time')
    # plt.ylabel('RMSE_y')
    # plt.legend()

    # plt.tight_layout()  # Ensures proper spacing between subplots
    # plt.show()

def RMSEfull(trueArray,estArray):
    print("RMSE")
    print(len(trueArray))
    print(len(estArray))
    print(len(estArray[0]))
    summ_x_array = []
    summ_y_array = []
    time_array = []
    for j in range(0,len(estArray[0])):
        summ_x = 0
        summ_y = 0
        time_array.append(j)
        for i in range(0,len(estArray)):
            summ_x += (trueArray[i][j+1][0] - estArray[i][j][0])**2
            summ_y += (trueArray[i][j+1][1] - estArray[i][j][1])**2
            
        summ_x_array.append(np.sqrt(summ_x)/len(estArray))
        summ_y_array.append(np.sqrt(summ_y)/len(estArray))

    # Create two subplots on the same figure
    plt.figure(figsize=(10, 6))
    
    plt.subplot(2, 1, 1)  # 2 rows, 1 column, first plot
    plt.plot(time_array, summ_x_array, label='RMSE_x')
    plt.xlabel('Time')
    plt.ylabel('RMSE_x')
    plt.legend()

    plt.subplot(2, 1, 2)  # 2 rows, 1 column, second plot
    plt.plot(time_array, summ_y_array, label='RMSE_y')
    plt.xlabel('Time')
    plt.ylabel('RMSE_y')
    plt.legend()

    plt.tight_layout()  # Ensures proper spacing between subplots
    plt.show()






def cppRun():
    # Replace './johann_code' with the actual path to your C++ program.
    cpp_program = 'sudo ./johann_code'

    # Run the C++ program
    try:
        subprocess.run(cpp_program, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")
    else:
        print("C++ program has been executed successfully.")


def monteCarlo(runs):
    trueArray = []
    estArray = []
    for i in range(0,runs):
        ati.main()
        cppRun()
        trueArray.append(fetchTrueState())
        estArray.append(fetchState())

    RMSEfull(trueArray,estArray)

    




#cppRun()
# newCustomPlot()
# calculateRMSE()

monteCarlo(2)


