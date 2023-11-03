import csv
import matplotlib.pyplot as plt
import numpy as np

def fetchState():
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

    print(odo)


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
        plt.scatter(lm[0], lm[1], marker='*',c="k", label='trueLM')


    # Scatter plot for trueX
    odo = list(zip(*odoX))  # Transpose trueX for plotting
    plt.plot(odo[0], odo[1], c='k', label='odo')

    plt.legend()
    plt.title(f"Custom Plot")
    plt.xlabel("X-coordinate")
    plt.ylabel("Y-coordinate")
    plt.grid(True)
    plt.show()


newCustomPlot()