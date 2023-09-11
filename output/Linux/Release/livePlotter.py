import numpy as np
import csv
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

plt.style.use('fivethirtyeight')

def fetchCoord(filename):
    x_coord = []
    y_coord = []
    with open(filename,'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x_coord.append(float(row[0]))
            y_coord.append(float(row[1]))
    return x_coord,y_coord

def fetchRobot():
    postion = []
    with open('positionCSV.csv','r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            postion.append(float(row[0]))
    return postion

# Read the lines from the CSV file
def fetchAndPlotLines():
    lines = []
    with open('linesCSV.csv', 'r') as file:
        csv_reader = csv.DictReader(file)
        for row in csv_reader:
            lines.append({
                'gradient': float(row['Gradient']),
                'intercept': float(row['Intercept']),
                'domain_min': float(row['Domain_Min']),
                'domain_max': float(row['Domain_Max']),
                'range_min': float(row['Range_Min']),
                'range_max': float(row['Range_Max'])
            })

    # Plot the lines
    for line in lines:
        x_values = [line['domain_min'], line['domain_max']]
        y_min = line['gradient']*x_values[0] + line['intercept']
        y_max = line['gradient']*x_values[1] + line['intercept']
        y_values = [y_min,y_max]
        #plt.plot(x_values, y_values, label=f"Line {line['gradient']}x+{line['intercept']}", color='b')
        plt.plot(x_values, y_values, color='b')



def rotate_point(x, y, angle):
    x_rotated = x * np.cos(angle) - y * np.sin(angle)
    y_rotated = x * np.sin(angle) + y * np.cos(angle)
    return x_rotated, y_rotated

# Function to draw a rotated triangle
def draw_rotated_triangle(ax, x, y, direction_angle):
    # Define the coordinates of the vertices of the triangle (Left Corner, Right Corner, Top Tip)
    #triangle = np.array([[0.0, 0.0], [110, 0.0], [55, 100]]) #accurate
    triangle = np.array([[0.0, 0.0], [330, 0.0], [165, 300]]) #scaled

    # Rotate the triangle based on the direction angle
    rotated_triangle = np.array([rotate_point(x, y, direction_angle) for x, y in triangle])

    # Plot the rotated triangle on the existing axis
    ax.fill(rotated_triangle[:, 0], rotated_triangle[:, 1], 'b')



def animate(i):
    #fetchFromCSV
    x1,y1=fetchCoord('fullMapCSV.csv')
    x2,y2=fetchCoord('landmarkCSV.csv')
    x3,y3=fetchCoord('cornersCSV.csv')
    x4,y4=fetchCoord('consensusCSV.csv')
    position = fetchRobot()
    #print("POSITION = ",position)

    

    plt.cla() #Clear Axis (so that we don't keep the old plot we clear it and write again)

    draw_rotated_triangle(plt.gca(),position[0],position[1],position[2])
    plt.plot(x1, y1, 'o', label='Points',markersize=1,color='r')
    plt.plot(x2, y2, 'X', label='Landmarks', markersize=4,color='g')
    plt.plot(x3, y3, 'X', label='RNC_points', markersize=4,color='b')
    plt.plot(x3, y3, 'o', label='Con_points', markersize=1,color='b')
    #fetchAndPlotLines()
    #plt.plot(x2, y2, 'X', label='RNC_lines', markersize=3,color='b')
    

    plt.legend(loc='upper left')
    plt.tight_layout()


#plot.gcf() <- get current figure
#animate <- function to get data
#interval <- 1000 = 1s. Every 1s the fuction will run
ani = FuncAnimation(plt.gcf(), animate, interval=1000)

plt.tight_layout()
plt.show()