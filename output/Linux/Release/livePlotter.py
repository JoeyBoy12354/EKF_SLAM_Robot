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
    #x3,y3=fetchCoord('cornersCSV.csv')
    #x4,y4=fetchCoord('linesCSV.csv')
    position = fetchRobot()
    print("POSITION = ",position)

    

    plt.cla() #Clear Axis (so that we don't keep the old plot we clear it and write again)

    draw_rotated_triangle(plt.gca(),position[0],position[1],position[2])
    plt.plot(x1, y1, 'o', label='Points',markersize=1,color='r')
    plt.plot(x2, y2, 'X', label='Landmarks', markersize=3,color='g')
    

    plt.legend(loc='upper left')
    plt.tight_layout()


#plot.gcf() <- get current figure
#animate <- function to get data
#interval <- 1000 = 1s. Every 1s the fuction will run
ani = FuncAnimation(plt.gcf(), animate, interval=1000)

plt.tight_layout()
plt.show()