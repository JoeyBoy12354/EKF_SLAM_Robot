import numpy as np
import csv
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math

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

    file.close()

    goal = []
    #Calculate goal that we want to move
    with open('motorCSV.csv','r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            goal.append(float(row[0]))
    
    d_x = goal[1]*math.cos(goal[0])
    d_y = goal[1]*math.sin(goal[0])
    x_goal = postion[0] + d_x
    y_goal = postion[1] + d_y

    if(len(goal)>2):
        d_x = goal[3]*math.cos(goal[2])
        d_y = goal[3]*math.cos(goal[2])
        x_true = postion[0] + d_x
        y_true = postion[1] + d_y
        angle_true = goal[2]
        true_move = [x_true,y_true,angle_true]
    else:
        true_move = [0,0,0]



    return postion,x_goal,y_goal,true_move

def fetchAndPlotGrid():
    x_coord = []
    y_coord = []
    trav_state = []
    with open('gridCSV.csv','r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x_coord.append(float(row[0]))
            y_coord.append(float(row[1]))
            trav_state.append(row[2])

    for i in range(0,len(x_coord)):
        if(trav_state[i] == 'true'):
            plt.plot(x_coord[i], y_coord[i], 'o', label='Points',markersize=0.5,color='orange')
        else(trav_state[i] == 'false'):
            plt.plot(x_coord[i], y_coord[i], 'o', label='Points',markersize=0.5,color='grey')
    

    return x_coord,y_coord


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

def rotate_pnt(x,y,angle):
    x_rot = x * np.cos(angle) - y * np.sin(angle)
    y_rot = x * np.sin(angle) + y * np.cos(angle)

    return [x_rot,y_rot]

# Function to draw a rotated triangle
def draw_rotated_triangle(ax, x, y, direction_angle):
    line_length = 300

    # Define the coordinates of the vertices of the triangle (Left Corner, Right Corner, Top Tip)
    #triangle = np.array([[0.0, 0.0], [80, 0.0], [40, 180]]) #accurate

    triangle = np.array([[x, y-40], [x, y+40], [x-90, y]]) #accurate + adjusted
    #triangle = np.array([[0.0, 0.0], [330, 0.0], [165, 300]]) #scaled

    #SWAPPED DUE TO MAP INVERSION
    if(direction_angle<0):
        triangle[1] = rotate_pnt(triangle[1][0],triangle[1][1],direction_angle) #Right Corner
        triangle[2] = rotate_pnt(triangle[2][0],triangle[2][1],direction_angle) #Top Tip
    elif(direction_angle>0):
        triangle[0] = rotate_pnt(triangle[0][0],triangle[0][1],direction_angle) #Left Corner
        triangle[2] = rotate_pnt(triangle[2][0],triangle[2][1],direction_angle) #Top Tip

    



    # Rotate the triangle based on the direction angle
    #rotated_triangle = np.array([rotate_point(x, y, direction_angle) for x, y in triangle])

    # Calculate the coordinates of the front of the triangle
    front_x = triangle[2][0]
    front_y = triangle[2][1]

    back_x = (triangle[0][0]+triangle[1][0])/2
    back_y = (triangle[0][1]+triangle[1][1])/2


    

    # Plot the front line
    ax.plot([front_x, back_x], [front_y, back_y], 'g')

    return triangle



def animate(i):
    #fetchFromCSV
    x1,y1=fetchCoord('fullMapCSV.csv')
    x2,y2=fetchCoord('landmarkCSV.csv')
    x3,y3=fetchCoord('cornersCSV.csv')
    x4,y4=fetchCoord('consensusCSV.csv')
    position,x_goal,y_goal,true_move = fetchRobot()

    #print("POSITION = ",position)

    #Adjustment
    #position[2] += -1*np.pi*7/8


    

    plt.cla() #Clear Axis (so that we don't keep the old plot we clear it and write again)
    #plt.axes().set_facecolor("black")

    #fetchAndPlotLines()
    triangle = draw_rotated_triangle(plt.gca(),position[0],position[1],position[2])
    plt.gca().fill(triangle[:, 0], triangle[:, 1], 'b')

    triangle = draw_rotated_triangle(plt.gca(),true_move[0],true_move[1],true_move[2])
    plt.gca().fill(triangle[:, 0], triangle[:, 1], 'y')

    plt.axhline(y=0, color='k', linestyle='--', linewidth=1)
    plt.axvline(x=0, color='k', linestyle='--', linewidth=1)
    plt.plot(x1, y1, 'o', label='Points',markersize=0.5,color='r')
    plt.plot(x4, y4, 'o', label='Con_points', markersize=1,color='purple')
    plt.plot(x2, y2, 'X', label='Landmarks', markersize=7,color='g')
    plt.plot(x3, y3, 'X', label='RNC_points', markersize=7,color='b')
    plt.plot(x_goal, y_goal, '8', label='Goal', markersize=7,color='c')

    
    
    #plt.plot(x2, y2, 'X', label='RNC_lines', markersize=3,color='b')
    

    #plt.legend(loc='upper left')
    
    
    plt.tight_layout()


#plot.gcf() <- get current figure
#animate <- function to get data
#interval <- 1000 = 1s. Every 1s the fuction will run

ani = FuncAnimation(plt.gcf(), animate, interval=1000)

plt.tight_layout()
plt.show()