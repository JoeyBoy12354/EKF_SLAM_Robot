import numpy as np
import csv
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math

plt.style.use('fivethirtyeight')

def fetchCoord(filename):
    x_coord = []
    y_coord = []
    try:
        with open(filename, 'r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                x_coord.append(float(row[0]))
                y_coord.append(float(row[1]))
    except FileNotFoundError:
        print(f"Error: The file '{filename}' was not found.")
    except Exception as e:
        print(f"An error occurred: {e}")
    else:
        return x_coord, y_coord

def fetchRobot(position, motor):
    position_data = []
    try:
        with open('positionCSV.csv', 'r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                position_data.append(float(row[0]))
    except FileNotFoundError:
        print("Error: The file 'positionCSV.csv' was not found.")
        return None

    try:
        goal = []
        # Calculate goal that we want to move
        with open('motorCSV.csv', 'r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                goal.append(float(row[0]))
    except FileNotFoundError:
        print("Error: The file 'motorCSV.csv' was not found.")
        return None
    except Exception as e:
        print(f"An error occurred while reading 'motorCSV.csv': {e}")
        return None
    else:
        file.close()

    d_x = goal[1] * math.cos(goal[0])
    d_y = goal[1] * math.sin(goal[0])
    x_goal = position_data[0] + d_x
    y_goal = position_data[1] + d_y

    if len(goal) > 2:
        d_x = goal[3] * math.cos(goal[2])
        d_y = goal[3] * math.cos(goal[2])
        x_true = position_data[0] + d_x
        y_true = position_data[1] + d_y
        angle_true = goal[2]
        true_move = [x_true, y_true, angle_true]
    else:
        true_move = [0, 0, 0]

    return position_data, x_goal, y_goal, true_move

def fetchAndPlotGrid(grid):
    x_coord = []
    y_coord = []
    trav_state = []
    try:
        with open(grid, 'r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                # Check for gaps
                if row[0] != ' ':
                    x_coord.append(float(row[0]))
                    y_coord.append(float(row[1]))
                    trav_state.append(int(row[2]))
    except FileNotFoundError:
        print(f"Error: The file '{grid}' was not found.")
        return None
    except Exception as e:
        print(f"An error occurred while reading '{grid}': {e}")
        return None
    else:
        file.close()

    for i in range(0, len(x_coord)):
        if trav_state[i] == 1:
            plt.plot(x_coord[i], y_coord[i], 'o', label='Points', markersize=2, color='orange')
        elif trav_state[i] == 0:
            plt.plot(x_coord[i], y_coord[i], 'o', label='Points', markersize=2, color='grey')

    return x_coord, y_coord


# Read the lines from the CSV file
def fetchAndPlotLines():
    #Initialize
    step = 1 # The x value will differ by X-amount on each iteration of while loop
    limit = 50 # The y value may be a maximum of y_intercept +- X
    
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

    print()
    # Plot the lines
    for line in lines:
        #Point of interception
        x_inter = line['domain_min']
        y_inter = line['gradient']*x_inter + line['intercept']
        print("y = ",line['gradient'],"x + ",line['intercept'])

        #Set bounds
        y_min = y_inter - limit
        y_max = y_inter + limit

        y_up = y_inter
        y_down = y_inter
        x_up = x_inter
        x_down = x_inter
        while((y_down>y_min and y_down<y_max) and (y_up>y_min and y_up<y_max)):
            y_up = line['gradient']*x_up + line['intercept']
            y_down = line['gradient']*x_down + line['intercept']
            x_up += step
            x_down -= step
            #print("y_d_u[",y_down,y_up,"] limit[",y_min,y_max,"]")
                    
        x_values = [x_up,x_down]
        y_values = [y_up,y_down]
        #print("x_values = ",x_values," y_values = ",y_values)
        plt.plot(x_values, y_values, color='b')

def fetchAndPlotLines2():
    #Initialize
    step = 1 # The x value will differ by X-amount on each iteration of while loop
    limit = 50 # The y value may be a maximum of y_intercept +- X
    
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
        #Point of interception
        x_max = line['domain_max']
        x_min = line['domain_min']
        y_max = line['gradient']*x_max + line['intercept']
        y_min = line['gradient']*x_min + line['intercept']
        #print("y = ",line['gradient'],"x + ",line['intercept'])

        #Check if line is vertical
        lineError = abs(line['range_max'] - line['range_min'])/2
        if(abs(y_max-line['range_max']*2) > lineError or abs(y_min-line['range_min']) < lineError):
            y_max = line['range_max']
            y_min = line['range_min']
            x_max = (y_max  - line['intercept'])/line['gradient']
            x_min = (y_min  - line['intercept'])/line['gradient']


        
        x_values = [x_max,x_min]
        y_values = [y_max,y_min]
        #print("x_values = ",x_values," y_values = ",y_values)
        #plt.plot(x_values, y_values, color='b')
        plt.plot(x_values, y_values)

def fetchAndPlotSection():
    x_coord = []
    y_coord = []
    
    with open('sectionCSV.csv','r') as file:
        csv_reader = csv.reader(file)
        

        X = []
        Y = []
        my_x = []
        my_y = []
        #len(csv_reader)
        for row in csv_reader:
            #Check for gaps
            
            if row[0] != ';':
                my_x.append(float(row[0]))
                my_y.append(float(row[1]))
                #sectionSize +=1
                #print("x = ",row[0]," ,",row[1])
                #plt.plot(float(row[0]), float(row[1]), 'o', label='Points',markersize=2,color=col)
                #mySection.append([float(row[0]),float(row[1])])
            else:
                if(len(my_x) > 1):
                    #print("SEQ : max= ",my_x[len(my_x) - 1], my_y[len(my_y) - 1],"  min = ",my_x[0], my_y[0])
                    plt.plot(my_x[len(my_x) - 1], my_y[len(my_y) - 1], 'o', label='Xnts',markersize=10)
                    X.append(my_x)
                    Y.append(my_y)
                    my_x = []
                    my_y = []
                    

        X.append(my_x)
        Y.append(my_y)

    
    for i in range(0,len(Y)):
        if(i%2 == 0):
            plt.plot(X[i], Y[i], '.', label='Points',markersize=2,color='blue')
        else:
            plt.plot(X[i], Y[i], '.', label='Points',markersize=2,color='orange')
        

    return



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

def plotTriangle(x,y):
    #triangle = [[x[0],y[0]],[x[1],y[1]],[x[2],y[2]]]

    colors = ['r', 'g', 'b']  # Red, Green, Blue for corners

    for i in range(3):
        plt.scatter(x[i], y[i], c=colors[i], s=100, label=f'Point {i+1}')
    plt.plot(x + [x[0]], y + [y[0]], 'k-', linewidth=2)


def plotBoundaries():

    plt.ylim([-1000,1000])
    plt.xlim([-700,3000])

    # plt.axhline(y=0, color='k', linestyle='--', linewidth=1)
    # plt.axvline(x=0, color='k', linestyle='--', linewidth=1)


    




def animate(i):
    folder = "k7/"

    folder8 = "k8/"
    folder6 = "k6/"
    folder5 = "k5/"
    folder4 = "k4/"
    folder3 = "k3/"

    grid = folder5+"gridCSV.csv"



    #fetchFromCSV
    x1,y1=fetchCoord(folder+'fullMapCSV.csv')
    x2,y2=fetchCoord("k3/"+'landmarkCSV.csv')
    #x3,y3=fetchCoord('cornersCSV.csv')
    x4,y4=fetchCoord(folder+'mapCSV.csv')
    x5,y5=fetchCoord("k3/"+'triangleCSV.csv')
    #position,x_goal,y_goal,true_move = fetchRobot()
    #print("POSITION = ",position)

    #Adjustment
    #position[2] += -1*np.pi*7/8


    

    plt.cla() #Clear Axis (so that we don't keep the old plot we clear it and write again)
    #plt.axes().set_facecolor("black")
    #fetchAndPlotSection()

    #fetchAndPlotLines()
    #triangle = draw_rotated_triangle(plt.gca(),position[0],position[1],position[2])
    #plt.gca().fill(triangle[:, 0], triangle[:, 1], 'b')

    #plotBoundaries()

    plotTriangle(x5,y5)

    #triangle = draw_rotated_triangle(plt.gca(),0,0,0) 
    #triangle = draw_rotated_triangle(plt.gca(),true_move[0],true_move[1],true_move[2])
    #plt.gca().fill(triangle[:, 0], triangle[:, 1], 'y')

    #plt.plot(x_goal, y_goal, '8', label='Goal', markersize=7,color='c')
    plt.axhline(y=0, color='k', linestyle='--', linewidth=1)
    plt.axvline(x=0, color='k', linestyle='--', linewidth=1)
    fetchAndPlotGrid(grid)

    plt.plot(x1, y1, 'o', label='Points',markersize=0.5,color='r')
    plt.plot(x4, y4, 'o', label='map_points', markersize=1,color='purple')#ConPoints are now representative of latest scan
    
    #plt.plot(x3, y3, 'X', label='RNC_points', markersize=7,color='b')
    plt.plot(x2, y2, 'X', label='Landmarks', markersize=7,color='g')
    

    


    
    
    #plt.plot(x2, y2, 'X', label='RNC_lines', markersize=3,color='b')
    

    #plt.legend(loc='upper left')
    
    
    plt.tight_layout()


#plot.gcf() <- get current figure
#animate <- function to get data
#interval <- 1000 = 1s. Every 1s the fuction will run

ani = FuncAnimation(plt.gcf(), animate, interval=1000)

plt.tight_layout()
plt.show()