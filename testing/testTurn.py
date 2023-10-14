import math
import matplotlib.pyplot as plt
import numpy as np

def calculate_wheel_positions(robot_point, closest_point, initial_pointing_direction):
    wheel_lidar_x = 81
    wheel_lidar_y = 71

    # Calculate the difference between the closest point and the robot point
    delta_x = closest_point[0] - robot_point[0]
    delta_y = closest_point[1] - robot_point[1]

    # Calculate the angle between the lidar and the closest point
    angle = math.atan2(delta_y, delta_x)

    # Subtract the initial pointing direction from this angle to get the relative angle
    relative_angle = angle - initial_pointing_direction

    # Calculate the new position of the wheels using trigonometry
    if relative_angle > 0:
        # Do left turn centered on left wheel
        Ax = robot_point[0] + wheel_lidar_x * math.cos(initial_pointing_direction)
        Ay = robot_point[1] + wheel_lidar_y * math.sin(initial_pointing_direction)
        Bx = robot_point[0]
        By = robot_point[1]
    else:
        # Do right turn centered on right wheel
        Ax = robot_point[0] + wheel_lidar_x * math.cos(initial_pointing_direction + math.pi)
        Ay = robot_point[1] + wheel_lidar_y * math.sin(initial_pointing_direction + math.pi)
        Bx = robot_point[0]
        By = robot_point[1]

    Cx = Ax + (Bx - Ax) * math.cos(-angle) - (By - Ay) * math.sin(-angle)
    Cy = Ay + (Bx - Ax) * math.sin(-angle) + (By - Ay) * math.cos(-angle)

    print("Angle to rotate = ",angle*180/math.pi)
    print("Wheel = ",Ax,", ",Ay)
    print("Lidar = ",Bx,", ",By)
    print("LidarNew = ",Cx,", ",Cy)

    return Ax, Ay, Bx, By, Cx, Cy


def new_wheel_position(robot,angle):
    w_x = 81
    w_y = 71
    w_theta = math.atan(w_y/w_x)
    w_r = math.sqrt(pow(81,2) + pow(71,2))

    print("w_theta = ",w_theta*180/math.pi)
    print("w_r = ",w_r)
    print()

    d_ang = angle - w_theta
    d_x = robot[0] - w_r*math.cos(d_ang)
    d_y = robot[1] - w_r*math.sin(d_ang)

    print("d_ang = ",d_ang*180/math.pi)
    print("d_x = ",robot[0]," - ",w_r*math.cos(d_ang)," = ",d_x)
    print("d_y = ",robot[1]," - ",w_r*math.sin(d_ang)," = ",d_y)

    wheel_x = robot[0] - d_x
    wheel_y = robot[0] + d_y

    return wheel_x,wheel_y

def plotRobot(robot):
    x = []
    y = []
    for i in range(0,360):
        wheel_x,wheel_y = new_wheel_position(robot,i*math.pi/180)
        x.append(wheel_x)
        y.append(wheel_y)

        if i == 0:
            plt.plot([robot[0], wheel_x], [robot[1], wheel_y], label="0 deg")

        if i == 90:
            plt.plot([robot[0], wheel_x], [robot[1], wheel_y], label="90 deg")


    plt.plot(x, y)
    plt.plot(robot[0],robot[1],marker='X')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Robot Wheel Positions')
    plt.grid(True)
    plt.legend()  # Add a legend to distinguish the lines
    plt.show()




# robot = [0,0]
# closest_point = [0,-200]
# angle = 0

#calculate_wheel_positions(robot,closest_point,angle)

# new_wheel_position(robot,angle)
# plotRobot(robot)







def rotate_point(point, angle):
    x, y = point
    angle_rad = np.deg2rad(angle)
    new_x = x * np.cos(angle_rad) - y * np.sin(angle_rad)
    new_y = x * np.sin(angle_rad) + y * np.cos(angle_rad)
    return new_x, new_y

# Define the vertices of the equilateral triangle
#triangle = [(0, 0), (1, 0), (0.5, np.sqrt(3)/2)]

triangle = [(0, 0), (81, 71), (81,-71)]

# Number of rotation angles (0 to 360 degrees)
num_angles = 91  # 361 angles for a full circle (0 to 360 degrees)

# Create a figure and axis for plotting
fig, ax = plt.subplots()

# Plot the initial triangle
triangle_x, triangle_y = zip(*triangle)
triangle_x = list(triangle_x) + [triangle_x[0]]  # Close the triangle
triangle_y = list(triangle_y) + [triangle_y[0]]
ax.plot(triangle_x, triangle_y, label='Original Triangle', color='blue')

# Rotate and plot the triangle for each angle
for angle in range(num_angles):
    rotated_triangle = [rotate_point(point, angle) for point in triangle]
    rotated_x, rotated_y = zip(*rotated_triangle)
    rotated_x = list(rotated_x) + [rotated_x[0]]  # Close the triangle
    rotated_y = list(rotated_y) + [rotated_y[0]]
    ax.plot(rotated_x, rotated_y, alpha=0.3, color='red')
    if(angle == 90):
        print("rotated triangle = ",rotated_triangle)


ax.set_aspect('equal', 'box')
ax.set_xlim(-110, 110)
ax.set_ylim(-110, 110)
ax.legend()
plt.grid(True)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Rotating an Equilateral Triangle')
plt.show()



def rotate_triangle(triangle, angle_degrees):
    # Convert the angle to radians
    angle_rad = np.deg2rad(angle_degrees)

    # Create a rotation matrix
    rotation_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                                [np.sin(angle_rad), np.cos(angle_rad)]])

    # Rotate each vertex of the triangle
    rotated_triangle = [np.dot(rotation_matrix, np.array(vertex)) for vertex in triangle]

    return rotated_triangle


triangle = [(0, 0), (81, 71), (81,-71)]
angle = 90
print(rotate_triangle(triangle,angle))

        