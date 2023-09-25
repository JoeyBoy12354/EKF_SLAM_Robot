import numpy as np
import csv
import matplotlib.pyplot as plt

def fetchCoord(filename):
    xy_coord = []
    with open(filename, 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            coord = [float(row[0]), float(row[1])]
            xy_coord.append(coord)
    return xy_coord

# Define line drawing parameters
threshold = 10.0  # Distance threshold to LiDAR point [mm]
max_line_length = 5000.0  # Maximum line length [mm]
max_number_lines = 100 #max number of lines

# Load LiDAR points
lidar_points = fetchCoord('fullMapCSV.csv')

# Function to draw orthogonal lines
def draw_lines(ax, x, y, direction):
    step_size = 10.0
    length = 0.0
    
    while length < max_line_length:
        ax.plot(x, y, 'b-')  # Plot a blue line
        
        # Check if the current point is close to a LiDAR point
        for point in lidar_points:
            dist = np.sqrt((x - point[0])**2 + (y - point[1])**2)
            if dist < threshold:
                return  # Stop drawing the line
        
        # Move in the specified direction
        if direction == 'horizontal':
            x += step_size
        elif direction == 'vertical':
            y += step_size
        
        length += step_size

# Create a plot
plt.figure(figsize=(8, 8))  # Adjust the figure size as needed

# Draw horizontal lines
for y in np.arange(0, max_number_lines, 10.0):
    draw_lines(plt.gca(), 0, y, 'horizontal')

# Draw vertical lines
for x in np.arange(0, max_number_lines, 10.0):
    draw_lines(plt.gca(), x, 0, 'vertical')

plt.scatter([point[0] for point in lidar_points], [point[1] for point in lidar_points], c='red', marker='.', label='LiDAR Points')
plt.xlabel('X (mm)')
plt.ylabel('Y (mm)')
plt.title('Lines and LiDAR Points')
plt.legend()
plt.grid()
plt.show()
