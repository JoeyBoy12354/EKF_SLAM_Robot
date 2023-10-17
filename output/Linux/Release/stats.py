import csv
import matplotlib.pyplot as plt

def fetchState():
    x_coord = []
    y_coord = []
    theta_coord = []
    with open('statsCSV.csv', 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x_coord.append(float(row[0]))
            y_coord.append(float(row[1]))
            theta_coord.append(float(row[2]))
    return x_coord, y_coord, theta_coord

# Fetch the data
x_coord, y_coord, theta_coord = fetchState()

# Create subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6))

# Plot x_coord on the first subplot
ax1.plot(x_coord)
ax1.set_title("X Coordinate")

# Plot y_coord on the second subplot
ax2.plot(y_coord)
ax2.set_title("Y Coordinate")

# Plot theta_coord on the third subplot
ax3.plot(theta_coord)
ax3.set_title("Theta Coordinate")

# Adjust layout
plt.tight_layout()

# Show the plots
plt.show()