import csv
import matplotlib.pyplot as plt

def fetchState():
    x_coord = []
    y_coord = []
    theta_coord = []
    z_z_q = []
    z_z_theta =[]
    gz_z_q=[]
    gz_z_theta=[]
    with open('statsCSV.csv', 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x_coord.append(float(row[0]))
            y_coord.append(float(row[1]))
            theta_coord.append(float(row[2]))
            z_z_q.append(float(row[3]))
            z_z_theta.append(float(row[4]))
            gz_z_q.append(float(row[5]))
            gz_z_theta.append(float(row[6]))

    return x_coord, y_coord, theta_coord, z_z_q,z_z_theta,gz_z_q,gz_z_theta

# Fetch the data
x_coord, y_coord, theta_coord, z_z_q, z_z_theta, gz_z_q, gz_z_theta = fetchState()

# Create subplots with 4 rows and 2 columns
fig, axs = plt.subplots(4, 2, figsize=(12, 12))

# Plot x_coord on the first subplot
axs[0, 0].plot(x_coord)
axs[0, 0].axhline(y=-10, color='r')
axs[0, 0].set_title("X Coordinate")

# Plot y_coord on the second subplot
axs[0, 1].plot(y_coord)
axs[0, 1].axhline(y=10, color='r')
axs[0, 1].set_title("Y Coordinate")

# Plot theta_coord on the third subplot
axs[1, 0].plot(theta_coord)
axs[1, 0].axhline(y=0, color='r')
axs[1, 0].set_title("Theta Coordinate")

axs[1, 1].plot(theta_coord)
axs[1, 1].axhline(y=0, color='r')
axs[1, 1].set_title("Theta Coordinate Duplicate")

axs[2, 0].plot(z_z_q)
axs[2, 0].set_title("Z-Z_cap q")

axs[2, 1].plot(z_z_theta)
axs[2, 1].set_title("Z-Z_cap theta")

axs[3, 0].plot(gz_z_q)
axs[3, 0].set_title("G*Z-Z_cap q")

axs[3, 1].plot(gz_z_theta)
axs[3, 1].set_title("G*Z-Z_cap theta")

# Adjust layout
plt.tight_layout()

# Show the plots
plt.show()