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
x_coord, y_coord, theta_coord,z_z_q,z_z_theta,gz_z_q,gz_z_theta = fetchState()

# Create subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6))

# Plot x_coord on the first subplot
ax1.plot(x_coord)
ax1.axhline(y=-10, color='r')
ax1.set_title("X Coordinate")

# Plot y_coord on the second subplot
ax2.plot(y_coord)
ax2.axhline(y=10, color='r')
ax2.set_title("Y Coordinate")

# Plot theta_coord on the third subplot
ax3.plot(theta_coord)
ax3.axhline(y=0, color='r')
ax3.set_title("Theta Coordinate")


ax3.plot(z_z_q)
ax3.set_title("Z-Z_cap q")

ax3.plot(z_z_theta)
ax3.set_title("Z-Z_cap theta")

ax3.plot(gz_z_q)
ax3.set_title("G*Z-Z_cap q")

ax3.plot(gz_z_theta)
ax3.set_title("G*Z-Z_cap theta")



# Adjust layout
plt.tight_layout()

# Show the plots
plt.show()