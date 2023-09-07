import matplotlib.pyplot as plt
import csv


def plotPoints():
    x_coord = []
    y_coord = []
    with open('mapCSV.csv','r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x_coord.append(float(row[0]))
            y_coord.append(float(row[1]))

    # Plot the points
    plt.figure()
    plt.plot(0,0,'X',markersize = 12,color = 'r')
    plt.plot(x_coord, y_coord, 'o')
    plt.xlabel('X (millimeters)')
    plt.ylabel('Y (millimeters)')
    plt.title('Plot of Points')
    plt.grid()
    plt.show()

def plotLines():
    # Lists to store line data
    lines = []

    # Read the lines from the CSV file
    with open('linesCSV.csv', 'r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
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
        plt.plot(x_values, y_values, label=f"Line {line['gradient']}x+{line['intercept']}")

    plt.xlabel('X (millimeters)')
    plt.ylabel('Y (millimeters)')
    plt.title('Straight Lines from CSV')
    plt.legend()
    plt.grid(True)
    plt.show()

def plotPointsAndLinesAndCorners():
    plt.plot(0,0,'X',markersize = 12,color = 'r')

    # Read points data from 'mapCSV.csv'
    x_coord = []
    y_coord = []
    with open('mapCSV.csv','r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x_coord.append(float(row[0]))
            y_coord.append(float(row[1]))

    # for i in range(0,len(x_coord)):
    #     print(x_coord[i],", ",y_coord[i])
    # Plot the points
    plt.plot(x_coord, y_coord, 'o', label='Points',markersize=3,color='r')
    file.close()

    # Lists to store line data
    lines = []

    # Read the lines from the CSV file
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
        plt.plot(x_values, y_values, label=f"Line {line['gradient']}x+{line['intercept']}", color='b')

    file.close()

    #Read Corners from CSV
    x_corn = []
    y_corn = []
    with open('cornersCSV.csv','r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x_corn.append(float(row[0]))
            y_corn.append(float(row[1]))

    # Plot the corners
    plt.plot(x_corn, y_corn, 'X', label='Corners', markersize=20,color='g')
    
    file.close()

    # Set labels, title, and show the plot
    plt.xlabel('X (millimeters)')
    plt.ylabel('Y (millimeters)')
    plt.title('Plot of Points and Lines and Corners')
    #plt.legend()
    plt.grid()
    plt.show()

def plotPointsAndLines():
    plt.plot(0,0,'X',markersize = 12,color = 'r')

    # Read points data from 'mapCSV.csv'
    x_coord = []
    y_coord = []
    with open('mapCSV.csv','r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x_coord.append(float(row[0]))
            y_coord.append(float(row[1]))

    # for i in range(0,len(x_coord)):
    #     print(x_coord[i],", ",y_coord[i])
    # Plot the points
    plt.plot(x_coord, y_coord, 'o', label='Points',markersize=0.7,color='r')
    file.close()

    # Lists to store line data
    lines = []

    # Read the lines from the CSV file
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

        # y_values = [line['range_min'],line['range_max']]
        plt.plot(x_values, y_values, label=f"Line {line['gradient']}x+{line['intercept']}", color='b')

        # print(line['domain_min'],", ", line['range_min'])
        # print(line['domain_max'],", ", line['range_max'])

    file.close()

    # Set labels, title, and show the plot
    plt.xlabel('X (millimeters)')
    plt.ylabel('Y (millimeters)')
    plt.title('Plot of Points and Lines and Corners')
    #plt.legend()
    plt.grid()
    plt.show()

def plotPointsAndCorners():
    plt.plot(0,0,'X',markersize = 12,color = 'r')
    # Read points data from 'mapCSV.csv'
    x_coord = []
    y_coord = []
    with open('mapCSV.csv','r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x_coord.append(float(row[0]))
            y_coord.append(float(row[1]))

    # for i in range(0,len(x_coord)):
    #     print(x_coord[i],", ",y_coord[i])
    # Plot the points
    plt.plot(x_coord, y_coord, 'o', label='Points',markersize=1,color='b')
    file.close()


    #Read Corners from CSV
    x_corn = []
    y_corn = []
    with open('cornersCSV.csv','r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x_corn.append(float(row[0]))
            y_corn.append(float(row[1]))

    # Plot the corners
    plt.plot(x_corn, y_corn, 'X', label='Corners', markersize=20,color='g')
    
    file.close()

    # Set labels, title, and show the plot
    plt.xlabel('X (meters)')
    plt.ylabel('Y (meters)')
    plt.title('Plot of Points and Lines and Corners')
    plt.legend()
    plt.grid()
    plt.show()

#plotPointsAndCorners()
plotPointsAndLinesAndCorners()
#plotPointsAndLines()
#plotLines()
#plotPoints()