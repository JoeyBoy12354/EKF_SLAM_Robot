import csv
import matplotlib.pyplot as plt
import numpy as np
import math

def fetchCoord(filename):
    x_coord = []
    y_coord = []
    with open(filename,'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x_coord.append(float((row[0])))
            y_coord.append(float((row[1])))
    return x_coord,y_coord

def writeCoord(corners,filename = 'cornersCSV.csv'):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        
        # Write the data to the CSV file
        writer.writerows(corners)

    return


def fit_with_least_squares(X,y):
    b = np.ones((X.shape[0],1))
    A = np.hstack((X,b))
    theta = np.linalg.lstsq(A,y,rcond=None)[0]
    return theta

def evaluate_model(X,y,theta,inlier_threshold):

    b = np.ones((X.shape[0],1))
    y = y.reshape((y.shape[0],1))
    A=np.hstack((y,X,b))
    theta = np.insert(theta,0,-1.)

    distances = np.abs(np.sum(A*theta,axis=1)) / np.sqrt(np.sum(np.power(theta[:-1],2)))
    inliers = distances <= inlier_threshold
    num_inliers = np.count_nonzero(inliers==True)

    return num_inliers

#def ransac(X,y,max_iters=200,samples_to_fit=2,inlier_threshold=0.15,min_inliers=35):
#worked for map 1

#1,50
#in_th = 0.05
def ransac(X,y,max_iters=500,samples_to_fit=2,inlier_threshold=0.09,min_inliers=5):
#def ransac(X,y,max_iters=200,samples_to_fit=2,inlier_threshold=8,min_inliers=35):

    best_model=None
    best_model_performance=0

    num_samples = X.shape[0]

    for i in range(max_iters):
        sample = np.random.choice(num_samples,size=samples_to_fit,replace=False)
        model_params = fit_with_least_squares(X[sample],y[sample])
        model_performace = evaluate_model(X,y,model_params,inlier_threshold)

        if( model_performace < min_inliers):
            continue
        if model_performace > best_model_performance:
            best_model = model_params
            best_model_performance = model_performace

    return best_model

def manager(x_coords,y_coords,inlier_threshold=0.05,min_inliers=7):
    
    sample_size = 100 # 500 worked well for map1
    num_samples = int(len(x_coords)/sample_size)
    best_models = []

    for i in range(0,num_samples):
        x = []
        y = []
        for j in range(0,sample_size):
            x.append([x_coords[i*sample_size + j]])
            y.append([y_coords[i*sample_size + j]])

        #print("x = ",x)
        x = np.array(x)
        #print("x = ",x)
        y = np.array(y)
        result = ransac(x,y,inlier_threshold=inlier_threshold,min_inliers=min_inliers)
        best_models.append(result)
        
        

        #Plotting
        m = result[0][0]
        b = result[1][0]
        if(i%2==0):
            plt.plot(x,m*x+b,'b',linewidth=4)
        else:
            plt.plot(x,m*x+b,'r',linewidth=4)

    return best_models

def calculate_intercept_angle(line1,line2):
    interAngle = np.pi/2
    m1 = line1[0][0]
    m2 = line2[0][0]
    b1 = line1[1][0]
    b2 = line2[1][0]
    #Is angle 90 degrees
    if(m1 * m2== -1):
        return interAngle
    else:
        #Absolute value to counter -90 being thrown out
        interAngle = abs(math.atan((m2 - m1)/(1 + m1*m2)))
        return interAngle
    
def calculate_intercept_point(line1,line2):
    m1 = line1[0][0]
    m2 = line2[0][0]
    b1 = line1[1][0]
    b2 = line2[1][0]

    #Find x-coordinate
    x = (b2 - b1)/(m1 - m2)
    #Find y-coordinate
    y = x * m1 + b1

    return x,y
    


def find_corners(best_models, angleThresh = 30*np.pi/180):
    corners = []
    for i in range(0,len(best_models)):

        
        if(i<len(best_models)-1):
            #Is next line corner
            interAngle = calculate_intercept_angle(best_models[i],best_models[i+1])
            x,y = calculate_intercept_point(best_models[i],best_models[i+1])


            if(interAngle < np.pi/2+angleThresh and interAngle > np.pi/2-angleThresh):
                corners.append([x,y])
                plt.plot(x, y, 'X', label='Points',markersize=20,color='g')
            
            elif(i<len(best_models)-2):
                #Angle between current and next next
                interAngle1 = calculate_intercept_angle(best_models[i],best_models[i+2])
                #Angle between next and next next
                interAngle2 = calculate_intercept_angle(best_models[i+1],best_models[i+2])
                
                ang1 = False
                ang2 = True
                if(interAngle1 < np.pi/2+angleThresh and interAngle1 > np.pi/2-angleThresh): ang1 = True
                if(interAngle2 < np.pi/2+angleThresh and interAngle2 > np.pi/2-angleThresh): ang2 = False

                if(ang1 == True and ang2 == True):
                    x,y = calculate_intercept_point(best_models[i],best_models[i+2])
                    corners.append([x,y])
                    plt.plot(x, y, 'X', label='Points',markersize=20,color='orange')

    return corners


def filter_corners(corners,x1,y1,duplicateThresh = 100, closenessThresh = 40):
    #Remove Duplicates
    clean_corners = []
    for i in range(0,len(corners)):
        dup = False
        for j in range(0,len(clean_corners)):
            dist = math.sqrt(pow(corners[i][0] - clean_corners[j][0] ,2) + pow(corners[i][1] - clean_corners[j][1] ,2))
            if(dist < duplicateThresh):
                dup = True
        if(dup == False):
            clean_corners.append(corners[i])

    #Remove Points far away from data
    close_corners = []
    
    for i in range(0,len(clean_corners)):
        dist_min = 10000000
        for j in range(0,len(x1)):
            dist_temp = math.sqrt(pow(x1[j] - clean_corners[i][0] ,2) + pow(y1[j] - clean_corners[i][1] ,2))
            if(dist_min > dist_temp):
                dist_min = dist_temp
            
        if(dist_min < closenessThresh):
            #print(i," ",clean_corners[i], " dist = ",dist_min)
            close_corners.append(clean_corners[i]) 


    # print("Num Corners = ",len(corners))
    # print("Num UniqueCorners = ",len(clean_corners))
    # print("Num CloseCorners = ",len(close_corners))
    return close_corners

def perform_stats(corner_sets):
    if len(corner_sets) < 2:
        print("At least two corner sets are required for comparison.")
        return

    num_corners_per_set = [len(corners) for corners in corner_sets]

    # Calculate statistics for the number of corners
    min_corners = min(num_corners_per_set)
    max_corners = max(num_corners_per_set)
    avg_corners = np.mean(num_corners_per_set)
    variance_corners = np.var(num_corners_per_set)
    std_dev_corners = np.std(num_corners_per_set)

    # Display the statistics for the number of corners
    print("Statistics for the number of corners:")
    print("Minimum number of corners in a set:", min_corners)
    print("Maximum number of corners in a set:", max_corners)
    print("Average number of corners in sets:", avg_corners)
    print("Variance of the number of corners:", variance_corners)
    print("Standard deviation of the number of corners:", std_dev_corners)

    # Calculate statistics for corresponding pairs of values
    num_sets = len(corner_sets)
    num_corners = min_corners  # Number of corners to consider in each set

    for i in range(num_corners):
        x_values = [corners[i][0] for corners in corner_sets]
        y_values = [corners[i][1] for corners in corner_sets]

        mean_x = np.mean(x_values)
        mean_y = np.mean(y_values)
        std_dev_x = np.std(x_values)
        std_dev_y = np.std(y_values)

        # Display the statistics for corresponding pairs of values
        print("\nStatistics for corner coordinates (Pair {}):".format(i + 1))
        print("Mean x-coordinate:", mean_x)
        print("Mean y-coordinate:", mean_y)
        print("Standard deviation of x-coordinate:", std_dev_x)
        print("Standard deviation of y-coordinate:", std_dev_y)


def brute_force(inlier_thresh,min_inliers):
    corner_sets = []
    for i in range(0,50):
        x1,y1=fetchCoord('map4.csv')
        best_models = manager(x1,y1,inlier_thresh,min_inliers)
        corners = find_corners(best_models)
        filtered_corner = filter_corners(corners,x1,y1)
        corner_sets.append(filtered_corner)

    perform_stats(corner_sets)





#brute_force(inlier_thresh=0.15,min_inliers=8)

x1,y1=fetchCoord('map4.csv')
best_models = manager(x1,y1,inlier_threshold=0.1,min_inliers=8)
plt.plot(x1, y1, 'o', label='Points',markersize=0.5,color='grey')

corners = find_corners(best_models)
filtered_corner = filter_corners(corners,x1,y1)



print(filtered_corner)

for i in range(0,len(filtered_corner)):
    plt.plot(filtered_corner[i][0], filtered_corner[i][1], 'X', label='Points',markersize=20,color='k')


# plt.ylim([-300,300])
# plt.xlim([-300,300])
plt.ylim([-2500,3000])
plt.xlim([-5000,1500])
plt.show()


















# def find_corners(best_models, angleThresh = 10*np.pi/180):
#     corners = []
#     for i in range(0,len(best_models)):

#         #Is the next line on the corner?
#         #Thus check myself to the next next line
#         if(i<len(best_models)-1):
#             interAngle = calculate_intercept_angle(best_models[i],best_models[i+1])


#             if(interAngle < np.pi/2+angleThresh and interAngle > np.pi/2-angleThresh):
#                 corners.append(interAngle)
#                 x,y = calculate_intercept_point(best_models[i],best_models[i+1])
#                 plt.plot(x, y, 'X', label='Points',markersize=20,color='g')
            
#             elif(i<len(best_models)-2):
#                 interAngle1 = calculate_intercept_angle(best_models[i],best_models[i+2])
#                 interAngle2 = calculate_intercept_angle(best_models[i+1],best_models[i+2])
                
#                 ang1 = False
#                 ang2 = True
#                 if(interAngle1 < np.pi/2+angleThresh and interAngle1 > np.pi/2-angleThresh): ang1 = True
#                 if(interAngle2 < np.pi/2+angleThresh and interAngle2 > np.pi/2-angleThresh): ang2 = False

#                 if(ang1 == True and ang2 == False):
#                     corners.append(interAngle)
#                     x,y = calculate_intercept_point(best_models[i],best_models[i+1])
#                     plt.plot(x, y, 'X', label='Points',markersize=20,color='g')
