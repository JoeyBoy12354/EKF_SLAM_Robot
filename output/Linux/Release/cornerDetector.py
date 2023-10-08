import csv
import numpy as np
import math



def fetchCoord(filename = 'mapCSV.csv'):
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

def ransac(X,y,max_iters=200,samples_to_fit=2,inlier_threshold=5,min_inliers=35):

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

def manager(x_coords,y_coords):
    #sample_size = 500 #worked well before lowerd lidar points
    sample_size = 500 #worked well for map1
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
        result = ransac(x,y)
        best_models.append(result)
        

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

    return corners

def filter_corners(corners,x1,y1,duplicateThresh = 40, closenessThresh = 40):
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
            print(i," ",clean_corners[i], " dist = ",dist_min)
            close_corners.append(clean_corners[i]) 


    print("Num Corners = ",len(corners))
    print("Num UniqueCorners = ",len(clean_corners))
    print("Num CloseCorners = ",len(close_corners))
    return close_corners



x1,y1=fetchCoord()
print("CD: NoPoints = ",len(x1))
best_models = manager(x1,y1)
print("CD: NoLines = ",len(best_models))
corners = find_corners(best_models)
filtered_corners = filter_corners(corners,x1,y1)
writeCoord(filtered_corners)

print("CD: NoCorners = ",len(filtered_corners))