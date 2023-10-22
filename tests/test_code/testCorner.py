import csv
import matplotlib.pyplot as plt
import numpy as np
import math
import time

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
def ransac(X,y,max_iters=500,samples_to_fit=2,inlier_threshold=0.15,min_inliers=8):
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

def manager(x_coords,y_coords,sample_size=100,max_iters=500,inlier_threshold=0.05,min_inliers=7):
    
    #sample_size = 100 # 500 worked well for map1
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
        result = ransac(x,y,max_iters = max_iters,inlier_threshold=inlier_threshold,min_inliers=min_inliers)
        best_models.append(result)
        
        # if(len(result) == 0):


        # #Plotting
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
                #plt.plot(x, y, 'X', label='Points',markersize=20,color='g')
            
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
                    #plt.plot(x, y, 'X', label='Points',markersize=20,color='orange')

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


def my_stats(corners_sets,corner):
    
    #I believe this testing site assumes that all corners are found all the time
    distances = []

    for j in range(0,len(corners_sets)):
            dist_min =1000000
            for z in range(0,len(corners_sets[j])):
                corner2 = [corners_sets[j][z][0],corners_sets[j][z][1]]
                dist = math.sqrt(pow(corner[0] - corner2[0] ,2) + pow(corner[1] - corner2[1] ,2))
                if(dist<dist_min):
                    dist_min = dist
            distances.append(dist_min)

    #print(distances)
    dist_avg = sum(distances)/len(distances)
    print("Average Distance Deviation = ",dist_avg)
    print("Max Distance Deviation = ",max(distances))
    print("Min Distance Deviation = ",min(distances))

    return dist_avg

                


def brute_force(mapCSV,sample_size,max_iters,inlier_thresh,min_inliers):
    print(mapCSV)
    corner_sets = []
    times = []
    for i in range(0,45):
        time_start = time.time()
        x1,y1=fetchCoord(mapCSV)
        best_models = manager(x1,y1,sample_size,max_iters,inlier_thresh,min_inliers)
        corners = find_corners(best_models)
        filtered_corner = filter_corners(corners,x1,y1)
        corner_sets.append(filtered_corner)
        times.append(time.time()-time_start)
        print(i," Corners Found = ",len(filtered_corner))
        
        
    corner = [corner_sets[0][0][0],corner_sets[0][0][1]]
    e1 = my_stats(corner_sets,corner)

    corner = [corner_sets[0][1][0],corner_sets[0][1][1]]
    e2 = my_stats(corner_sets,corner)

    e_avg = (e1+e2)/2
    print("e_avg = ",e_avg)
    print("time_avg = ",sum(times)/len(times))



#brute_force('map2.csv',sample_size=100,max_iters=200,inlier_thresh=0.3,min_inliers=5)

# brute_force('map3.csv',sample_size=100,max_iters=200,inlier_thresh=0.5,min_inliers=5)
# brute_force('map1.csv',sample_size=100,max_iters=200,inlier_thresh=0.5,min_inliers=5)
# #brute_force('map2.csv',sample_size=100,max_iters=200,inlier_thresh=0.4,min_inliers=5)

# brute_force('map4.csv',sample_size=100,max_iters=200,inlier_thresh=0.5,min_inliers=5)
# brute_force('map5.csv',sample_size=100,max_iters=200,inlier_thresh=0.5,min_inliers=5)
# brute_force('map6.csv',sample_size=100,max_iters=200,inlier_thresh=0.5,min_inliers=5)



# brute_force('map3.csv',sample_size=80,max_iters=200,inlier_thresh=1.2,min_inliers=6)
# brute_force('map1.csv',sample_size=80,max_iters=200,inlier_thresh=1.2,min_inliers=6)

# brute_force('map4.csv',sample_size=80,max_iters=200,inlier_thresh=1.2,min_inliers=6)
# brute_force('map5.csv',sample_size=80,max_iters=200,inlier_thresh=1.2,min_inliers=6)
# brute_force('map6.csv',sample_size=80,max_iters=200,inlier_thresh=1.2,min_inliers=6)

#brute_force('map7.csv',sample_size=80,max_iters=200,inlier_thresh=0.8,min_inliers=5)

#Works fairly well for large rooms.(Did miss once during 3x20 Trial runs thus about a 44/45 Hit ratio)
brute_force('map7.csv',sample_size=80,max_iters=200,inlier_thresh=1.3,min_inliers=6)


# brute_force('map3.csv',sample_size=100,max_iters=200,inlier_thresh=0.15,min_inliers=4)

x1,y1=fetchCoord('map7.csv')
best_models = manager(x1,y1,sample_size=80,max_iters=200,inlier_threshold=1.2,min_inliers=6)
#best_models = manager(x1,y1,sample_size=100,max_iters=200,inlier_threshold=0.4,min_inliers=5)
plt.plot(x1, y1, 'o', label='Points',markersize=0.5,color='grey')

corners = find_corners(best_models)
filtered_corner = filter_corners(corners,x1,y1)



print(filtered_corner)

for i in range(0,len(filtered_corner)):
    plt.plot(filtered_corner[i][0], filtered_corner[i][1], 'X', label='Points',markersize=20,color='k')


# plt.ylim([-300,300])
# plt.xlim([-300,300])
plt.ylim([-2500,3000])
plt.xlim([-3000,5000])
plt.show()





#max_iters=500,samples_to_fit=2,inlier_threshold=0.15,min_inliers=8, 100 samples

# [0.0, 1.6795418857670543, 1.6024588285219787, 1.7227192175520005, 1.5792777141710888, 1.735168732093464, 1.5400806965457638, 1.714095958040051, 1.886048262317713, 1.539348353613621]
# AVERAGE DEVIATION =  1.4998739648622734
# [0.0, 1.3711593239250213, 1.4165041909955283, 1.2686815544415655, 1.2175013046744794, 1.3833955196118533, 0.3963472833197897, 1.3494471652502809, 1.2375097328341806, 1.207178055131322]
# AVERAGE DEVIATION =  1.0847724130184022
# e_avg =  1.2923231889403377
# time_avg =  9.041740822792054

# sample_size=100,max_iters=200,inlier_thresh=0.15,min_inliers=4
# [0.0, 0.6459454488178182, 0.07610942272824515, 0.2545409382061566, 0.29782761923989093, 0.13865517032738775, 0.021271862680991835, 1.0674948274723868, 0.655211695645616, 0.6482362319941747, 0.2953704379176504, 0.8697181432529707, 0.07247004426869422, 0.1432060654037948, 0.21170868519604297, 0.30692293685258676, 0.6376839545382601, 0.2469799452388555, 0.12857372801943043, 0.6236768967757662]       
# AVERAGE DEVIATION =  0.36708020272883607
# [0.0, 0.2850801753448996, 0.16749312054582402, 0.13710180762001747, 1.593684642866083, 1.4913196396321542, 0.3316370596515697, 0.32989369569346905, 1.8290689918403016, 0.962057521474797, 0.17301349019242684, 0.8804122716385442, 1.54282769673374, 0.35180766064713076, 0.8805050696039433, 0.3387959101508535, 0.32771786375983436, 2.0416209958750975, 0.4352370427529045, 0.3632409788065563]
# AVERAGE DEVIATION =  0.7231257817415074
# e_avg =  0.5451029922351718
# time_avg =  3.6853674411773683

# sample_size=50,max_iters=200,inlier_thresh=0.15,min_inliers=4
# [0.0, 2.294403907496763, 0.6494437832100356, 1.6524816122364505, 0.02444893155445461, 0.037585910399321375, 0.1365174745940637, 0.20750159286545422, 3.4947792255708925, 0.6154582261568483, 0.8847516627717656, 0.45628811996924673, 0.5116233451687628, 0.44448649103389004, 0.537819984785843, 0.13621807576040398, 0.13651747459413702, 1.0862564852640157, 0.03817928374595871, 0.44450504779746813]       
# AVERAGE DEVIATION =  0.6894633317487889
# [0.0, 0.4043014487865095, 0.894219624791461, 1.8188596818694627, 0.8122826240958245, 1.8405514241800867, 0.8486385766654658, 0.8400828097302188, 0.8413453128824592, 0.19359576289094532, 0.8100817449485499, 0.8162551624031072, 0.8413453128843952, 0.40058665798235116, 0.9699988471898414, 0.08152501027426365, 0.8459041143231274, 0.12063036340736935, 0.8037540687163622, 0.8127181795685051]
# AVERAGE DEVIATION =  0.7498338363795154
# e_avg =  0.7196485840641522
# time_avg =  7.151280677318573










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
