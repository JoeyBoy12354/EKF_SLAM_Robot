"""
Extended Kalman Filter SLAM example

author: Atsushi Sakai (@Atsushi_twi)
"""

import csv

import math

import matplotlib.pyplot as plt
import numpy as np

# EKF state covariance
Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)]) ** 2

#  Simulation parameter
Q_sim = np.diag([10, np.deg2rad(1.0)]) ** 2
#Q_sim = np.diag([0.05, np.deg2rad(0.5)]) ** 2
R_sim = np.diag([1.0, np.deg2rad(10.0)]) ** 2

all_lm_inter = []

#DT = 0.1  # time tick [s]
DT = 1  # time tick [s]
#SIM_TIME = 50.0  # simulation time [s]
SIM_TIME = 50 # simulation time [s]
MAX_RANGE = 30000.0  # maximum observation range
M_DIST_TH = 100.0  # Threshold of Mahalanobis distance for data association.
STATE_SIZE = 3  # State size [x,y,yaw]
LM_SIZE = 2  # LM state size [x,y]

show_animation = True


def ekf_slam(xEst, PEst, u, z):
    # Predict
    S = STATE_SIZE
    G, Fx = jacob_motion(xEst[0:S], u)
    xEst[0:S] = motion_model(xEst[0:S], u)
    #print("xExt after motion = \n",xEst)
    # print("motion_model G = \n",G)

    # print("PEst Initial = \n",PEst)
    # print("G(motion) = \n",G)
    # print("C(noise) = \n",Cx)
    # print("G.T @ PEst @ G = \n",G.T @ PEst[0:S, 0:S] @ G)
    # print("Fx.T @ Cx @ Fx = \n",Fx.T @ Cx @ Fx)
    PEst[0:S, 0:S] = G.T @ PEst[0:S, 0:S] @ G + Fx.T @ Cx @ Fx
    #print("PEst = \n", PEst)


    initP = np.eye(2)

    lm_group = []

    # Update
    for iz in range(len(z[:, 0])):  # for each observation
        #print("in lm = ",z[iz, 0:2])
        min_id = search_correspond_landmark_id(xEst, PEst, z[iz, 0:2])

        nLM = calc_n_lm(xEst)
        if min_id == nLM:
            # print("New LM")
            # print("New LM at = ",calc_landmark_position(xEst, z[iz, :]))
            # Extend state and covariance matrix
            
            xAug = np.vstack((xEst, calc_landmark_position(xEst, z[iz, :])))
            PAug = np.vstack((np.hstack((PEst, np.zeros((len(xEst), LM_SIZE)))),
                              np.hstack((np.zeros((LM_SIZE, len(xEst))), initP))))
            xEst = xAug
            PEst = PAug
        
        #this will fetch the x and y of the new landmark or of the assosciated stored landmark
        lm = get_landmark_position_from_state(xEst, min_id)
        lm_group.append( [lm[0],lm[1]] ) 
        y, S, H = calc_innovation(lm, xEst, PEst, z[iz, 0:2], min_id)

        K = (PEst @ H.T) @ np.linalg.inv(S)
        # print("S^(-1) = \n",np.linalg.inv(S))
        # print("PEst @ H.T = \n",PEst @ H.T)

        # print("K = \n",K)
        
        #print("y = \n",y)
        #print("K = \n",K)
        xEst = xEst + (K @ y)
        #print("xExt after correction = \n",xEst)
        #print("xEst = ",xEst)
        PEst = (np.eye(len(xEst)) - (K @ H)) @ PEst

    #print("XEst prior = ",xEst[2]*180/np.pi)
    xEst[2] = pi_2_pi(xEst[2])
    #print("XEst prior = ",xEst[2]*180/np.pi)
    all_lm_inter.append(lm_group)


    return xEst, PEst


def calc_input():
    v = 100.0  # [m/s]
    yaw_rate = 0.1  # [rad/s]
    u = np.array([[v, yaw_rate]]).T
    return u


def observation(xTrue, xd, u, RFID):
    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    z = np.zeros((0, 3))

    for i in range(len(RFID[:, 0])):

        dx = RFID[i, 0] - xTrue[0, 0]
        dy = RFID[i, 1] - xTrue[1, 0]

        #print("obs dx,dy = ",[dx,dy])
        d = math.hypot(dx, dy)
        angle = pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])
        if d <= MAX_RANGE:
            #print("dn = d + noise = ",dn," = ",d," + ",np.random.randn() * Q_sim[0, 0] ** 0.5)
            dn = d + np.random.randn() * Q_sim[0, 0] ** 0.5  # add noise
            print("dn = d + noise = ",dn," = ",d," + ",np.random.randn() * Q_sim[0, 0] ** 0.5)
            angle_n = angle + np.random.randn() * Q_sim[1, 1] ** 0.5  # add noise
            zi = np.array([dn, angle_n, i])
            z = np.vstack((z, zi))

    # add noise to input
    u_prenoise = [u[0, 0],u[1, 0]]
    ud = np.array([[
        u[0, 0] + np.random.randn() * R_sim[0, 0] ** 0.5,
        u[1, 0] + np.random.randn() * R_sim[1, 1] ** 0.5]]).T

        print("u[0] = u[0] + noise ",u[0, 0]," = ",u_prenoise[0]," + ",R_sim[0, 0] ** 0.5)
        print("u[1] = u[1] + noise ",u[1, 0]," = ",u_prenoise[1]," + ",R_sim[1, 1] ** 0.5)

    xd = motion_model(xd, ud)
    return xTrue, z, xd, ud


def motion_model(x, u):
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT]])

    x = (F @ x) + (B @ u)
    return x


def calc_n_lm(x):
    n = int((len(x) - STATE_SIZE) / LM_SIZE)
    return n


def jacob_motion(x, u):
    Fx = np.hstack((np.eye(STATE_SIZE), np.zeros(
        (STATE_SIZE, LM_SIZE * calc_n_lm(x)))))

    jF = np.array([[0.0, 0.0, -DT * u[0, 0] * math.sin(x[2, 0])],
                   [0.0, 0.0, DT * u[0, 0] * math.cos(x[2, 0])],
                   [0.0, 0.0, 0.0]], dtype=float)
    
    G = np.eye(STATE_SIZE) + Fx.T @ jF @ Fx

    return G, Fx,


def calc_landmark_position(x, z):
    # print("z = ",z)
    # print("x = ",x)
    zp = np.zeros((2, 1))

    zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
    zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])

    #print("zp = ",zp)

    return zp


def get_landmark_position_from_state(x, ind):
    lm = x[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]

    return lm


def search_correspond_landmark_id(xAug, PAug, zi):
    """
    Landmark association with Mahalanobis distance
    """
    
    #print("\nIn landmark Association")
    nLM = calc_n_lm(xAug)

    min_dist = []

    #print("Observed Landmark polar = ",zi[0],", ",zi[1])
    #print("Observed Landmark carti = ",zi[0]*np.cos(zi[1]),", ",zi[0]*np.sin(zi[1]),")")

    for i in range(nLM):
        lm = get_landmark_position_from_state(xAug, i)
        #print("Stored Landmark lm = ",lm[0],", ",lm[1],"Index = ",i)
        #print("z = \n",zi)
        y, S, H = calc_innovation(lm, xAug, PAug, zi, i)
        #print("y = \n",y)
        #print("min distance = ",y.T @ np.linalg.inv(S) @ y)
        min_dist.append(y.T @ np.linalg.inv(S) @ y)

    min_dist.append(M_DIST_TH)  # new landmark

    min_id = min_dist.index(min(min_dist))

    return min_id


def calc_innovation(lm, xEst, PEst, z, LMid):
    delta = lm - xEst[0:2]
    #print("xEst = ",xEst[0:2])
    #print("stored = ",lm)
    #print("x = ",xEst[0],", y = ",xEst[1]," ang = ",xEst[2]*180/np.pi)
    #print("deltaX = ",delta[0]," deltaY = ",delta[1])
    #print("delta = ",delta)
    q = (delta.T @ delta)[0, 0]
    #print("q_matrix =\n",(delta.T @ delta))

    z_angle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
    #print("z_angle = ",z_angle*180/np.pi," -> ",pi_2_pi(z_angle)*180/np.pi)
    zp = np.array([[math.sqrt(q), pi_2_pi(z_angle)]])
    #print("z_cap = \n",zp)

    #print("z = \n",z)
    #print("zp = \n",zp)
    y = (z - zp).T
    #print("y(before pi2pi) = \n",y)
    #print("y = \n",y)
    y[1] = pi_2_pi(y[1])
    #print("after y = \n",y)
    H = jacob_h(q, delta, xEst, LMid + 1)
    S = H @ PEst @ H.T + Cx[0:2, 0:2]
    # print("H_observation_jacob = \n",H)
    # print("PEst = \n",PEst)
    # print("Cx (noise) = \n",Cx[0:2, 0:2])
    # print("S = \n",S)

    return y, S, H


def jacob_h(q, delta, x, i):
    sq = math.sqrt(q)
    G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                  [delta[1, 0], - delta[0, 0], - q, - delta[1, 0], delta[0, 0]]])

    G = G / q
    nLM = calc_n_lm(x)
    F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
    F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                    np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))

    F = np.vstack((F1, F2))

    H = G @ F

    return H


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def write_lm_csv(lm, filename='atsi_lmCSV.csv'):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        for group in lm:
            for item in group:
                flattened = [item[0][0], item[1][0]]
                writer.writerow(flattened)


def write_lm_csv2(lm,filename = 'atsi_lmCSV.csv'):
    #print("atsi_lm")

    #print("lm = ",lm)
    landmarks = []
    for i in range(0,len(lm)):
        for j in range(0,len(lm[i])):
            landmarks.append(lm[i][j])

    
    #print(landmarks)
    with open(filename, mode='w', newline='') as file:
        
        writer = csv.writer(file)
        for i in range(0,len(lm)):
            # Write the data to the CSV file
            writer.writerows(landmarks)
    return

def write_ati_odo_csv(hox,hoy,filename = 'atsi_odoCSV.csv'):
    #print("atsi_u")
    print("hox = ",hox)
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        for i in range(0,len(hox)):
        
            # Write the data to the CSV file
            writer.writerow([hox[i],hoy[i]])
    
    return

def write_ati_path_csv(hx,hy,filename = 'atsi_pathCSV.csv'):
    #print("atsi_u")
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        for i in range(0,len(hx)):
        
            # Write the data to the CSV file
            writer.writerow([hx[i],hy[i]])
    
    return

def write_ati_lm_true_csv(lm_tx,lm_ty,filename = 'atsi_lm_trueCSV.csv'):
    #print("atsi_u")
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        for i in range(0,len(lm_tx)):
            # Write the data to the CSV file
            writer.writerow([lm_tx[i],lm_ty[i]])
    
    return




def write_u_csv(u,filename = 'atsi_uCSV.csv'):
    #print("atsi_u")
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        for i in range(0,len(u)):
        
            # Write the data to the CSV file
            writer.writerows(u[i])
    
    return





def fetchState():
    
    state = []
    with open('ekf_uCSV.csv', 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x = float(row[0])
            y = float(row[1])
            state.append([x,y])

    return state

def fetchLandmarks():
    landmarks = []
    current_group = []  # Initialize an empty list to store the current group of 3 landmarks
    with open('ekf_lmCSV.csv', 'r') as file:
        csv_reader = csv.reader(file, delimiter=',')  # Specify the delimiter as a comma
        for row in csv_reader:
            # Convert the values to floats
            row = [float(value) for value in row]

            current_group.append([row[0], row[1]])  # Store x and y as a list
            current_group.append([row[2], row[3]])
            current_group.append([row[4], row[5]])

            # Check if we have collected 3 landmarks in the current group
            if len(current_group) == 3:
                landmarks.append(current_group)
                current_group = []  # Reset the current group for the next 3 landmarks

    return landmarks

def fetchTrueState():
    state = []
    with open('atsi_pathCSV.csv', 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x = float(row[0])
            y = float(row[1])
            state.append([x,y])

    return state

def fetchTrueLM():
    state = []
    with open('atsi_lm_trueCSV.csv', 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x = float(row[0])
            y = float(row[1])
            state.append([x,y])

    return state

def fetchOdo():
    state = []
    with open('atsi_odoCSV.csv', 'r') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            x = float(row[0])
            y = float(row[1])
            state.append([x,y])

    return state


def newCustomPlot():
    myLM = fetchLandmarks()
    myX = fetchState()
    trueX = fetchTrueState()
    trueLM = fetchTrueLM()
    odo = fetchOdo()


    # Scatter plot for myX
    myX = list(zip(*myX))  # Transpose myX for plotting
    plt.plot(myX[0], myX[1], c='orange', label='myX')

    # Scatter plot for atiLM
    for lm in myLM:
        lm = list(zip(*lm))  # Transpose each set of landmarks
        plt.scatter(lm[0], lm[1],c='yellow' ,marker='x')


    # Scatter plot for trueX
    trueX = list(zip(*trueX))  # Transpose trueX for plotting
    plt.scatter(trueX[0], trueX[1], c='blue', label='trueX')

    # Scatter plot for trueLM
    for lm in trueLM:
        lm = list(zip(*lm))  # Transpose each set of landmarks
        plt.scatter(lm[0], lm[1], marker='o', label='trueLM')

    plt.legend()
    plt.title(f"Custom Plot at time {time}")
    plt.xlabel("X-coordinate")
    plt.ylabel("Y-coordinate")
    plt.grid(True)
    plt.show()

def plotCustom(myLM,myX,atiX,atiLM,trueX,trueLM,time):

    # Scatter plot for atiX
    atiX = list(zip(*atiX))  # Transpose atiX for plotting
    plt.plot(atiX[0], atiX[1], c='red', label='atiX')

    # Scatter plot for atiLM
    for lm in atiLM:
        lm = list(zip(*lm))  # Transpose each set of landmarks
        plt.scatter(lm[0], lm[1],c='g' ,marker='x')

    # Scatter plot for myX
    myX = list(zip(*myX))  # Transpose myX for plotting
    plt.plot(myX[0], myX[1], c='orange', label='myX')

    # Scatter plot for atiLM
    for lm in myLM:
        lm = list(zip(*lm))  # Transpose each set of landmarks
        plt.scatter(lm[0], lm[1],c='yellow' ,marker='x')


    # # Scatter plot for trueX
    # trueX = list(zip(*trueX))  # Transpose trueX for plotting
    # plt.scatter(trueX[0], trueX[1], c='blue', label='trueX')

    # # Scatter plot for trueLM
    # for lm in trueLM:
    #     lm = list(zip(*lm))  # Transpose each set of landmarks
    #     plt.scatter(lm[0], lm[1], marker='o', label='trueLM')

    plt.legend()
    plt.title(f"Custom Plot at time {time}")
    plt.xlabel("X-coordinate")
    plt.ylabel("Y-coordinate")
    plt.grid(True)
    plt.show()

def main():
    print(__file__ + " start!!")

    time = 0.0

    # RFID positions [x, y]
    # RFID = np.array([[10.0, -2.0],
    #                  [15.0, 10.0],
    #                  [3.0, 15.0],
    #                  [-5.0, 20.0]])
    
    RFID = np.array([[800.0, 800.0],
                     [750.0, -700.0],
                     [-1000.0, -700.0]])

    # State Vector [x y yaw v]'
    xEst = np.zeros((STATE_SIZE, 1))
    xTrue = np.zeros((STATE_SIZE, 1))
    PEst = np.eye(STATE_SIZE)
    # print("PEST")
    # print(PEst)

    xDR = np.zeros((STATE_SIZE, 1))  # Dead reckoning

    # history
    hxEst = xEst
    hxTrue = xTrue
    hxDR = xTrue

    all_u = []
    all_lm = []

    atsi_state = []
    atsi_lm = []
    all_time = []
    true_state = []
    true_lm = []
    my_state = []
    my_lm = []

    my_lm = fetchLandmarks()
    my_state = fetchState()

    while SIM_TIME >= time:
        #print("\ni = ",time)
        time += DT
        u = calc_input()

        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)

        # lm_group = []
        # for i in range(len(RFID[:, 0])):
        #     myLM_x = RFID[i, 0] + np.random.randn() * Q_sim[0, 0] * 25
        #     myLM_y = RFID[i, 1] + np.random.randn() * Q_sim[0, 0] * 25
        #     lm_group.append([myLM_x,myLM_y])
            
        lm_group = []
        for i in range(0,len(z)):
            #print("xTrue, = ",xTrue[0, 0]," ",xTrue[1, 0])
            myLM_x = z[i][0]*np.cos(z[i][1]) + xTrue[0, 0]
            myLM_y = z[i][0]*np.sin(z[i][1]) + xTrue[1, 0]


            lm_group.append([myLM_x,myLM_y])

        all_lm.append(z)
        all_u.append(ud)
        
        # print("ud = ",ud)
        # write_u_csv(ud)
        #print("z = ",z)
        # write_lm_csv(z)

        xEst, PEst = ekf_slam(xEst, PEst, ud, z)

        x_state = xEst[0:STATE_SIZE]
        

        #Johann Storage Functions
        atsi_lm_temp = []
        for i in range(calc_n_lm(xEst)):
            x = xEst[STATE_SIZE + i * 2]
            y = xEst[STATE_SIZE + i * 2 + 1]
            atsi_lm_temp.append([x,y])
        atsi_lm.append(atsi_lm_temp)
        
        x = xEst[0]
        y = xEst[1]
        atsi_state.append([x,y])
            
        

        # store data history
        hxEst = np.hstack((hxEst, x_state))
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            plt.plot(RFID[:, 0], RFID[:, 1], "*k")
            plt.plot(xEst[0], xEst[1], ".r")

            # plot landmark
            for i in range(calc_n_lm(xEst)):
                plt.plot(xEst[STATE_SIZE + i * 2],
                         xEst[STATE_SIZE + i * 2 + 1], "xg")

            plt.plot(hxTrue[0, :],
                     hxTrue[1, :], "-b")
            plt.plot(hxDR[0, :],
                     hxDR[1, :], "-k")
            plt.plot(hxEst[0, :],
                     hxEst[1, :], "-r")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)

    # all_lm = np.array(all_lm_inter)
    all_lm = np.array(all_lm)
    all_u = np.array(all_u)
    write_u_csv(all_u)
    write_lm_csv2(all_lm)

    write_ati_path_csv(hxTrue[0, :],hxTrue[1, :])
    write_ati_odo_csv(hxDR[0, :],hxDR[1, :])
    write_ati_lm_true_csv(RFID[:, 0], RFID[:, 1])

    print("hxTrue")
    print(hxTrue)
    print(hxTrue[0, :])
    print(hxTrue[1, :])



    

    plotCustom(my_lm,my_state,atsi_state,atsi_lm,true_state,true_lm,all_time)


if __name__ == '__main__':
    main()