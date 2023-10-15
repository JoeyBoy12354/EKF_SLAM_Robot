Started in Main

 i = 0
------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------


Attempt 0
SLAMTEC LIDAR S/N: ABCF99F6C9E59AD4C5E59CF7571D3414SLAMTEC Lidar health status : 0
Entering scan loop
I have reached max NoPoints in Lidar_functioncount = 1
IN triangFunction 
NAVI,GRID: Wheels = (81,71),(81,-71)
NAVI,GRID: tri_rot = (0,0),(81,71),(81,-71)
NAVI,GRID: tri_shift = (0,0),(81,71),(81,-71)
EKF: ++C = (0,0)
EKF: ++Cangle = 0
EKF: ++Cd_x = -0
EKF: ++Cd_y = 0
EKF: ++X = 0
EKF: ++Y = 0

EKF: distance = 0mm   |  Angle0
EKF: d_x: -0 + 0 = 0
EKF: d_y: 0 + 0 = 0
EKF: d_theta: 0

 MAIN: PREDICTED POSITION: x=0, y=0, w=0 deg
Fit Cartesian for: (0,0) | 0
Could be: 180

Number of CAR points8316ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 2

 CSV: Saving Full Map (overwriting old)


GRID: In grid data process
GRID:SAVED TO CSV 


IN update Movement Grid
A
NAVI,GRID: deltaX = -200 = 200 - 0
NAVI,GRID: deltaY = 0 = -0 - 0
NAVI,GRID angle = 90 = 180 - 0

NAVI,GRID: Wheels = (81,71),(81,-71)
NAVI,GRID: tri_rot = (0,0),(81,71),(81,-71)
NAVI,GRID: tri_shift = (0,0),(81,71),(81,-71)
NAVI,GRID: deltaX = -210 = -200 - 10
NAVI,GRID: deltaY = -152 = 0 - 152
NAVI,GRID: dist1 = 67204 = 44100 - 23104
NAVI,GRID rotate angle = 90
NAVI,GRID around A = (81,71)
NAVI,GRID from B = (0,0)
NAVI,GRID resulting C (10,152)
NAVI,GRID to visit: (-200,0)
NAVI,GRID then distance = 0
NAVI: Set angle = 90 deg Set Distance = 0mm
NAVI: Run python

MC:TURN:  90.0002104591498  deg
MC: LEFT_Thread
LEFT SPEED =  1.206127186035307  rotations/s
RIGHT SPEED =  0.054823963001604865  rotations/s
MC GO FORWARD FOR:  0.0
MC: FORWARD_Thread
LEFT SPEED =  0.0  rotations/s
RIGHT SPEED =  67.00166134185304  rotations/s
MC: Angle turned =  1.5763078928538246
MC: distance moved =  0.0

ret/cpp = 0

Main_end: ekf.w = 1.57631 ekf.distance = 0
LEAVNG RUN

 i = 1
------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------


Attempt 0
SLAMTEC LIDAR S/N: ABCF99F6C9E59AD4C5E59CF7571D3414SLAMTEC Lidar health status : 0
Entering scan loop
I have reached max NoPoints in Lidar_functioncount = 1
IN triangFunction 
NAVI,GRID: Wheels = (81,71),(81,-71)
NAVI,GRID: tri_rot = (0,0),(81,71),(81,-71)
NAVI,GRID: tri_shift = (0,0),(81,71),(81,-71)
EKF: ++C = (10.4475,152.39)
EKF: ++Cangle = 1.57631
EKF: ++Cd_x = 0
EKF: ++Cd_y = 0
EKF: ++X = 0
EKF: ++Y = 0

EKF: distance = 0mm   |  Angle90.3158
EKF: d_x: 0 + 10 = 10
EKF: d_y: 0 + 152 = 10
EKF: d_theta: 1.57631

 MAIN: PREDICTED POSITION: x=10, y=152, w=90.3158 deg
Fit Cartesian for: (10,152) | 90.3158
Could be: 89.6842

Number of CAR points8387ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 3
Re-Observed: (762.622,-728.785)  Stored:(774.007,-731.661) 
Re-Observed: (817.339,463.472)  Stored:(787.78,482.198) 
No of New Points = 0

IN STORE STATE POINTS


GRID: In grid data process
GRID:SAVED TO CSV 


IN update Movement Grid
A
NAVI,GRID: deltaX = -210 = 200 - 10
NAVI,GRID: deltaY = 48 = -200 - 152
NAVI,GRID angle = 90 = 167.125 - 90.3158

NAVI,GRID: Wheels = (80.5525,70.6099),(-61.4454,71.3925)
NAVI,GRID: tri_rot = (0,-0),(70.5525,-81.3901),(-71.4454,-80.6075)
NAVI,GRID: tri_shift = (10,152),(80.5525,70.6099),(-61.4454,71.3925)
NAVI,GRID: deltaX = -361.943 = -200 - 161.943
NAVI,GRID: deltaY = 58.8376 = 200 - 141.162
NAVI,GRID: dist1 = 134464 = 131002 - 3461.86
NAVI,GRID rotate angle = 90
NAVI,GRID around A = (80.5525,70.6099)
NAVI,GRID from B = (10,152)
NAVI,GRID resulting C (161.943,141.162)
NAVI,GRID to visit: (-200,200)
NAVI,GRID then distance = 0
NAVI: Set angle = 90 deg Set Distance = 0mm
NAVI: Run python

MC:TURN:  90.0002104591498  deg
MC: LEFT_Thread
LEFT SPEED =  1.2066287326504443  rotations/s
RIGHT SPEED =  0.0548467605750202  rotations/s
MC GO FORWARD FOR:  0.0
MC: FORWARD_Thread
LEFT SPEED =  64.98766656337156  rotations/s
RIGHT SPEED =  64.98766656337156  rotations/s
MC: Angle turned =  1.5763078928538246
MC: distance moved =  10.210176124166818

ret/cpp = 0

Main_end: ekf.w = 1.57631 ekf.distance = 10.2102
LEAVNG RUN

 i = 2
------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------


Attempt 0
SLAMTEC LIDAR S/N: ABCF99F6C9E59AD4C5E59CF7571D3414SLAMTEC Lidar health status : 0
Entering scan loop
I have reached max NoPoints in Lidar_functioncount = 1
IN triangFunction 
NAVI,GRID: Wheels = (80.5525,70.6099),(-61.4454,71.3925)
NAVI,GRID: tri_rot = (0,-0),(70.5525,-81.3901),(-71.4454,-80.6075)
NAVI,GRID: tri_shift = (10,152),(80.5525,70.6099),(-61.4454,71.3925)
EKF: ++C = (162.33,140.713)
EKF: ++Cangle = 3.15262
EKF: ++Cd_x = 10.2096
EKF: ++Cd_y = -0.112546
EKF: ++X = 20.2096
EKF: ++Y = 151.887

EKF: distance = 10.2102mm   |  Angle90.3158
EKF: d_x: 0.0562738 + 161.943 = 161.999
EKF: d_y: 10.21 + 141.162 = 161.999
EKF: d_theta: 1.57631

 MAIN: PREDICTED POSITION: x=171.999, y=303.372, w=180.632 deg
Fit Cartesian for: (171.999,303.372) | 180.632
Could be: -0.631579

Number of CAR points8360ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 3
No of New Points = 1

IN STORE STATE POINTS


GRID: In grid data process
GRID:SAVED TO CSV 


IN update Movement Grid
A
NAVI,GRID: deltaX = -371.999 = 200 - 171.999
NAVI,GRID: deltaY = 96.6276 = -400 - 303.372
NAVI,GRID angle = 90 = 165.439 - 180.632

NAVI,GRID: Wheels = (90.2212,233.27),(91.7864,375.261)
NAVI,GRID: tri_rot = (-0,0),(-81.7777,-70.1028),(-80.2125,71.8885)
NAVI,GRID: tri_shift = (171.999,303.372),(90.2212,233.27),(91.7864,375.261)
NAVI,GRID: deltaX = -360.324 = -200 - 160.324
NAVI,GRID: deltaY = 248.508 = 400 - 151.492
NAVI,GRID: dist1 = 191590 = 129833 - 61756.3
NAVI,GRID rotate angle = 90
NAVI,GRID around A = (90.2212,233.27)
NAVI,GRID from B = (171.999,303.372)
NAVI,GRID resulting C (160.324,151.492)
NAVI,GRID to visit: (-200,400)
NAVI,GRID then distance = 0
NAVI: Set angle = 90 deg Set Distance = 0mm
NAVI: Run python

MC:TURN:  90.0002104591498  deg
MC: LEFT_Thread
LEFT SPEED =  1.1916596585262036  rotations/s
RIGHT SPEED =  0.054166348114827435  rotations/s
MC GO FORWARD FOR:  0.0
MC: FORWARD_Thread
LEFT SPEED =  4.169039619903386  rotations/s
RIGHT SPEED =  4.169039619903386  rotations/s
MC: Angle turned =  1.5763078928538246
MC: distance moved =  10.210176124166818

ret/cpp = 0

Main_end: ekf.w = 1.57631 ekf.distance = 10.2102
LEAVNG RUN

 i = 3
------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------


Attempt 0
SLAMTEC LIDAR S/N: ABCF99F6C9E59AD4C5E59CF7571D3414SLAMTEC Lidar health status : 0
Entering scan loop
I have reached max NoPoints in Lidar_functioncount = 1
IN triangFunction 
NAVI,GRID: Wheels = (90.2212,233.27),(91.7864,375.261)
NAVI,GRID: tri_rot = (-0,0),(-81.7777,-70.1028),(-80.2125,71.8885)
NAVI,GRID: tri_shift = (171.999,303.372),(90.2212,233.27),(91.7864,375.261)
EKF: ++C = (159.872,151.107)
EKF: ++Cangle = 4.72892
EKF: ++Cd_x = -0.168816
EKF: ++Cd_y = -10.2088
EKF: ++X = 171.83
EKF: ++Y = 293.164

EKF: distance = 10.2102mm   |  Angle90.3158
EKF: d_x: 0.0562738 + 160.324 = 160.38
EKF: d_y: 10.21 + 151.492 = 160.38
EKF: d_theta: 1.57631

 MAIN: PREDICTED POSITION: x=332.379, y=465.074, w=270.947 deg
Fit Cartesian for: (332.379,465.074) | 270.947
Could be: -90.9474

Number of CAR points8354ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 3
No of New Points = 1

IN STORE STATE POINTS


GRID: In grid data process
GRID:SAVED TO CSV 


IN update Movement Grid
A
NAVI,GRID: deltaX = 267.621 = -600 - 332.379
NAVI,GRID: deltaY = -65.0743 = -400 - 465.074
NAVI,GRID angle = 90 = -13.6667 - 270.947

NAVI,GRID: Wheels = (262.728,547.237),(404.709,544.889)
NAVI,GRID: tri_rot = (0,0),(-69.651,82.1628),(72.3296,79.815)
NAVI,GRID: tri_shift = (332.379,465.074),(262.728,547.237),(404.709,544.889)
NAVI,GRID: deltaX = 419.435 = 600 - 180.565
NAVI,GRID: deltaY = -77.5862 = 400 - 477.586
NAVI,GRID: dist1 = 181945 = 175926 - 6019.61
NAVI,GRID rotate angle = 90
NAVI,GRID around A = (262.728,547.237)
NAVI,GRID from B = (332.379,465.074)
NAVI,GRID resulting C (180.565,477.586)
NAVI,GRID to visit: (600,400)
NAVI,GRID then distance = 0
NAVI: Set angle = 90 deg Set Distance = 0mm
NAVI: Run python

MC:TURN:  90.0002104591498  deg
MC: LEFT_Thread
LEFT SPEED =  1.0846232057374088  rotations/s
RIGHT SPEED =  0.04930105480624585  rotations/s
MC GO FORWARD FOR:  0.0
MC: FORWARD_Thread
LEFT SPEED =  66.76701687360713  rotations/s
RIGHT SPEED =  66.76701687360713  rotations/s
MC: Angle turned =  1.5763078928538246
MC: distance moved =  10.210176124166818

ret/cpp = 0

Main_end: ekf.w = 1.57631 ekf.distance = 10.2102
LEAVNG RUN
