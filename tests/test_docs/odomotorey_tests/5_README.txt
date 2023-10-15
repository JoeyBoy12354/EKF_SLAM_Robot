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
EKF: distance = 0mm   |  Angle0
EKF: d_x: -0
EKF: d_y: 0
EKF: d_theta: 0

 MAIN: PREDICTED POSITION: x=0, y=0, w=0 deg
Fit Cartesian for: (0,0) | 0
Could be: 180

Number of CAR points8206ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 2

 CSV: Saving Full Map (overwriting old)


GRID: In grid data process
GRID:SAVED TO CSV 


IN update Movement Grid
A
NAVI,GRID: deltaX = 200 = -200 - 0
NAVI,GRID: deltaY = -200 = 200 - 0
NAVI,GRID angle = 90 = -45 - 0

NAVI,GRID: Wheels = (81,71),(81,-71)
NAVI,GRID: tri_rot = (0,0),(81,71),(81,-71)
NAVI,GRID: tri_shift = (0,0),(81,71),(81,-71)
NAVI,GRID: deltaX = 190 = 200 - 10
NAVI,GRID: deltaY = -352 = -200 - 152
NAVI,GRID: dist1 = 160004 = 36100 - 123904
NAVI,GRID rotate angle = 90
NAVI,GRID around A = (81,71)
NAVI,GRID from B = (0,0)
NAVI,GRID resulting C (10,152)
NAVI,GRID to visit: (200,-200)
NAVI,GRID then distance = 0
NAVI: Set angle = 90 deg Set Distance = 0mm
NAVI: Run python

MC:TURN:  90.0002104591498  deg
MC: LEFT_Thread
LEFT SPEED =  1.1268924652130692  rotations/s
RIGHT SPEED =  0.05122238478241224  rotations/s
MC GO FORWARD FOR:  0.0
MC: FORWARD_Thread
LEFT SPEED =  0.0  rotations/s
RIGHT SPEED =  95.9355901189387  rotations/s
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
EKF: distance = 0mm   |  Angle90.3158
EKF: d_x: 0
EKF: d_y: 0
EKF: d_theta: 90.3158

 MAIN: PREDICTED POSITION: x=10.4475, y=152.39, w=90.3158 deg
Fit Cartesian for: (10.4475,152.39) | 90.3158
Could be: 89.6842

Number of CAR points8286ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 1
Re-Observed: (763.526,-742.499)  Stored:(802.477,-711.939) 
No of New Points = 0

IN STORE STATE POINTS


GRID: In grid data process
GRID:SAVED TO CSV 


IN update Movement Grid
A
NAVI,GRID: deltaX = -210.448 = 200 - 10.4475
NAVI,GRID: deltaY = 247.61 = -400 - 152.39
NAVI,GRID angle = 90 = 130.362 - 90.3158

NAVI,GRID: Wheels = (81,71),(-60.9978,71.7826)
NAVI,GRID: tri_rot = (0,-0),(70.5525,-81.3901),(-71.4454,-80.6075)
NAVI,GRID: tri_shift = (10.4475,152.39),(81,71),(-60.9978,71.7826)
NAVI,GRID: deltaX = -362.39 = -200 - 162.39
NAVI,GRID: deltaY = 258.448 = 400 - 141.552
NAVI,GRID: dist1 = 198122 = 131327 - 66795.1
NAVI,GRID rotate angle = 90
NAVI,GRID around A = (81,71)
NAVI,GRID from B = (10.4475,152.39)
NAVI,GRID resulting C (162.39,141.552)
NAVI,GRID to visit: (-200,400)
NAVI,GRID then distance = 0
NAVI: Set angle = 90 deg Set Distance = 0mm
NAVI: Run python

MC:TURN:  90.0002104591498  deg
MC: LEFT_Thread
LEFT SPEED =  1.235927099141043  rotations/s
RIGHT SPEED =  0.05617850450641104  rotations/s
MC GO FORWARD FOR:  0.0
MC: FORWARD_Thread
LEFT SPEED =  63.66581663630844  rotations/s
RIGHT SPEED =  63.66581663630844  rotations/s
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
NAVI,GRID: Wheels = (81,71),(-60.9978,71.7826)
NAVI,GRID: tri_rot = (0,-0),(70.5525,-81.3901),(-71.4454,-80.6075)
NAVI,GRID: tri_shift = (10.4475,152.39),(81,71),(-60.9978,71.7826)
EKF: distance = 10.2102mm   |  Angle90.3158
EKF: d_x: 10.2096
EKF: d_y: -0.112546
EKF: d_theta: 90.3158

 MAIN: PREDICTED POSITION: x=172.987, y=140.99, w=180.632 deg
Fit Cartesian for: (172.987,140.99) | 180.632
Could be: -0.631579

Number of CAR points8289ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 2
Re-Observed: (793.95,-752.512)  Stored:(801.997,-710.688) 
No of New Points = 1

IN STORE STATE POINTS


GRID: In grid data process
GRID:SAVED TO CSV 


IN update Movement Grid
A
NAVI,GRID: deltaX = 27.0127 = -200 - 172.987
NAVI,GRID: deltaY = -340.99 = 200 - 140.99
NAVI,GRID angle = 90 = -85.4706 - 180.632

NAVI,GRID: Wheels = (91.2096,70.8874),(92.7748,212.879)
NAVI,GRID: tri_rot = (-0,0),(-81.7777,-70.1028),(-80.2125,71.8885)
NAVI,GRID: tri_shift = (172.987,140.99),(91.2096,70.8874),(92.7748,212.879)
NAVI,GRID: deltaX = 38.6876 = 200 - 161.312
NAVI,GRID: deltaY = -189.11 = -200 - -10.8903
NAVI,GRID: dist1 = 37259.2 = 1496.73 - 35762.5
NAVI,GRID rotate angle = 90
NAVI,GRID around A = (91.2096,70.8874)
NAVI,GRID from B = (172.987,140.99)
NAVI,GRID resulting C (161.312,-10.8903)
NAVI,GRID to visit: (200,-200)
NAVI,GRID then distance = 0
NAVI: Set angle = 90 deg Set Distance = 0mm
NAVI: Run python

MC:TURN:  90.0002104591498  deg
MC: LEFT_Thread
LEFT SPEED =  1.2252738567701473  rotations/s
RIGHT SPEED =  0.05569426621682488  rotations/s
MC GO FORWARD FOR:  0.0
MC: FORWARD_Thread
LEFT SPEED =  65.92744419993713  rotations/s
RIGHT SPEED =  65.92744419993713  rotations/s
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
NAVI,GRID: Wheels = (91.2096,70.8874),(92.7748,212.879)
NAVI,GRID: tri_rot = (-0,0),(-81.7777,-70.1028),(-80.2125,71.8885)
NAVI,GRID: tri_shift = (172.987,140.99),(91.2096,70.8874),(92.7748,212.879)
EKF: distance = 10.2102mm   |  Angle90.3158
EKF: d_x: -0.168816
EKF: d_y: -10.2088
EKF: d_theta: 90.3158

 MAIN: PREDICTED POSITION: x=160.692, y=-21.4842, w=270.947 deg
Fit Cartesian for: (160.692,-21.4842) | 270.947
Could be: -90.9474

Number of CAR points8228ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 2
No of New Points = 0

IN STORE STATE POINTS


GRID: In grid data process
GRID:SAVED TO CSV 


IN update Movement Grid
A
NAVI,GRID: deltaX = 39.3082 = -200 - 160.692
NAVI,GRID: deltaY = -178.516 = 200 - -21.4842
NAVI,GRID angle = 90 = -77.5819 - 270.947

NAVI,GRID: Wheels = (91.0408,60.6787),(233.021,58.3308)
NAVI,GRID: tri_rot = (0,0),(-69.651,82.1628),(72.3296,79.815)
NAVI,GRID: tri_shift = (160.692,-21.4842),(91.0408,60.6787),(233.021,58.3308)
NAVI,GRID: deltaX = 191.122 = 200 - 8.8779
NAVI,GRID: deltaY = -191.028 = -200 - -8.97235
NAVI,GRID: dist1 = 73019.2 = 36527.7 - 36491.6
NAVI,GRID rotate angle = 90
NAVI,GRID around A = (91.0408,60.6787)
NAVI,GRID from B = (160.692,-21.4842)
NAVI,GRID resulting C (8.8779,-8.97235)
NAVI,GRID to visit: (200,-200)
NAVI,GRID then distance = 0
NAVI: Set angle = 90 deg Set Distance = 0mm
NAVI: Run python

MC:TURN:  90.0002104591498  deg
MC: LEFT_Thread
LEFT SPEED =  1.10189678544782  rotations/s
RIGHT SPEED =  0.050086217520355455  rotations/s
MC GO FORWARD FOR:  0.0
MC: FORWARD_Thread
LEFT SPEED =  4.743936480647862  rotations/s
RIGHT SPEED =  4.743936480647862  rotations/s
MC: Angle turned =  1.5763078928538246
MC: distance moved =  10.210176124166818

ret/cpp = 0

Main_end: ekf.w = 1.57631 ekf.distance = 10.2102
LEAVNG RUN
