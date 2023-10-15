odroid@odroid-bullseye:~$ cd Desktop/Johann/rplidar_sdk-website/output/Linux/Release/
odroid@odroid-bullseye:~/Desktop/Johann/rplidar_sdk-website/output/Linux/Release$ sudo ./johann_code
[sudo] password for odroid: 
Sorry, try again.
[sudo] password for odroid: 
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

Number of CAR points9080ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 3

 CSV: Saving Full Map (overwriting old)


GRID: In grid data process
GRID:SAVED TO CSV 


IN update Movement Grid
A
NAVI,GRID: deltaX = 200 = -200 - 0
NAVI,GRID: deltaY = -200 = 200 - 0
NAVI,GRID angle = 0 = -45 - 0

NAVI,GRID: Wheels = (81,71),(81,-71)
NAVI,GRID: tri_rot = (0,0),(81,71),(81,-71)
NAVI,GRID: tri_shift = (0,0),(81,71),(81,-71)
NAVI,GRID: deltaX = 200 = 200 - 0
NAVI,GRID: deltaY = -200 = -200 - 0
NAVI,GRID: dist1 = 80000 = 40000 - 40000
NAVI,GRID rotate angle = 0
NAVI,GRID around A = (81,-71)
NAVI,GRID from B = (0,0)
NAVI,GRID resulting C (0,0)
NAVI,GRID to visit: (200,-200)
NAVI,GRID then distance = 100
NAVI: Set angle = 0 deg Set Distance = 100mm
NAVI: Run python

MC:TURN:  0.0  deg
MC: FORWARD_Thread
TIME =  0.08818435668945312 s
LEFT SPEED =  0.566993987109054  rotations/s, ROTATIONS =  0.05
RIGHT SPEED =  0.0  rotations/s, ROTATIONS =  0.0
MC GO FORWARD FOR:  100.0
MC: FORWARD_Thread
TIME =  0.5397474765777588 s
LEFT SPEED =  0.9263591247711326  rotations/s, ROTATIONS =  0.5
RIGHT SPEED =  0.0  rotations/s, ROTATIONS =  0.0
MC: Angle turned =  4.105263157894738
MC: distance moved =  102.10176124166827

ret/cpp = 0

Main_end: ekf.w = 0.0716504 ekf.distance = 102.102
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
EKF: distance = 102.102mm   |  Angle4.10526
EKF: d_x: -101.84
EKF: d_y: 7.30937
EKF: d_theta: 4.10526

 MAIN: PREDICTED POSITION: x=-106.715, y=13.2903, w=4.10526 deg
Fit Cartesian for: (-106.715,13.2903) | 4.10526
Could be: 175.895

Number of CAR points8227ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 2
No of New Points = 1

IN STORE STATE POINTS


GRID: In grid data process
GRID:SAVED TO CSV 


IN update Movement Grid
A
NAVI,GRID: deltaX = -93.2852 = 200 - -106.715
NAVI,GRID: deltaY = -213.29 = 200 - 13.2903
NAVI,GRID angle = 0 = -113.623 - 4.10526

NAVI,GRID: Wheels = (-20.8398,78.3094),(-31.0054,-63.3263)
NAVI,GRID: tri_rot = (0,0),(85.875,65.0191),(75.7093,-76.6165)
NAVI,GRID: tri_shift = (-106.715,13.2903),(-20.8398,78.3094),(-31.0054,-63.3263)
NAVI,GRID: deltaX = -93.2852 = -200 - -106.715
NAVI,GRID: deltaY = -213.29 = -200 - 13.2903
NAVI,GRID: dist1 = 54194.9 = 8702.13 - 45492.7
NAVI,GRID rotate angle = 0
NAVI,GRID around A = (-31.0054,-63.3263)
NAVI,GRID from B = (-106.715,13.2903)
NAVI,GRID resulting C (-106.715,13.2903)
NAVI,GRID to visit: (-200,-200)
NAVI,GRID then distance = 100
NAVI: Set angle = 0 deg Set Distance = 100mm
NAVI: Run python

MC:TURN:  0.0  deg
MC: FORWARD_Thread
TIME =  0.045375823974609375 s
LEFT SPEED =  1.1019083648591845  rotations/s, ROTATIONS =  0.05
RIGHT SPEED =  0.0  rotations/s, ROTATIONS =  0.0
MC GO FORWARD FOR:  100.0
MC: FORWARD_Thread
TIME =  0.48302674293518066 s
LEFT SPEED =  1.035139373364048  rotations/s, ROTATIONS =  0.5
RIGHT SPEED =  0.0  rotations/s, ROTATIONS =  0.0
MC: Angle turned =  4.105263157894738
MC: distance moved =  102.10176124166827

ret/cpp = 0

Main_end: ekf.w = 0.0716504 ekf.distance = 102.102
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
NAVI,GRID: Wheels = (-20.8398,78.3094),(-31.0054,-63.3263)
NAVI,GRID: tri_rot = (0,0),(85.875,65.0191),(75.7093,-76.6165)
NAVI,GRID: tri_shift = (-106.715,13.2903),(-20.8398,78.3094),(-31.0054,-63.3263)
EKF: distance = 102.102mm   |  Angle4.10526
EKF: d_x: -101.055
EKF: d_y: 14.5812
EKF: d_theta: 4.10526

 MAIN: PREDICTED POSITION: x=-212.204, y=34.186, w=8.21053 deg
Fit Cartesian for: (-212.204,34.186) | 8.21053
Could be: 171.789

Number of CAR points8194ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 3
Re-Observed: (762.795,632.251)  Stored:(782.357,591.445) 
No of New Points = 1

IN STORE STATE POINTS


GRID: In grid data process
GRID:SAVED TO CSV 


IN update Movement Grid
A
NAVI,GRID: deltaX = 12.2043 = 200 - -212.204
NAVI,GRID: deltaY = -234.186 = 200 - 34.186
NAVI,GRID angle = 0 = -87.0168 - 8.21053

NAVI,GRID: Wheels = (-121.895,92.8906),(-142.174,-47.6539)
NAVI,GRID: tri_rot = (0,0),(90.3093,58.7046),(70.0302,-81.8399)
NAVI,GRID: tri_shift = (-212.204,34.186),(-121.895,92.8906),(-142.174,-47.6539)
NAVI,GRID: deltaX = 12.2043 = -200 - -212.204
NAVI,GRID: deltaY = -234.186 = -200 - 34.186
NAVI,GRID: dist1 = 54992 = 148.946 - 54843.1
NAVI,GRID rotate angle = 0
NAVI,GRID around A = (-142.174,-47.6539)
NAVI,GRID from B = (-212.204,34.186)
NAVI,GRID resulting C (-212.204,34.186)
NAVI,GRID to visit: (-200,-200)
NAVI,GRID then distance = 100
NAVI: Set angle = 0 deg Set Distance = 100mm
NAVI: Run python

MC:TURN:  0.0  deg
MC: FORWARD_Thread
TIME =  0.0014526844024658203 s
LEFT SPEED =  34.419038240603975  rotations/s, ROTATIONS =  0.05
RIGHT SPEED =  0.0  rotations/s, ROTATIONS =  0.0
MC GO FORWARD FOR:  100.0
MC: FORWARD_Thread
TIME =  0.5230069160461426 s
LEFT SPEED =  0.9560103024639299  rotations/s, ROTATIONS =  0.5
RIGHT SPEED =  0.0  rotations/s, ROTATIONS =  0.0
MC: Angle turned =  4.105263157894738
MC: distance moved =  102.10176124166827

ret/cpp = 0

Main_end: ekf.w = 0.0716504 ekf.distance = 102.102
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
NAVI,GRID: Wheels = (-121.895,92.8906),(-142.174,-47.6539)
NAVI,GRID: tri_rot = (0,0),(90.3093,58.7046),(70.0302,-81.8399)
NAVI,GRID: tri_shift = (-212.204,34.186),(-121.895,92.8906),(-142.174,-47.6539)
EKF: distance = 102.102mm   |  Angle4.10526
EKF: d_x: -99.7521
EKF: d_y: 21.7783
EKF: d_theta: 4.10526

 MAIN: PREDICTED POSITION: x=-315.927, y=62.5801, w=12.3158 deg
Fit Cartesian for: (-315.927,62.5801) | 12.3158
Could be: 167.684

Number of CAR points8233ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 3
No of New Points = 1

IN STORE STATE POINTS


GRID: In grid data process
GRID:SAVED TO CSV 


IN update Movement Grid
A
NAVI,GRID: deltaX = -84.0727 = 400 - -315.927
NAVI,GRID: deltaY = -262.58 = 200 - 62.5801
NAVI,GRID angle = 0 = -107.754 - 12.3158

NAVI,GRID: Wheels = (-221.647,114.669),(-251.936,-24.0633)
NAVI,GRID: tri_rot = (0,0),(94.2802,52.0888),(63.9917,-86.6433)
NAVI,GRID: tri_shift = (-315.927,62.5801),(-221.647,114.669),(-251.936,-24.0633)
NAVI,GRID: deltaX = -84.0727 = -400 - -315.927
NAVI,GRID: deltaY = -262.58 = -200 - 62.5801
NAVI,GRID: dist1 = 76016.5 = 7068.22 - 68948.3
NAVI,GRID rotate angle = 0
NAVI,GRID around A = (-251.936,-24.0633)
NAVI,GRID from B = (-315.927,62.5801)
NAVI,GRID resulting C (-315.927,62.5801)
NAVI,GRID to visit: (-400,-200)
NAVI,GRID then distance = 100
NAVI: Set angle = 0 deg Set Distance = 100mm
NAVI: Run python

MC:TURN:  0.0  deg
MC: FORWARD_Thread
TIME =  0.0009644031524658203 s
LEFT SPEED =  51.84553770086527  rotations/s, ROTATIONS =  0.05
RIGHT SPEED =  0.0  rotations/s, ROTATIONS =  0.0
MC GO FORWARD FOR:  100.0
MC: FORWARD_Thread
TIME =  0.4723012447357178 s
LEFT SPEED =  1.0586463736291472  rotations/s, ROTATIONS =  0.5
RIGHT SPEED =  0.0  rotations/s, ROTATIONS =  0.0
MC: Angle turned =  4.105263157894738
MC: distance moved =  102.10176124166827

ret/cpp = 0

Main_end: ekf.w = 0.0716504 ekf.distance = 102.102
LEAVNG RUN

 i = FINALRUN
------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------


Attempt 0
SLAMTEC LIDAR S/N: ABCF99F6C9E59AD4C5E59CF7571D3414SLAMTEC Lidar health status : 0
Entering scan loop
I have reached max NoPoints in Lidar_functioncount = 1
IN triangFunction 
NAVI,GRID: Wheels = (-221.647,114.669),(-251.936,-24.0633)
NAVI,GRID: tri_rot = (0,0),(94.2802,52.0888),(63.9917,-86.6433)
NAVI,GRID: tri_shift = (-315.927,62.5801),(-221.647,114.669),(-251.936,-24.0633)
EKF: distance = 102.102mm   |  Angle4.10526
EKF: d_x: -97.937
EKF: d_y: 28.8635
EKF: d_theta: 4.10526

 MAIN: PREDICTED POSITION: x=-417.351, y=98.3267, w=16.4211 deg
Fit Cartesian for: (-417.351,98.3267) | 16.4211
Could be: 163.579

Number of CAR points8259ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 3
No of New Points = 1

IN STORE STATE POINTS


GRID: In grid data process
GRID:SAVED TO CSV 



Main_end: ekf.w = 0.0716504 ekf.distance = 102.102
LEAVNG RUN
