INIT_sigma_r = 100
INIT_sigma_theta = 0.1
INIT_sigma_odo_x = 0.01
INIT_sigma_odo_y = 0.01
INIT_sigma_odo_theta = 0.017
MCAL: IN motor calibration
TIME =  0.5311963558197021 s
LEFT SPEED =  0.847144365863719  rotations/s, ROTATIONS =  0.45 , TICKS =  9
RIGHT SPEED =  0.9412715176263544  rotations/s, ROTATIONS =  0.5 , TICKS =  10
TIME =  0.4973134994506836 s
LEFT SPEED =  1.0054020261912935  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.9048618235721642  rotations/s, ROTATIONS =  0.45 , TICKS =  9
TIME =  0.5028233528137207 s
LEFT SPEED =  0.9943850006211492  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.8949465005590344  rotations/s, ROTATIONS =  0.45 , TICKS =  9
TIME =  0.5071561336517334 s
LEFT SPEED =  0.887300714988527  rotations/s, ROTATIONS =  0.45 , TICKS =  9
RIGHT SPEED =  0.9858896833205856  rotations/s, ROTATIONS =  0.5 , TICKS =  10
TIME =  1.5464377403259277 s
LEFT SPEED =  1.2609625005588763  rotations/s, ROTATIONS =  1.95 , TICKS =  39
RIGHT SPEED =  1.2932948723680782  rotations/s, ROTATIONS =  2.0 , TICKS =  40
TIME =  0.5407772064208984 s
LEFT SPEED =  0.8321356644787196  rotations/s, ROTATIONS =  0.45 , TICKS =  9
RIGHT SPEED =  0.9245951827541328  rotations/s, ROTATIONS =  0.5 , TICKS =  10
TIME =  0.5012462139129639 s
LEFT SPEED =  0.9975137689255839  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.8977623920330254  rotations/s, ROTATIONS =  0.45 , TICKS =  9
TIME =  0.5324933528900146 s
LEFT SPEED =  0.9389788572689919  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.8450809715420927  rotations/s, ROTATIONS =  0.45 , TICKS =  9
TIME =  0.5151605606079102 s
LEFT SPEED =  0.9705711932023289  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.873514073882096  rotations/s, ROTATIONS =  0.45 , TICKS =  9
TIME =  1.5212674140930176 s
LEFT SPEED =  1.3146932495049883  rotations/s, ROTATIONS =  2.0 , TICKS =  40
RIGHT SPEED =  1.1832239245544895  rotations/s, ROTATIONS =  1.8 , TICKS =  36
MCAL: time Left =  0.008766233766233766 s  0.0012337662337662328 s
MCAL: time Right =  0.009 s  0.001 s

 YOU HAVE 10s TO MOVE ME TO STARTING POSITION 

ret/cpp = 0

 i = 0
------------------------------------------------------------------------------------------------------------


Attempt 0
SLAMTEC LIDAR S/N: ABCF99F6C9E59AD4C5E59CF7571D3414SLAMTEC Lidar health status : 0
Entering scan loop
I have reached max NoPoints in Lidar_functioncount = 1

 MAIN: after_motion State: x=0, y=0, w=0 deg
Fit Cartesian for: (0,0) | 0

Number of CAR points8957ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 4
(760.321,821.006) | (-1036.3,1019.15) | (776.753,-695.419) | (-1014.9,-594.33) | 

 i = 0
ObsLM.x = 760.321ObsLM.y = 821.006
EstLM.x = 760.321EstLM.y = 821.006
z.r = 1118.99z.theta = 132.802
z_cap.r = 1118.99z_cap.theta = 132.802
(z-z_cap).r = 0(z-z_cap).theta = 0

 EKF: State2: x=0, y=0, w=0 deg
(760.321,821.006) | 

 i = 1
ObsLM.x = -1036.3ObsLM.y = 1019.15
EstLM.x = -1036.3EstLM.y = 1019.15
z.r = 1453.47z.theta = 44.5219
z_cap.r = 1453.47z_cap.theta = 44.5219
(z-z_cap).r = 0(z-z_cap).theta = 0

 EKF: State2: x=0, y=0, w=0 deg
(760.321,821.006) | (-1036.3,1019.15) | 

 i = 2
ObsLM.x = 776.753ObsLM.y = -695.419
EstLM.x = 776.753EstLM.y = -695.419
z.r = 1042.57z.theta = 221.838
z_cap.r = 1042.57z_cap.theta = 221.838
(z-z_cap).r = 0(z-z_cap).theta = 0

 EKF: State2: x=0, y=0, w=0 deg
(760.321,821.006) | (-1036.3,1019.15) | (776.753,-695.419) | 

 i = 3
ObsLM.x = -1014.9ObsLM.y = -594.33
EstLM.x = -1014.9EstLM.y = -594.33
z.r = 1176.12z.theta = 329.647
z_cap.r = 1176.12z_cap.theta = 329.647
(z-z_cap).r = 0(z-z_cap).theta = 0

 EKF: State2: x=0, y=0, w=0 deg
(760.321,821.006) | (-1036.3,1019.15) | (776.753,-695.419) | (-1014.9,-594.33) | 

 MAIN: after_ekf State: x=0, y=0, w=0 deg
(760.321,821.006) | (-1036.3,1019.15) | (776.753,-695.419) | (-1014.9,-594.33) | 

 CSV: Saving Full Map (overwriting old)
GRID: In grid data process
GRID:SAVED TO CSV
IN update Movement Grid
NAVI,GRID: tri_shift = (0,0),(81,71),(81,-71)
NAVI,GRID around A = (81,-71) from B = (0,0) resulting C (0,0) to visit: (200,-200)
NAVI: Set angle = 0 deg Set Distance = 100mm
MC:TURN:  0.0  deg
MC GO FORWARD FOR:  100.0
TIME =  0.6019847393035889 s
LEFT SPEED =  0.6644686715193867  rotations/s, ROTATIONS =  0.4 , TICKS =  8
RIGHT SPEED =  0.8305858393992334  rotations/s, ROTATIONS =  0.5 , TICKS =  10
MC: Angle turned =  0.0
MC: distance moved =  81.68140899333463
ret/cpp = 0
LEAVNG RUN

 i = 1
------------------------------------------------------------------------------------------------------------


Attempt 0
SLAMTEC LIDAR S/N: ABCF99F6C9E59AD4C5E59CF7571D3414SLAMTEC Lidar health status : 0
Entering scan loop
I have reached max NoPoints in Lidar_functioncount = 1

 MAIN: after_motion State: x=-81.6814, y=0, w=0 deg
Fit Cartesian for: (-81.6814,0) | 0

Number of CAR points8934ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 5
(775.879,824.24) | (812.856,-686.388) | (-1014.16,998.504) | (-976.039,-615.205) | (781.782,630.659) | 

 i = 0
ObsLM.x = 775.879ObsLM.y = 824.24
EstLM.x = 760.321EstLM.y = 821.006
z.r = 1189.45z.theta = 136.135
z_cap.r = 1176.02z_cap.theta = 135.723
(z-z_cap).r = 13.4279(z-z_cap).theta = 0.411669

 EKF: State2: x=-81.6814, y=-0.0544037, w=-0.0921532 deg
(760.321,821.007) | (-1036.3,1019.15) | (776.753,-695.419) | (-1014.9,-594.33) | 

 i = 1
ObsLM.x = 812.856ObsLM.y = -686.388
EstLM.x = 776.753EstLM.y = -695.419
z.r = 1127.5z.theta = 217.589
z_cap.r = 1104.74z_cap.theta = 219.101
(z-z_cap).r = 22.7616(z-z_cap).theta = -1.51156

 EKF: State2: x=-81.6814, y=0.101341, w=0.179646 deg
(760.321,821.007) | (-1036.3,1019.15) | (776.754,-695.422) | (-1014.9,-594.33) | 

 i = 2
ObsLM.x = -1014.16ObsLM.y = 998.504
EstLM.x = -1036.3EstLM.y = 1019.15
z.r = 1366.14z.theta = 46.7756
z_cap.r = 1396.33z_cap.theta = 46.69
(z-z_cap).r = -30.1973(z-z_cap).theta = 0.0856369

 EKF: State2: x=-81.6814, y=0.107353, w=0.172766 deg
(760.321,821.007) | (-1036.3,1019.14) | (776.754,-695.422) | (-1014.9,-594.33) | 

 i = 3
ObsLM.x = -976.039ObsLM.y = -615.205
EstLM.x = -1014.9EstLM.y = -594.33
z.r = 1085.58z.theta = 325.3
z_cap.r = 1106.46z_cap.theta = 327.331
(z-z_cap).r = -20.8792(z-z_cap).theta = -2.0316

 EKF: State2: x=-81.6814, y=0.226719, w=0.428136 deg
(760.321,821.008) | (-1036.3,1019.14) | (776.754,-695.421) | (-1014.9,-594.327) | 

 i = 4
ObsLM.x = 781.782ObsLM.y = 630.659
EstLM.x = 781.782EstLM.y = 630.659
z.r = 1069.12z.theta = 143.438
z_cap.r = 1069.12z_cap.theta = 143.438
(z-z_cap).r = 0(z-z_cap).theta = 0

 EKF: State2: x=-81.6814, y=0.226719, w=0.428136 deg
(760.321,821.008) | (-1036.3,1019.14) | (776.754,-695.421) | (-1014.9,-594.327) | (781.782,630.659) | 

 MAIN: after_ekf State: x=-81.6814, y=0.226719, w=0.428136 deg
(760.321,821.008) | (-1036.3,1019.14) | (776.754,-695.421) | (-1014.9,-594.327) | (781.782,630.659) | 
No of New Points = 1
GRID: In grid data process
GRID:SAVED TO CSV
IN update Movement Grid
NAVI,GRID: tri_shift = (-81.6814,0.226719),(-0.153139,70.6195),(-1.21421,-71.3766)
NAVI,GRID around A = (-1.21421,-71.3766) from B = (-81.6814,0.226719) resulting C (-81.6814,0.226719) to visit: (-200,-200)
NAVI: Set angle = 0 deg Set Distance = 100mm
MC:TURN:  0.0  deg
MC GO FORWARD FOR:  100.0
TIME =  0.4776155948638916 s
LEFT SPEED =  1.0468669896394136  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.8374935917115309  rotations/s, ROTATIONS =  0.4 , TICKS =  8
MC: Angle turned =  0.0
MC: distance moved =  102.10176124166827
ret/cpp = 0
LEAVNG RUN

 i = 2
------------------------------------------------------------------------------------------------------------


Attempt 0
SLAMTEC LIDAR S/N: ABCF99F6C9E59AD4C5E59CF7571D3414SLAMTEC Lidar health status : 0
Entering scan loop
I have reached max NoPoints in Lidar_functioncount = 1

 MAIN: after_motion State: x=-183.78, y=0.989655, w=0.428136 deg
Fit Cartesian for: (-183.78,0.989655) | 0.428136

Number of CAR points9002ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 4
(773.332,826.634) | (822.562,-682.219) | (-966.761,-624.816) | (-1015.33,992.856) | 

 i = 0
ObsLM.x = 773.332ObsLM.y = 826.634
EstLM.x = 760.321EstLM.y = 821.008
z.r = 1264.02z.theta = 138.789
z_cap.r = 1250.5z_cap.theta = 138.595
(z-z_cap).r = 13.5194(z-z_cap).theta = 0.19421

 EKF: State2: x=-183.78, y=0.927092, w=0.378674 deg
(760.321,821.009) | (-1036.3,1019.14) | (776.754,-695.421) | (-1014.9,-594.327) | (781.782,630.659) | 

 i = 1
ObsLM.x = 822.562ObsLM.y = -682.219
EstLM.x = 776.754EstLM.y = -695.421
z.r = 1216.31z.theta = 213.792
z_cap.r = 1186.39z_cap.theta = 215.562
(z-z_cap).r = 29.9192(z-z_cap).theta = -1.77033

 EKF: State2: x=-183.782, y=1.2857, w=0.711735 deg
(760.321,821.009) | (-1036.3,1019.14) | (776.755,-695.424) | (-1014.9,-594.327) | (781.782,630.659) | 

 i = 2
ObsLM.x = -966.761ObsLM.y = -624.816
EstLM.x = -1014.9EstLM.y = -594.327
z.r = 1002.53z.theta = 320.641
z_cap.r = 1022.5z_cap.theta = 323.661
(z-z_cap).r = -19.9755(z-z_cap).theta = -3.02036

 EKF: State2: x=-183.784, y=1.59537, w=1.14673 deg
(760.32,821.01) | (-1036.3,1019.14) | (776.756,-695.424) | (-1014.9,-594.322) | (781.782,630.659) | 

 i = 3
ObsLM.x = -1015.33ObsLM.y = 992.856
EstLM.x = -1036.3EstLM.y = 1019.14
z.r = 1293.86z.theta = 48.8607
z_cap.r = 1327.47z_cap.theta = 48.8966
(z-z_cap).r = -33.6158(z-z_cap).theta = -0.0358448

 EKF: State2: x=-183.784, y=1.65038, w=1.16179 deg
(760.32,821.01) | (-1036.3,1019.14) | (776.756,-695.424) | (-1014.9,-594.323) | (781.782,630.659) | 

 MAIN: after_ekf State: x=-183.784, y=1.65038, w=1.16179 deg
(760.32,821.01) | (-1036.3,1019.14) | (776.756,-695.424) | (-1014.9,-594.323) | (781.782,630.659) | 
No of New Points = 0
GRID: In grid data process
GRID:SAVED TO CSV
IN update Movement Grid
NAVI,GRID: tri_shift = (-183.784,1.65038),(-101.361,70.9935),(-104.24,-70.9774)
NAVI,GRID around A = (-104.24,-70.9774) from B = (-183.784,1.65038) resulting C (-183.784,1.65038) to visit: (-200,-200)
NAVI: Set angle = 0 deg Set Distance = 100mm
MC:TURN:  0.0  deg
MC GO FORWARD FOR:  100.0
TIME =  0.544884443283081 s
LEFT SPEED =  0.8258631817209247  rotations/s, ROTATIONS =  0.45 , TICKS =  9
RIGHT SPEED =  0.917625757467694  rotations/s, ROTATIONS =  0.5 , TICKS =  10
MC: Angle turned =  0.0
MC: distance moved =  91.89158511750145
ret/cpp = 0
LEAVNG RUN

 i = FINALRUN 0
------------------------------------------------------------------------------------------------------------


Attempt 0
SLAMTEC LIDAR S/N: ABCF99F6C9E59AD4C5E59CF7571D3414SLAMTEC Lidar health status : 0
Entering scan loop
I have reached max NoPoints in Lidar_functioncount = 1

 MAIN: after_motion State: x=-275.657, y=3.51355, w=1.16179 deg
Fit Cartesian for: (-275.657,3.51355) | 1.16179

Number of CAR points9064ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 5
(774.872,858.482) | (-1044.04,963.124) | (-915.977,-652.415) | (874.13,-666.441) | (790.387,662.845) | 

 i = 0
ObsLM.x = 774.872ObsLM.y = 858.482
EstLM.x = 760.32EstLM.y = 821.01
z.r = 1354.47z.theta = 139.698
z_cap.r = 1319.68z_cap.theta = 140.561
(z-z_cap).r = 34.7899(z-z_cap).theta = -0.863049

 EKF: State2: x=-275.66, y=3.67568, w=1.34913 deg
(760.323,821.012) | (-1036.3,1019.14) | (776.756,-695.423) | (-1014.9,-594.323) | (781.782,630.659) | 

 i = 1
ObsLM.x = -1044.04ObsLM.y = 963.124
EstLM.x = -1036.3EstLM.y = 1019.14
z.r = 1229.21z.theta = 49.9612
z_cap.r = 1268.76z_cap.theta = 51.8157
(z-z_cap).r = -39.5493(z-z_cap).theta = -1.85452

 EKF: State2: x=-275.666, y=4.12262, w=1.70884 deg
(760.323,821.012) | (-1036.29,1019.14) | (776.756,-695.423) | (-1014.9,-594.323) | (781.782,630.659) | 

 i = 2
ObsLM.x = -915.977ObsLM.y = -652.415
EstLM.x = -915.977EstLM.y = -652.415
z.r = 917.082z.theta = 312.574
z_cap.r = 917.082z_cap.theta = 312.574
(z-z_cap).r = 0(z-z_cap).theta = 0

 EKF: State2: x=-275.666, y=4.12262, w=1.70884 deg
(760.323,821.012) | (-1036.29,1019.14) | (776.756,-695.423) | (-1014.9,-594.323) | (781.782,630.659) | (-915.977,-652.415) | 

 i = 3
ObsLM.x = 874.13ObsLM.y = -666.441
EstLM.x = 874.13EstLM.y = -666.441
z.r = 1331.05z.theta = 208.542
z_cap.r = 1331.05z_cap.theta = 208.542
(z-z_cap).r = 0(z-z_cap).theta = 0

 EKF: State2: x=-275.666, y=4.12262, w=1.70884 deg
(760.323,821.012) | (-1036.29,1019.14) | (776.756,-695.423) | (-1014.9,-594.323) | (781.782,630.659) | (-915.977,-652.415) | (874.13,-666.441) | 

 i = 4
ObsLM.x = 790.387ObsLM.y = 662.845
EstLM.x = 781.782EstLM.y = 630.659
z.r = 1253.15z.theta = 146.579
z_cap.r = 1229.12z_cap.theta = 147.644
(z-z_cap).r = 24.027(z-z_cap).theta = -1.06552

 EKF: State2: x=-275.668, y=4.26012, w=1.83608 deg
(760.323,821.013) | (-1036.29,1019.14) | (776.756,-695.423) | (-1014.9,-594.323) | (781.784,630.659) | (-915.977,-652.415) | (874.13,-666.44) | 

 MAIN: after_ekf State: x=-275.668, y=4.26012, w=1.83608 deg
(760.323,821.013) | (-1036.29,1019.14) | (776.756,-695.423) | (-1014.9,-594.323) | (781.784,630.659) | (-915.977,-652.415) | (874.13,-666.44) | 
No of New Points = 1
GRID: In grid data process
GRID:SAVED TO CSV
LEAVNG RUN

 i = FINALRUN 1
------------------------------------------------------------------------------------------------------------


Attempt 0
SLAMTEC LIDAR S/N: ABCF99F6C9E59AD4C5E59CF7571D3414SLAMTEC Lidar health status : 0
Entering scan loop
I have reached max NoPoints in Lidar_functioncount = 1

 MAIN: after_motion State: x=-275.668, y=4.26012, w=1.83608 deg
Fit Cartesian for: (-275.668,4.26012) | 1.83608

Number of CAR points9084ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 5
(784.698,845.913) | (-923.877,-643.967) | (864.547,-677.462) | (-1010.68,972.823) | (797.728,647.952) | 

 i = 0
ObsLM.x = 784.698ObsLM.y = 845.913
EstLM.x = 760.323EstLM.y = 821.013
z.r = 1353.79z.theta = 139.724
z_cap.r = 1319.23z_cap.theta = 139.912
(z-z_cap).r = 34.5656(z-z_cap).theta = -0.188896

 EKF: State2: x=-275.667, y=4.19825, w=1.86347 deg
(760.326,821.015) | (-1036.29,1019.14) | (776.756,-695.423) | (-1014.9,-594.323) | (781.784,630.659) | (-915.977,-652.415) | (874.13,-666.44) | 

 i = 1
ObsLM.x = -923.877ObsLM.y = -643.967
EstLM.x = -915.977EstLM.y = -652.415
z.r = 916.675z.theta = 313.139
z_cap.r = 917.135z_cap.theta = 312.416
(z-z_cap).r = -0.460083(z-z_cap).theta = 0.722252

 EKF: State2: x=-275.665, y=4.10367, w=1.73714 deg
(760.326,821.014) | (-1036.29,1019.14) | (776.756,-695.423) | (-1014.9,-594.323) | (781.784,630.659) | (-915.976,-652.416) | (874.13,-666.441) | 

 i = 2
ObsLM.x = 864.547ObsLM.y = -677.462
EstLM.x = 874.13EstLM.y = -666.441
z.r = 1328.39z.theta = 209.132
z_cap.r = 1331.04z_cap.theta = 208.513
(z-z_cap).r = -2.64832(z-z_cap).theta = 0.618897

 EKF: State2: x=-275.663, y=3.96116, w=1.63797 deg
(760.326,821.014) | (-1036.29,1019.14) | (776.756,-695.423) | (-1014.9,-594.323) | (781.784,630.659) | (-915.976,-652.416) | (874.13,-666.44) | 

 i = 3
ObsLM.x = -1010.68ObsLM.y = 972.823
EstLM.x = -1036.29EstLM.y = 1019.14
z.r = 1216.12z.theta = 51.1765
z_cap.r = 1268.52z_cap.theta = 51.5194
(z-z_cap).r = -52.4(z-z_cap).theta = -0.342875

 EKF: State2: x=-275.667, y=4.21578, w=1.70263 deg
(760.326,821.014) | (-1036.29,1019.14) | (776.756,-695.423) | (-1014.9,-594.323) | (781.784,630.659) | (-915.976,-652.416) | (874.13,-666.44) | 

 i = 4
ObsLM.x = 797.728ObsLM.y = 647.952
EstLM.x = 781.784EstLM.y = 630.659
z.r = 1251.63z.theta = 147.345
z_cap.r = 1229.08z_cap.theta = 147.654
(z-z_cap).r = 22.5493(z-z_cap).theta = -0.309052

 EKF: State2: x=-275.667, y=4.21482, w=1.73588 deg
(760.326,821.015) | (-1036.29,1019.14) | (776.756,-695.423) | (-1014.9,-594.323) | (781.787,630.66) | (-915.976,-652.416) | (874.13,-666.44) | 

 MAIN: after_ekf State: x=-275.667, y=4.21482, w=1.73588 deg
(760.326,821.015) | (-1036.29,1019.14) | (776.756,-695.423) | (-1014.9,-594.323) | (781.787,630.66) | (-915.976,-652.416) | (874.13,-666.44) | 
No of New Points = 0
GRID: In grid data process
GRID:SAVED TO CSV
LEAVNG RUN

 i = FINALRUN 2
------------------------------------------------------------------------------------------------------------


Attempt 0
SLAMTEC LIDAR S/N: ABCF99F6C9E59AD4C5E59CF7571D3414SLAMTEC Lidar health status : 0
Entering scan loop
I have reached max NoPoints in Lidar_functioncount = 1

 MAIN: after_motion State: x=-275.667, y=4.21482, w=1.73588 deg
Fit Cartesian for: (-275.667,4.21482) | 1.73588

Number of CAR points9079ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 5
(782.912,847.566) | (-923.084,-644.988) | (867.516,-673.479) | (-1007.26,971.047) | (796.954,648.313) | 

 i = 0
ObsLM.x = 782.912ObsLM.y = 847.566
EstLM.x = 760.326EstLM.y = 821.015
z.r = 1353.45z.theta = 139.72
z_cap.r = 1319.26z_cap.theta = 140.011
(z-z_cap).r = 34.1924(z-z_cap).theta = -0.290693

 EKF: State2: x=-275.666, y=4.15597, w=1.78701 deg
(760.329,821.016) | (-1036.29,1019.14) | (776.756,-695.423) | (-1014.9,-594.323) | (781.787,630.66) | (-915.976,-652.416) | (874.13,-666.44) | 

 i = 1
ObsLM.x = -923.084ObsLM.y = -644.988
EstLM.x = -915.976EstLM.y = -652.416
z.r = 916.808z.theta = 313.137
z_cap.r = 917.106z_cap.theta = 312.495
(z-z_cap).r = -0.297485(z-z_cap).theta = 0.64212

 EKF: State2: x=-275.665, y=4.08793, w=1.67516 deg
(760.329,821.016) | (-1036.29,1019.14) | (776.756,-695.423) | (-1014.9,-594.323) | (781.787,630.66) | (-915.975,-652.417) | (874.13,-666.44) | 

 i = 2
ObsLM.x = 867.516ObsLM.y = -673.479
EstLM.x = 874.13EstLM.y = -666.44
z.r = 1328.89z.theta = 208.98
z_cap.r = 1331.03z_cap.theta = 208.574
(z-z_cap).r = -2.13367(z-z_cap).theta = 0.40585

 EKF: State2: x=-275.663, y=3.9885, w=1.61037 deg
(760.329,821.016) | (-1036.29,1019.14) | (776.756,-695.423) | (-1014.9,-594.323) | (781.787,630.66) | (-915.975,-652.417) | (874.13,-666.439) | 

 i = 3
ObsLM.x = -1007.26ObsLM.y = 971.047
EstLM.x = -1036.29EstLM.y = 1019.14
z.r = 1212.61z.theta = 51.2815
z_cap.r = 1268.49z_cap.theta = 51.5462
(z-z_cap).r = -55.8785(z-z_cap).theta = -0.264724

 EKF: State2: x=-275.668, y=4.30827, w=1.66543 deg
(760.329,821.016) | (-1036.29,1019.13) | (776.756,-695.423) | (-1014.9,-594.323) | (781.787,630.66) | (-915.975,-652.417) | (874.13,-666.439) | 

 i = 4
ObsLM.x = 796.954ObsLM.y = 648.313
EstLM.x = 781.787EstLM.y = 630.66
z.r = 1251.1z.theta = 147.354
z_cap.r = 1229.03z_cap.theta = 147.695
(z-z_cap).r = 22.069(z-z_cap).theta = -0.341564

 EKF: State2: x=-275.668, y=4.30323, w=1.70277 deg
(760.329,821.016) | (-1036.29,1019.13) | (776.756,-695.423) | (-1014.9,-594.323) | (781.789,630.661) | (-915.975,-652.417) | (874.13,-666.439) | 

 MAIN: after_ekf State: x=-275.668, y=4.30323, w=1.70277 deg
(760.329,821.016) | (-1036.29,1019.13) | (776.756,-695.423) | (-1014.9,-594.323) | (781.789,630.661) | (-915.975,-652.417) | (874.13,-666.439) | 
No of New Points = 0
GRID: In grid data process
GRID:SAVED TO CSV
LEAVNG RUN

 i = FINALRUN 3
------------------------------------------------------------------------------------------------------------


Attempt 0
SLAMTEC LIDAR S/N: ABCF99F6C9E59AD4C5E59CF7571D3414SLAMTEC Lidar health status : 0
Entering scan loop
I have reached max NoPoints in Lidar_functioncount = 1

 MAIN: after_motion State: x=-275.668, y=4.30323, w=1.70277 deg
Fit Cartesian for: (-275.668,4.30323) | 1.70277

Number of CAR points9077ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 5
(782.401,848.561) | (-922.458,-646.397) | (866.892,-677.405) | (-1011.84,969.654) | (796.059,651.125) | 

 i = 0
ObsLM.x = 782.401ObsLM.y = 848.561
EstLM.x = 760.329EstLM.y = 821.016
z.r = 1353.62z.theta = 139.71
z_cap.r = 1319.21z_cap.theta = 140.047
(z-z_cap).r = 34.4098(z-z_cap).theta = -0.337193

 EKF: State2: x=-275.667, y=4.23236, w=1.76481 deg
(760.332,821.018) | (-1036.29,1019.13) | (776.756,-695.423) | (-1014.9,-594.323) | (781.789,630.661) | (-915.975,-652.417) | (874.13,-666.439) | 

 i = 1
ObsLM.x = -922.458ObsLM.y = -646.397
EstLM.x = -915.975EstLM.y = -652.417
z.r = 917.419z.theta = 313.066
z_cap.r = 917.16z_cap.theta = 312.513
(z-z_cap).r = 0.259033(z-z_cap).theta = 0.552344

 EKF: State2: x=-275.666, y=4.18832, w=1.6688 deg
(760.332,821.018) | (-1036.29,1019.13) | (776.756,-695.423) | (-1014.9,-594.323) | (781.789,630.661) | (-915.975,-652.417) | (874.13,-666.439) | 

 i = 2
ObsLM.x = 866.892ObsLM.y = -677.405
EstLM.x = 874.13EstLM.y = -666.439
z.r = 1330.42z.theta = 209.149
z_cap.r = 1331.08z_cap.theta = 208.584
(z-z_cap).r = -0.66333(z-z_cap).theta = 0.565007

 EKF: State2: x=-275.663, y=4.05253, w=1.57936 deg
(760.332,821.018) | (-1036.29,1019.13) | (776.756,-695.423) | (-1014.9,-594.323) | (781.789,630.661) | (-915.975,-652.417) | (874.131,-666.439) | 

 i = 3
ObsLM.x = -1011.84ObsLM.y = 969.654
EstLM.x = -1036.29EstLM.y = 1019.13
z.r = 1214.22z.theta = 51.0988
z_cap.r = 1268.44z_cap.theta = 51.5755
(z-z_cap).r = -54.2135(z-z_cap).theta = -0.476706

 EKF: State2: x=-275.67, y=4.43049, w=1.66079 deg
(760.332,821.018) | (-1036.28,1019.13) | (776.756,-695.423) | (-1014.9,-594.323) | (781.789,630.661) | (-915.974,-652.417) | (874.131,-666.438) | 

 i = 4
ObsLM.x = 796.059ObsLM.y = 651.125
EstLM.x = 781.789EstLM.y = 630.661
z.r = 1251.73z.theta = 147.232
z_cap.r = 1228.98z_cap.theta = 147.705
(z-z_cap).r = 22.7489(z-z_cap).theta = -0.473114

 EKF: State2: x=-275.67, y=4.43952, w=1.71403 deg
(760.332,821.018) | (-1036.28,1019.13) | (776.756,-695.423) | (-1014.9,-594.323) | (781.791,630.662) | (-915.974,-652.417) | (874.131,-666.438) | 

 MAIN: after_ekf State: x=-275.67, y=4.43952, w=1.71403 deg
(760.332,821.018) | (-1036.28,1019.13) | (776.756,-695.423) | (-1014.9,-594.323) | (781.791,630.662) | (-915.974,-652.417) | (874.131,-666.438) | 
No of New Points = 0
GRID: In grid data process
GRID:SAVED TO CSV
LEAVNG RUN
