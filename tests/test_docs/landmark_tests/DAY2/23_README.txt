INIT_sigma_r = 10
INIT_sigma_theta = 0.04
INIT_sigma_odo_x = 0.01
INIT_sigma_odo_y = 0.01
INIT_sigma_odo_theta = 0.017
MCAL: IN motor calibration
TIME =  0.5526306629180908 s
LEFT SPEED =  0.8142870640290505  rotations/s, ROTATIONS =  0.45 , TICKS =  9
RIGHT SPEED =  0.9047634044767227  rotations/s, ROTATIONS =  0.5 , TICKS =  10
TIME =  0.4717085361480713 s
LEFT SPEED =  1.0599765780855996  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.9539789202770397  rotations/s, ROTATIONS =  0.45 , TICKS =  9
TIME =  0.5116236209869385 s
LEFT SPEED =  0.9772809141131598  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.8795528227018438  rotations/s, ROTATIONS =  0.45 , TICKS =  9
TIME =  0.4553658962249756 s
LEFT SPEED =  0.8784144867150486  rotations/s, ROTATIONS =  0.4 , TICKS =  8
RIGHT SPEED =  1.0980181083938108  rotations/s, ROTATIONS =  0.5 , TICKS =  10
TIME =  1.464888572692871 s
LEFT SPEED =  1.3652915568338728  rotations/s, ROTATIONS =  2.0 , TICKS =  40
RIGHT SPEED =  1.2628946900713325  rotations/s, ROTATIONS =  1.85 , TICKS =  37
TIME =  0.6104927062988281 s
LEFT SPEED =  0.8190106037978717  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.7371095434180845  rotations/s, ROTATIONS =  0.45 , TICKS =  9
TIME =  0.5473945140838623 s
LEFT SPEED =  0.7307343966891107  rotations/s, ROTATIONS =  0.4 , TICKS =  8
RIGHT SPEED =  0.9134179958613883  rotations/s, ROTATIONS =  0.5 , TICKS =  10
TIME =  0.5131633281707764 s
LEFT SPEED =  0.9743486577310612  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.876913791957955  rotations/s, ROTATIONS =  0.45 , TICKS =  9
TIME =  0.46332764625549316 s
LEFT SPEED =  1.0791499364237906  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.9712349427814115  rotations/s, ROTATIONS =  0.45 , TICKS =  9
TIME =  1.5007460117340088 s
LEFT SPEED =  1.3326705414256856  rotations/s, ROTATIONS =  2.0 , TICKS =  40
RIGHT SPEED =  1.1994034872831172  rotations/s, ROTATIONS =  1.8 , TICKS =  36
MCAL: time Left =  0.009 s  0.001 s
MCAL: time Right =  0.008999999999999998 s  0.0010000000000000009 s

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

Number of CAR points8943ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 4
(775.78,806.177) | (-1007.86,1040.04) | (756.303,-714.976) | (-1032.84,-569.973) | 

 i = 0
ObsLM.x = 775.78ObsLM.y = 806.177
EstLM.x = 775.78EstLM.y = 806.177
z.r = 1118.82z.theta = 133.899
z_cap.r = 1118.82z_cap.theta = 133.899
(z-z_cap).r = 0(z-z_cap).theta = 0

 EKF: State2: x=0, y=0, w=0 deg
(775.78,806.177) | 

 i = 1
ObsLM.x = -1007.86ObsLM.y = 1040.04
EstLM.x = -1007.86EstLM.y = 1040.04
z.r = 1448.27z.theta = 45.9002
z_cap.r = 1448.27z_cap.theta = 45.9002
(z-z_cap).r = 0(z-z_cap).theta = 0

 EKF: State2: x=0, y=0, w=0 deg
(775.78,806.177) | (-1007.86,1040.04) | 

 i = 2
ObsLM.x = 756.303ObsLM.y = -714.976
EstLM.x = 756.303EstLM.y = -714.976
z.r = 1040.76z.theta = 223.391
z_cap.r = 1040.76z_cap.theta = 223.391
(z-z_cap).r = 0(z-z_cap).theta = 0

 EKF: State2: x=0, y=0, w=0 deg
(775.78,806.177) | (-1007.86,1040.04) | (756.303,-714.976) | 

 i = 3
ObsLM.x = -1032.84ObsLM.y = -569.973
EstLM.x = -1032.84EstLM.y = -569.973
z.r = 1179.67z.theta = 331.108
z_cap.r = 1179.67z_cap.theta = 331.108
(z-z_cap).r = 0(z-z_cap).theta = 0

 EKF: State2: x=0, y=0, w=0 deg
(775.78,806.177) | (-1007.86,1040.04) | (756.303,-714.976) | (-1032.84,-569.973) | 

 MAIN: after_ekf State: x=0, y=0, w=0 deg
(775.78,806.177) | (-1007.86,1040.04) | (756.303,-714.976) | (-1032.84,-569.973) | 

 CSV: Saving Full Map (overwriting old)
GRID: In grid data process
GRID:SAVED TO CSV
IN update Movement Grid
NAVI,GRID: tri_shift = (0,0),(81,71),(81,-71)
NAVI,GRID around A = (81,-71) from B = (0,0) resulting C (0,0) to visit: (-200,0)
NAVI: Set angle = 0 deg Set Distance = 100mm
MC:TURN:  0.0  deg
MC GO FORWARD FOR:  100.0
TIME =  0.49645161628723145 s
LEFT SPEED =  1.0071474915104628  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.9064327423594166  rotations/s, ROTATIONS =  0.45 , TICKS =  9
MC: Angle turned =  0.0
MC: distance moved =  102.10176124166827
ret/cpp = 0
LEAVNG RUN

 i = 1
------------------------------------------------------------------------------------------------------------


Attempt 0
SLAMTEC LIDAR S/N: ABCF99F6C9E59AD4C5E59CF7571D3414SLAMTEC Lidar health status : 0
Entering scan loop
I have reached max NoPoints in Lidar_functioncount = 1

 MAIN: after_motion State: x=-102.102, y=0, w=0 deg
Fit Cartesian for: (-102.102,0) | 0

Number of CAR points8990ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 3
(754.101,826.397) | (794.517,-686.65) | (-1034.81,998.773) | 

 i = 0
ObsLM.x = 754.101ObsLM.y = 826.397
EstLM.x = 775.78EstLM.y = 806.177
z.r = 1189.96z.theta = 136.015
z_cap.r = 1191.89z_cap.theta = 137.438
(z-z_cap).r = -1.92383(z-z_cap).theta = -1.42323

 EKF: State2: x=-102.102, y=0.291697, w=0.536169 deg
(775.772,806.161) | (-1007.87,1040.04) | (756.304,-714.974) | (-1032.84,-569.975) | 

 i = 1
ObsLM.x = 794.517ObsLM.y = -686.65
EstLM.x = 756.304EstLM.y = -714.974
z.r = 1129.52z.theta = 216.921
z_cap.r = 1117.35z_cap.theta = 219.267
(z-z_cap).r = 12.1716(z-z_cap).theta = -2.34528

 EKF: State2: x=-102.102, y=0.772112, w=1.21236 deg
(775.768,806.17) | (-1007.87,1040.04) | (756.385,-715.059) | (-1032.84,-569.977) | 

 i = 2
ObsLM.x = -1034.81ObsLM.y = 998.773
EstLM.x = -1007.87EstLM.y = 1040.04
z.r = 1366z.theta = 45.7246
z_cap.r = 1378.58z_cap.theta = 47.7142
(z-z_cap).r = -12.5881(z-z_cap).theta = -1.98955

 EKF: State2: x=-102.102, y=1.13475, w=1.6703 deg
(775.765,806.176) | (-1007.78,1039.96) | (756.388,-715.052) | (-1032.84,-569.978) | 

 MAIN: after_ekf State: x=-102.102, y=1.13475, w=1.6703 deg
(775.765,806.176) | (-1007.78,1039.96) | (756.388,-715.052) | (-1032.84,-569.978) | 
No of New Points = 0
GRID: In grid data process
GRID:SAVED TO CSV
IN update Movement Grid
NAVI,GRID: tri_shift = (-102.102,1.13475),(-19.0667,69.7436),(-23.2057,-72.1961)
NAVI,GRID around A = (-23.2057,-72.1961) from B = (-102.102,1.13475) resulting C (-102.102,1.13475) to visit: (-200,-200)
NAVI: Set angle = 0 deg Set Distance = 100mm
MC:TURN:  0.0  deg
MC GO FORWARD FOR:  100.0
TIME =  0.5367631912231445 s
LEFT SPEED =  0.7452075823018032  rotations/s, ROTATIONS =  0.4 , TICKS =  8
RIGHT SPEED =  0.931509477877254  rotations/s, ROTATIONS =  0.5 , TICKS =  10
MC: Angle turned =  0.0
MC: distance moved =  81.68140899333463
ret/cpp = 0
LEAVNG RUN

 i = 2
------------------------------------------------------------------------------------------------------------


Attempt 0
SLAMTEC LIDAR S/N: ABCF99F6C9E59AD4C5E59CF7571D3414SLAMTEC Lidar health status : 0
Entering scan loop
I have reached max NoPoints in Lidar_functioncount = 1

 MAIN: after_motion State: x=-183.749, y=3.51561, w=1.6703 deg
Fit Cartesian for: (-183.749,3.51561) | 1.6703

Number of CAR points9021ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 4
(855.605,752.455) | (767.135,-755.187) | (-937.75,1073.84) | (-1012.25,-541.034) | 

 i = 0
ObsLM.x = 855.605ObsLM.y = 752.455
EstLM.x = 775.765EstLM.y = 806.176
z.r = 1281.08z.theta = 142.554
z_cap.r = 1250.97z_cap.theta = 138.416
(z-z_cap).r = 30.1079(z-z_cap).theta = 4.13761

 EKF: State2: x=-183.712, y=1.51889, w=-0.152062 deg
(775.973,806.358) | (-1007.77,1039.96) | (756.383,-715.065) | (-1032.84,-569.975) | 

 i = 1
ObsLM.x = 767.135ObsLM.y = -755.187
EstLM.x = 756.383EstLM.y = -715.065
z.r = 1215.2z.theta = 218.666
z_cap.r = 1182.06z_cap.theta = 217.468
(z-z_cap).r = 33.1384(z-z_cap).theta = 1.19724

 EKF: State2: x=-183.721, y=2.25387, w=-0.326862 deg
(775.976,806.365) | (-1007.77,1039.96) | (756.643,-715.245) | (-1032.84,-569.977) | 

 i = 2
ObsLM.x = -937.75ObsLM.y = 1073.84
EstLM.x = -1007.77EstLM.y = 1039.96
z.r = 1310.29z.theta = 55.1945
z_cap.r = 1325.1z_cap.theta = 51.8734
(z-z_cap).r = -14.8124(z-z_cap).theta = 3.32104

 EKF: State2: x=-183.72, y=2.46448, w=-0.957939 deg
(775.981,806.364) | (-1007.7,1039.84) | (756.638,-715.246) | (-1032.84,-569.978) | 

 i = 3
ObsLM.x = -1012.25ObsLM.y = -541.034
EstLM.x = -1032.84EstLM.y = -569.978
z.r = 990.883z.theta = 327.694
z_cap.r = 1024.06z_cap.theta = 326.972
(z-z_cap).r = -33.1725(z-z_cap).theta = 0.722252

 EKF: State2: x=-183.708, y=1.62021, w=-1.17859 deg
(775.98,806.353) | (-1007.69,1039.83) | (756.639,-715.257) | (-1032.57,-569.801) | 

 MAIN: after_ekf State: x=-183.708, y=1.62021, w=-1.17859 deg
(775.98,806.353) | (-1007.69,1039.83) | (756.639,-715.257) | (-1032.57,-569.801) | 
No of New Points = 1
GRID: In grid data process
GRID:SAVED TO CSV
IN update Movement Grid
NAVI,GRID: tri_shift = (-183.708,1.62021),(-104.186,74.2713),(-101.265,-67.6987)
NAVI,GRID around A = (-101.265,-67.6987) from B = (-183.708,1.62021) resulting C (-183.708,1.62021) to visit: (-200,-200)
NAVI: Set angle = 0 deg Set Distance = 100mm
MC:TURN:  0.0  deg
MC GO FORWARD FOR:  100.0
TIME =  0.5660781860351562 s
LEFT SPEED =  0.7949431917732523  rotations/s, ROTATIONS =  0.45 , TICKS =  9
RIGHT SPEED =  0.8832702130813914  rotations/s, ROTATIONS =  0.5 , TICKS =  10
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

 MAIN: after_motion State: x=-275.581, y=-0.269901, w=-1.17859 deg
Fit Cartesian for: (-275.581,-0.269901) | -1.17859

Number of CAR points9066ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 5
(876.249,762.023) | (-917.687,1068.19) | (805.238,-761.274) | (870.825,556.26) | (-974.658,-548.39) | 

 i = 0
ObsLM.x = 876.249ObsLM.y = 762.023
EstLM.x = 876.249EstLM.y = 762.023
z.r = 1381.23z.theta = 147.682
z_cap.r = 1381.23z_cap.theta = 147.682
(z-z_cap).r = 0(z-z_cap).theta = 0

 EKF: State2: x=-275.581, y=-0.269901, w=-1.17859 deg
(775.98,806.353) | (-1007.69,1039.83) | (756.639,-715.257) | (-1032.57,-569.801) | (876.249,762.023) | 

 i = 1
ObsLM.x = -917.687ObsLM.y = 1068.19
EstLM.x = -1007.69EstLM.y = 1039.83
z.r = 1246.55z.theta = 60.1741
z_cap.r = 1271.93z_cap.theta = 56.0374
(z-z_cap).r = -25.374(z-z_cap).theta = 4.13668

 EKF: State2: x=-275.592, y=0.701959, w=-2.11183 deg
(775.982,806.363) | (-1007.59,1039.63) | (756.636,-715.247) | (-1032.56,-569.801) | (876.257,762.02) | 

 i = 2
ObsLM.x = 805.238ObsLM.y = -761.274
EstLM.x = 756.636EstLM.y = -715.247
z.r = 1322.42z.theta = 217.295
z_cap.r = 1256.22z_cap.theta = 216.857
(z-z_cap).r = 66.2062(z-z_cap).theta = 0.438525

 EKF: State2: x=-275.6, y=3.20682, w=-1.99081 deg
(775.984,806.385) | (-1007.62,1039.65) | (757.151,-715.574) | (-1032.55,-569.803) | (876.268,762.029) | 

 i = 3
ObsLM.x = 870.825ObsLM.y = 556.26
EstLM.x = 870.825EstLM.y = 556.26
z.r = 1272.85z.theta = 156.237
z_cap.r = 1272.85z_cap.theta = 156.237
(z-z_cap).r = 0(z-z_cap).theta = 0

 EKF: State2: x=-275.6, y=3.20682, w=-1.99081 deg
(775.984,806.385) | (-1007.62,1039.65) | (757.151,-715.574) | (-1032.55,-569.803) | (876.268,762.029) | (870.825,556.26) | 

 i = 4
ObsLM.x = -974.658ObsLM.y = -548.39
EstLM.x = -1032.55EstLM.y = -569.803
z.r = 890.473z.theta = 323.715
z_cap.r = 949.377z_cap.theta = 324.865
(z-z_cap).r = -58.9044(z-z_cap).theta = -1.14979

 EKF: State2: x=-275.593, y=0.933815, w=-1.96873 deg
(775.982,806.365) | (-1007.59,1039.63) | (757.163,-715.602) | (-1032.12,-569.454) | (876.257,762.022) | (870.817,556.255) | 

 MAIN: after_ekf State: x=-275.593, y=0.933815, w=-1.96873 deg
(775.982,806.365) | (-1007.59,1039.63) | (757.163,-715.602) | (-1032.12,-569.454) | (876.257,762.022) | (870.817,556.255) | 
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

 MAIN: after_motion State: x=-275.593, y=0.933815, w=-1.96873 deg
Fit Cartesian for: (-275.593,0.933815) | -1.96873

Number of CAR points9093ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 5
(861.794,577.218) | (-932,1060.14) | (814.281,-744.164) | (867.025,779.52) | (-966.417,-558.065) | 

 i = 0
ObsLM.x = 861.794ObsLM.y = 577.218
EstLM.x = 870.817EstLM.y = 556.255
z.r = 1275.05z.theta = 155.099
z_cap.r = 1273.83z_cap.theta = 156.123
(z-z_cap).r = 1.22241(z-z_cap).theta = -1.0246

 EKF: State2: x=-275.589, y=1.13066, w=-1.59924 deg
(775.982,806.365) | (-1007.6,1039.63) | (757.163,-715.601) | (-1032.12,-569.454) | (876.257,762.022) | (870.831,556.256) | 

 i = 1
ObsLM.x = -932ObsLM.y = 1060.14
EstLM.x = -1007.6EstLM.y = 1039.63
z.r = 1245.94z.theta = 59.8071
z_cap.r = 1270.56z_cap.theta = 56.4204
(z-z_cap).r = -24.6123(z-z_cap).theta = 3.38675

 EKF: State2: x=-275.592, y=2.53456, w=-2.35262 deg
(775.983,806.377) | (-1007.5,1039.44) | (757.157,-715.584) | (-1032.11,-569.451) | (876.263,762.027) | (870.843,556.257) | 

 i = 2
ObsLM.x = 814.281ObsLM.y = -744.164
EstLM.x = 757.157EstLM.y = -715.584
z.r = 1321.13z.theta = 216.769
z_cap.r = 1257.88z_cap.theta = 217.165
(z-z_cap).r = 63.2485(z-z_cap).theta = -0.396711

 EKF: State2: x=-275.582, y=5.49505, w=-2.11127 deg
(775.984,806.395) | (-1007.55,1039.48) | (757.631,-715.889) | (-1032.09,-569.449) | (876.271,762.037) | (870.86,556.273) | 

 i = 3
ObsLM.x = 867.025ObsLM.y = 779.52
EstLM.x = 876.271EstLM.y = 762.037
z.r = 1380.1z.theta = 147.997
z_cap.r = 1378.09z_cap.theta = 148.814
(z-z_cap).r = 2.01038(z-z_cap).theta = -0.817423

 EKF: State2: x=-275.581, y=5.50052, w=-1.97316 deg
(775.984,806.395) | (-1007.55,1039.48) | (757.632,-715.888) | (-1032.09,-569.449) | (876.29,762.044) | (870.859,556.275) | 

 i = 4
ObsLM.x = -966.417ObsLM.y = -558.065
EstLM.x = -1032.09EstLM.y = -569.449
z.r = 891.549z.theta = 322.767
z_cap.r = 950.195z_cap.theta = 324.738
(z-z_cap).r = -58.6458(z-z_cap).theta = -1.9716

 EKF: State2: x=-275.589, y=2.76746, w=-1.80464 deg
(775.983,806.377) | (-1007.51,1039.44) | (757.653,-715.924) | (-1031.69,-569.104) | (876.269,762.027) | (870.842,556.263) | 

 MAIN: after_ekf State: x=-275.589, y=2.76746, w=-1.80464 deg
(775.983,806.377) | (-1007.51,1039.44) | (757.653,-715.924) | (-1031.69,-569.104) | (876.269,762.027) | (870.842,556.263) | 
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

 MAIN: after_motion State: x=-275.589, y=2.76746, w=-1.80464 deg
Fit Cartesian for: (-275.589,2.76746) | -1.80464

Number of CAR points9096ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 5
(863.64,576.101) | (-929.373,1064.41) | (-972.735,-555.287) | (812.682,-746.142) | (870.942,778.342) | 

 i = 0
ObsLM.x = 863.64ObsLM.y = 576.101
EstLM.x = 870.842EstLM.y = 556.263
z.r = 1275.36z.theta = 155.09
z_cap.r = 1273.05z_cap.theta = 156.033
(z-z_cap).r = 2.31238(z-z_cap).theta = -0.943331

 EKF: State2: x=-275.586, y=2.88569, w=-1.46915 deg
(775.983,806.377) | (-1007.51,1039.44) | (757.653,-715.924) | (-1031.69,-569.105) | (876.269,762.027) | (870.864,556.269) | 

 i = 1
ObsLM.x = -929.373ObsLM.y = 1064.41
EstLM.x = -1007.51EstLM.y = 1039.44
z.r = 1246.7z.theta = 59.8403
z_cap.r = 1268.92z_cap.theta = 56.2429
(z-z_cap).r = -22.2198(z-z_cap).theta = 3.59748

 EKF: State2: x=-275.582, y=4.28915, w=-2.29955 deg
(775.983,806.386) | (-1007.43,1039.28) | (757.643,-715.905) | (-1031.67,-569.098) | (876.279,762.036) | (870.881,556.272) | 

 i = 2
ObsLM.x = -972.735ObsLM.y = -555.287
EstLM.x = -1031.67EstLM.y = -569.098
z.r = 893.951z.theta = 323.547
z_cap.r = 948.915z_cap.theta = 325.124
(z-z_cap).r = -54.9647(z-z_cap).theta = -1.57731

 EKF: State2: x=-275.604, y=1.33639, w=-2.16835 deg
(775.983,806.371) | (-1007.39,1039.24) | (757.658,-715.939) | (-1031.31,-568.787) | (876.264,762.018) | (870.857,556.252) | 

 i = 3
ObsLM.x = 812.682ObsLM.y = -746.142
EstLM.x = 757.658EstLM.y = -715.939
z.r = 1320.26z.theta = 216.651
z_cap.r = 1257.82z_cap.theta = 216.936
(z-z_cap).r = 62.4409(z-z_cap).theta = -0.284983

 EKF: State2: x=-275.582, y=4.3834, w=-1.98102 deg
(775.983,806.386) | (-1007.44,1039.28) | (758.115,-716.23) | (-1031.26,-568.77) | (876.279,762.037) | (870.88,556.275) | 

 i = 4
ObsLM.x = 870.942ObsLM.y = 778.342
EstLM.x = 876.279EstLM.y = 762.037
z.r = 1383.3z.theta = 147.96
z_cap.r = 1378.7z_cap.theta = 148.646
(z-z_cap).r = 4.60132(z-z_cap).theta = -0.685724

 EKF: State2: x=-275.582, y=4.24738, w=-1.88647 deg
(775.983,806.385) | (-1007.44,1039.28) | (758.117,-716.231) | (-1031.26,-568.772) | (876.317,762.056) | (870.878,556.275) | 

 MAIN: after_ekf State: x=-275.582, y=4.24738, w=-1.88647 deg
(775.983,806.385) | (-1007.44,1039.28) | (758.117,-716.231) | (-1031.26,-568.772) | (876.317,762.056) | (870.878,556.275) | 
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

 MAIN: after_motion State: x=-275.582, y=4.24738, w=-1.88647 deg
Fit Cartesian for: (-275.582,4.24738) | -1.88647

Number of CAR points8199ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 5
(863.376,578.017) | (-930.703,1063.96) | (-971.258,-551.768) | (814.872,-746.892) | (868.267,781.485) | 

 i = 0
ObsLM.x = 863.376ObsLM.y = 578.017
EstLM.x = 870.878EstLM.y = 556.275
z.r = 1275.32z.theta = 155.149
z_cap.r = 1272.44z_cap.theta = 156.175
(z-z_cap).r = 2.87756(z-z_cap).theta = -1.02639

 EKF: State2: x=-275.579, y=4.35213, w=-1.52247 deg
(775.983,806.385) | (-1007.43,1039.28) | (758.118,-716.231) | (-1031.26,-568.773) | (876.316,762.056) | (870.905,556.282) | 

 i = 1
ObsLM.x = -930.703ObsLM.y = 1063.96
EstLM.x = -1007.43EstLM.y = 1039.28
z.r = 1245.78z.theta = 59.7953
z_cap.r = 1267.55z_cap.theta = 56.2561
(z-z_cap).r = -21.7717(z-z_cap).theta = 3.53919

 EKF: State2: x=-275.569, y=5.83042, w=-2.34319 deg
(775.984,806.392) | (-1007.37,1039.13) | (758.105,-716.209) | (-1031.24,-568.763) | (876.33,762.07) | (870.925,556.29) | 

 i = 2
ObsLM.x = -971.258ObsLM.y = -551.768
EstLM.x = -1031.24EstLM.y = -568.763
z.r = 891.572z.theta = 323.631
z_cap.r = 949.313z_cap.theta = 325.095
(z-z_cap).r = -57.7415(z-z_cap).theta = -1.46387

 EKF: State2: x=-275.604, y=2.53664, w=-2.24232 deg
(775.983,806.38) | (-1007.31,1039.07) | (758.126,-716.251) | (-1030.87,-568.446) | (876.307,762.043) | (870.894,556.261) | 

 i = 3
ObsLM.x = 814.872ObsLM.y = -746.892
EstLM.x = 758.126EstLM.y = -716.251
z.r = 1323.17z.theta = 216.741
z_cap.r = 1259.07z_cap.theta = 217.055
(z-z_cap).r = 64.1024(z-z_cap).theta = -0.313465

 EKF: State2: x=-275.571, y=5.81421, w=-2.04989 deg
(775.984,806.392) | (-1007.37,1039.13) | (758.584,-716.541) | (-1030.82,-568.422) | (876.329,762.07) | (870.924,556.292) | 

 i = 4
ObsLM.x = 868.267ObsLM.y = 781.485
EstLM.x = 876.329EstLM.y = 762.07
z.r = 1382.04z.theta = 147.908
z_cap.r = 1377.97z_cap.theta = 148.764
(z-z_cap).r = 4.06995(z-z_cap).theta = -0.856314

 EKF: State2: x=-275.571, y=5.71235, w=-1.92883 deg
(775.984,806.391) | (-1007.37,1039.12) | (758.587,-716.542) | (-1030.82,-568.425) | (876.363,762.085) | (870.922,556.292) | 

 MAIN: after_ekf State: x=-275.571, y=5.71235, w=-1.92883 deg
(775.984,806.391) | (-1007.37,1039.12) | (758.587,-716.542) | (-1030.82,-568.425) | (876.363,762.085) | (870.922,556.292) | 
No of New Points = 0
GRID: In grid data process
GRID:SAVED TO CSV
LEAVNG RUN
