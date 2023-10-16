INIT_sigma_r = 100
INIT_sigma_theta = 0.1
INIT_sigma_odo_x = 0.01
INIT_sigma_odo_y = 0.01
INIT_sigma_odo_theta = 0.017
MCAL: IN motor calibration
TIME =  0.6339812278747559 s
LEFT SPEED =  0.709800196306283  rotations/s, ROTATIONS =  0.45 , TICKS =  9
RIGHT SPEED =  0.7886668847847588  rotations/s, ROTATIONS =  0.5 , TICKS =  10
TIME =  0.5347585678100586 s
LEFT SPEED =  0.7480011056916369  rotations/s, ROTATIONS =  0.4 , TICKS =  8
RIGHT SPEED =  0.9350013821145461  rotations/s, ROTATIONS =  0.5 , TICKS =  10
TIME =  0.5611708164215088 s
LEFT SPEED =  0.8909943022133889  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.80189487199205  rotations/s, ROTATIONS =  0.45 , TICKS =  9
TIME =  0.47731804847717285 s
LEFT SPEED =  1.0475195765070926  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.8380156612056742  rotations/s, ROTATIONS =  0.4 , TICKS =  8
TIME =  1.9858074188232422 s
LEFT SPEED =  1.0071470078327978  rotations/s, ROTATIONS =  2.0 , TICKS =  40
RIGHT SPEED =  0.9819683326369779  rotations/s, ROTATIONS =  1.95 , TICKS =  39
TIME =  0.5856473445892334 s
LEFT SPEED =  0.8537561121372359  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.7683805009235124  rotations/s, ROTATIONS =  0.45 , TICKS =  9
TIME =  0.6192705631256104 s
LEFT SPEED =  0.8074015297552292  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.7266613767797062  rotations/s, ROTATIONS =  0.45 , TICKS =  9
TIME =  0.5919582843780518 s
LEFT SPEED =  0.844654113634597  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.7601887022711373  rotations/s, ROTATIONS =  0.45 , TICKS =  9
TIME =  0.5763719081878662 s
LEFT SPEED =  0.7807458927254384  rotations/s, ROTATIONS =  0.45 , TICKS =  9
RIGHT SPEED =  0.8674954363615982  rotations/s, ROTATIONS =  0.5 , TICKS =  10
TIME =  2.037736177444458 s
LEFT SPEED =  0.9814813233125285  rotations/s, ROTATIONS =  2.0 , TICKS =  40
RIGHT SPEED =  0.9078702240640889  rotations/s, ROTATIONS =  1.85 , TICKS =  37
MCAL: time Left =  0.008763157894736842 s  0.001236842105263156 s
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

Traceback (most recent call last):
  File "/home/odroid/Desktop/Johann/rplidar_sdk-website/output/Linux/Release/cornerDetector.py", line 190, in <module>
    corners = find_corners(best_models)
  File "/home/odroid/Desktop/Johann/rplidar_sdk-website/output/Linux/Release/cornerDetector.py", line 128, in find_corners
    interAngle = calculate_intercept_angle(best_models[i],best_models[i+1])
  File "/home/odroid/Desktop/Johann/rplidar_sdk-website/output/Linux/Release/cornerDetector.py", line 96, in calculate_intercept_angle
    m2 = line2[0][0]
TypeError: 'NoneType' object is not subscriptable
Number of CAR points9008ret/cpp = 256
DATA: NUMBER OF CORNERS POINTS = 2
(-1206.43,863.321) | (975.798,-668.649) | 

Gain = 
8.13146e-09 | 8.13146e-09
-5.81889e-09 | -5.81889e-09
4.22817e-16 | 4.22817e-16
-8.13146e-05 | -8.13146e-05
5.81889e-05 | 5.81889e-05

 EKF: State2: x=0, y=0, w=0 deg
(-1206.43,863.321) | 

Gain = 
-8.24831e-09 | -8.24831e-09
5.65202e-09 | 5.65202e-09
-1.69978e-14 | -1.69978e-14
-8.17615e-13 | -8.17615e-13
5.75114e-13 | 5.75114e-13
8.24832e-05 | 8.24832e-05
-5.65202e-05 | -5.65202e-05

 EKF: State2: x=0, y=0, w=0 deg
(-1206.43,863.321) | (975.798,-668.649) | 

 MAIN: after_ekf State: x=0, y=0, w=0 deg
(-1206.43,863.321) | (975.798,-668.649) | 

 CSV: Saving Full Map (overwriting old)
GRID: In grid data process
GRID:SAVED TO CSV
IN update Movement Grid
NAVI,GRID: tri_shift = (0,0),(81,71),(81,-71)
NAVI,GRID around A = (81,-71) from B = (0,0) resulting C (0,0) to visit: (-200,0)
NAVI: Set angle = 0 deg Set Distance = 100mm
MC:TURN:  0.0  deg
MC GO FORWARD FOR:  100.0
TIME =  0.5807783603668213 s
LEFT SPEED =  0.8609136188961974  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.688730895116958  rotations/s, ROTATIONS =  0.4 , TICKS =  8
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

Number of CAR points9060ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 2
(-1911.78,979.41) | (144.105,-748.067) | 

Gain = 
1.75663e-08 | 1.75663e-08
-0.000572501 | -0.000572501
-4.82793e-06 | -4.82793e-06
2.19944e-07 | 2.19944e-07
3.07353e-07 | 3.07353e-07
-2.67916e-07 | -2.67916e-07
-3.90987e-07 | -3.90987e-07
-8.78074e-05 | -8.78074e-05
4.77752e-05 | 4.77752e-05

 EKF: State2: x=-102.102, y=0, w=0 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.78,979.41) | 

Gain = 
-6.40823e-09 | -6.40823e-09
0.00105499 | 0.00105499
7.71511e-06 | 7.71511e-06
-4.05305e-07 | -4.05305e-07
-5.66382e-07 | -5.66382e-07
4.93709e-07 | 4.93709e-07
7.205e-07 | 7.205e-07
-2.12183e-07 | -2.12183e-07
-2.86565e-07 | -2.86565e-07
3.22091e-05 | 3.22091e-05
-9.45601e-05 | -9.45601e-05

 EKF: State2: x=-102.102, y=0, w=0 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.78,979.41) | (144.105,-748.067) | 

 MAIN: after_ekf State: x=-102.102, y=0, w=0 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.78,979.41) | (144.105,-748.067) | 
No of New Points = 0
GRID: In grid data process
GRID:SAVED TO CSV
IN update Movement Grid
NAVI,GRID: tri_shift = (-102.102,0),(-21.1018,71),(-21.1018,-71)
NAVI,GRID around A = (-21.1018,-71) from B = (-102.102,0) resulting C (-102.102,0) to visit: (-200,200)
NAVI: Set angle = 0 deg Set Distance = 100mm
MC:TURN:  0.0  deg
MC GO FORWARD FOR:  100.0
TIME =  0.5991740226745605 s
LEFT SPEED =  0.8344821054960412  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.751033894946437  rotations/s, ROTATIONS =  0.45 , TICKS =  9
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

 MAIN: after_motion State: x=-204.204, y=0, w=0 deg
Fit Cartesian for: (-204.204,0) | 0

Number of CAR points8204ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 2
(-1906.6,967.93) | (152.697,-751.172) | 

Gain = 
2.6157e-08 | 2.6157e-08
-0.00210717 | -0.00210717
-1.02717e-05 | -1.02717e-05
3.4432e-07 | 3.4432e-07
4.8116e-07 | 4.8116e-07
-4.19422e-07 | -4.19422e-07
-6.12088e-07 | -6.12088e-07
-8.60968e-05 | -8.60968e-05
5.05737e-05 | 5.05737e-05
-1.44652e-06 | -1.44652e-06
-5.65699e-07 | -5.65699e-07

 EKF: State2: x=-204.204, y=-0.0493058, w=-0.0497493 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.78,979.41) | (144.105,-748.067) | 

Gain = 
-1.33881e-08 | -1.33881e-08
0.003463 | 0.003463
1.42182e-05 | 1.42182e-05
-5.8148e-07 | -5.8148e-07
-8.12574e-07 | -8.12574e-07
7.08312e-07 | 7.08312e-07
1.03368e-06 | 1.03368e-06
-9.60209e-07 | -9.60209e-07
-1.22762e-06 | -1.22762e-06
4.61565e-05 | 4.61565e-05
-8.86156e-05 | -8.86156e-05

 EKF: State2: x=-204.204, y=-0.16428, w=-0.140533 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.78,979.41) | (144.106,-748.067) | 

 MAIN: after_ekf State: x=-204.204, y=-0.16428, w=-0.140533 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.78,979.41) | (144.106,-748.067) | 
No of New Points = 0
GRID: In grid data process
GRID:SAVED TO CSV
IN update Movement Grid
NAVI,GRID: tri_shift = (-204.204,-0.16428),(-123.378,71.0342),(-123.03,-70.9654)
NAVI,GRID around A = (-123.03,-70.9654) from B = (-204.204,-0.16428) resulting C (-204.204,-0.16428) to visit: (-200,-200)
NAVI: Set angle = 0 deg Set Distance = 100mm
MC:TURN:  0.0  deg
MC GO FORWARD FOR:  100.0
TIME =  0.5456163883209229 s
LEFT SPEED =  0.8247552852743807  rotations/s, ROTATIONS =  0.45 , TICKS =  9
RIGHT SPEED =  0.9163947614159785  rotations/s, ROTATIONS =  0.5 , TICKS =  10
MC: Angle turned =  0.0
MC: distance moved =  91.89158511750145
ret/cpp = 0
LEAVNG RUN

 i = 3
------------------------------------------------------------------------------------------------------------


Attempt 0
SLAMTEC LIDAR S/N: ABCF99F6C9E59AD4C5E59CF7571D3414SLAMTEC Lidar health status : 0
Entering scan loop
I have reached max NoPoints in Lidar_functioncount = 1

 MAIN: after_motion State: x=-296.095, y=-0.389668, w=-0.140533 deg
Fit Cartesian for: (-296.095,-0.389668) | -0.140533

Number of CAR points8233ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 2
(-1895.59,960.731) | (168.41,-754.764) | 

Gain = 
-3.49498e-06 | -3.49498e-06
-0.00400073 | -0.00400073
-1.36483e-05 | -1.36483e-05
3.99963e-07 | 3.99963e-07
5.58917e-07 | 5.58917e-07
-4.87201e-07 | -4.87201e-07
-7.11002e-07 | -7.11002e-07
-8.41225e-05 | -8.41225e-05
5.34372e-05 | 5.34372e-05
-3.58057e-06 | -3.58057e-06
-1.77974e-06 | -1.77974e-06

 EKF: State2: x=-296.095, y=-0.388484, w=-0.184505 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.78,979.409) | (144.106,-748.067) | 

Gain = 
5.03581e-06 | 5.03581e-06
0.00599891 | 0.00599891
1.6646e-05 | 1.6646e-05
-6.24218e-07 | -6.24218e-07
-8.72296e-07 | -8.72296e-07
7.6037e-07 | 7.6037e-07
1.10965e-06 | 1.10965e-06
-1.91659e-06 | -1.91659e-06
-2.18544e-06 | -2.18544e-06
5.77521e-05 | 5.77521e-05
-8.18806e-05 | -8.18806e-05

 EKF: State2: x=-296.095, y=-0.742322, w=-0.421952 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.78,979.409) | (144.108,-748.068) | 

 MAIN: after_ekf State: x=-296.095, y=-0.742322, w=-0.421952 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.78,979.409) | (144.108,-748.068) | 
No of New Points = 1
GRID: In grid data process
GRID:SAVED TO CSV
IN update Movement Grid
NAVI,GRID: tri_shift = (-296.095,-0.742322),(-215.62,70.8523),(-214.575,-71.1439)
NAVI,GRID around A = (-214.575,-71.1439) from B = (-296.095,-0.742322) resulting C (-296.095,-0.742322) to visit: (-200,-200)
NAVI: Set angle = 0 deg Set Distance = 100mm
MC:TURN:  0.0  deg
MC GO FORWARD FOR:  100.0
TIME =  0.565314769744873 s
LEFT SPEED =  0.8844630049656236  rotations/s, ROTATIONS =  0.5 , TICKS =  10
RIGHT SPEED =  0.7960167044690613  rotations/s, ROTATIONS =  0.45 , TICKS =  9
MC: Angle turned =  0.0
MC: distance moved =  102.10176124166827
ret/cpp = 0
LEAVNG RUN

 i = FINALRUN 0
------------------------------------------------------------------------------------------------------------


Attempt 0
SLAMTEC LIDAR S/N: ABCF99F6C9E59AD4C5E59CF7571D3414SLAMTEC Lidar health status : 0
Entering scan loop
I have reached max NoPoints in Lidar_functioncount = 1

 MAIN: after_motion State: x=-398.194, y=-1.49424, w=-0.421952 deg
Fit Cartesian for: (-398.194,-1.49424) | -0.421952

Number of CAR points9089ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 2
(-1887.02,948.562) | (189.046,-751.142) | 

Gain = 
-1.88502e-05 | -1.88502e-05
-0.00659593 | -0.00659593
-1.70363e-05 | -1.70363e-05
4.3752e-07 | 4.3752e-07
6.114e-07 | 6.114e-07
-5.3295e-07 | -5.3295e-07
-7.77767e-07 | -7.77767e-07
-8.1484e-05 | -8.1484e-05
5.68678e-05 | 5.68678e-05
-6.19432e-06 | -6.19432e-06
-3.71024e-06 | -3.71024e-06

 EKF: State2: x=-398.195, y=-1.4288, w=-0.494133 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.77,979.407) | (144.108,-748.068) | 

Gain = 
2.41129e-05 | 2.41129e-05
0.00883676 | 0.00883676
1.80436e-05 | 1.80436e-05
-6.15649e-07 | -6.15649e-07
-8.60321e-07 | -8.60321e-07
7.49932e-07 | 7.49932e-07
1.09442e-06 | 1.09442e-06
-3.06019e-06 | -3.06019e-06
-3.0785e-06 | -3.0785e-06
6.86446e-05 | 6.86446e-05
-7.38398e-05 | -7.38398e-05

 EKF: State2: x=-398.198, y=-2.23318, w=-0.939258 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.77,979.407) | (144.111,-748.069) | 

 MAIN: after_ekf State: x=-398.198, y=-2.23318, w=-0.939258 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.77,979.407) | (144.111,-748.069) | 
No of New Points = 0
GRID: In grid data process
GRID:SAVED TO CSV
LEAVNG RUN

 i = FINALRUN 1
------------------------------------------------------------------------------------------------------------


Attempt 0
SLAMTEC LIDAR S/N: ABCF99F6C9E59AD4C5E59CF7571D3414SLAMTEC Lidar health status : 0
Entering scan loop
I have reached max NoPoints in Lidar_functioncount = 1

 MAIN: after_motion State: x=-398.198, y=-2.23318, w=-0.939258 deg
Fit Cartesian for: (-398.198,-2.23318) | -0.939258

Number of CAR points8219ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 2
(-1891.06,930.401) | (197.217,-744.929) | 

Gain = 
-3.62397e-05 | -3.62397e-05
-0.00903075 | -0.00903075
-1.85368e-05 | -1.85368e-05
4.36972e-07 | 4.36972e-07
6.10633e-07 | 6.10633e-07
-5.32282e-07 | -5.32282e-07
-7.76792e-07 | -7.76792e-07
-8.04051e-05 | -8.04051e-05
5.75873e-05 | 5.75873e-05
-8.40096e-06 | -8.40096e-06
-5.95111e-06 | -5.95111e-06

 EKF: State2: x=-398.2, y=-2.28853, w=-1.15486 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.77,979.404) | (144.111,-748.069) | 

Gain = 
4.7039e-05 | 4.7039e-05
0.0121253 | 0.0121253
1.91356e-05 | 1.91356e-05
-6.17885e-07 | -6.17885e-07
-8.63446e-07 | -8.63446e-07
7.52657e-07 | 7.52657e-07
1.0984e-06 | 1.0984e-06
-4.45309e-06 | -4.45309e-06
-3.9553e-06 | -3.9553e-06
7.18331e-05 | 7.18331e-05
-7.03421e-05 | -7.03421e-05

 EKF: State2: x=-398.207, y=-3.49962, w=-1.74214 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.77,979.405) | (144.115,-748.07) | 

 MAIN: after_ekf State: x=-398.207, y=-3.49962, w=-1.74214 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.77,979.405) | (144.115,-748.07) | 
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

 MAIN: after_motion State: x=-398.207, y=-3.49962, w=-1.74214 deg
Fit Cartesian for: (-398.207,-3.49962) | -1.74214

Number of CAR points8196ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 2
(-1912.63,904.398) | (209.239,-737.641) | 

Gain = 
-5.39884e-05 | -5.39884e-05
-0.0113866 | -0.0113866
-1.93603e-05 | -1.93603e-05
4.27123e-07 | 4.27123e-07
5.9687e-07 | 5.9687e-07
-5.20285e-07 | -5.20285e-07
-7.59283e-07 | -7.59283e-07
-7.92801e-05 | -7.92801e-05
5.8172e-05 | 5.8172e-05
-1.05181e-05 | -1.05181e-05
-8.39781e-06 | -8.39781e-06

 EKF: State2: x=-398.211, y=-3.9923, w=-2.25312 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.77,979.402) | (144.115,-748.071) | 

Gain = 
7.07677e-05 | 7.07677e-05
0.0153054 | 0.0153054
1.94642e-05 | 1.94642e-05
-6.04629e-07 | -6.04629e-07
-8.44922e-07 | -8.44922e-07
7.36509e-07 | 7.36509e-07
1.07483e-06 | 1.07483e-06
-5.9022e-06 | -5.9022e-06
-4.62793e-06 | -4.62793e-06
7.48621e-05 | 7.48621e-05
-6.65697e-05 | -6.65697e-05

 EKF: State2: x=-398.223, y=-5.75487, w=-3.0316 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.77,979.403) | (144.119,-748.072) | 

 MAIN: after_ekf State: x=-398.223, y=-5.75487, w=-3.0316 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.77,979.403) | (144.119,-748.072) | 
No of New Points = 1
GRID: In grid data process
GRID:SAVED TO CSV
LEAVNG RUN

 i = FINALRUN 3
------------------------------------------------------------------------------------------------------------


Attempt 0
SLAMTEC LIDAR S/N: ABCF99F6C9E59AD4C5E59CF7571D3414SLAMTEC Lidar health status : 0
Entering scan loop
I have reached max NoPoints in Lidar_functioncount = 1

 MAIN: after_motion State: x=-398.223, y=-5.75487, w=-3.0316 deg
Fit Cartesian for: (-398.223,-5.75487) | -3.0316

Number of CAR points8229ret/cpp = 0
DATA: NUMBER OF CORNERS POINTS = 2
(-1927.55,871.346) | (224.038,-726.623) | 

Gain = 
-6.50488e-05 | -6.50488e-05
-0.012427 | -0.012427
-1.81635e-05 | -1.81635e-05
3.76551e-07 | 3.76551e-07
5.262e-07 | 5.262e-07
-4.58682e-07 | -4.58682e-07
-6.69383e-07 | -6.69383e-07
4.20602e-06 | 4.20602e-06
3.56219e-06 | 3.56219e-06
-1.14316e-05 | -1.14316e-05
-1.00195e-05 | -1.00195e-05
-8.58666e-05 | -8.58666e-05
5.00328e-05 | 5.00328e-05

 EKF: State2: x=-398.223, y=-5.75487, w=-3.0316 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.77,979.403) | (144.119,-748.072) | (-1927.55,871.346) | 

Gain = 
9.40582e-05 | 9.40582e-05
0.0183093 | 0.0183093
1.95154e-05 | 1.95154e-05
-5.83047e-07 | -5.83047e-07
-8.14763e-07 | -8.14763e-07
7.1022e-07 | 7.1022e-07
1.03647e-06 | 1.03647e-06
-6.21518e-06 | -6.21518e-06
-5.13098e-06 | -5.13098e-06
7.77067e-05 | 7.77067e-05
-6.26391e-05 | -6.26391e-05
-1.07874e-06 | -1.07874e-06
-6.65655e-08 | -6.65655e-08

 EKF: State2: x=-398.24, y=-8.31403, w=-4.06589 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.76,979.404) | (144.124,-748.072) | (-1927.55,871.346) | 

 MAIN: after_ekf State: x=-398.24, y=-8.31403, w=-4.06589 deg
(-1206.43,863.321) | (975.798,-668.649) | (-1911.76,979.404) | (144.124,-748.072) | (-1927.55,871.346) | 
No of New Points = 0
GRID: In grid data process
GRID:SAVED TO CSV
LEAVNG RUN