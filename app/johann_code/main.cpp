//Probalistic Robotics
//#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <random>
#include <vector>
#include <cmath>
#include "robot.h"

using namespace std;
using namespace Eigen;
using namespace CSV_Functions;
using namespace Data_Functions;
using namespace Landmark_Functions;
using namespace Lidar_Functions;
using namespace Navigation_Functions;
using namespace Mapping_Functions;

//Thread Stuff
mutex mtx; // Mutex for synchronization
condition_variable cv; // Condition variable for signalin
atomic<bool> stopFlag(false); // Atomic flag to control threadSlave loop
atomic<bool> startFlag(false); // Atomic flag to control threadSlave loop

float grid_ystep = 400;
float grid_xstep = 400;
float boundThresh = 340;//If distance between gridPoint and lidarPoint <= Xmm then return false 





//g++ -g main.cpp Data_Functions.cpp  CSV_Functions.cpp EKF_Functions.cpp Landmark_Functions.cpp Simulation_Functions.cpp -o prog2

void testEKF(){
    cout<<"THIS IS BROKEN DUE TO EKF updateMotion being removed!!"<<endl;
    ExtendedKalmanFilter ekf;


    CarPoint A;
    CarPoint B;
    CarPoint C;
    CarPoint D;
    CarPoint E;
    // A.x = 10.14655999;
    // B.x = 17.81377777;
    // C.x = 15.07757899;
    A.x = 2;
    B.x = -2;
    C.x = -2;
    D.x = 2;
    E.x = 2;

    // A.y = -0.19454059;
    // B.y = 0.56453221;
    // C.y = 1.37651164;
    A.y = 2;
    B.y = 2;
    C.y = -2;
    D.y = -2;
    E.y = 2;


    ekf.TestValues.push_back(A);
    ekf.TestValues.push_back(B);
    ekf.TestValues.push_back(C);
    ekf.TestValues.push_back(D);
    //ekf.TestValues.push_back(E);


    // ekf.v = 1.3094206;
    // ekf.w = -0.12391118;

    ekf.runEKF();
    cout<<"\n\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!EKF HAS BEEN RUN!!!!!!!!!!!!!!!!!!!!!!!\n\n\n";


    ekf.runEKF();
    cout<<"\n\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!EKF HAS BEEN RUN!!!!!!!!!!!!!!!!!!!!!!!\n\n\n";
    for(int i =0;i<130;i++){
        cout<<"\ni = "<<i<<endl;
        ekf.runEKF();
    }
}

void testLidar(){
    vector<PolPoint> lidarDataPoints;//can be replaced with array for speed
    bool error = true;
    //initializeLidar(lidarDataPoints,error);
    runLidar(lidarDataPoints,error);
}

void testLandmarkIdentification(bool& firstRun){
    //LandmarkProcessing();
    
    
    
    //Run Lidar
    vector<PolPoint> lidarDataPoints;//can be replaced with array for speed
    bool error = true;
    int count = 0;
    while(error == true && count<5){
        cout<<"\nAttempt "<<count<<endl;
        runLidar(lidarDataPoints, error);
        count +=1;
    }
    cout<<"count = "<<count<<endl;
    
    cout<<"Main: Lidar Run complete"<<endl;

    vector<PolPoint> sortedPoints = sortPoints(lidarDataPoints);
    cout<<"Sorting completed";

    vector<CarPoint> carPoints = convertCartesian(sortedPoints);
    //fitCartesian(carPoints,x,y,angle); for this test we are not moving

    saveCarToCSV(carPoints);
    cout<<"\nNumber of CAR points"<<carPoints.size(); 

    if(firstRun == true){
        saveCarToFullMapCSV(carPoints);
        firstRun = false;
    }else{
        //storeMapPoints(carPoints,ekf.State);
    }

    LandmarkProcessing2(carPoints);
}

// void testLidarLandmark(){
//     vector<PolPoint> lidarDataPoints;//can be replaced with array for speed
//         bool error = true;
//         int count = 0;
//         while(error == true && count<5){
//             cout<<"\nAttempt "<<count<<endl;
//             runLidar(lidarDataPoints, error);
//             count +=1;
//         }
//         cout<<"count = "<<count<<endl;

//      if(error == false){
//             //Predict Position

//             //cout<<"\n MAIN: after_motion State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;
//             //cout << "\nEKF 6\nState =\n" << ekf.State << "\n";
//             ExtendedKalmanFilter ekf;


//             ekf.State(0) = -200;
//             ekf.State(1) = -95;
//             //ekf.State(2) = PI/2;
//             //Process Data
//             vector<CarPoint> carPoints;
//             lidarDataProcessing(lidarDataPoints,carPoints,ekf.State[0],ekf.State[1],ekf.State[2]);
//             saveCarToFullMapCSV(carPoints);
//             saveCarToCSV(carPoints);
//      }
// }

void testPython(){
    int ret;
    ret = system("python3 motorControl.py ok go");

    cout << "ret/cpp = " << ret << endl;

    return;
}

void testMotor(){
    float angle = -PI/2;
    float distance = 0;

    writeMotorToCSV(angle,distance);
    
    int ret;
    ret = system("python3 motorControl.py ok go");
    cout << "ret/cpp = " << ret << endl;

    cout<<"Set angle = "<<angle*180/(PI)<<" Set Distance = "<<distance<<endl;
    readMotorFromCSV(angle,distance);
    cout<<"Read angle = "<<angle<<" Read Distance = "<<distance<<endl;

}


void threadSlave(int n, string a, vector<int>& vect){
    while (!stopFlag) { // Check the stop flag to determine whether to continue
        cout << "SLAVE: This is n = " << n << endl;
        cout << "SLAVE: This is a = " << a << endl;

        vect.clear();
        for (int i = 0; i < n; i++) {
            vect.push_back(i);
        }

        // Notify the main thread that the vector is filled
        {
            lock_guard<mutex> lock(mtx);
            cv.notify_all();
        }
    }
}

void testThread() {
    vector<int> vect;
    int size = 1000000;
    thread t1(threadSlave, size, "StringFromMain", ref(vect));

    int timer = 0;
    while (timer < 500) {
        {
            unique_lock<mutex> lock(mtx);
            cv.wait(lock, [&vect, size] { return vect.size() == size; });

            // Do something with the filled vector
            // For example, copy its contents to another data structure

            // Reset the vector
            vect.clear();
        }

        // Continue your processing after vector is filled

        timer = timer + 1;
        cout << "\nMAIN: vector is full in time = " << timer << endl;
    }

    cout << "Signal threadSlave to stop" << endl;
    stopFlag.store(true); // Set the stop flag to signal threadSlave to stop

    // Wait for the t1 thread to join
    t1.join();
    cout << "t1 joined" << endl;
}

void testMap(){
    // vector<CarPoint> path;
    // cout<<"TEST RUN"<<endl;

    // ILidarDriver * drv = nullptr;
    // sl_result op_result;
    // vector<PolPoint> lidarDataPoints;
    // int NoPoints = 8192;
    // bool error;
    // sl_u32 timeout = 3000;

    // initializeLidar(drv,error);
    // fetchScan(drv, op_result, lidarDataPoints, NoPoints, error, timeout);
    // lidarDataPoints.clear();
    // fetchScan(drv, op_result, lidarDataPoints, NoPoints, error, timeout);

    //saveCarToFullMapCSV(carPoints);    

    vector<CarPoint> map;
    vector<vector<GridPoint>> gridNew;
    readCarFromFullMapCSV(map);
    gridMakeDots(map,gridNew);
    saveGridToCSV(gridNew);

}



void calibrateMotors(){
    writeMotorStateToCSV(true);

    int ret;
    ret = system("python3 motorControl.py ok go");
    cout << "ret/cpp = " << ret << endl;

    writeMotorStateToCSV(false);
    return;
}


//This process will only use the latest scan to update the EKF and RANSAC
void simRun(ExtendedKalmanFilter& ekf, bool final){
    //cout<<"In SIM RUN"<<endl;

    if(final == false){
        ekf.updateMotion();
        cout<<"MAIN: after_motion State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;
        //cout << "\nEKF 6\nState =\n" << ekf.State << "\n";
 
    }
    
    //Run EKF
    ekf.runEKF();

    cout<<"MAIN: after_ekf State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;

    for(int i =3;i<dim;i=i+2){
        if(ekf.State[i] != 0 && ekf.State[i+1] != 0){
            cout<<"("<<ekf.State[i]<<","<<ekf.State[i+1]<<") | ";
        }
    }
    cout<<endl;
    

    
}




void simRun1(){
    ExtendedKalmanFilter ekf;
    cout<<"sigma_r = "<<ekf.sigma_r<<endl;
    cout<<"sigma_theta = "<<ekf.sigma_theta<<endl;
    cout<<"sigma_odo_x = "<<ekf.sigma_odo_x<<endl;
    cout<<"sigma_odo_y = "<<ekf.sigma_odo_y<<endl;
    cout<<"sigma_odo_theta = "<<ekf.sigma_odo_theta<<endl;
    
    ekf.w=0;
    ekf.distance =0;
    ekf.TestValues.push_back({768.848,-625.26});
    ekf.TestValues.push_back({765.078,893.023});
    simRun(ekf,false);

    ekf.w=0;
    ekf.distance =102.10176124166827;
    ekf.TestValues.clear();
    ekf.TestValues.push_back({754.029,691.722});
    ekf.TestValues.push_back({811.1,-597.757});
    simRun(ekf,false);

    ekf.w=0;
    ekf.distance =91.89158511750145;
    ekf.TestValues.clear();
    ekf.TestValues.push_back({794.409,684.405});
    ekf.TestValues.push_back({836.933,-594.593});
    simRun(ekf,false);

    ekf.w=0;
    ekf.distance =102.10176124166827;
    ekf.TestValues.clear();
    ekf.TestValues.push_back({857.585,-599.617});
    ekf.TestValues.push_back({-1181.81,-635.516});
    simRun(ekf,false);

    ekf.w=0;
    ekf.distance =102.10176124166827;
    ekf.TestValues.clear();
    ekf.TestValues.push_back({878.672,-569.8});
    ekf.TestValues.push_back({-1216.67,-650.351});
    simRun(ekf,false);

    cout<<"-----------------------------------------"<<endl;

    ekf.TestValues.clear();
    ekf.TestValues.push_back({857.36,-602.621});
    ekf.TestValues.push_back({-1234.59,-614.237});
    simRun(ekf,true);

    ekf.TestValues.clear();
    ekf.TestValues.push_back({843.613,-622.543});
    ekf.TestValues.push_back({-1261.33,-586.234});
    simRun(ekf,true);

    ekf.TestValues.clear();
    ekf.TestValues.push_back({825.285,-648.551});
    ekf.TestValues.push_back({-1218.59,-556.074});
    simRun(ekf,true);

    cout<<"SORCERY"<<endl;
    return;
}

void simRun2(){
    ExtendedKalmanFilter ekf;
    cout<<"sigma_r = "<<ekf.sigma_r<<endl;
    cout<<"sigma_theta = "<<ekf.sigma_theta<<endl;
    cout<<"sigma_odo_x = "<<ekf.sigma_odo_x<<endl;
    cout<<"sigma_odo_y = "<<ekf.sigma_odo_y<<endl;
    cout<<"sigma_odo_theta = "<<ekf.sigma_odo_theta<<endl;
    
    ekf.w=0;
    ekf.distance =0;
    ekf.TestValues.push_back({793.426,806.399});
    ekf.TestValues.push_back({759.393,-641.327});
    simRun(ekf,false);

    ekf.w=0;
    ekf.distance =102.1;
    ekf.TestValues.clear();
    ekf.TestValues.push_back({803.1,442.369});
    ekf.TestValues.push_back({772.873,-644.434});
    simRun(ekf,false);

    ekf.w=0;
    ekf.distance =91.89158511750145;
    ekf.TestValues.clear();
    ekf.TestValues.push_back({852.718,402.318});
    ekf.TestValues.push_back({765.385,-667.562});
    simRun(ekf,false);

    cout<<"-----------------------------------------"<<endl;

    ekf.TestValues.clear();
    ekf.TestValues.push_back({826.94,432.343});
    ekf.TestValues.push_back({804.766,-652.828});
    simRun(ekf,true);

    ekf.TestValues.clear();
    ekf.TestValues.push_back({829.686,448.039});
    ekf.TestValues.push_back({794.628,-619.954});
    simRun(ekf,true);

    ekf.TestValues.clear();
    ekf.TestValues.push_back({813.554,486.389});
    ekf.TestValues.push_back({817.647,-594.362});
    simRun(ekf,true);

    cout<<"SORCERY"<<endl;
    return;
}

void simRun3(){
    ExtendedKalmanFilter ekf;
    cout<<"sigma_r = "<<ekf.sigma_r<<endl;
    cout<<"sigma_theta = "<<ekf.sigma_theta<<endl;
    cout<<"sigma_odo_x = "<<ekf.sigma_odo_x<<endl;
    cout<<"sigma_odo_y = "<<ekf.sigma_odo_y<<endl;
    cout<<"sigma_odo_theta = "<<ekf.sigma_odo_theta<<endl;
    
    ekf.w=0;
    ekf.distance =0;
    ekf.TestValues.push_back({763.226,821.155});
    ekf.TestValues.push_back({-1027.73,1024.49});
    ekf.TestValues.push_back({768.733,-702.785});
    ekf.TestValues.push_back({765.71,597.332});
    ekf.TestValues.push_back({-1027.62,-585.177});
    simRun(ekf,false);

    ekf.w=0;
    ekf.distance =91.89158511750145;
    ekf.TestValues.clear();
    ekf.TestValues.push_back({807.103,795.645});
    ekf.TestValues.push_back({-978.576,1050.07});
    ekf.TestValues.push_back({768.434,-722.174});
    ekf.TestValues.push_back({-1016.22,-564.596});
    ekf.TestValues.push_back({804.18,580.847});
    simRun(ekf,false);

    ekf.w=0;
    ekf.distance =91.89158511750145;
    ekf.TestValues.clear();
    ekf.TestValues.push_back({847.053,773.736});
    ekf.TestValues.push_back({783.505,-741.488});
    ekf.TestValues.push_back({-998.814,-552.505});
    ekf.TestValues.push_back({-927.949,1063.87});
    ekf.TestValues.push_back({840.862,559.079});
    simRun(ekf,false);

    ekf.w=0;
    ekf.distance =102.10176124166827;
    ekf.TestValues.clear();
    ekf.TestValues.push_back({881.274,511.251});
    ekf.TestValues.push_back({-907.695,1095.98});
    ekf.TestValues.push_back({-1001.92,-519.256});
    ekf.TestValues.push_back({769.952,-789.791});
    ekf.TestValues.push_back({893.194,733.529});
    simRun(ekf,false);

    cout<<"-----------------------------------------"<<endl;

    ekf.TestValues.clear();
    ekf.TestValues.push_back({888.939,486.888});
    ekf.TestValues.push_back({-888.473,1107.94});
    ekf.TestValues.push_back({771.318,-795.827});
    ekf.TestValues.push_back({908.979,712.507});
    ekf.TestValues.push_back({-1011.13,-507.74});
    simRun(ekf,true);

    ekf.TestValues.clear();
    ekf.TestValues.push_back({903.269,451.617});
    ekf.TestValues.push_back({-857.008,1126.25});
    ekf.TestValues.push_back({-1026.89,-480.472});
    ekf.TestValues.push_back({731.178,-839.262});
    ekf.TestValues.push_back({925.612,678.594});
    simRun(ekf,true);

    for(int i =0;i<10;i++){
        ekf.TestValues.clear();
        ekf.TestValues.push_back({915.606,424.298});
        ekf.TestValues.push_back({-821.015,1146.44});
        ekf.TestValues.push_back({704.389,-872.277});
        ekf.TestValues.push_back({948.087,644.084});
        ekf.TestValues.push_back({-1041.74,-459.041});
        simRun(ekf,true);
    }
    saveStatsToCSV(ekf.stats);

    cout<<"SORCERY"<<endl;
    return;
}

void simRun4(){
    ExtendedKalmanFilter ekf;
    
    ekf.w=0;
    ekf.distance =0;
    ekf.TestValues.push_back({763.226,821.155});
    ekf.TestValues.push_back({-1027.73,1024.49});
    ekf.TestValues.push_back({768.733,-702.785});
    simRun(ekf,false);

    ekf.w=0;
    ekf.distance =91.89158511750145;
    ekf.TestValues.clear();
    ekf.TestValues.push_back({807.103,795.645});
    ekf.TestValues.push_back({-978.576,1050.07});
    ekf.TestValues.push_back({768.434,-722.174});
    simRun(ekf,false);

    cout<<"-----------------------------------------"<<endl;

    ekf.TestValues.clear();
    ekf.TestValues.push_back({807.103,795.645});
    ekf.TestValues.push_back({-978.576,1050.07});
    ekf.TestValues.push_back({768.434,-722.174});
    simRun(ekf,true);

    ekf.TestValues.clear();
    ekf.TestValues.push_back({807.103,795.645});
    ekf.TestValues.push_back({-978.576,1050.07});
    ekf.TestValues.push_back({768.434,-722.174});
    simRun(ekf,true);

    ekf.TestValues.clear();
    ekf.TestValues.push_back({807.103,795.645});
    ekf.TestValues.push_back({-978.576,1050.07});
    ekf.TestValues.push_back({768.434,-722.174});
    simRun(ekf,true);

    cout<<"SORCERY"<<endl;
    return;
}

void simRun5(){
    ExtendedKalmanFilter ekf;
    vector<vector<float>> states;
    vector<float> state;
    float x = -10;
    float w = -1*(PI/180);
    float y = (x/(cos(w)))*sin(w);

    w=0;
    y=10;

    vector<float> groundtruth{ x, y, w };
    
    ekf.w=0;
    ekf.distance =0;
    ekf.TestValues.push_back({800,800});
    ekf.TestValues.push_back({750,-700});
    ekf.TestValues.push_back({-1000,-700});
    simRun(ekf,false);

    //Set distance = 110 and angle = 0
    //Set true landmark distance = 100 and angle = 10 degrees 
    //Thus landmarks must converge to (100,17.63269806) | 10

    CarPoint LM1 = {800+x,800+y};
    CarPoint LM2 = {750+x,-700+y};
    CarPoint LM3 = {-1000+x,-700+y};

    cout<<"LM1 = "<<LM1<<endl;
    cout<<"LM2 = "<<LM2<<endl;
    cout<<"LM3 = "<<LM3<<endl;

    float deltaX = LM1.x - groundtruth[0];
    float deltaY = LM1.y - groundtruth[1];
    float q = deltaX * deltaX + deltaY * deltaY;
    float e_q = sqrt(q);
    float e_theta = (PI - atan2(deltaY, deltaX)) - groundtruth[2];
    cout<<"GM_LM1: e_q = "<<e_q<<" e_theta = "<<e_theta*180/PI<<endl;

    deltaX = LM2.x - groundtruth[0];
    deltaY = LM2.y - groundtruth[1];
    q = deltaX * deltaX + deltaY * deltaY;
    e_q = sqrt(q);
    e_theta = (PI - atan2(deltaY, deltaX)) - groundtruth[2];
    cout<<"GM_LM2: e_q = "<<e_q<<" e_theta = "<<e_theta*180/PI<<endl;

    deltaX = LM3.x - groundtruth[0];
    deltaY = LM3.y - groundtruth[1];
    q = deltaX * deltaX + deltaY * deltaY;
    e_q = sqrt(q);
    e_theta = (PI - atan2(deltaY, deltaX)) - groundtruth[2];
    cout<<"GM_LM3: e_q = "<<e_q<<" e_theta = "<<e_theta*180/PI<<endl;

    

    ekf.w=0;
    ekf.distance =0;
    ekf.TestValues.clear();
    ekf.TestValues.push_back(LM1);
    ekf.TestValues.push_back(LM2);
    ekf.TestValues.push_back(LM3);
    simRun(ekf,false);

    cout<<"-----------------------------------------"<<endl;
    int runs = 130;

    for(int i =0;i<runs;i++){
        ekf.w=0;
        ekf.distance =0;
        ekf.TestValues.clear();
        ekf.TestValues.push_back(LM1);
        ekf.TestValues.push_back(LM2);
        ekf.TestValues.push_back(LM3);
        simRun(ekf,false);
    }

    saveStatsToCSV(ekf.stats);


    //Do stats
    vector<float> stddev(3,0);
    vector<float> avg(3,0);

    for(int i=0;i<states.size();i++){
        avg[0]+=states[i][0];
        avg[1]+=states[i][1];
        avg[2]+=states[i][2];

        stddev[0]+= abs(states[i][0] - groundtruth[0]);
        stddev[1]+= abs(states[i][1] - groundtruth[1]);
        stddev[2]+= abs(states[i][2] - groundtruth[2]);
    }

    cout<<"runs = "<<runs<<endl;
    cout<<"stddev x:"<<stddev[0]/states.size()<<" y:"<<stddev[1]/states.size()<<" theta:"<<(stddev[2]/states.size())*180/PI<<endl;
    cout<<"avg x:"<<avg[0]/states.size()<<" y:"<<avg[1]/states.size()<<" theta:"<<(avg[2]/states.size())*180/PI<<endl;
    cout<<"gt x:"<<groundtruth[0]<<"  gt y:"<<groundtruth[1]<<"  gt theta:"<<groundtruth[2]*180/PI<<endl;

    return;
}




void atSim(){
    //I want to be able to feed it landmark locations
    //I want to be able to feed it distance locations
    ExtendedKalmanFilter ekf;

    vector<vector<float>> u;
    //vector<vector<CarPoint>> lm;
    vector<vector<PolPoint>> lm;

    atsi_u_read(u);
    //atsi_lm_read(lm);
    atsi_lm_read2(lm);
    cout<<"check sizes = "<<lm.size()<<" "<<u.size()<<endl;

    
    

    vector<vector<float>> states;
    vector<float> curr_state;
    vector<vector<float>> landmarks;
    vector<float> curr_lm;


    for(int i=0;i<u.size();i++){
        cout<<"\n i ="<<i<<endl;
        cout<<"u[0][0]= "<<u[i][0]<<" u[0][1]= "<<u[i][1]<<endl;
        
        ekf.w = u[i][1];
        ekf.distance = u[i][0];
        ekf.TestValues.clear();

        //We must now convert the landmarks to x and y
        // vecotr<CarPoint> lm_converted;
        // for(int j =0;j<3;j++){
        //     Carpoint newPoint;
        //     newPoint.x = landmarks[i+j][0]*cos(landmarks[i+j][1]);
        //     newPoint.y = landmarks[i+j][0]*sin(landmarks[i+j][1]);
        //     lm_converted.push_back(newPoint);
        //     cout<<"lm_converted = "<<newPoint<<endl;
        // }

        cout<<"Main:landmarks"<<endl;

        //Polar Landmark
        vector<PolPoint> lm_polar;
        for(int j =0;j<lm[i].size();j++){
            lm_polar.push_back(lm[i][j]);
            cout<<"Main: L"<<j<<"= "<<lm[i][j].distance<<", "<<lm[i][j].angle<<" = "<<lm[i][j].distance*cos(lm[i][j].angle)<<","<<lm[i][j].distance*sin(lm[i][j].angle)<<endl;
        }

        ekf.TestPolValues = lm_polar;
        
        // for(int j=0;j<lm[i].size();j++){
        //     ekf.TestValues.push_back(lm_converted);
        // }

        simRun(ekf,false);

        //Only x and y needed  
        curr_state.clear();
        curr_state.push_back(ekf.State[0]);
        curr_state.push_back(ekf.State[1]);
        states.push_back(curr_state);

        curr_lm.clear();
        for(int i=3;i<dim;i=i+2){
            curr_lm.push_back(ekf.State[i]);
            curr_lm.push_back(ekf.State[i+1]); 
        }
        
        landmarks.push_back(curr_lm);
        cout<<"states["<<i<<"][0]= "<<states[i][0]<<" states["<<i<<"][1]= "<<states[i][1]<<endl;
        cout<<"landmarks["<<i<<"][0]= "<<landmarks[i][0]<<" landmarks["<<i<<"][1]= "<<landmarks[i][1]<<endl;
        cout<<"curr_lm = ("<<curr_lm[0]<<","<<curr_lm[1]<<"),("<<curr_lm[2]<<","<<curr_lm[3]<<"),("<<curr_lm[4]<<","<<curr_lm[5]<<")"<<endl;
        cout<<endl;
    }

    for(int i =0;i<int(curr_lm.size()/2);i++){
        cout<<"curr_lm["<<i<<"]= ()"<<curr_lm[i*2]<<", "<<curr_lm[i*2+1]<<")"<<endl;

    }



    cout<<"Completed RUN"<<endl;

    atsi_lm_write(landmarks);
    cout<<"complete lm write"<<endl;
    atsi_u_write(states);        
    cout<<"complete u states"<<endl;

    return;
        

        

    

    
}


void randomFitting(vector<PolPoint>& lidarDataPoints,vector<CarPoint> carPoints, vector<PolPoint> polarCornerPoints, ExtendedKalmanFilter& ekf, float d, float w,float& acc){
        ekf.w = w;
        ekf.distance = d;

        ekf.updateMotion();
        ekf.TestPolValues = polarCornerPoints;


        //Run EKF
        ekf.runEKF();
        float ekfX = ekf.State(0);
        float ekfY = ekf.State(1);
        float ekfCosAngle = cos(ekf.State(2));
        float ekfSinAngle = sin(ekf.State(2));
        

        //fitCartesian(carPoints,ekf.State(0),ekf.State(1),ekf.State(2));

        //I think accuracy would be better confirmed by corner distances
        //Once again we will assume only 4 corners max
        CarPoint c1(ekf.State(3),ekf.State(4));
        CarPoint c2(ekf.State(5),ekf.State(6));
        CarPoint c3(ekf.State(7),ekf.State(8));
        CarPoint c4(ekf.State(9),ekf.State(10));
        CarPoint c5(ekf.State(11),ekf.State(12));
        CarPoint c6(ekf.State(13),ekf.State(14));
        vector<CarPoint> Stored_vec{c1,c2,c3,c4,c5,c6};
        vector<float> distances;
        CarPoint printPoint;

        cout<<"randomFitting Connection Data: ";
        for(int i =0;i<Stored_vec.size();i++){
            
            CarPoint StatePoint = Stored_vec[i];
            float dist = 1000000000000;
            if(StatePoint.x !=0 && StatePoint.y !=0){
                for(int j =0;j<polarCornerPoints.size();j++){
                    //Convert Corner to Cart
                    float r = polarCornerPoints[i].distance;
                    float ang = polarCornerPoints[i].angle;
                    CarPoint LiDARPoint = {r*cos(ang),r*sin(ang)};

                    // Apply rotation first
                    float rotatedX = LiDARPoint.x * ekfCosAngle - LiDARPoint.y * ekfSinAngle;
                    float rotatedY = LiDARPoint.x * ekfSinAngle + LiDARPoint.y * ekfCosAngle;

                    // Then apply translation
                    LiDARPoint.x = rotatedX + ekfX;
                    LiDARPoint.y = rotatedY + ekfY;

                    //Get minimum distance
                    if(pointDistance(LiDARPoint,StatePoint) < dist){
                        dist = pointDistance(LiDARPoint,StatePoint);
                        printPoint = LiDARPoint;
                    }
                }
                //pushback minimum distance
                distances.push_back(dist);
                cout<<"StatePnt = "<<StatePoint<<", LiDARPoint = "<<printPoint<<" dist = "<<dist<<"  &&  ";
            }
        }
        cout<<endl;

        //Get average minimum distance
        for(int i=0;i<distances.size();i++){
            acc+=distances[i];
        }
        acc = acc/distances.size();



        // //We must confirm that this thing is actually making some fucking sense
        // //Polar Corner Points are not fitted, pls remember this
        // for(int i =0;i<Stored_vec.size();i++){
            
        //     CarPoint StoredPoint = Stored_vec[i];
        //     if(StoredPoint.x !=0 && StoredPoint.y !=0){
        //         float deltaX = StoredPoint.x - ekf.State(0);
        //         float deltaY = StoredPoint.y - ekf.State(1);
        //         double q = pow(deltaX,2) + pow(deltaY,2);

        //         Matrix<float, 2, 1> z_cap_m;
        //         z_cap_m(0) = sqrt(q);
        //         z_cap_m(1) = (atan2(deltaY, deltaX)) - ekf.State(2);
        //         z_cap_m(1) = pi_2_pi(z_cap_m(1));

        //         float dist=10000;

        //         for(int j=0;j<polarCornerPoints.size();j++){
        //             Matrix<float, 2, 1> z;
        //             z(0) = polarCornerPoints[j].distance;
        //             z(1) = polarCornerPoints[j].angle;

        //             CarPoint Stored = {z_cap_m(0)*cos(z_cap_m(1)),z_cap_m(0)*sin(z_cap_m(1))};
        //             CarPoint Observed = {z(0)*cos(z(1)),z(0)*sin(z(1))};

        //             if( dist>pointDistance(Stored,Observed)){
        //                 dist = pointDistance(Stored,Observed);
        //             }
        //         }

        //         distances.push_back(dist);
        //         cout<<"StatePnt = "

        //     }
        // }

        
        // for(int i=0;i<distances.size();i++){
        //     acc+=distances[i];
        // }
        // acc = acc/distances.size();


        return;
    }


ExtendedKalmanFilter runThread(ExtendedKalmanFilter ekf, vector<PolPoint> lidarDataPoints, float& accuracy, vector<CarPoint> carPoints, vector<PolPoint> polarCornerPoints, bool& second, bool firstRun2) {
    cout<<"Running ScanMatch Threads"<<endl;

    float a1 = 0;//0
    float a2 = 0;//4
    float a3 = 0;//4
    float a4 = 0;//8
    float a5 = 0;//8

    float a6 = 0;//12
    float a7 = 0;//12
    float a8 = 0;//16
    float a9 = 0;//16
    float a10 = 0;//20
    float a11 = 0;//20

    float a12 = 0;//24
    float a13 = 0;//24
    float a14 = 0;//28
    float a15 = 0;//28

    float a16 = 0;//32
    float a17 = 0;//32
    float a18 = 0;//36
    float a19 = 0;//36

    vector<PolPoint> v1(lidarDataPoints.begin(), lidarDataPoints.end());
    vector<PolPoint> v2(lidarDataPoints.begin(), lidarDataPoints.end());
    vector<PolPoint> v3(lidarDataPoints.begin(), lidarDataPoints.end());
    vector<PolPoint> v4(lidarDataPoints.begin(), lidarDataPoints.end());
    vector<PolPoint> v5(lidarDataPoints.begin(), lidarDataPoints.end());

    vector<PolPoint> v6(lidarDataPoints.begin(), lidarDataPoints.end());
    vector<PolPoint> v7(lidarDataPoints.begin(), lidarDataPoints.end());
    vector<PolPoint> v8(lidarDataPoints.begin(), lidarDataPoints.end());
    vector<PolPoint> v9(lidarDataPoints.begin(), lidarDataPoints.end());
    vector<PolPoint> v10(lidarDataPoints.begin(), lidarDataPoints.end());
    vector<PolPoint> v11(lidarDataPoints.begin(), lidarDataPoints.end());

    vector<PolPoint> v12(lidarDataPoints.begin(), lidarDataPoints.end());
    vector<PolPoint> v13(lidarDataPoints.begin(), lidarDataPoints.end());
    vector<PolPoint> v14(lidarDataPoints.begin(), lidarDataPoints.end());
    vector<PolPoint> v15(lidarDataPoints.begin(), lidarDataPoints.end());

    vector<PolPoint> v16(lidarDataPoints.begin(), lidarDataPoints.end());
    vector<PolPoint> v17(lidarDataPoints.begin(), lidarDataPoints.end());
    vector<PolPoint> v18(lidarDataPoints.begin(), lidarDataPoints.end());
    vector<PolPoint> v19(lidarDataPoints.begin(), lidarDataPoints.end());

    ExtendedKalmanFilter ekf1 = ekf;
    ExtendedKalmanFilter ekf2 = ekf;
    ExtendedKalmanFilter ekf3 = ekf;
    ExtendedKalmanFilter ekf4 = ekf;
    ExtendedKalmanFilter ekf5 = ekf;

    ExtendedKalmanFilter ekf6 = ekf;
    ExtendedKalmanFilter ekf7 = ekf;
    ExtendedKalmanFilter ekf8 = ekf;
    ExtendedKalmanFilter ekf9 = ekf;
    ExtendedKalmanFilter ekf10 = ekf;
    ExtendedKalmanFilter ekf11 = ekf;

    ExtendedKalmanFilter ekf12 = ekf;
    ExtendedKalmanFilter ekf13 = ekf;
    ExtendedKalmanFilter ekf14 = ekf;
    ExtendedKalmanFilter ekf15 = ekf;

    ExtendedKalmanFilter ekf16 = ekf;
    ExtendedKalmanFilter ekf17 = ekf;
    ExtendedKalmanFilter ekf18 = ekf;
    ExtendedKalmanFilter ekf19 = ekf;

    thread thread1(randomFitting, std::ref(v1), carPoints, polarCornerPoints, std::ref(ekf1), ekf.distance, ekf.w, std::ref(a1));
    thread thread2(randomFitting, std::ref(v2), carPoints, polarCornerPoints, std::ref(ekf2), ekf.distance, ekf.w + 4 * PI / 180, std::ref(a2));
    thread thread3(randomFitting, std::ref(v3), carPoints, polarCornerPoints, std::ref(ekf3), ekf.distance, ekf.w - 4 * PI / 180, std::ref(a3));
    thread thread4(randomFitting, std::ref(v4), carPoints, polarCornerPoints, std::ref(ekf4), ekf.distance, ekf.w + 8 * PI / 180, std::ref(a4));
    thread thread5(randomFitting, std::ref(v5), carPoints, polarCornerPoints, std::ref(ekf5), ekf.distance, ekf.w - 8 * PI / 180, std::ref(a5));

    thread thread6(randomFitting, std::ref(v6), carPoints, polarCornerPoints, std::ref(ekf6), ekf.distance, ekf.w + 12 * PI / 180, std::ref(a6));
    thread thread7(randomFitting, std::ref(v7), carPoints, polarCornerPoints, std::ref(ekf7), ekf.distance, ekf.w - 12 * PI / 180, std::ref(a7));
    thread thread8(randomFitting, std::ref(v8), carPoints, polarCornerPoints, std::ref(ekf8), ekf.distance, ekf.w + 16 * PI / 180, std::ref(a8));
    thread thread9(randomFitting, std::ref(v9), carPoints, polarCornerPoints, std::ref(ekf9), ekf.distance, ekf.w - 16 * PI / 180, std::ref(a9));
    thread thread10(randomFitting, std::ref(v10), carPoints, polarCornerPoints, std::ref(ekf10), ekf.distance, ekf.w + 20 * PI / 180, std::ref(a10));
    thread thread11(randomFitting, std::ref(v11), carPoints, polarCornerPoints, std::ref(ekf11), ekf.distance, ekf.w - 20 * PI / 180, std::ref(a11));

    thread thread12(randomFitting, std::ref(v12), carPoints, polarCornerPoints, std::ref(ekf12), ekf.distance, ekf.w + 24 * PI / 180, std::ref(a12));
    thread thread13(randomFitting, std::ref(v13), carPoints, polarCornerPoints, std::ref(ekf13), ekf.distance, ekf.w - 24 * PI / 180, std::ref(a13));
    thread thread14(randomFitting, std::ref(v14), carPoints, polarCornerPoints, std::ref(ekf14), ekf.distance, ekf.w + 28 * PI / 180, std::ref(a14));
    thread thread15(randomFitting, std::ref(v15), carPoints, polarCornerPoints, std::ref(ekf15), ekf.distance, ekf.w - 28 * PI / 180, std::ref(a15));

    thread thread16(randomFitting, std::ref(v16), carPoints, polarCornerPoints, std::ref(ekf16), ekf.distance, ekf.w + 32 * PI / 180, std::ref(a16));
    thread thread17(randomFitting, std::ref(v17), carPoints, polarCornerPoints, std::ref(ekf17), ekf.distance, ekf.w - 32 * PI / 180, std::ref(a17));
    thread thread18(randomFitting, std::ref(v18), carPoints, polarCornerPoints, std::ref(ekf18), ekf.distance, ekf.w + 36 * PI / 180, std::ref(a18));
    thread thread19(randomFitting, std::ref(v19), carPoints, polarCornerPoints, std::ref(ekf19), ekf.distance, ekf.w - 36 * PI / 180, std::ref(a19));



    thread1.join();
    thread2.join();
    thread3.join();
    thread4.join();
    thread5.join();

    thread6.join();
    thread7.join();
    thread8.join();
    thread9.join();
    thread10.join();
    thread11.join();

    thread12.join();
    thread13.join();
    thread14.join();
    thread15.join();

    thread16.join();
    thread17.join();
    thread18.join();
    thread19.join();

    vector<float> acc_vect{ a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19};
    //float max = *max_element (acc_vect.begin(), acc_vect.end());

    float max = *min_element (acc_vect.begin(), acc_vect.end()); //This is due to us now using average distance

    accuracy = max;

    //cout<<"\n\na1 = "<<a1<<", a2 = "<<a2<<", a3 = "<<a3<<", a4 = "<<a4<<", a5 = "<<a5<<endl;

    cout<<"\n\n";
    cout<<", a"<<0<<" = "<<acc_vect[0]<<endl;
    int c = 1;
    for(int i =1;i<acc_vect.size();i=i+2){
        cout<<", a"<<c*4<<" = "<<acc_vect[i];
        cout<<", a"<<-c*4<<" = "<<acc_vect[i+1];
        c+=1;
        cout<<endl;
    }
    
    if(second == false){
        //Save the 0 scan
        vector<CarPoint> testPoints;
        for(int i =0;i<carPoints.size();i++){
            testPoints.push_back(carPoints[i]);
        }
        //This saves the black odometry reading
        fitCartesian(testPoints,ekf1.State(0),ekf1.State(1),ekf1.State(2));
        saveCarMotionToCSV(testPoints);
        second = true;
    }

    if(firstRun2 == true){
        return ekf1;
    }



    if(max == a1){
        return ekf1;
    }else if(max == a2){
        return ekf2;
    }else if(max == a3){
        return ekf3;
    }else if(max == a4){
        return ekf4;
    }else if(max == a5){
        return ekf5;
    }
    
    else if(max == a6){
        return ekf6;
    }else if(max == a7){
        return ekf7;
    }else if(max == a8){
        return ekf8;
    }else if(max == a9){
        return ekf9;
    }else if(max == a10){
        return ekf10;
    }else if(max == a11){
        return ekf11;
    }

    else if(max == a12){
        return ekf12;
    }else if(max == a13){
        return ekf13;
    }else if(max == a14){
        return ekf14;
    }else if(max == a15){
        return ekf15;
    }

    else if(max == a16){
        return ekf16;
    }else if(max == a17){
        return ekf17;
    }else if(max == a18){
        return ekf18;
    }else if(max == a19){
        return ekf19;
    }





}


void fullRun2(ExtendedKalmanFilter& ekf,bool& mapped, bool& home, bool firstRun, bool firstRun2, bool finalRun,bool postMap,vector<CarPoint>& path, vector<PolPoint> lidarDataPoints){
    

    //Run Lidar
    //vector<PolPoint> lidarDataPoints;//can be replaced with array for speed
    bool error = false;


    if(error == false){

        ExtendedKalmanFilter ekf_old = ekf;

        // if(firstRun == false){
        //     vector<float> accuracy;
        //     cout<<"\n MAIN: b4_thread State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;
        //     //float angle = runThread(ekf, lidarDataPoints,accuracy);
        //     ekf.w = runThread(ekf, lidarDataPoints,accuracy);
        //     cout<<"\n MAIN: afta_thread State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;
        //     // for(int i =0;i<accuracy.size();i++){
        //     //     cout<<"a"<<i+1<<":"<<accuracy[i]<<endl;
        //     // }
        // }


        

    
        //Predict Position
        ekf.updateMotion();
        cout<<"\n MAIN: after_motion State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;


        //Process Data
        vector<CarPoint> carPoints;
        vector<PolPoint> polarCornerPoints;
        lidarDataProcessing2(lidarDataPoints,carPoints,polarCornerPoints);


        // vector<CarPoint> testPoints;

        // for(int i =0;i<carPoints.size();i++){
        //     testPoints.push_back(carPoints[i]);
        // }

        // //This saves the black odometry reading
        // fitCartesian(testPoints,ekf.State(0),ekf.State(1),ekf.State(2));
        // saveCarMotionToCSV(testPoints);

        //Set Corners and do EKF
        ekf.TestPolValues = polarCornerPoints;
        // ekf.runEKF();

        bool second = false;
        cout<<"\n MAIN: b4_thread State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;
        float accuracy = 0;


        ekf = runThread(ekf_old, lidarDataPoints,accuracy,  carPoints, polarCornerPoints,second,firstRun2);
        cout<<"Accuracy = "<<accuracy<<endl;
        cout<<"\n MAIN: afta_thread State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;
        

        
        //5th Corner thread fix
        if(( (ekf.State(11) != 0 && ekf.State(12)!=0) || (noCorners == 1 && ekf.noNewCorners>0) || accuracy > 300) && firstRun2 == false) {
            cout<<"\n5th Corner was added OR noCorners = 1 OR Acc>300 AGAIN assume this is problematic and will be solved with thread tests"<<endl;
            cout<<"\n MAIN: b4_thread State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;

            ExtendedKalmanFilter ekf_1 = ekf;
            float accuracy_1 = accuracy;

            ekf_old.State(2) = ekf_old.State(2) + 82*PI/180;
            cout<<"\n MAIN ThreadFix 2p: b4_thread State: x="<<ekf_old.State[0]<<", y="<<ekf_old.State[1]<<", w="<<ekf_old.State[2]*180/PI<<" deg"<<endl;
            ExtendedKalmanFilter ekf_2p = runThread(ekf_old, lidarDataPoints,accuracy,  carPoints, polarCornerPoints,second,firstRun2);
            float accuracy_2p = accuracy;

            ekf_old.State(2) = ekf_old.State(2) - 2*82*PI/180;
            cout<<"\n MAIN ThreadFix 2n: b4_thread State: x="<<ekf_old.State[0]<<", y="<<ekf_old.State[1]<<", w="<<ekf_old.State[2]*180/PI<<" deg"<<endl;
            ExtendedKalmanFilter ekf_2n = runThread(ekf_old, lidarDataPoints,accuracy,  carPoints, polarCornerPoints,second,firstRun2);
            float accuracy_2n = accuracy;
            

            

            if(accuracy_1<accuracy_2p && accuracy_1<accuracy_2n){
                cout<<"First Thread stage was closer"<<endl;
                ekf = ekf_1;
                accuracy = accuracy_1;
            }else if(accuracy_2p<accuracy_1 && accuracy_2p<accuracy_2n){
                cout<<"2p Thread stage was closer"<<endl;
                ekf = ekf_2p;
                accuracy = accuracy_2p;
            }
            else if(accuracy_2n<accuracy_1 && accuracy_2n<accuracy_2p){
                cout<<"2n Thread stage was closer"<<endl;
                ekf = ekf_2n;
                accuracy = accuracy_2n;
            }


            cout<<"Accuracy = "<<accuracy<<endl;
            cout<<"\n MAIN: afta_thread State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;

        }

        if(noCorners == 1 && ekf.noNewCorners>0){
            cout<<"Resetting Landmarks of EKF"<<endl;
            ekf_old.State(0) = ekf.State(0);
            ekf_old.State(1) = ekf.State(1);
            ekf_old.State(2) = ekf.State(2);
            ekf.State = ekf_old.State;
        }
        

        cout<<"\n MAIN: after_ekf State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;
        for(int i =3;i<dim;i=i+2){
            if(ekf.State[i] != 0 && ekf.State[i+1] != 0){
                cout<<"("<<ekf.State[i]<<","<<ekf.State[i+1]<<") | ";
            }
        }
        cout<<endl;

        //Store Data for plotting
        if(firstRun == true){
            cout<<"MAIN: Save Car To Full Map"<<endl;
            saveCarToFullMapCSV(carPoints);
        }else{
            cout<<"MAIN: Store Map Points"<<endl;
            storeMapPoints(carPoints,ekf.State);
        }

        //Get Grid
        vector<vector<GridPoint>> gridNew;
        gridDataProcess(gridNew, ekf.State, firstRun);
            
        //Complete Robot Movement
        if(firstRun == false){
            if(finalRun == false && postMap == false){
                mapped = mapMovement(ekf.State,gridNew,path);// Move the robot to the location
                motorDataProcessing(ekf.w,ekf.distance);//Set Ekf variables to result from motor functions
            }else if(finalRun == false && postMap == true){
                home = postMapMovement(ekf.State,path,home);// Move the robot to the location
                motorDataProcessing(ekf.w,ekf.distance);//Set Ekf variables to result from motor functions
            }else{
                cout<<"MAIN: I DID NOT FUCKING MOVE"<<endl;
            }
        }else{
            cout<<"FIRST RUN NO MOVEMENT"<<endl;
            //Set motors
            motorControlGrid(0,0);
            motorDataProcessing(ekf.w,ekf.distance);//Set Ekf variables to result from motor functions

        }

    }else{
        cout<<" NO PROCESSING DUE TO LIDAR ERROR"<<endl;
    }
        
    

    cout<<"LEAVNG RUN"<<endl;
    
}


void fullRunClean(ExtendedKalmanFilter& ekf,bool& mapped, bool& home, bool firstRun, bool finalRun,bool postMap,vector<CarPoint>& path, vector<PolPoint> lidarDataPoints){
    

    //Run Lidar
    //vector<PolPoint> lidarDataPoints;//can be replaced with array for speed
    bool error = false;


    if(error == false){
        ExtendedKalmanFilter ekf_old = ekf;
        //Predict Position
        ekf.updateMotion();
        

        cout<<"\n MAIN: after_motion State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;

        //Process Data
        
        vector<CarPoint> carPoints;
        vector<PolPoint> polarCornerPoints;
        lidarDataProcessingCleaning(lidarDataPoints,carPoints,polarCornerPoints);

        ekf.TestPolValues = polarCornerPoints;
        //Run EKF
        ekf.runEKF();



        if((ekf.State(11) != 0 && ekf.State(12)!=0) && firstRun == false){
            bool second = false;
            cout<<"\n5th Corner was added assume this is problematic and will be solved with thread tests"<<endl;
            cout<<"\n MAIN: b4_thread State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;
            float accuracy;
            ekf = runThread(ekf_old, lidarDataPoints,accuracy,  carPoints, polarCornerPoints,second,firstRun);
            cout<<"\n MAIN: afta_thread State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;

        }

        cout<<"\n MAIN: after_ekf State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;
        for(int i =3;i<dim;i=i+2){
            if(ekf.State[i] != 0 && ekf.State[i+1] != 0){
                cout<<"("<<ekf.State[i]<<","<<ekf.State[i+1]<<") | ";
            }
        }
        cout<<endl;


        storeMapPointsCleaning(carPoints,ekf.State);

        //Get Grid
        vector<vector<GridPoint>> gridNew;
        gridDataProcess(gridNew, ekf.State, firstRun);
            
        //Complete Robot Movement
  
        if(finalRun == false && postMap == false){
            mapped = mapMovement(ekf.State,gridNew,path);// Move the robot to the location
            motorDataProcessing(ekf.w,ekf.distance);//Set Ekf variables to result from motor functions
        }else if(finalRun == false && postMap == true){
            home = postMapMovement(ekf.State,path,home);// Move the robot to the location
            motorDataProcessing(ekf.w,ekf.distance);//Set Ekf variables to result from motor functions
        }else{
            cout<<"MAIN: I DID NOT FUCKING MOVE"<<endl;
        }
        

    }else{
        cout<<" NO PROCESSING DUE TO LIDAR ERROR"<<endl;
    }
        
    

    cout<<"LEAVNG RUN"<<endl;
    
}

void testRun(){
    ExtendedKalmanFilter ekf;
    bool mapped = false;
    bool home = false;
    bool firstRun = true;
    bool firstRun2 = true;
    bool finalRun = false;
    bool postMap = false;


    vector<CarPoint> path;
    cout<<"TEST RUN"<<endl;

    ILidarDriver * drv = nullptr;
    sl_result op_result;
    vector<PolPoint> lidarDataPoints;
    int NoPoints = 8192;
    bool error;
    sl_u32 timeout = 3000;

    //calibrateMotors();

    initializeLidar(drv,error);
    cout<<"\n\nTime to do some mapping"<<endl;
    grid_ystep = 500;
    grid_xstep = 500;
    boundThresh = 280;
    cout<<"gridx = "<<grid_xstep<<", gridy = "<<grid_ystep<<", boundThresh = "<<boundThresh<<endl;

    int count = 0;
    while(mapped == false && count<40){
        
        fetchScan(drv, op_result, lidarDataPoints, NoPoints, error, timeout);
        lidarDataPoints.clear();
        fetchScan(drv, op_result, lidarDataPoints, NoPoints, error, timeout);
        if(lidarDataPoints.size()!=0){
            
            cout<<"MAIN: FetchedScan = "<<lidarDataPoints.size()<<endl;
            cout<<"\n i = "<<count<<endl;
            cout<<"------------------------------------------------------------------------------------------------------------\n\n";
            fullRun2(ekf,mapped,home,firstRun,firstRun2,finalRun,postMap,path,lidarDataPoints);
            if(firstRun == false){
                firstRun2 = false;
            }
            firstRun = false; //DO NOT CHANGE THIS KEEP IT HERE DO NOT MOVE IT INSIDE FULLRUN OR GOD HELP ME
            count = count+1;
            
        }else{
            cout<<"MAIN: No Scan Fetched"<<endl;
        }
        lidarDataPoints.clear();

        string s; 
        cout<<"please enter something: ";
        cin>>s;
    }

    ekf.distance = 0;
    ekf.w = 0;


    cout<<"\n\nTime to do some cleaning"<<endl;
    grid_ystep = 300;
    grid_xstep = 300;
    boundThresh = 250;
    cout<<"gridx = "<<grid_xstep<<", gridy = "<<grid_ystep<<", boundThresh = "<<boundThresh<<endl;

    count=0;
    firstRun = true;
    mapped = false;
    while(mapped == false && count<40){
        
        fetchScan(drv, op_result, lidarDataPoints, NoPoints, error, timeout);
        lidarDataPoints.clear();
        fetchScan(drv, op_result, lidarDataPoints, NoPoints, error, timeout);
        if(lidarDataPoints.size()!=0){
            cout<<"MAIN: FetchedScan = "<<lidarDataPoints.size()<<endl;
            cout<<"\n i = "<<count<<endl;
            cout<<"------------------------------------------------------------------------------------------------------------\n\n";
            fullRunClean(ekf,mapped,home,firstRun,finalRun,postMap,path,lidarDataPoints);
            firstRun = false; //DO NOT CHANGE THIS KEEP IT HERE DO NOT MOVE IT INSIDE FULLRUN OR GOD HELP ME
            count = count+1;
        }else{
            cout<<"MAIN: No Scan Fetched"<<endl;
        }
        lidarDataPoints.clear();

        string s; 
        cout<<"please enter something: ";
        cin>>s;
    }

    ekf.distance = 0;
    ekf.w = 0;


    //Time to go home
    cout<<"\n\nTime to go home"<<endl;
    mapped = false;
    postMap = true;
    while(mapped == false && count<40){
        
        fetchScan(drv, op_result, lidarDataPoints, NoPoints, error, timeout);
        lidarDataPoints.clear();
        fetchScan(drv, op_result, lidarDataPoints, NoPoints, error, timeout);
        if(lidarDataPoints.size()!=0){
            cout<<"MAIN: FetchedScan = "<<lidarDataPoints.size()<<endl;
            cout<<"\n i = "<<count<<endl;
            cout<<"------------------------------------------------------------------------------------------------------------\n\n";
            fullRunClean(ekf,mapped,home,firstRun,finalRun,postMap,path,lidarDataPoints);
            firstRun = false; //DO NOT CHANGE THIS KEEP IT HERE DO NOT MOVE IT INSIDE FULLRUN OR GOD HELP ME
            count = count+1;
        }else{
            cout<<"MAIN: No Scan Fetched"<<endl;
        }
        lidarDataPoints.clear();

        string s; 
        cout<<"please enter something: ";
        cin>>s;
    }

    fullRunClean(ekf,mapped,home,firstRun,finalRun,postMap,path,lidarDataPoints);
    
    // fetchScan(drv, op_result, lidarDataPoints, NoPoints, error, timeout);
    // lidarDataPoints.clear();
    // fetchScan(drv, op_result, lidarDataPoints, NoPoints, error, timeout);
    // cout<<"POST LOOP RUN"<<endl;
    // finalRun = true;
    // fullRun2(ekf,mapped,home,firstRun,finalRun,postMap,path,lidarDataPoints);
    // lidarDataPoints.clear();
        

    // cout<<"Fully Mapped Room"<<endl;

    // finalRun = false;
    // postMap = true;
    // count = 0;
    // path.clear();
    // while(home == false){
    //     fetchScan(drv, op_result, lidarDataPoints, NoPoints, error, timeout);
    //     cout<<"\n i = "<<count<<endl;
    //     cout<<"------------------------------------------------------------------------------------------------------------\n\n";
    //     fullRun2(ekf,mapped,home,firstRun,finalRun,postMap,path,lidarDataPoints);
    // }

    // // ekf.distance = 0;
    // // ekf.w = 0;
    
    // finalRun = true;
    // fetchScan(drv, op_result, lidarDataPoints, NoPoints, error, timeout);

    // cout<<"I am Home"<<endl;

    delete drv;
    drv = NULL;

}


float RANSACstats(vector<vector<CarPoint>> CornerList, CarPoint TestCorner){
    float dist_avg = 0;
    float max_dist = -100;
    float min_dist = 10000;

    for(int i = 0;i<CornerList.size();i++){
        float dist = 100000;
        for(int j =0;j<CornerList[i].size();j++){
            if(pointDistance(TestCorner,CornerList[i][j]) < dist ){
                dist = pointDistance(TestCorner,CornerList[i][j]);
            }
        }

        dist_avg+=dist;

        if(dist > max_dist){
            max_dist = dist;
        }
        if(dist < min_dist){
            min_dist = dist;
        }

    }

    cout<<"Average Dist Deviation = "<<dist_avg/CornerList.size()<<endl;
    cout<<"Max Dist = "<<max_dist<<endl;
    cout<<"Min Dist = "<<min_dist<<endl;

    return dist_avg/CornerList.size();
}





float testRANSAC(vector<CarPoint> dataPoints){
    vector<vector<CarPoint>> CornerList;
    vector<CarPoint> FiltCarCorners;
    cout<<"A dataLen = "<<dataPoints.size()<<endl;

    vector<float> y;
    vector<float> x;

    for(int i =0;i<dataPoints.size();i++){
        x.push_back(dataPoints[i].x);
        y.push_back(dataPoints[i].y);
    }

    cout<<"B x size = "<<x.size()<<endl;
    int sample_size = 80;
    int max_iters= 600;
    float inlier_thresh=0.002; //0.0005
    int min_inlier = 5; // 5
    float angleThreshold = 20.0 * M_PI / 180.0;
    float distanceThreshold = 120;
    float closenessThreshold = 40;
    
    
    for(int i =0;i<100;i++){
        vector<VectorXd> bestModels = manager2(x, y, sample_size, max_iters, inlier_thresh, min_inlier);
        vector<Vector2d> corners = findCorners2(bestModels, angleThreshold);
        vector<Vector2d> filteredCorners = filterCorners2(corners, x, y, distanceThreshold, closenessThreshold);

        
        FiltCarCorners.clear();
        for (int i =0;i<filteredCorners.size();i++) {
            // Access the values inside the VectorXf
            CarPoint lm;
            lm.x = filteredCorners[i][0];
            lm.y = filteredCorners[i][1];
            FiltCarCorners.push_back(lm);
        }

        //cout<<i<<" Corners Found = "<<FiltCarCorners.size()<<endl;
        CornerList.push_back(FiltCarCorners);
    }


    CarPoint TestCorner = CornerList[0][0];
    float dist_avg = RANSACstats(CornerList,TestCorner);
    // TestCorner = CornerList[0][1];
    // dist_avg += RANSACstats(CornerList,TestCorner);


    cout<<"Overall Dist Average = "<<dist_avg<<endl;;


    cout<<"Filtered Corners = "<<endl;
    for(int i =0;i<FiltCarCorners.size();i++){
        cout<<FiltCarCorners[i]<<endl;
    }
    cout<<endl;
    cout<<"No Corners = "<<FiltCarCorners.size()<<endl;

    return dist_avg;
}

void testRANSACallMaps(){
    float mapError = 0;
    vector<CarPoint> dataPoints;

    cout<<"map9"<<endl;
    readCarFromCSVTest(dataPoints, "map9.csv");
    mapError += testRANSAC(dataPoints);
    dataPoints.clear();

    cout<<"map8"<<endl;
    readCarFromCSVTest(dataPoints, "map8.csv");
    mapError += testRANSAC(dataPoints);
    dataPoints.clear();

    cout<<"map7"<<endl;
    readCarFromCSVTest(dataPoints, "map7.csv");
    mapError += testRANSAC(dataPoints);
    dataPoints.clear();

    cout<<"map6"<<endl;
    readCarFromCSVTest(dataPoints, "map6.csv");
    mapError += testRANSAC(dataPoints);
    dataPoints.clear();

    cout<<"map5"<<endl;
    readCarFromCSVTest(dataPoints, "map5.csv");
    mapError += testRANSAC(dataPoints);
    dataPoints.clear();

    cout<<"map4"<<endl;
    readCarFromCSVTest(dataPoints, "map4.csv");
    mapError += testRANSAC(dataPoints);
    dataPoints.clear();

    cout<<"map3"<<endl;
    readCarFromCSVTest(dataPoints, "map3.csv");
    mapError += testRANSAC(dataPoints);
    dataPoints.clear();

    cout<<"map2"<<endl;
    readCarFromCSVTest(dataPoints, "map2.csv");
    mapError += testRANSAC(dataPoints);
    dataPoints.clear();

    cout<<"map1"<<endl;
    readCarFromCSVTest(dataPoints, "map1.csv");
    mapError += testRANSAC(dataPoints);
    dataPoints.clear();

    mapError = mapError/9;
    cout<<"Map Error = "<<mapError<<endl;
}


void testRANSAC2(){
    vector<CarPoint> dataPoints;

    cout<<"map10"<<endl;
    readCarFromCSVTest(dataPoints, "map10.csv");
    getRANSACCorners(dataPoints);
    //dataPoints.clear();

}


void testLM(){
    bool firstRun = true;
    int noRuns = 8;
    for(int i =0;i<noRuns;i++){
        testLandmarkIdentification(firstRun);
    }
}

int main() {
    //cout<<"Started in Main"<<endl;
    
    //testPython();

    //testLM();

    //testMotor();
    
    //testLidar();

    testRun();

    //testRANSAC2();

    //testRANSACallMaps();
    

    //simRun3();

    //testLidarLandmark();
    //simRun5();
    
    //atSim();

    //testRun();

    //testMap();

    //testThread();

    //testLidarThread();

  
    return 0;
}




// WELCOME TO THE GRAVEYARD







// void fullRunYes(ExtendedKalmanFilter& ekf,bool& mapped, bool& home, bool firstRun, bool finalRun,bool postMap,vector<CarPoint>& path, vector<PolPoint> lidarDataPoints){
    

//     //Run Lidar
//     //vector<PolPoint> lidarDataPoints;//can be replaced with array for speed
//     bool error = false;


//     if(error == false){

//         ExtendedKalmanFilter ekf_old = ekf;

//         // if(firstRun == false){
//         //     vector<float> accuracy;
//         //     cout<<"\n MAIN: b4_thread State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;
//         //     //float angle = runThread(ekf, lidarDataPoints,accuracy);
//         //     ekf.w = runThread(ekf, lidarDataPoints,accuracy);
//         //     cout<<"\n MAIN: afta_thread State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;
//         //     // for(int i =0;i<accuracy.size();i++){
//         //     //     cout<<"a"<<i+1<<":"<<accuracy[i]<<endl;
//         //     // }
//         // }


        

    
//         //Predict Position
//         ekf.updateMotion();
//         cout<<"\n MAIN: after_motion State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;


//         //Process Data
//         vector<CarPoint> carPoints;
//         vector<PolPoint> polarCornerPoints;
//         lidarDataProcessing2(lidarDataPoints,carPoints,polarCornerPoints);


//         vector<CarPoint> testPoints;

//         for(int i =0;i<carPoints.size();i++){
//             testPoints.push_back(carPoints[i]);
//         }

//         //This saves the black odometry reading
//         fitCartesian(testPoints,ekf.State(0),ekf.State(1),ekf.State(2));
//         saveCarMotionToCSV(testPoints);

//         //Set Corners and do EKF
//         ekf.TestPolValues = polarCornerPoints;
//         ekf.runEKF();

//         //5th Corner thread fix
//         if((ekf.State(11) != 0 && ekf.State(12)!=0) || (noCorners == 1 && ekf.noNewCorners>0){
//             cout<<"\n5th Corner was added OR noCorners = 1 assume this is problematic and will be solved with thread tests"<<endl;
//             cout<<"\n MAIN: b4_thread State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;
//             vector<float> accuracy;
//             ekf = runThread(ekf_old, lidarDataPoints,accuracy,  carPoints, polarCornerPoints);
//             cout<<"\n MAIN: afta_thread State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;

//         }

//         // if((ekf.State(11) != 0 && ekf.State(12)!=0) || (noCorners == 1 && ekf.noNewCorners>0)){
//         //     cout<<"\n5th Corner was added OR noCorners = 1 AGAIN assume this is problematic and will be solved with thread tests"<<endl;
//         //     cout<<"\n MAIN: b4_thread State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;
            
//         //     if(ekf_old.State(2)<0){
//         //         ekf_old.State(2) = ekf_old.State(2) - PI/2;
//         //     }else{
//         //         ekf_old.State(2) = ekf_old.State(2) + PI/2;
//         //     }
            
//         //     vector<float> accuracy;
//         //     ekf = runThread(ekf_old, lidarDataPoints,accuracy,  carPoints, polarCornerPoints);
//         //     cout<<"\n MAIN: afta_thread State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;

//         // }

//         // if(noCorners == 1 && ekf.noNewCorners>0){
//         //     cout<<"Resetting Landmarks of EKF"<<endl;
//         //     ekf_old.State(0) = ekf.State(0);
//         //     ekf_old.State(1) = ekf.State(1);
//         //     ekf_old.State(2) = ekf.State(2);
//         //     ekf.State = ekf_old.State;
//         // }
        

//         cout<<"\n MAIN: after_ekf State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;
//         for(int i =3;i<dim;i=i+2){
//             if(ekf.State[i] != 0 && ekf.State[i+1] != 0){
//                 cout<<"("<<ekf.State[i]<<","<<ekf.State[i+1]<<") | ";
//             }
//         }
//         cout<<endl;

//         //Store Data for plotting
//         if(firstRun == true){
//             cout<<"MAIN: Save Car To Full Map"<<endl;
//             saveCarToFullMapCSV(carPoints);
//         }else{
//             cout<<"MAIN: Store Map Points"<<endl;
//             storeMapPoints(carPoints,ekf.State);
//         }

//         //Get Grid
//         vector<vector<GridPoint>> gridNew;
//         gridDataProcess(gridNew, ekf.State, firstRun);
            
//         //Complete Robot Movement
//         if(firstRun == false){
//             if(finalRun == false && postMap == false){
//                 mapped = mapMovement(ekf.State,gridNew,path);// Move the robot to the location
//                 motorDataProcessing(ekf.w,ekf.distance);//Set Ekf variables to result from motor functions
//             }else if(finalRun == false && postMap == true){
//                 home = postMapMovement(ekf.State,path,home);// Move the robot to the location
//                 motorDataProcessing(ekf.w,ekf.distance);//Set Ekf variables to result from motor functions
//             }else{
//                 cout<<"MAIN: I DID NOT FUCKING MOVE"<<endl;
//             }
//         }else{
//             cout<<"FIRST RUN NO MOVEMENT"<<endl;
//             //Set motors
//             motorControlGrid(0,0);
//             motorDataProcessing(ekf.w,ekf.distance);//Set Ekf variables to result from motor functions

//         }

//     }else{
//         cout<<" NO PROCESSING DUE TO LIDAR ERROR"<<endl;
//     }
        
    

//     cout<<"LEAVNG RUN"<<endl;
    
// }