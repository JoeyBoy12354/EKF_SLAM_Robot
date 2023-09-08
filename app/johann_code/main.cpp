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




//g++ -g main.cpp Data_Functions.cpp  CSV_Functions.cpp EKF_Functions.cpp Landmark_Functions.cpp Simulation_Functions.cpp -o prog2

void testEKF(){
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
    ekf.t = 0.1;

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
    runLidar(lidarDataPoints);
}

void testLandmarkIdentification(){
    LandmarkProcessing();
}

void testLidarLandmark(){
    vector<PolPoint> lidarDataPoints;//can be replaced with array for speed
    runLidar(lidarDataPoints);
    int ret;
    ret = system("python3 CSV_Files/motorControl.py ok go");
    cout << "ret/cpp = " << ret << endl;
}

void testPython(){
    int ret;
    ret = system("python3 motorControl.py ok go");

    cout << "ret/cpp = " << ret << endl;

    return;
}

void testMotor(){
    float angle = -PI/2;
    float distance = 0;
    float time;

    writeMotorToCSV(angle,distance);
    
    int ret;
    ret = system("python3 motorControl.py ok go");
    cout << "ret/cpp = " << ret << endl;

    cout<<"Set angle = "<<angle*180/(PI)<<" Set Distance = "<<distance<<endl;
    readMotorFromCSV(angle,distance,time);
    cout<<"Read angle = "<<angle<<" Read Distance = "<<distance<<" Time = "<<time<<endl;

}



void fullRun(ExtendedKalmanFilter ekf){
    bool mapped = false;
    bool firstRun = true;
    
    if(mapped == false){
        //Run Lidar
        vector<PolPoint> lidarDataPoints;//can be replaced with array for speed
        runLidar(lidarDataPoints);
        

        cout<<"Main: Lidar Run complete"<<endl;

        //Process Data
        lidarDataProcessing(lidarDataPoints);

        //Run EKF
        ekf.runEKF();

        //Store Data for plotting
        if(firstRun == true){
            saveCarToFullMapCSV(lidarDataPoints);
            firstRun = false;
        }else{
            StoreMapAndStatePoints(lidarDataPoints,ekf.Data);
        }
            

        //Complete Robot Movement
        mapped = updateMovement(ekf.State);// Move the robot to the location
        motorDataProcessing(ekf.w,ekf.v,ekf.t);//Send odometry to ekf

        cout<<"Main: ekf.w = "<<ekf.w<<" ekf.v = "<<ekf.v<<" ekf.t = "<<ekf.t<<endl;
        
    }else{
        cout<<"MAP COMPLETED";
    }
    
}

void testRun(){
    ExtendedKalmanFilter ekf;
    for(int i =0;i<2;i++){
        fullRun(ekf);
    }
    
}




int main() {
    cout<<"Started in Main"<<endl;
    
    //testPython();
    //testLandmarkIdentification();
    //testMotor();
    //testLidar();

    testRun();
    
    
  
    return 0;
}









