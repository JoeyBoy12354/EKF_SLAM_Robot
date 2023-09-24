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

    writeMotorToCSV(angle,distance);
    
    int ret;
    ret = system("python3 motorControl.py ok go");
    cout << "ret/cpp = " << ret << endl;

    cout<<"Set angle = "<<angle*180/(PI)<<" Set Distance = "<<distance<<endl;
    readMotorFromCSV(angle,distance);
    cout<<"Read angle = "<<angle<<" Read Distance = "<<distance<<endl;

}


//This process will only use the latest scan to update the EKF and RANSAC
void fullRun(ExtendedKalmanFilter ekf,bool& mapped, bool& firstRun){
    
    
    if(mapped==false){
        //Run Lidar
        vector<PolPoint> lidarDataPoints;//can be replaced with array for speed
        //runLidar(vector<PolPoint>& lidarDataPoints);
        runLidar(lidarDataPoints);
        

        cout<<"Main: Lidar Run complete"<<endl;

        //Process Data
        vector<CarPoint> carPoints;
        //lidarDataProcessing(vector<PolPoint> dataPoints, vector<CarPoint>& carPoints)
        lidarDataProcessing(lidarDataPoints,carPoints);

        //Run EKF
        ekf.runEKF();

        //Store Data for plotting
        if(firstRun == true){
            saveCarToFullMapCSV(carPoints);
            firstRun = false;
        }else{
            storeMapPoints(carPoints);
            storeStatePoints(ekf.State);
        }
            

        //Complete Robot Movement
        mapped = updateMovement(ekf.State);// Move the robot to the location
        motorDataProcessing(ekf.w,ekf.distance);//Send odometry to ekf

        cout<<"Main: ekf.w = "<<ekf.w<<" ekf.distance = "<<ekf.distance<<endl;
        
    }else{
        cout<<"MAP COMPLETED !"<<endl;
    }

    cout<<"LEAVNG FULL RUN"<<endl;
    
}



//This process will use the full map with all historic values to update the EKF and RANSAC
void fullRunfullLandmark(ExtendedKalmanFilter ekf,bool& mapped, bool& firstRun){
    
    
    if(mapped==false){
        //Run Lidar
        vector<PolPoint> lidarDataPoints;//can be replaced with array for speed
        //runLidar(vector<PolPoint>& lidarDataPoints);
        runLidar(lidarDataPoints);
        

        cout<<"Main: Lidar Run complete"<<endl;
        

        //Process Data
        vector<CarPoint> carPoints;
        lidarDataProcessingFull(lidarDataPoints,carPoints,firstRun);

        //Run EKF (Note this means that graph will updat i-1 robot positions)
        ekf.runEKF();

        storeStatePoints(ekf.State);

        //Complete Robot Movement
        mapped = updateMovement(ekf.State);// Move the robot to the location
        motorDataProcessing(ekf.w,ekf.distance);//Send odometry to ekf

        cout<<"Main: ekf.w = "<<ekf.w<<" ekf.distance = "<<ekf.distance<<endl;
        
    }else{
        cout<<"MAP COMPLETED !"<<endl;
    }

    cout<<"LEAVNG FULL RUN"<<endl;
    
}



void testRun(){
    ExtendedKalmanFilter ekf;
    bool mapped = false;
    bool firstRun = true;
    
    for(int i =0;i<2;i++){
        cout<<"IN RUN LOOP: "<<i<<endl;
        cout<<"Mapped = "<<mapped<<endl;
        fullRunfullLandmark(ekf,mapped,firstRun);
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










