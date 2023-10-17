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
        storeMapPoints(carPoints);
    }

    LandmarkProcessing2(carPoints);
}

void testLidarLandmark(){
    vector<PolPoint> lidarDataPoints;//can be replaced with array for speed
    bool error = true;
    runLidar(lidarDataPoints,error);
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
        cout<<"\n MAIN: after_motion State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;
        //cout << "\nEKF 6\nState =\n" << ekf.State << "\n";
 
    }
    
    //Run EKF
    ekf.runEKF();

    cout<<"MAIN: after_ekf State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;

    // for(int i =3;i<dim;i=i+2){
    //     if(ekf.State[i] != 0 && ekf.State[i+1] != 0){
    //         cout<<"("<<ekf.State[i]<<","<<ekf.State[i+1]<<") | ";
    //     }
    // }
    cout<<endl;
    

    
}



//This process will only use the latest scan to update the EKF and RANSAC
void fullRun(ExtendedKalmanFilter& ekf,bool& mapped, bool& firstRun, int finalRun){
    
    
    if(mapped==false){

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
        
        //cout<<"Main: Lidar Run complete"<<endl;

        if(error == false){
            //Predict Position
            if(finalRun <= 1){
                ekf.updateMotion();
            }

            cout<<"\n MAIN: after_motion State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;
            //cout << "\nEKF 6\nState =\n" << ekf.State << "\n";

            //Process Data
            vector<CarPoint> carPoints;
            lidarDataProcessing(lidarDataPoints,carPoints,ekf.State[0],ekf.State[1],ekf.State[2]);

            
            //Run EKF
            ekf.runEKF();

            cout<<"\n MAIN: after_ekf State: x="<<ekf.State[0]<<", y="<<ekf.State[1]<<", w="<<ekf.State[2]*180/PI<<" deg"<<endl;

            for(int i =3;i<dim;i=i+2){
                if(ekf.State[i] != 0 && ekf.State[i+1] != 0){
                    cout<<"("<<ekf.State[i]<<","<<ekf.State[i+1]<<") | ";
                }
            }
            cout<<endl;

            //Store Data for plotting
            if(firstRun == true){
                saveCarToFullMapCSV(carPoints);
                firstRun = false;
            }else{
                storeMapPoints(carPoints);
                storeStatePoints(ekf.State);
            }

            //Get Grid
            vector<vector<GridPoint>> gridNew;
            gridDataProcess(gridNew, ekf.State, firstRun);
                

            //Complete Robot Movement
            // mapped = updateMovement(ekf.State);// Move the robot to the location
            //motorDataProcessing(ekf.w,ekf.distance);//Send odometry to ekf
            if(finalRun == 0){
                mapped = updateMovementGrid(ekf.State,gridNew,ekf.lidar_x,ekf.lidar_y);// Move the robot to the location
                motorDataProcessing(ekf.w,ekf.distance);
            }
            
            //motorControlGrid(ekf.w,ekf.distance);//Send odometry to ekf


            //cout<<"\nMain_end: ekf.w = "<<ekf.w<<" ekf.distance = "<<ekf.distance<<endl;
    
        }else{
            cout<<" NO PROCESSING DUE TO LIDAR ERROR"<<endl;
        }
        
    }else{
        cout<<"MAP COMPLETED !"<<endl;
    }
    

    cout<<"LEAVNG RUN"<<endl;
    
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

    ekf.TestValues.clear();
    ekf.TestValues.push_back({915.606,424.298});
    ekf.TestValues.push_back({-821.015,1146.44});
    ekf.TestValues.push_back({704.389,-872.277});
    ekf.TestValues.push_back({948.087,644.084});
    ekf.TestValues.push_back({-1041.74,-459.041});
    simRun(ekf,true);

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
    float w = -10*(PI/180);
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

    state.push_back(ekf.State(0));
    state.push_back(ekf.State(1));
    state.push_back(ekf.State(2));
    states.push_back(state);

    cout<<"-----------------------------------------"<<endl;

    for(int i =0;i<1000;i++){
        ekf.TestValues.clear();
        ekf.TestValues.push_back(LM1);
        ekf.TestValues.push_back(LM2);
        ekf.TestValues.push_back(LM3);
        simRun(ekf,true);

        state.clear();
        state.push_back(ekf.State(0));
        state.push_back(ekf.State(1));
        state.push_back(ekf.State(2));
        states.push_back(state);
    }


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

    cout<<"stddev x:"<<stddev[0]/states.size()<<" y:"<<stddev[1]/states.size()<<" theta:"<<(stddev[2]/states.size())*180/PI<<endl;
    cout<<"avg x:"<<avg[0]/states.size()<<" y:"<<avg[1]/states.size()<<" theta:"<<(avg[2]/states.size())*180/PI<<endl;
    cout<<"gt x:"<<groundtruth[0]<<"  gt y:"<<groundtruth[1]<<"  gt theta:"<<groundtruth[2]*180/PI<<endl;

    return;
}



void testRun(){
    ExtendedKalmanFilter ekf;
    bool mapped = false;
    bool firstRun = true;
    int finalRun = 0;

    calibrateMotors();
    
    for(int i =0;i<3;i++){
        cout<<"\n i = "<<i<<endl;
        cout<<"------------------------------------------------------------------------------------------------------------\n\n";
        // cout<<"IN RUN LOOP: "<<i<<endl;
        // cout<<"Mapped = "<<mapped<<endl;
        fullRun(ekf,mapped,firstRun,finalRun);
        
        firstRun = false;
    }

    finalRun = 1;
    for(int i =0;i<4;i++){
        cout<<"\n i = "<<"FINALRUN "<<i<<endl;
        cout<<"------------------------------------------------------------------------------------------------------------\n\n";
        fullRun(ekf,mapped,firstRun,finalRun);
        ekf.distance = 0;
        ekf.w = 0;
        finalRun = 2;
    }
    
    
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

    //testRun();
    
    simRun5();
  
    return 0;
}




// WELCOME TO THE GRAVEYARD



//This process will use the full map with all historic values to update the EKF and RANSAC
// void fullRunfullLandmark(ExtendedKalmanFilter& ekf,bool& mapped, bool& firstRun, bool& calibration){

    
//     if(mapped==false){
//         //Run Lidar
//         vector<PolPoint> lidarDataPoints;//can be replaced with array for speed
//         runLidar(lidarDataPoints);
//         cout<<"Main: Lidar Run complete"<<endl;
        
//         //Process Data
//         vector<CarPoint> carPoints;
//         lidarDataProcessingFull(lidarDataPoints,carPoints,firstRun);


//         //Run EKF (Note this means that graph will updat i-1 robot positions)
//         ekf.runEKF();

//         cout << "\nMAIN: EKF\nmu =\n" << ekf.State << "\n";

//         storeStatePoints(ekf.State);

//         //Complete Robot Movement
//         mapped = updateMovement(ekf.State);// Move the robot to the location
//         motorDataProcessing(ekf.w,ekf.distance);//Send odometry to ekf

//         cout<<"Main: ekf.w = "<<ekf.w<<" ekf.distance = "<<ekf.distance<<endl;
        
//     }else{
//         cout<<"MAP COMPLETED !"<<endl;
//     }

//     cout<<"LEAVNG FULL RUN FULL LANDMARK"<<endl;
    
// }





// if(calibration==false){
//         cout<<"MAIN: Calibration"<<endl;
//         //Since it is the first run we need direction calibration
//         ExtendedKalmanFilter ekf1;
//         ExtendedKalmanFilter ekf2;
//         ExtendedKalmanFilter ekf3;
//         float caliDistance = 20; //mm
//         float caliThreshold = caliDistance; //mm

        
//         //Get Lidar Reading 1
//         vector<PolPoint> lidarDataPoints;//can be replaced with array for speed
//         runLidar(lidarDataPoints);
//         cout<<"Main Cali: Lidar Run complete"<<endl;
//         vector<CarPoint> carPoints;
//         lidarDataProcessingCali(lidarDataPoints,carPoints);
//         ekf1.distance = 0; 
//         ekf1.w = 0;
//         ekf1.runEKF();

//         //Move Robot
//         moveCalibration(caliDistance);

//         //Get Lidar Reading 2
//         lidarDataPoints.clear();//can be replaced with array for speed
//         runLidar(lidarDataPoints);
//         cout<<"Main Cali: Lidar Run complete"<<endl;
//         carPoints.clear();
//         lidarDataProcessingCali(lidarDataPoints,carPoints);
//         ekf2.distance = caliDistance; 
//         ekf2.w = 0;
//         ekf2.runEKF();

//         float caliAngle1;
//         getCaliAngle(ekf1.State,ekf2.State,caliDistance,caliAngle1);

//         //Move Robot
//         moveCalibration(caliDistance);

//         //Get Lidar Reading 2
//         lidarDataPoints.clear();//can be replaced with array for speed
//         runLidar(lidarDataPoints);
//         cout<<"Main Cali: Lidar Run complete"<<endl;
//         carPoints.clear();
//         lidarDataProcessingCali(lidarDataPoints,carPoints);
//         ekf3.distance = caliDistance; 
//         ekf3.w = 0;
//         ekf3.runEKF();

//         float caliAngle2;
//         getCaliAngle(ekf2.State,ekf3.State,caliDistance,caliAngle2);

        

//         float caliAngle = (caliAngle1+caliAngle2)/2;

//         cout<<"\n !! caliAngle1 = "<<caliAngle1<<"rads "<<caliAngle1*180/PI<<"deg"<<endl;
//         cout<<"!! caliAngle2 = "<<caliAngle2<<"rads "<<caliAngle2*180/PI<<"deg"<<endl;
//         cout<<"!! caliAngle_avg = "<<caliAngle<<"rads "<<caliAngle*180/PI<<"deg"<<endl<<endl;
//         ekf.w = caliAngle;
//         calibration = true;

//     }



