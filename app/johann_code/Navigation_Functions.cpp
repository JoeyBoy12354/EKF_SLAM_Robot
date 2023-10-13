#include "robot.h"

using namespace Landmark_Functions;
using namespace CSV_Functions;
using namespace Data_Functions;


int noNavTrials = 0;
vector<bool> LandmarksExplored;
double foundRadius = 250;//Robot must be within Xmm of LM for LM to be considered Explored
double closeness = 200;//Robot should travel distance: (Robot->LM) - closeness mm

float theta;
float dist;

namespace Navigation_Functions{
    //Determine Direction and send control signals to motors
    bool updateMovement(MatrixXf State){

        CarPoint Robot;
        Robot.x = State(0);
        Robot.y = State(1);

        updateExplorations(State,Robot);

        if(noNavTrials<2){
            for(int i = 0; i<LandmarksExplored.size();i++){
                if(LandmarksExplored[i] != true){
                    CarPoint LM;
                    LM.x = State(i+3);
                    LM.y = State(i*2+4);

                    landmarkExplore(LM, State);
                    motorControl();
                    noNavTrials = 0;

                    //SHOULD BE TRUE PLEASE CHANGE 
                    return false;
                }
            }

            noNavTrials++;
            randomExplore();
            motorControl();

            //SHOULD BE TRUE PLEASE CHANGE 
            return false;
        }else{
            cout<<"MAP EXPLORED SET DIST = 100, theta = 0"<<endl;
            dist = 100;
            theta = 0;
            //mapped room
            cout<<"\nNAVI: EXPLORED MAP \n"<<endl;
            
            return true;
        }
    }


    void motorControl(){

        cout<<"!!!!!!!!!!!!!! !!!!!!!!! TESTING CONST theta & DIST"<<endl;
        

        cout<<"Navi: Set angle = "<<theta*180/(PI)<<" deg Set Distance = "<<dist<<"mm"<<endl;
        theta = 0;
        dist = 0;


        //Send to motors
        writeMotorToCSV(theta,dist);
        cout<<"Navi test: Set angle = "<<theta*180/(PI)<<" deg Set Distance = "<<dist<<"mm"<<endl;
    
        int ret;
        ret = system("python3 motorControl.py ok go");
        cout << "ret/cpp = " << ret << endl;

        

    }

    //Set velocity and direction to a new landmark
    void landmarkExplore(CarPoint LM, MatrixXf State){
        cout<<"NAVI: Landmark Explore "<<LM.x<<", "<<LM.y<<endl;
        cout<<"NAVI: State Position "<<State(0)<<", "<<State(1)<<State(2)*180/PI<<endl;
        //Calculate required distance and rotation
        float deltaX = LM.x - State(0);
        float deltaY = LM.y - State(1);
        dist = deltaX*deltaX + deltaY*deltaY;
        dist = sqrt(dist) - closeness;
        theta = atan2(deltaY,deltaX) - State(2);

        //FIX
        //theta = theta;

        //Ensure distance > 0 
        if(dist<0){
            dist = 0;
        }
            
        // Normalize theta to the range [0, 2π]
        while (theta < -2 * PI ) {
            theta += 2 * PI;  // Add 2π until theta becomes more than -2π
        }
        while (theta >= 2 * PI) {
            theta -= 2 * PI;  // Subtract 2π until theta becomes less than 2π
        }
       
        
        return;
    }

    //Set velocity and direction randomly
    void randomExplore(){
        cout<<"NAVI: Random Explore "<<endl;
        dist = rand() % 300;
        theta = rand() % 2*PI;
        
        
        return;

    }

    //Check if we have explored any new landmarks
    void updateExplorations(MatrixXf State, CarPoint Robot){
        int NoLandmarks = (State.rows() - 3)/2;

        //Check if new landmarks have been found and add them if so
        for(int i=LandmarksExplored.size(); i<NoLandmarks; i++){
            LandmarksExplored.push_back(false);
        }
        

        
        CarPoint LM; 
        
        for(int i=0; i<NoLandmarks; i++){
            LM.x = State(i+3);
            LM.y = State(i*2+4);

            //if distance between self and landmark < Xmm it is explored.
            if(pointDistance(Robot, LM) < foundRadius){
                LandmarksExplored[i] = true;
            }
        }

        
    }





    //Set motors and start them
    void motorControlGrid(float angle, float distance){

        // cout<<"!!!!!!!!!!!!!! !!!!!!!!! TESTING CONST theta & DIST"<<endl;

        cout<<"Navi: Set angle = "<<angle*180/(PI)<<" deg Set Distance = "<<distance<<"mm"<<endl;
        // angle = 0;
        // distance = 0;


        //Send to motors
        writeMotorToCSV(angle,distance);

        cout<<"NAVI: Run python"<<endl;
    
        int ret;
        ret = system("python3 motorControl.py ok go");
        cout << "ret/cpp = " << ret << endl;

        

    }

    //Set distance and angle to go to nearest unexplored grid point
    bool updateMovementGrid(MatrixXf State, vector<vector<GridPoint>> gridMap){
        cout<<"IN update Movement Grid"<<endl;
        //take grid map
        bool mapped = true;
        float smallDistance = 10000000;
        CarPoint closestPoint;
        CarPoint robotPoint;
        robotPoint.x = State(0);
        robotPoint.y = State(1);

        cout<<"A"<<endl;

        //find closest non-traversed point
        for(int i =0;i<gridMap.size();i++){
            for(int j=0;j<gridMap[i].size();j++){
                CarPoint tempPoint;
                tempPoint.x = gridMap[i][j].x;
                tempPoint.y = gridMap[i][j].y;

                //Only check points that have not been traversed
                if(gridMap[i][j].trav == false){
                    mapped = false;
                    float temp_dist = pointDistance(tempPoint,robotPoint);
                    if(temp_dist<smallDistance){
                        smallDistance = temp_dist;
                        closestPoint = tempPoint;
                    }
                }  
            }
        }

        
        //Set destination
        float deltaX = closestPoint.x - State(0);
        float deltaY = closestPoint.y - State(1);
        float distance = deltaX*deltaX + deltaY*deltaY;
        distance = sqrt(distance) - closeness;
        float angle = atan2(deltaY,deltaX) - State(2);

        cout<<"GRID: dot to visit: "<<closestPoint<<endl;
        cout<<"GRID: movement: "<<distance<<"mm "<<angle*180/PI<<" deg"<<endl;

        //Set motors
        if(mapped == false){
            cout<<"GRID: updateMoveent, force==noMovement"<<endl;
            motorControlGrid(angle,distance);
        }
        
        return mapped;
    }




    // //Find array of gridPoints to traverse through
    void pathFinder(vector<vector<GridPoint>> gridMap, MatrixXf State,GridPoint goal){
        //Assume there is a grid point at (0,0)

        GridPoint myRobot;
        myRobot.x = State(0);
        myRobot.y = State(1);

        //Get the start node as the gridpoint nearest to myRobot

        for(int i =0;i<gridMap.size();i++){
           // for(int j =0;j<gridMap[i].size();j++)
        }


        cout<<"\nNot yet implemented"<<endl;
        return;
    }

    //Note that start and goal must be gridpoints in gridmap
    // vector<GridPoint> pathFinder(vector<vector<GridPoint>>& gridMap, const GridPoint& start, const GridPoint& goal) {
        
    
    // }


}