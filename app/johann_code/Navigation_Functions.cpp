#include "robot.h"

using namespace Landmark_Functions;
using namespace CSV_Functions;
using namespace Data_Functions;


int noNavTrials = 0;
vector<bool> LandmarksExplored;
double foundRadius = 50;//Robot must be within Xmm of LM for LM to be considered Explored

float theta;
float dist;

namespace Navigation_Functions{
    //Determine Direction and send control signals to motors
    bool updateMovement(MatrixXf State){

        cout<<"\n\nUpdate Movement Entered"<<endl;

        CarPoint Robot;
        Robot.x = State(0);
        Robot.y = State(1);

        updateExplorations(State,Robot);

        if(noNavTrials<10){
            for(int i = 0; i<LandmarksExplored.size();i++){
                if(LandmarksExplored[i] != true){
                    CarPoint LM;
                    LM.x = State(i+3);
                    LM.y = State(i*2+4);

                    landmarkExplore(LM, State);
                    motorControl();
                    noNavTrials = 0;
                    cout<<"NAVI: LANDMARK EXPLORE"<<endl;
                    //SHOULD BE TRUE PLEASE CHANGE 
                    return false;
                }
            }

            noNavTrials++;
            randomExplore();
            motorControl();
            cout<<"NAVI: RANDOM EXPLORE"<<endl;

            //SHOULD BE TRUE PLEASE CHANGE 
            return false;
        }else{
            //mapped room
            
            
            return false;
        }
    }


    void motorControl(){
        float time;

        //Send to motors
        writeMotorToCSV(theta,dist);
        cout<<"Navi: Set angle = "<<theta*180/(PI)<<" Set Distance = "<<dist<<"mm"<<endl;
    
        int ret;
        ret = system("python3 motorControl.py ok go");
        cout << "ret/cpp = " << ret << endl;

        

    }

    //Set velocity and direction to a new landmark
    void landmarkExplore(CarPoint LM, MatrixXf State){
        cout<<"Landmark Explore "<<LM.x<<", "<<LM.y<<endl;
        //Calculate required distance and rotation
        float deltaX = LM.x - State(0);
        float deltaY = LM.y - State(1);
        dist = deltaX*deltaX + deltaY*deltaY;
        dist = sqrt(dist);
        theta = atan2(deltaY,deltaX) - State(2);
       
        
        return;
    }

    //Set velocity and direction randomly
    void randomExplore(){
        cout<<"Random Explore "<<endl;
        dist = rand() % 30;
        theta = rand() % 30;
        
        
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

    //Move through the map post mapping
    void pathFinder(){
        cout<<"\nNot yet implemented"<<endl;
        return;
    }


}