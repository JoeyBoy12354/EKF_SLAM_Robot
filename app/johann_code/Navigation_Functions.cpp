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
            dist = 0;
            theta = 0;
            //mapped room
            cout<<"\nNAVI: EXPLORED MAP \n"<<endl;
            
            return true;
        }
    }


    void motorControl(){
        float time;

        //Send to motors
        writeMotorToCSV(theta,dist);
        cout<<"Navi: Set angle = "<<theta*180/(PI)<<" deg Set Distance = "<<dist<<"mm"<<endl;
    
        int ret;
        ret = system("python3 motorControl.py ok go");
        cout << "ret/cpp = " << ret << endl;

        

    }

    //Set velocity and direction to a new landmark
    void landmarkExplore(CarPoint LM, MatrixXf State){
        cout<<"NAVI: Landmark Explore "<<LM.x<<", "<<LM.y<<endl;
        //Calculate required distance and rotation
        float deltaX = LM.x - State(0);
        float deltaY = LM.y - State(1);
        dist = deltaX*deltaX + deltaY*deltaY;
        dist = sqrt(dist) - closeness;
        theta = atan2(deltaY,deltaX) - State(2);

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

    //Move through the map post mapping
    void pathFinder(){
        cout<<"\nNot yet implemented"<<endl;
        return;
    }


}