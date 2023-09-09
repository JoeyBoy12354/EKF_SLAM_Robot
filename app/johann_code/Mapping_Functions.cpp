#include "robot.h"

using namespace CSV_Functions;
using namespace Landmark_Functions;

namespace Mapping_Functions{

    void StoreMapAndStatePoints(vector<CarPoint> lidardata, Matrix<float, dim, 1> State){
        cout<<"Entered STOREMAPEANDPOINTS"<<endl;
        float distance_threshold = 30;//If two points are greater than Xmm then keep this point.

        vector<CarPoint> oldmap;
        vector<CarPoint> landmarks;
        vector<float> position;

        readCarFromFullMapCSV(oldmap);//Fetch all current points
        

        //Compare with new points (only update if points are different)
        //If a point is less than Xmm away from oldmap point then remove it from circulation
        for(int i=0; i<oldmap.size();i++){
            for(int j=0; j<lidardata.size();j++){
                    vector<CarPoint> temp;
                    if(pointDistance(lidardata[j],oldmap[i]) > distance_threshold){
                        temp.push_back(lidardata[j]);
                    }
                    lidardata = temp;
            }
        }
    
        //Move Position into vector
        position.push_back(State[0]);
        position.push_back(State[1]);
        position.push_back(State[2]);
        

        //Convert State Landmarks to points
        for(int i = 3;i<State.size();i=i+2){
            CarPoint LM;
            LM.x = State[i];
            LM.y = State[i+1];
            landmarks.push_back(LM);
        }

        cout<<"UPDATING FULL MAP";

        //Update full map
        appendCarToFullMapCSV(lidardata);
        saveLandmarkToCSV(landmarks);
        savePositionToCSV(position);
    }

    

}