#include "robot.h"

using namespace CSV_Functions;
using namespace Landmark_Functions;

namespace Mapping_Functions{

    void StoreMapAndStatePoints(vector<CarPoint> lidardata, MatrixXf State){
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
        position.append(State[0]);
        position.append(State[1]);
        position.append(State[2]);
        

        //Convert State Landmarks to points
        for(int i = 3;i<State.size();i=i+2){
            CarPoint LM;
            LM.x = State[i];
            LM.y = State[i+1];
            landmarks.append(LM);
        }

        //Update full map
        appendCarToFullMapCSV(lidardata);
        saveLandmarkToCSV(landmarks);
        savePositionToCSV(position);
    }

    

}