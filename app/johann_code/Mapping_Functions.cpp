#include "robot.h"

using namespace CSV_Functions;
using namespace Landmark_Functions;

namespace Mapping_Functions{

    void StoreMapPoints(vector<CarPoint> lidardata){
        float distance_threshold = 30;//If two points are greater than Xmm then keep this point.

        //Fetch all current points
        vector<CarPoint> oldmap;
        readCarFromFullMapCSV(oldmap);
        

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

        //Update full map
        //oldmap.insert(oldmap.end(), lidardata.begin(), lidardata.end());
        //saveCarToFullMapCSV(oldmap)
        appendCarToFullMapCSV(lidardata);
    }

    

}