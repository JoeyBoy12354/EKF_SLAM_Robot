#include "robot.h"


using namespace CSV_Functions;
using namespace Landmark_Functions;
using namespace Simulation_Functions;
using namespace Mapping_Functions;

namespace Data_Functions{

    vector<CarPoint> convertCartesian(vector<PolPoint>& dataPoints){
        vector<CarPoint> cartesianPoints;
        for (const PolPoint& polarPoint : dataPoints) {
            double angleRad = polarPoint.angle * PI / 180;
            double x = polarPoint.distance * cos(angleRad);
            double y = polarPoint.distance * sin(angleRad);
            cartesianPoints.push_back({x, y});
        }

        return cartesianPoints;
    }

    int getIndex(vector<double> v, double K)
    {
        auto it = find(v.begin(), v.end(), K);
    
        // If element was found
        if (it != v.end()) 
        {
        
            // calculating the index
            // of K
            int index = it - v.begin();
            return index;
        }
        else {
            // If the element is not
            // present in the vector
            cout<<"\nINDEX WAS NOT FOUND - BE WORRIED WE DID NOT CONSIDER THIS"<<endl;
            return -1;
        }
    }
    
    void lidarDataProcessingCali(vector<PolPoint> dataPoints, vector<CarPoint>& carPoints){
        cout<<"\n lidarDataProcessing Calibration"<<endl;

        carPoints = convertCartesian(dataPoints);

        cout<<"\nRANSAC"<<endl;
        vector<Line> detected_lines = RANSAC(carPoints);
        writeLinesToCSV(detected_lines);
        writeConsensusToCSV(detected_lines);
        

        vector<CarPoint> closestPoints = findNearestPoint(detected_lines);
        writeCornersToCSV(closestPoints);
        cout<<"\n Number of Closest Points Found:"<<closestPoints.size()<<endl;
        
     
    }

    void lidarDataProcessingFull(vector<PolPoint> dataPoints, vector<CarPoint>& carPoints, bool firstRun){
        cout<<"\n lidarDataProcessing Full"<<endl;

        carPoints = convertCartesian(dataPoints);
        saveCarToCSV(carPoints);
        cout<<"\nNumber of CAR points"<<carPoints.size();

        //Store Data for plotting
        if(firstRun == true){
            saveCarToFullMapCSV(carPoints);
        }else{
            storeMapPoints(carPoints);
        }

        readCarFromFullMapCSV(carPoints);

        cout<<"\nRANSAC"<<endl;
        vector<Line> detected_lines = RANSAC(carPoints);
        writeLinesToCSV(detected_lines);
        writeConsensusToCSV(detected_lines);
        

        vector<CarPoint> closestPoints = findNearestPoint(detected_lines);
        writeCornersToCSV(closestPoints);
        cout<<"\n Number of Closest Points Found:"<<closestPoints.size()<<endl;
        
     
    }

    void lidarDataProcessing(vector<PolPoint> dataPoints, vector<CarPoint>& carPoints){
        cout<<"\n lidarDataProcessing"<<endl;

        carPoints = convertCartesian(dataPoints);
        saveCarToCSV(carPoints);
        cout<<"\nNumber of CAR points"<<carPoints.size(); 

        cout<<"\nRANSAC"<<endl;
        vector<Line> detected_lines = RANSAC(carPoints);
        writeLinesToCSV(detected_lines);
        writeConsensusToCSV(detected_lines);
        

        vector<CarPoint> closestPoints = findNearestPoint(detected_lines);
        writeCornersToCSV(closestPoints);
        cout<<"\n Number of Closest Points Found:"<<closestPoints.size()<<endl;
        
     
    }

    void LandmarkProcessing(){
        
        vector<CarPoint> carPoints;
        readCarFromCSV(carPoints);
        cout<<"\n Number of Points Read:"<<carPoints.size()<<endl;

        cout<<"\n RANSAC\n"<<endl;
        vector<Line> detected_lines = RANSAC(carPoints);
        writeLinesToCSV(detected_lines);
        

        cout<<"\n Number of Lines Found:"<<detected_lines.size()<<endl;

        cout<<"\n findNearestPoints\n"<<endl;
        vector<CarPoint> closestPoints = findNearestPoint(detected_lines);
        writeCornersToCSV(closestPoints);

        cout<<"\n Number of Closest Points Found:"<<closestPoints.size()<<endl;

    }

    //This function will take the inputed values and set the EKF
    void motorDataProcessing(float& ekf_w,float&ekf_dist){
        //cout<<"Process motor Data"<<endl;
        float theta;
        float dist;
      

        
        readMotorFromCSV(theta,dist);
        cout<<"DATA: SEND TO EKF: Read angle = "<<theta<<" Read Distance = "<<endl;
        
        ekf_w = theta;
        ekf_dist = dist;

    }


    void getCaliAngle(MatrixXf State1,MatrixXf State2, float distThresh, float& caliAngle){
        //Get list 1
        vector<CarPoint> LMlist1
        CarPoint LM; 
        for(int i=0; i<State1.rows(); i++){
            LM.x = State(i+3);
            LM.y = State(i*2+4);
            LMlist1.append(LM);
        }

        //Get list 2
        vector<CarPoint> LMlist2
        CarPoint LM; 
        for(int i=0; i<State1.rows(); i++){
            LM.x = State(i+3);
            LM.y = State(i*2+4);
            LMlist2.append(LM);
        }

        vector<float> totalAngles;
        float sumAngles = 0;
        for(int i=0; i<LMlist1.size(); i++){
            for(int j=0; j<LMlist2.size(); j++){
                //data assosciation
                if(pointDistance(LMlist1[i],LMlist2[j]) <= distThresh){
                    //calculate angle;
                    float delta_x = LMlist1[0] - LMlist2[0];
                    float delta_y = LMlist1[1] - LMlist2[0];
                    float angle = atan2(delta_y,delta_x);

                    totalAngles.append(angle);
                    sumAngles+=angle;
                }
            }
        }

        caliAngle = sumAngles/totalAngle.size();

        return;
    }
}





