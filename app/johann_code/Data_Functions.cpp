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

    void fitCartesian(vector<CarPoint>& carPoints, float x, float y, float angle){

        if(x==0 and y==0 and angle==0){
            return; //just due to me being scared
        }else{

            // Calculate trigonometric values for the angle
            // float cosAngle = cos(angle);
            // float sinAngle = sin(angle);

            //map reversal
            float cosAngle = cos(-angle);
            float sinAngle = sin(-angle);

            for (int i = 0; i < carPoints.size(); i++) {
                // Apply rotation first
                float rotatedX = carPoints[i].x * cosAngle - carPoints[i].y * sinAngle;
                float rotatedY = carPoints[i].x * sinAngle + carPoints[i].y * cosAngle;

                // Then apply translation
                carPoints[i].x = rotatedX + x;
                carPoints[i].y = rotatedY + y;
            }
        }

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
    

 
    void lidarDataProcessingFull(vector<PolPoint> dataPoints, vector<CarPoint>& carPoints, bool firstRun){
        
        //I AM NOT CURRENTLY USING THIS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

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

        //Reading The Full Map means we will identify all landmarks at all times (this could be problematic)
        readCarFromFullMapCSV(carPoints);

        cout<<"\nRANSAC"<<endl;
        vector<Line> detected_lines = RANSAC(carPoints);
        writeLinesToCSV(detected_lines);
        writeConsensusToCSV(detected_lines);
        

        vector<CarPoint> closestPoints = findNearestPoint(detected_lines);
        writeCornersToCSV(closestPoints);
        cout<<"\n Number of Closest Points Found:"<<closestPoints.size()<<endl;
        
     
    }

    void lidarDataProcessing(vector<PolPoint> dataPoints, vector<CarPoint>& carPoints, float x, float y, float angle){
        cout<<"\n lidarDataProcessing"<<endl;

        carPoints = convertCartesian(dataPoints);
        fitCartesian(carPoints,x,y,angle);

        saveCarToCSV(carPoints);
        cout<<"\nNumber of CAR points"<<carPoints.size(); 



        // cout<<"\nRANSAC"<<endl;
        // vector<Line> detected_lines = RANSAC(carPoints);
        // writeLinesToCSV(detected_lines);
        // writeConsensusToCSV(detected_lines);

        cout<<"\nANSAC"<<endl;
        vector<CarPoint> closestPoints = ANSAC_CORNER(carPoints);
        writeCornersToCSV(closestPoints);

        // cout<<"\nGRADIENT CORNERS"<<endl;
        // vector<CarPoint> closestPoints = gradientAnalysis(carPoints);
        // writeCornersToCSV(closestPoints);
        

        // vector<CarPoint> closestPoints = findNearestPoint(detected_lines);
        // writeCornersToCSV(closestPoints);
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

    void LandmarkProcessing2(vector<CarPoint> carPoints){
        cout<<"\n Number of Points Read:"<<carPoints.size()<<endl;

        cout<<"\n RANSAC_Manager\n"<<endl;
        vector<Line> detected_lines = RANSAC_Manager(carPoints);
        writeLinesToCSV(detected_lines);
        

        cout<<"\n Number of Lines Found:"<<detected_lines.size()<<endl;
        for(int i =0;i<detected_lines.size();i++){
            cout<<"y = "<<detected_lines[i].gradient<<"x + "<<detected_lines[i].intercept<<" | conLen = "<<detected_lines[i].ConsensusPoints.size()<<endl;
        }
        

    }

    //This function will take the inputed values and set the EKF
    void motorDataProcessing(float& ekf_w,float&ekf_dist){
        //cout<<"Process motor Data"<<endl;
        float theta;
        float dist;
      

        
        readMotorFromCSV(theta,dist);
        cout<<"DATA: SEND TO EKF: Read angle = "<<theta<<" Read Distance = "<<endl;
        cout<<"DATA:FOR TESTING I FORCE THIS TO BE 0 and 0 was "<<endl;
        theta = 0;
        dist = 0;
        cout<<"DATA: SEND TO EKF (TEST_VAL): Read angle = "<<theta<<" Read Distance = "<<endl;
        
        ekf_w = theta;
        ekf_dist = dist;

        

    }


    void getCaliAngle(MatrixXf State1,MatrixXf State2, float distThresh, float& caliAngle){
        //Get list 1
        vector<CarPoint> LMlist1;
        CarPoint LM; 
        for(int i=0; i<State1.rows(); i++){
            LM.x = State1(i+3);
            LM.y = State1(i*2+4);
            LMlist1.push_back(LM);
        }

        //Get list 2
        vector<CarPoint> LMlist2; 
        for(int i=0; i<State2.rows(); i++){
            LM.x = State2(i+3);
            LM.y = State2(i*2+4);
            LMlist2.push_back(LM);
        }

        vector<float> totalAngles;
        float sumAngles = 0;
        for(int i=0; i<LMlist1.size(); i++){
            for(int j=0; j<LMlist2.size(); j++){
                //data assosciation
                if(pointDistance(LMlist1[i],LMlist2[j]) <= distThresh){
                    //calculate angle;
                    float delta_x = LMlist1[i].x - LMlist2[j].x;
                    float delta_y = LMlist1[i].y - LMlist2[j].y;
                    float angle = atan2(delta_y,delta_x);

                    totalAngles.push_back(angle);
                    sumAngles+=angle;
                }
            }
        }

        caliAngle = sumAngles/totalAngles.size();

        return;
    }
}





