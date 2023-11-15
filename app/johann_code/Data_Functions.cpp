#include "robot.h"


using namespace CSV_Functions;
using namespace Landmark_Functions;
using namespace Simulation_Functions;
using namespace Mapping_Functions;

namespace Data_Functions{

    //int noCorners = 0;//If distance between gridPoint and lidarPoint <= Xmm then return false 
    int noCorners = 0;//If distance between gridPoint and lidarPoint <= Xmm then return false (global)


    bool compareByAngle(const PolPoint& a, const PolPoint& b) {
        return a.angle < b.angle;
    }

    vector<PolPoint> sortPoints(vector<PolPoint> dataPoints) {
        vector<PolPoint> sortedPoints;

        // Sort dataPoints based on angle in ascending order
        sort(dataPoints.begin(), dataPoints.end(), compareByAngle);
        cout<<"START Point = "<<dataPoints[0].angle<<" END Point "<<dataPoints[dataPoints.size()-1].angle;

        cout<<"START LEN = "<<dataPoints.size();

        //Delete duplicates
        dataPoints.erase(std::unique(dataPoints.begin(), dataPoints.end(), 
                     [](const PolPoint& a, const PolPoint& b) {
                         return a.angle == b.angle;
                     }), dataPoints.end());

        cout<<"END LEN = "<<dataPoints.size();
        // Copy the sorted dataPoints to sortedPoints
        sortedPoints = dataPoints;

        return sortedPoints;
    }

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

        vector<CarPoint> corners;
        readCornersFromCSV(corners);

        cout<<"Fit Cartesian for: ("<<x<<","<<y<<") | "<<angle*180/PI<<endl;
        // cout<<"Could be: "<<(PI-angle)*180/PI<<endl;
        if(x==0 and y==0 and angle==0){
            return; //just due to me being scared
        }else{

            // Calculate trigonometric values for the angle
            float cosAngle = cos(angle);
            float sinAngle = sin(angle);

            //map reversal
            // float cosAngle = cos(-angle);
            // float sinAngle = sin(-angle);

            for (int i = 0; i < carPoints.size(); i++) {
                // Apply rotation first
                float rotatedX = carPoints[i].x * cosAngle - carPoints[i].y * sinAngle;
                float rotatedY = carPoints[i].x * sinAngle + carPoints[i].y * cosAngle;

                // Then apply translation
                carPoints[i].x = rotatedX + x;
                carPoints[i].y = rotatedY + y;
            }

            for (int i = 0; i < corners.size(); i++) {
                // Apply rotation first
                float rotatedX = corners[i].x * cosAngle - corners[i].y * sinAngle;
                float rotatedY = corners[i].x * sinAngle + corners[i].y * cosAngle;

                // Then apply translation
                corners[i].x = rotatedX + x;
                corners[i].y = rotatedY + y;
            }

            writeCornersToCSV(corners);

            
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
    


    void lidarDataProcessing(vector<PolPoint> dataPoints, vector<CarPoint>& carPoints,vector<PolPoint>& cornerPolarPoints){
        //cout<<"\n lidarDataProcessing"<<endl;

        carPoints = convertCartesian(dataPoints);


        //cout<<"\n RANSAC py RUN \n"<<endl;
        //vector<CarPoint> cornerPoints = getCorners();

        vector<CarPoint> cornerPoints;
        vector<CarPoint> temp;
        
        int maxSize = 2;
        for(int i =0;i<5;i++){
            temp = getRANSACCorners(carPoints);
            if(temp.size()>maxSize){
                cornerPoints = temp;
            }   
        }

        //This is emergency fetch
        int noRuns = 0;
        while(cornerPoints.size()<2 && noRuns<10){
            cout<<"Too few Corners!"<<endl;
            cornerPoints = getRANSACCorners(carPoints);
            noRuns+=1;
        }

        noCorners = cornerPoints.size();


        //Convert cornerPoints to polar
        for(int i =0;i<cornerPoints.size();i++){
            PolPoint newPoint;
            newPoint.distance = sqrt(pow(cornerPoints[i].x,2) + pow(cornerPoints[i].y,2));
            newPoint.angle = atan2(cornerPoints[i].y,cornerPoints[i].x);
            cornerPolarPoints.push_back(newPoint);

        return;
     
    }

    void lidarDataProcessing2(vector<PolPoint> dataPoints, vector<CarPoint>& carPoints,vector<PolPoint>& cornerPolarPoints){
    //cout<<"\n lidarDataProcessing"<<endl;

    carPoints = convertCartesian(dataPoints);

    saveCarToCSV(carPoints);
    cout<<"\nNumber of CAR points"<<carPoints.size(); 

    //cout<<"\n RANSAC py RUN \n"<<endl;
    //vector<CarPoint> cornerPoints = getCorners();

    vector<CarPoint> cornerPoints;
    vector<CarPoint> temp;
    
    int maxSize = 2;
    for(int i =0;i<5;i++){
        temp = getRANSACCorners(carPoints);
        if(temp.size()>maxSize){
            cornerPoints = temp;
        }   
    }

    //This is emergency fetch
    int noRuns = 0;
    while(cornerPoints.size()<2 && noRuns<10){
        cout<<"Too few Corners!"<<endl;
        cornerPoints = getRANSACCorners(carPoints);
        noRuns+=1;
    }

    noCorners = cornerPoints.size();


    //Convert cornerPoints to polar
    for(int i =0;i<cornerPoints.size();i++){
        PolPoint newPoint;
        newPoint.distance = sqrt(pow(cornerPoints[i].x,2) + pow(cornerPoints[i].y,2));
        newPoint.angle = atan2(cornerPoints[i].y,cornerPoints[i].x);
        cornerPolarPoints.push_back(newPoint);
    }


    cout<<"DATA: NUMBER OF CORNERS POINTS = "<<cornerPoints.size()<<endl;
    for(int i =0;i<cornerPoints.size();i++){
        cout<<cornerPoints[i]<<" | ";
    }
    cout<<endl;

    return;
     
    }


    
    void lidarDataProcessingCleaning(vector<PolPoint> dataPoints, vector<CarPoint>& carPoints,vector<PolPoint>& cornerPolarPoints){
    //cout<<"\n lidarDataProcessing"<<endl;

    carPoints = convertCartesian(dataPoints);

    // saveCarToCSV(carPoints);
    // cout<<"\nNumber of CAR points"<<carPoints.size(); 

    //cout<<"\n RANSAC py RUN \n"<<endl;
    //vector<CarPoint> cornerPoints = getCorners();

    vector<CarPoint> cornerPoints;
    for(int i =0;i<5;i++){
        if(cornerPoints.size()<2){
            cout<<"Too few Corners!"<<endl;
            cornerPoints = getRANSACCorners(carPoints);
        }    
    }
    
    //Convert cornerPoints to polar
    for(int i =0;i<cornerPoints.size();i++){
        PolPoint newPoint;
        newPoint.distance = sqrt(pow(cornerPoints[i].x,2) + pow(cornerPoints[i].y,2));
        newPoint.angle = atan2(cornerPoints[i].y,cornerPoints[i].x);
        cornerPolarPoints.push_back(newPoint);
    }


    cout<<"DATA: NUMBER OF CORNERS POINTS = "<<cornerPoints.size()<<endl;
    for(int i =0;i<cornerPoints.size();i++){
        cout<<cornerPoints[i]<<" | ";
    }
    cout<<endl;

    return;
     
    }

    

    void LandmarkProcessing(){
        
        vector<CarPoint> carPoints;
        readCarFromCSV(carPoints);
        cout<<"\n Number of Points Read:"<<carPoints.size()<<endl;

        // cout<<"\n RANSAC\n"<<endl;
        // vector<Line> detected_lines = RANSAC(carPoints);
        // writeLinesToCSV(detected_lines);
        
        
        

        // cout<<"\n Number of Lines Found:"<<detected_lines.size()<<endl;

        // cout<<"\n findNearestPoints\n"<<endl;
        // vector<CarPoint> closestPoints = findNearestPoint(detected_lines);
        // writeCornersToCSV(closestPoints);

        // cout<<"\n Number of Closest Points Found:"<<closestPoints.size()<<endl;

    }

    void LandmarkProcessing2(vector<CarPoint> carPoints){
        cout<<"\n Number of Points Read:"<<carPoints.size()<<endl;

        // cout<<"\n RANSAC_Manager\n"<<endl;
        // vector<Line> detected_lines = RANSAC_Manager(carPoints);
        // writeLinesToCSV(detected_lines);
        // writeConsensusToCSV(detected_lines);
        
        

        // cout<<"\n Number of Lines Found:"<<detected_lines.size()<<endl;
        // for(int i =0;i<detected_lines.size();i++){
        //     cout<<"y = "<<detected_lines[i].gradient<<"x + "<<detected_lines[i].intercept<<" | conLen = "<<detected_lines[i].ConsensusPoints.size()<<endl;
        // }

        // vector<CarPoint> closestPoints =  findFancyCorners(detected_lines);
        // writeCornersToCSV(closestPoints);


        // cout<<"\n ANSAC RUN \n"<<endl;
        // vector<CarPoint> closestPoints = ANSAC_CORNER(carPoints);
        // writeCornersToCSV(closestPoints);

        
        // cout<<"\n GRADIENT ANALYSOIS RUN \n"<<endl;
        // vector<CarPoint> closestPoints = gradientAnalysis(carPoints);
        // writeCornersToCSV(closestPoints);

        cout<<"\n RANSAC py RUN \n"<<endl;
        vector<CarPoint> closestPoints = getCorners();


        cout<<"NUMBER OF CLOSEST POINTS = "<<closestPoints.size()<<endl;

        

    }

    //This function will take the inputed values and set the EKF
    void motorDataProcessing(float& ekf_w,float&ekf_dist){
        //cout<<"Process motor Data"<<endl;
        float theta;
        float dist;
      

        
        readMotorFromCSV(theta,dist);
        //cout<<"DATA: SEND TO EKF: Read angle = "<<theta<<" Read Distance = "<<endl;
        // cout<<"DATA:FOR TESTING I FORCE THIS TO BE 0 and 0 was "<<endl;
        // theta = 0;
        // dist = 0;
        // cout<<"DATA: SEND TO EKF (TEST_VAL): Read angle = "<<theta<<" Read Distance = "<<endl;
        
        ekf_w = theta;
        ekf_dist = dist;
    }

    // Function to calculate the perpendicular distance from a point to a point
    double pointDistance(CarPoint pointA, CarPoint pointB) {
        return sqrt( pow((pointA.x - pointB.x),2) + pow((pointA.y - pointB.y),2));
    }

    double gridPointDistance(GridPoint pointA, GridPoint pointB) {
        return sqrt( pow((pointA.x - pointB.x),2) + pow((pointA.y - pointB.y),2));
    }

    // Function to calculate the perpendicular distance from a point to a line
    double perpendicularDistance(const CarPoint& point, Line& line) {
        return fabs((line.gradient * point.x - point.y + line.intercept)) / sqrt(1 + line.gradient * line.gradient);
    }
    

    float pi_2_pi(float angle) {
        while (angle <= -M_PI) {
        angle += 2.0 * M_PI;
        }
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }
        return angle;
    }

}





