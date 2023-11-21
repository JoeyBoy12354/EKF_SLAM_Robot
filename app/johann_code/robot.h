#ifndef ROBOT_H
#define ROBOT_H

//#include <C:/Johann/Uni/Modules/2023/EPR/Project/Libraries/eigen-3.4.0/Eigen/Dense>
#include </home/odroid/Desktop/Johann/Libraries/eigen-3.4.0/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <random>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic> // for atomic flag
#include <unordered_set>

//#include <sstream>
//#include <numeric>
//#include <algorithm>


#include <queue>
#include <set>

//Lidar Functions Includes
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

using namespace std;
using namespace Eigen;
using namespace sl;



#define N 30 //Number of landmarks
#define dim 2*N+3//Initial Dimension of Matrices in EKF
#define PI 3.14159265358979323846 

// #define grid_ystep 400
// #define grid_xstep 400

extern float grid_ystep;
extern float grid_xstep;
extern float boundThresh;//If distance between gridPoint and lidarPoint <= Xmm then return false 

//extern int noCorners;//number of corners found

struct PolPoint {
    double angle;
    double distance;
};

struct CarPoint {
    double x;
    double y;

    //CarPoint(double x, double y) : x(x), y(y) {}
    CarPoint(double x = 0, double y = 0) : x(x), y(y) {}

    bool operator==(const CarPoint& other) const {
        return x == other.x && y == other.y;
    }

    friend ostream& operator<<(ostream& os, const CarPoint& point) {
        os << "(" << point.x << "," << point.y << ")";
        return os;
    }
};

struct CornerPoint {
    double x;
    double y;
    double angle;

    bool operator==(const CornerPoint& other) const {
        return x == other.x && y == other.y;
    }

    friend ostream& operator<<(ostream& os, const CornerPoint& point) {
        os << "(" << point.x << "," << point.y << ")|" << point.angle*180/PI;
        return os;
    }
};


struct GridPoint {
    double x;
    double y;
    bool trav = false;//Has this point been traversed

    bool operator==(const GridPoint& other) const {
        return x == other.x && y == other.y && trav == other.trav;
    }

    friend ostream& operator<<(ostream& os, const GridPoint& point) {
        os << "(" << point.x << "," << point.y << ") " << point.trav;
        return os;
    }
};

struct GridNode {
    int x;
    int y;
    bool traversable; // Indicates if the point has been traversed
    GridNode* parent; // Pointer to the parent node

    // Constructor for initialization
    GridNode(int x, int y) : x(x), y(y), traversable(true), parent(nullptr) {}
};



struct Line {
    double gradient;
    double intercept;
    double domain_max = -1000;
    double domain_min = 1000;
    double range_max = -1000;
    double range_min = 1000;
    CarPoint corner1 = {1000,1000};
    CarPoint corner2 = {1000,1000};

    vector<CarPoint> ConsensusPoints;
};


namespace CSV_Functions{
    void savePolToCSV(const vector<PolPoint>& points, const string& filename);
    void appendPolToCSV(const vector<PolPoint>& points, const string& filename);
    void saveCarToCSV(const vector<CarPoint>& points);
    void readCarFromCSV(vector<CarPoint>& points);
    void appendCarToCSV(const vector<CarPoint>& points);

    //Test
    void readCarFromCSVTest(vector<CarPoint>& points ,string TmapCSV);

    //Full Map
    void saveCarToFullMapCSV(vector<CarPoint>& points);
    void appendCarToFullMapCSV(vector<CarPoint>& points);
    void readCarFromFullMapCSV(vector<CarPoint>& points);
    void saveCarMotionToCSV(vector<CarPoint>& points);

    //EKF
    void saveLandmarkToCSV(vector<CarPoint> landmarks);
    void savePositionToCSV(vector<float> position);

    //Lines
    void saveSectionsToCSV(vector<vector<CarPoint>> points);
    void writeLinesToCSV(const vector<Line>& lines);
    void writeConsensusToCSV(const vector<Line>& lines);

    //Corners
    void writeCornersToCSV(const vector<CarPoint>& corners);
    void readCornersFromCSV(vector<CarPoint>& corners);

    //Motor
    void writeMotorToCSV(float angle, float distance);
    void readMotorFromCSV(float& angle, float& distance);
    void writeMotorStateToCSV(bool state);

    //Grid
    void saveGridToCSV(vector<vector<GridPoint>> points);
    void readGridFromCSV(vector<vector<GridPoint>>& points);

    //Triangle
    void saveTriangleToCSV(vector<CarPoint> triangle);

    //Stats
    void saveStatsToCSV(vector<vector<float>> states);

    //Atsis
    void atsi_u_read(vector<vector<float>>& input);
    void atsi_lm_read(vector<vector<CarPoint>>& lm);
    void atsi_u_write(vector<vector<float>>& input);
    void atsi_lm_write(vector<vector<float>>& lm);
    void atsi_lm_read2(vector<vector<PolPoint>>& lm);

    //Sonar
    void saveSonarIndicationToCSV(float distance);
    void saveSonarIndicationTurnToCSV(float distance);


}

namespace Data_Functions{
    extern int noCorners; // Declaration, not a definition

    //processing
    void fitCartesian(vector<CarPoint>& carPoints, float x, float y, float angle);
    vector<CarPoint> convertCartesian(vector<PolPoint>& dataPoints);
    int getIndex(vector<double> v, double K);
    vector<PolPoint> sortPoints(vector<PolPoint> dataPoints);
    bool compareByAngle(const PolPoint& a, const PolPoint& b);

    //The boys from Landmark.cpp
    double pointDistance(CarPoint pointA, CarPoint pointB);
    double perpendicularDistance(const CarPoint& point, Line& line);
    double gridPointDistance(GridPoint pointA, GridPoint pointB);

    //landmark
    void lidarDataProcessing(vector<PolPoint> dataPoints, vector<CarPoint>& carPoints,vector<PolPoint>& cornerPolarPoints);
    void lidarDataProcessing2(vector<PolPoint> dataPoints, vector<CarPoint>& carPoints,vector<PolPoint>& corner);

    void LandmarkProcessing();
    void LandmarkProcessing2(vector<CarPoint> carPoints);

    float pi_2_pi(float angle);


    //calibration
    //void getCaliAngle(MatrixXf State1,MatrixXf State2, float distThresh, float& caliAngle);
    
    //motor
    void motorDataProcessing(float& ekf_w,float&ekf_dist);

    //Cleaning
    void lidarDataProcessingCleaning(vector<PolPoint> dataPoints, vector<CarPoint>& carPoints,vector<PolPoint>& cornerPolarPoints);
}

namespace Landmark_Functions{
    vector<CarPoint> getCorners();
    VectorXf fitWithLeastSquares(VectorXf& x, VectorXf& y);
    int getSampleIndex(int numSamples);
    int evaluateModel(VectorXf& x, VectorXf& y, VectorXf& theta, float inlierThreshold);
    VectorXf ransac(VectorXf& X, VectorXf& y, int maxIters, float inlierThreshold, int minInliers, int samplesToFit);
    vector<VectorXf> manager(vector<float>& xCoords, vector<float>& yCoords, int sampleSize, int maxIters, float inlierThreshold, int minInliers);

    float calculateInterceptAngle(VectorXf& line1, VectorXf& line2);
    VectorXf calculateInterceptPoint(VectorXf& line1, VectorXf& line2);
    vector<VectorXf> findCorners(vector<VectorXf>& bestModels, float angleThreshold);
    vector<VectorXf> filterCorners(vector<VectorXf>& corners, vector<float>& xCoords, vector<float>& yCoords, float duplicateThreshold, float closenessThreshold);

    VectorXd fitWithLeastSquares2(const MatrixXd& X, const VectorXd& y);
    int evaluateModel2(const MatrixXd& X, const VectorXd& y, const VectorXd& theta, double inlierThreshold);
    VectorXd ransac2(const MatrixXd& X, const VectorXd& y, int maxIters, double inlierThreshold, int minInliers, int samplesToFit);
    vector<VectorXd> manager2(const vector<float>& xCoords, const vector<float>& yCoords, int sampleSize, int maxIters, double inlierThreshold, int minInliers);

    double calculateInterceptAngle2(const Vector2d& line1, const Vector2d& line2);
    Vector2d calculateInterceptPoint2(const Vector2d& line1, const Vector2d& line2);
    vector<Vector2d> findCorners2(const vector<VectorXd>& bestModels, double angleThreshold);
    vector<Vector2d> filterCorners2(const vector<Vector2d>& corners, vector<float>& xCoords, vector<float>& yCoords, double duplicateThreshold, double closenessThreshold);
    vector<Vector2d> filterCorners2_2(const vector<Vector2d>& closeCorners, vector<float>& xCoords, vector<float>& yCoords, double doubleLineThreshold);

    vector<CarPoint> getRANSACCorners(vector<CarPoint> dataPoints);
    vector<CarPoint> getMiniRANSACCorners(vector<CarPoint> dataPoints);
    bool getInlierCheck(vector<CarPoint> dataPoints,CarPoint corner);
}

namespace Simulation_Functions{
    vector<PolPoint> generateLidarData();
    double generateGaussian(double mean, double stddev);
    vector<CarPoint> landmarkNoise(vector<CarPoint> Landmarks);
}

namespace Lidar_Functions{
    void print_usage(int argc, const char * argv[]);
    bool checkSLAMTECLIDARHealth(ILidarDriver * drv);
    void ctrlc(int);
    int runLidar(vector<PolPoint>& lidarDataPoints, bool& error);

    //int initializeLidar(vector<PolPoint>& lidarDataPoints, bool& error, int NoPoints);
    int initializeLidar(ILidarDriver*& driver, bool& error);
    void fetchScan(ILidarDriver * drv, sl_result op_result, vector<PolPoint>& lidarDataPoints, int NoPoints, bool& error,sl_u32 timeout);
}

namespace Mapping_Functions{
    
    float storeMapPoints(vector<CarPoint> lidardata,Matrix<float, dim, 1> State);
    void storeStatePoints(Matrix<float, dim, 1> State);

    void getMapBounds(vector<CarPoint> map, vector<float>& bounds);
    void getMapBoundsSpecific(vector<CarPoint> map, vector<float>& bounds, GridPoint dot);

    void gridDataProcess(vector<vector<GridPoint>>& gridNew,Matrix<float, dim, 1> State, bool firstRun);
    void gridDataAssosciationPath(vector<vector<GridPoint>>& gridOld, vector<CarPoint> dumpedPath);
    void gridDataAssosciationMoveSimple(vector<vector<GridPoint>>& gridNew, Matrix<float, dim, 1> State);
    void gridDataAssosciationMap(vector<vector<GridPoint>> gridOld, vector<vector<GridPoint>>& gridNew);

    bool gridDotBoundCheck(vector<CarPoint> searchMap,GridPoint point);
    bool gridDotLidarCheck(vector<CarPoint> searchMap, GridPoint point,float distThresh);
    void gridMakeDots(vector<CarPoint> mapdata, vector<vector<GridPoint>>& points);
    void gridMakeDots2(vector<CarPoint> mapdata, vector<vector<GridPoint>>& points);
    bool gridDotBigBoundCheck(GridPoint point,vector<float> bounds);

    void mapConverter(vector<vector<GridPoint>> gridOld, vector<vector<GridNode>>& gridNew);

    void storeMapPointsCleaning(vector<CarPoint> lidardata,Matrix<float, dim, 1> State);
}

namespace Navigation_Functions{
    //Grid
    bool postMapMovement(MatrixXf State, vector<CarPoint>& path, bool home);
    bool mapMovement(MatrixXf State, vector<vector<GridPoint>> gridMap, vector<CarPoint>& path);
    void updateMovement(CarPoint closestPoint,MatrixXf State);
    void motorControlGrid(float angle, float distance);

    //Triangle 
    vector<CarPoint> translateTriangle(vector<CarPoint> triangle,CarPoint reference);
    vector<CarPoint> rotateTriangle(const vector<CarPoint> triangle,double angleRad);
    CarPoint triangularRepositioning(MatrixXf State, float angle);

    //PostMap
    // vector<GridPoint> pathFinder(vector<vector<GridPoint>> gridMap, MatrixXf State,GridPoint goal);
    // vector<GridPoint> findNeighbours(vector<vector<GridPoint>> gridMap, GridPoint point, float defaultVal,vector<CarPoint>& indexes);
    // void postMapMovement(MatrixXf State);

    GridPoint stateToGridDot(MatrixXf State, vector<vector<GridPoint>> gridMap);

    //Navi2
    vector<CarPoint> pathFinder(MatrixXf State,CarPoint Goal);
    vector<GridNode*> bfs(std::vector<std::vector<GridNode>>& gridMap, GridNode& start, GridNode& goal);
    vector<GridNode*> findNeighboursBFS2(GridNode& point,vector<vector<GridNode>>& gridMap);
    vector<CarPoint> pathShortening(vector<CarPoint> path);
    
    CarPoint findNextPoint(MatrixXf State, vector<vector<GridPoint>> gridMap, bool& mapped);
    CarPoint findNextPointRadius(MatrixXf State, vector<vector<GridPoint>> gridMap, bool& mapped);
    CarPoint findNextPointStraight(MatrixXf State, vector<vector<GridPoint>> gridMap, bool& mapped);

    float wallAvoidance(MatrixXf State, float angle);
    float wallAvoidanceForward(MatrixXf State, float angle, CarPoint postPoint, CarPoint goalPoint);
    void escape(CarPoint closestPoint,MatrixXf State,float avoid_dist);
    float wallAvoidanceTurn(MatrixXf State);
    
}

class ExtendedKalmanFilter {
public:
    //Testing
    vector<CarPoint> TestValues;
    vector<CarPoint> landmarks;
    vector<PolPoint> TestPolValues;

    //Motion
    // float v = 1.3094206;
    // float w = -0.12391118;
    // float t = 0.1;

    float sigma_r = 100;//*
    float sigma_theta = 0.1;//* 
    float sigma_odo_x = 0.01;
    float sigma_odo_y = 0.01;
    float sigma_odo_theta = 0.017;
    
    int noNewCorners = 0;

    float w = 0;
    float distance = 0;
    float lidar_x = 0;
    float lidar_y = 0;
    Matrix<float, dim, 1> State;
    
    
    float initialLandmarkCovariance_AtSi = 1;

    Matrix<float, 2, 1> z;
    Matrix<float, 2, 1> z_cap;

    //Purely stats
    Matrix<float, 2, 1> delat_gz;
    vector<vector<float>> stats;
    

    ExtendedKalmanFilter();
    void updateMotion();
    void updateCovarianceOfRobot();
    void getEstimatedObservationJacobian(float deltaX, float deltaY, float q);
    void getGainMatrix();
    void updateStateOfLandmark();
    void updateCovarianceOfLandmark();
    void runEKF();
    vector<CarPoint> observeEnvironment();
    void calculateNoise();
    float mahalanobisDistance(CarPoint StoredPoint,int LMindex);
    float directDistance(CarPoint StoredPoint);
    void isNewLandmark2();

private:
    Matrix<float, dim, dim> Covariance;
    Matrix<float, dim, dim> Motion_Jacobian;
    Matrix<float, dim, dim> Motion_Noise;
    Matrix<float, 2, 2> Coordinate_Uncertainty;
    Matrix<float, 5, dim> F;
    // Matrix<float, 2, 1> z;
    // Matrix<float, 2, 1> z_cap;
    Matrix<float, 2, dim> Observation_Jacobian;
    Matrix<float, dim, 2> Gain;

    vector<CarPoint> Landmarks;
    CarPoint EstimatedLandmark;
    CarPoint ObservedLandmark;
    PolPoint ObservedPolarLandmark;

    //Every degree error = 1 (remember to conver to radians)
    //Every mm error = 0.1
    int NoLandmarksFound = 0;
    int LandmarkIndex = 0;

    // float sigma_r = 100;
    // float sigma_theta = 0.1;
    // float sigma_odo_x = 0.01;
    // float sigma_odo_y = 0.01;
    // float sigma_odo_theta = 0.017;

    

    

    // float sigma_r = 100;//*
    // float sigma_theta = 17;//* 
    // float sigma_odo_x = 0.01;
    // float sigma_odo_y = 0.01;
    // float sigma_odo_theta = 0.17;


    //Covariance for EKF simulation
    // Q = np.diag([
    //             0.1,  # variance of location on x-axis
    //             0.1,  # variance of location on y-axis
    //             np.deg2rad(1.0),  # variance of yaw angle
    //             1.0  # variance of velocity
    // ]) ** 2  # predict state covariance

    // R = np.diag([1.0, 1.0]) ** 2  # Observation x,y position covariance

    // float sigma_r = 0.0000005;
    // float sigma_theta = 0.0000005;

    bool LandmarkIsNew = false;
};

#endif





// namespace Landmark_Functions{
    
//     double perpendicularDistance(const CarPoint& point, Line& line);
//     void findInliers(vector<Line>& lines, vector<CarPoint>& points);
//     vector<Line> houghTransform(vector<CarPoint>& points, int num_theta_bins = 360, int num_rho_bins = 500);
//     double pointDistance(CarPoint pointA, CarPoint pointB);
//     vector<CarPoint> findCorners(vector<Line> lines);
//     vector<CarPoint> findLazyCorners(vector<Line> lines);
//     vector<CarPoint> findFancyCorners(vector<Line> lines);

//     vector<Line> RANSAC(vector<CarPoint> laserdata);
//     vector<CarPoint> ANSAC_CORNER(vector<CarPoint> laserdata);
//     void LeastSquaresLineEstimate(vector<CarPoint> SelectedPoints, double& c, double& m);
//     vector<CarPoint> findNearestPoint(vector<Line> lines);
//     float getAngle(Line l1, Line l2);
//     CarPoint getIntercept(Line line1, Line line2);
//     bool checkForHoles(vector<CarPoint> points);
//     // vector<CarPoint> gradientAnalysis(vector<CarPoint> laserdata);
//     // float distanceBetweenPointandSample(CarPoint point,vector<CarPoint> sample);

//     vector<Line> RANSAC_Manager(vector<CarPoint> laserdata);
//     vector<Line> RANSAC2(vector<CarPoint> laserdata);
//     vector<CarPoint> gradientAnalysis(vector<CarPoint> laserdata);
//     float distanceBetweenPointandSample(CarPoint point,vector<CarPoint> sample);
// }
