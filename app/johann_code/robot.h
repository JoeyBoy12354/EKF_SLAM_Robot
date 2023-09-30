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
#include <random>

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

#define N 10 //Number of landmarks
#define dim 2*N+3//Initial Dimension of Matrices in EKF
#define PI 3.14159265358979323846 

struct PolPoint {
    double angle;
    double distance;
};

struct CarPoint {
    double x;
    double y;

    bool operator==(const CarPoint& other) const {
        return x == other.x && y == other.y;
    }

    friend ostream& operator<<(ostream& os, const CarPoint& point) {
        os << "(" << point.x << "," << point.y << ")";
        return os;
    }
};

struct GridPoint {
    double x;
    double y;
    bool trav = false;

    bool operator==(const GridPoint& other) const {
        return x == other.x && y == other.y && trav == other.trav;
    }

    friend ostream& operator<<(ostream& os, const GridPoint& point) {
        os << "(" << point.x << "," << point.y << ") " << point.trav;
        return os;
    }
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

    //Full Map
    void saveCarToFullMapCSV(const vector<CarPoint>& points);
    void appendCarToFullMapCSV(const vector<CarPoint>& points);
    void readCarFromFullMapCSV(vector<CarPoint>& points);

    //EKF
    void saveLandmarkToCSV(vector<CarPoint> landmarks);
    void savePositionToCSV(vector<float> position);

    //Lines
    void writeLinesToCSV(const vector<Line>& lines);
    void writeConsensusToCSV(const vector<Line>& lines);

    //Corners
    void writeCornersToCSV(const vector<CarPoint>& corners);
    void readCornersFromCSV(vector<CarPoint>& corners);

    //Motor
    void writeMotorToCSV(float angle, float distance);
    void readMotorFromCSV(float& angle, float& distance);

    //Grid
    void saveGridToCSV(vector<vector<GridPoint>> points);
    void readGridFromCSV(vector<vector<GridPoint>>& points);


}

namespace Data_Functions{
    //processing
    void fitCartesian(vector<CarPoint>& carPoints, float x, float y, float angle);
    vector<CarPoint> convertCartesian(vector<PolPoint>& dataPoints);
    int getIndex(vector<double> v, double K);

    //landmark
    void lidarDataProcessingFull(vector<PolPoint> dataPoints, vector<CarPoint>& carPoints, bool firstRun);
    void lidarDataProcessing(vector<PolPoint> dataPoints, vector<CarPoint>& carPoints, float x, float y, float angle);
    void LandmarkProcessing();

    //calibration
    void lidarDataProcessingCali(vector<PolPoint> dataPoints, vector<CarPoint>& carPoints);
    void getCaliAngle(MatrixXf State1,MatrixXf State2, float distThresh, float& caliAngle);
    
    //motor
    void motorDataProcessing(float& ekf_w,float&ekf_dist);
}

namespace Landmark_Functions{
    double perpendicularDistance(const CarPoint& point, Line& line);
    void findInliers(vector<Line>& lines, vector<CarPoint>& points);
    vector<Line> houghTransform(vector<CarPoint>& points, int num_theta_bins = 360, int num_rho_bins = 500);
    double pointDistance(CarPoint pointA, CarPoint pointB);
    vector<CarPoint> findCorners(vector<Line> lines);
    vector<CarPoint> findLazyCorners(vector<Line> lines);

    vector<Line> RANSAC(vector<CarPoint> laserdata);
    void LeastSquaresLineEstimate(vector<CarPoint> SelectedPoints, double& c, double& m);
    vector<CarPoint> findNearestPoint(vector<Line> lines);
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
    int runLidar(vector<PolPoint>& lidarDataPoints);
}

namespace Mapping_Functions{
    void storeMapPoints(vector<CarPoint> lidardata);
    void storeStatePoints(Matrix<float, dim, 1> State);

    void getMapBounds(vector<CarPoint> map, vector<float>& bounds);

    void gridDataProcess(vector<vector<GridPoint>>& gridNew,Matrix<float, dim, 1> State, bool firstRun);
    void gridDataAssosciationMove(vector<vector<GridPoint>>& gridNew, Matrix<float, dim, 1> State);
    void gridDataAssosciationMap(vector<vector<GridPoint>> gridOld, vector<vector<GridPoint>>& gridNew);
    bool gridDotBoundCheck(vector<CarPoint> searchMap, GridPoint point,float distThresh, vector<float> bounds);
    void gridMakeDots(vector<CarPoint> mapdata, vector<vector<GridPoint>>& points);
}

namespace Navigation_Functions{
    bool updateMovement(MatrixXf State);
    void landmarkExplore(CarPoint LM, MatrixXf State);
    void randomExplore();
    void updateExplorations(MatrixXf State, CarPoint Robot);
    void pathFinder();
    void motorControl();
    void moveCalibration(float distance);
}



class ExtendedKalmanFilter {
public:
    //Testing
    vector<CarPoint> TestValues;

    //Motion
    // float v = 1.3094206;
    // float w = -0.12391118;
    // float t = 0.1;

    float w = 0;
    float distance = 0;
    Matrix<float, dim, 1> State;
    
    

    ExtendedKalmanFilter();
    void updateMotion();
    void updateCovarianceOfRobot();
    void isNewLandmark();
    void getEstimatedObservation(float deltaX, float deltaY, float q);
    void getEstimatedObservationJacobian(float deltaX, float deltaY, float q);
    void getGainMatrix();
    void updateStateOfLandmark();
    void updateCovarianceOfLandmark();
    void runEKF();
    vector<CarPoint> observeEnvironment();

private:
    Matrix<float, dim, dim> Covariance;
    Matrix<float, dim, dim> Motion_Jacobian;
    Matrix<float, dim, dim> Motion_Noise;
    Matrix<float, 2, 2> Coordinate_Uncertainty;
    Matrix<float, 5, dim> F;
    Matrix<float, 2, 1> z;
    Matrix<float, 2, 1> z_cap;
    Matrix<float, 2, dim> Observation_Jacobian;
    Matrix<float, dim, 2> Gain;

    vector<CarPoint> Landmarks;
    CarPoint EstimatedLandmark;
    CarPoint ObservedLandmark;

    int NoLandmarksFound = 0;
    int LandmarkIndex = 0;
    float sigma_r = 0.5;
    float sigma_theta = 0.5;

    // float sigma_r = 0.0000005;
    // float sigma_theta = 0.0000005;

    bool LandmarkIsNew = false;
};

#endif
