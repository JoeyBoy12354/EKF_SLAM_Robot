#include "robot.h"

using namespace CSV_Functions;



namespace Landmark_Functions{

    vector<CarPoint> getCorners(){
        int ret;
        ret = system("python3 cornerDetector.py ok go");
        //ret = system("/usr/bin/python3 cornerDetector.py ok go");
        cout << "ret/cpp = " << ret << endl;

        vector<CarPoint> corners;
        readCornersFromCSV(corners);

        //cout<<"LM: Number of Corners = "<<corners.size();

        return corners;
    }

    
    // int evaluateModel(const MatrixXd& X, const VectorXd& y, const VectorXd& theta, double inlierThreshold) {
    //     int numInliers = 0;
    //     VectorXd b = VectorXd::Ones(X.rows());
    //     VectorXd yReshaped = y;
    //     MatrixXd A(X.rows(), X.cols() + 2);
        
    //     A << yReshaped, X, b;
    //     VectorXd thetaExtended(theta.size() + 1);
    //     thetaExtended << -1.0, theta;
        
    //     VectorXd distances = (A * thetaExtended).array().abs() / thetaExtended.segment(1, thetaExtended.size() - 1).norm();
        
    //     for (int i = 0; i < distances.size(); ++i) {
    //         if (distances(i) <= inlierThreshold) {
    //             numInliers++;
    //         }
    //     }
        
    //     return numInliers;
    // }

    // VectorXd ransac(const MatrixXd& X, const VectorXd& y, int maxIters, double inlierThreshold, int minInliers, int samplesToFit) {
    //     VectorXd bestModel;
    //     int bestModelPerformance = 0;
        
    //     int numSamples = X.rows();
    //     int numFeatures = X.cols();
        
    //     random_device rd;
    //     mt19937 gen(rd());
    //     uniform_int_distribution<int> distribution(0, numSamples - 1);
        
    //     for (int i = 0; i < maxIters; ++i) {
    //         vector<int> sampleIndices;
    //         for (int j = 0; j < samplesToFit; ++j) {
    //             int index = distribution(gen);
    //             sampleIndices.push_back(index);
    //         }
            
    //         MatrixXd sampledX(samplesToFit, numFeatures);
    //         VectorXd sampledy(samplesToFit);
            
    //         for (int j = 0; j < samplesToFit; ++j) {
    //             sampledX.row(j) = X.row(sampleIndices[j]);
    //             sampledy(j) = y(sampleIndices[j]);
    //         }
            
    //         VectorXd modelParams = fitWithLeastSquares(sampledX, sampledy);
    //         int modelPerformance = evaluateModel(X, y, modelParams, inlierThreshold);
            
    //         if (modelPerformance < minInliers) {
    //             continue;
    //         }
            
    //         if (modelPerformance > bestModelPerformance) {
    //             bestModel = modelParams;
    //             bestModelPerformance = modelPerformance;
    //         }
    //     }
        
    //     return bestModel;
    // }

    // vector<VectorXd> manager(const vector<double>& xCoords, const vector<double>& yCoords, int sampleSize, int maxIters, double inlierThreshold, int minInliers) {
    //     int numSamples = xCoords.size() / sampleSize;
    //     vector<VectorXd> bestModels;
        
    //     for (int i = 0; i < numSamples; ++i) {
    //         vector<double> x, y;
            
    //         for (int j = 0; j < sampleSize; ++j) {
    //             x.push_back(xCoords[i * sampleSize + j]);
    //             y.push_back(yCoords[i * sampleSize + j]);
    //         }
            
    //         MatrixXd xMatrix = Map<MatrixXd>(x.data(), sampleSize, 1);
    //         VectorXd yVector = Map<VectorXd>(y.data(), sampleSize);
            
    //         VectorXd result = ransac(xMatrix, yVector, maxIters, inlierThreshold, minInliers);
    //         bestModels.push_back(result);
    //     }
        
    //     return bestModels;
    // }












}