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


    VectorXd fitWithLeastSquares(const MatrixXd& X, const VectorXd& y) {
        int numRows = X.rows();
        int numCols = X.cols();

        // Augment the feature matrix X with a column of ones for the bias term
        MatrixXd A(numRows, numCols + 1);
        A << X, MatrixXd::Ones(numRows, 1);

        // Compute the transpose of A
        MatrixXd A_transpose = A.transpose();

        // Compute A_transpose * A and A_transpose * y
        MatrixXd A_transpose_A = A_transpose * A;
        VectorXd A_transpose_y = A_transpose * y;

        // Solve the linear system using matrix multiplication
        VectorXd theta(numCols + 1);
        theta = A_transpose_A.inverse() * A_transpose_y;

        return theta;
    }   

    
    int evaluateModel(const MatrixXd& X, const VectorXd& y, const VectorXd& theta, double inlierThreshold) {
        int numInliers = 0;
        VectorXd b = VectorXd::Ones(X.rows());
        VectorXd yReshaped = y;
        MatrixXd A(X.rows(), X.cols() + 2);
        
        A << yReshaped, X, b;
        VectorXd thetaExtended(theta.size() + 1);
        thetaExtended << -1.0, theta;
        
        VectorXd distances = (A * thetaExtended).array().abs() / thetaExtended.segment(1, thetaExtended.size() - 1).norm();
        
        for (int i = 0; i < distances.size(); ++i) {
            if (distances(i) <= inlierThreshold) {
                numInliers++;
            }
        }
        
        return numInliers;
    }

    VectorXd ransac(const MatrixXd& X, const VectorXd& y, int maxIters, double inlierThreshold, int minInliers, int samplesToFit) {
        VectorXd bestModel;
        int bestModelPerformance = 0;
        
        int numSamples = X.rows();
        int numFeatures = X.cols();
        
        random_device rd;
        mt19937 gen(rd());
        uniform_int_distribution<int> distribution(0, numSamples - 1);
        
        for (int i = 0; i < maxIters; ++i) {
            vector<int> sampleIndices;
            for (int j = 0; j < samplesToFit; ++j) {
                int index = distribution(gen);
                sampleIndices.push_back(index);
            }
            
            MatrixXd sampledX(samplesToFit, numFeatures);
            VectorXd sampledy(samplesToFit);
            
            for (int j = 0; j < samplesToFit; ++j) {
                sampledX.row(j) = X.row(sampleIndices[j]);
                sampledy(j) = y(sampleIndices[j]);
            }
            
            VectorXd modelParams = fitWithLeastSquares(sampledX, sampledy);
            int modelPerformance = evaluateModel(X, y, modelParams, inlierThreshold);
            
            if (modelPerformance < minInliers) {
                continue;
            }
            
            if (modelPerformance > bestModelPerformance) {
                bestModel = modelParams;
                bestModelPerformance = modelPerformance;
            }
        }
        
        return bestModel;
    }

    vector<VectorXd> manager(const vector<double>& xCoords, const vector<double>& yCoords, int sampleSize, int maxIters, double inlierThreshold, int minInliers) {
        int numSamples = xCoords.size() / sampleSize;
        vector<VectorXd> bestModels;
        
        for (int i = 0; i < numSamples; ++i) {
            vector<double> x, y;
            
            for (int j = 0; j < sampleSize; ++j) {
                x.push_back(xCoords[i * sampleSize + j]);
                y.push_back(yCoords[i * sampleSize + j]);
            }
            
            MatrixXd xMatrix = Map<MatrixXd>(x.data(), sampleSize, 1);
            VectorXd yVector = Map<VectorXd>(y.data(), sampleSize);
            
            VectorXd result = ransac(xMatrix, yVector, maxIters, inlierThreshold, minInliers);
            bestModels.push_back(result);
        }
        
        return bestModels;
    }












}