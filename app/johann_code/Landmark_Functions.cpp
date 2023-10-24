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


    VectorXd fitWithLeastSquares(MatrixXd& X, VectorXd& y) {
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
  
    int evaluateModel(MatrixXd& X, VectorXd& y, VectorXd& theta, float inlierThreshold) {
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

    VectorXd ransac(MatrixXd& X, VectorXd& y, int maxIters, float inlierThreshold, int minInliers, int samplesToFit) {
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

    vector<VectorXd> manager(vector<float>& xCoords, vector<float>& yCoords, int sampleSize, int maxIters, float inlierThreshold, int minInliers) {
        int numSamples = xCoords.size() / sampleSize;
        vector<VectorXd> bestModels;
        
        for (int i = 0; i < numSamples; ++i) {
            vector<float> x, y;
            
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


    float calculateInterceptAngle(VectorXd& line1, VectorXd& line2) {
        float interAngle = PI / 2;
        float m1 = line1(0);
        float m2 = line2(0);
        float b1 = line1(1);
        float b2 = line2(1);

        if (m1 * m2 == -1) {
            return interAngle;
        } else {
            interAngle = abs(atan((m2 - m1) / (1 + m1 * m2)));
            return interAngle;
        }
    }

    VectorXd calculateInterceptPoint(VectorXd& line1, VectorXd& line2) {
        double m1 = line1(0);
        double m2 = line2(0);
        double b1 = line1(1);
        double b2 = line2(1);

        double x = (b2 - b1) / (m1 - m2);
        double y = x * m1 + b1;

        VectorXd result(2);
        result << x, y;
        return result;
    }

    vector<VectorXd> findCorners(vector<VectorXd>& bestModels, float angleThreshold) {
        vector<VectorXd> corners;

        for (size_t i = 0; i < bestModels.size(); ++i) {
            if (i < bestModels.size() - 1) {
                float interAngle = calculateInterceptAngle(bestModels[i], bestModels[i + 1]);
                VectorXd interceptPoint = calculateInterceptPoint(bestModels[i], bestModels[i + 1]);

                if (interAngle < PI / 2 + angleThreshold && interAngle > PI / 2 - angleThreshold) {
                    corners.push_back(interceptPoint);
                } else if (i < bestModels.size() - 2) {
                    float interAngle1 = calculateInterceptAngle(bestModels[i], bestModels[i + 2);
                    float interAngle2 = calculateInterceptAngle(bestModels[i + 1], bestModels[i + 2);
                    bool ang1 = false;
                    bool ang2 = true;

                    if (interAngle1 < PI / 2 + angleThreshold && interAngle1 > PI / 2 - angleThreshold) ang1 = true;
                    if (interAngle2 < PI / 2 + angleThreshold && interAngle2 > PI / 2 - angleThreshold) ang2 = false;

                    if (ang1 && ang2) {
                        VectorXd interceptPoint = calculateInterceptPoint(bestModels[i], bestModels[i + 2]);
                        corners.push_back(interceptPoint);
                    }
                }
            }
        }
        return corners;
    }

    vector<VectorXd> filterCorners(vector<VectorXd>& corners, vector<float>& xCoords, vector<float>& yCoords, float duplicateThreshold, float closenessThreshold) {
        vector<VectorXd> cleanCorners;

        for (size_t i = 0; i < corners.size(); ++i) {
            bool duplicate = false;

            for (size_t j = 0; j < cleanCorners.size(); ++j) {
                float dist = sqrt(pow(corners[i](0) - cleanCorners[j](0), 2) + pow(corners[i](1) - cleanCorners[j](1), 2));

                if (dist < duplicateThreshold) {
                    duplicate = true;
                    break;
                }
            }

            if (!duplicate) {
                cleanCorners.push_back(corners[i]);
            }
        }

        vector<VectorXd> closeCorners;

        for (size_t i = 0; i < cleanCorners.size(); ++i) {
            float distMin = 10000000;

            for (size_t j = 0; j < xCoords.size(); ++j) {
                float distTemp = sqrt(pow(xCoords[j] - cleanCorners[i](0), 2) + pow(yCoords[j] - cleanCorners[i](1), 2));

                if (distMin > distTemp) {
                    distMin = distTemp;
                }
            }

            if (distMin < closenessThreshold) {
                closeCorners.push_back(cleanCorners[i]);
            }
        }

        return closeCorners;
    }
}