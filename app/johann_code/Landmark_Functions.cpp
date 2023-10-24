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


    //Assume there will only ever be 2 samples
    VectorXf fitWithLeastSquares(const VectorXf& x, const VectorXf& y) {
        float m = (y(0)-y(1))/(x(0)-x(1));
        float c = y(0) - x(0)*m;

        VectorXf theta;
        theta(0) = m;
        theta(1) = c;

        return theta;
    }

    int getSampleIndex(int numSamples){
        random_device rd; // obtain a random number from hardware
        mt19937 gen(rd()); // seed the generator
        uniform_int_distribution<> distr(0, numSamples); // define the range
        return distr(gen);
    }
  
    int evaluateModel(const VectorXf& x, const VectorXf& y, const VectorXf& theta, float inlierThreshold) {
        int numInliers = 0;  // Initialize the inlier count to 0

        // Create a vector 'b' filled with 1s, used to represent the bias term in the model
        VectorXf b = VectorXf::Ones(x.rows());

        // Duplicate 'y' as 'yReshaped' (may not be necessary but for consistency)
        VectorXf yReshaped = y;

        // Create a matrix 'A' that augments 'yReshaped' and 'X' for model evaluation
        MatrixXf A(x.rows(), 3);
        A.col(0) = yReshaped;  // First column is 'yReshaped'
        A.rightCols(2) = x;    // Last two columns are 'X'

        // Create a 'thetaExtended' vector with an extra element
        // and set the first element to 0, while copying the rest from 'theta'
        VectorXf thetaExtended = VectorXf::Zero(theta.size() + 1);
        thetaExtended.segment(1, theta.size()) = theta;

        // Calculate distances by multiplying 'A' with 'thetaExtended'
        // and scale them by the norm of the model parameters (excluding bias term)
        VectorXf distances = (A * thetaExtended).array().abs() / thetaExtended.segment(1, thetaExtended.size() - 1).norm();

        // Iterate through the distances
        for (int i = 0; i < distances.size(); ++i) {
            // If the distance is less than or equal to the specified threshold, increment inlier count
            if (distances(i) <= inlierThreshold) {
                numInliers++;
            }
        }

    return numInliers;  // Return the total count of inliers
}

    VectorXf ransac(const VectorXf& X, const VectorXf& y, int maxIters, float inlierThreshold, int minInliers, int samplesToFit) {
        VectorXf bestModel = VectorXf::Zero(3);
        int bestModelPerformance = 0;

        int numSamples = X.rows();

        for (int i = 0; i < maxIters; ++i) {
            vector<int> sampleIndices;

            //Fetch random samples indexes
            for (int j = 0; j < samplesToFit; ++j) {
                sampleIndices.push_back(getSampleIndex(numSamples));
            }

            //Create sample vectors
            VectorXf sampledX(samplesToFit);
            VectorXf sampledy(samplesToFit);

            //Assign samples to vectors
            for (int j = 0; j < samplesToFit; ++j) {
                sampledX(j) = X(sampleIndices[j]);
                sampledy(j) = y(sampleIndices[j]);
            }

            VectorXf modelParams = fitWithLeastSquares(sampledX, sampledy);
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

    vector<VectorXf> manager(const vector<float>& xCoords, const vector<float>& yCoords, int sampleSize, int maxIters, float inlierThreshold, int minInliers) {
        int numSamples = xCoords.size() / sampleSize;
        vector<VectorXf> bestModels;

        cout<<"FUck man numSamples = "<<numSamples<<endl;

        for (int i = 0; i < numSamples; ++i) {
            cout<<"WHAT????";
            vector<float> x;
            vector<float> y;
            cout<<"Whats";

            for (int j = 0; j < sampleSize; ++j) {
                cout<<"ares";
                x.push_back(xCoords[i * sampleSize + j]);
                y.push_back(yCoords[i * sampleSize + j]);
            }

            cout<<"YAYA";

            VectorXf xVector = Map<VectorXf>(x.data(), x.size());
            VectorXf yVector = Map<VectorXf>(y.data(), y.size());

            VectorXf result = ransac(xVector, yVector, maxIters, inlierThreshold, minInliers);
            bestModels.push_back(result);
        }

        return bestModels;
    }


    float calculateInterceptAngle(VectorXf& line1, VectorXf& line2) {
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

    VectorXf calculateInterceptPoint(VectorXf& line1, VectorXf& line2) {
        double m1 = line1(0);
        double m2 = line2(0);
        double b1 = line1(1);
        double b2 = line2(1);

        double x = (b2 - b1) / (m1 - m2);
        double y = x * m1 + b1;

        VectorXf result(2);
        result << x, y;
        return result;
    }

    vector<VectorXf> findCorners(vector<VectorXf>& bestModels, float angleThreshold) {
        vector<VectorXf> corners;

        for (size_t i = 0; i < bestModels.size(); ++i) {
            if (i < bestModels.size() - 1) {
                float interAngle = calculateInterceptAngle(bestModels[i], bestModels[i + 1]);
                VectorXf interceptPoint = calculateInterceptPoint(bestModels[i], bestModels[i + 1]);

                if (interAngle < PI / 2 + angleThreshold && interAngle > PI / 2 - angleThreshold) {
                    corners.push_back(interceptPoint);
                } else if (i < bestModels.size() - 2) {
                    float interAngle1 = calculateInterceptAngle(bestModels[i], bestModels[i + 2]);
                    float interAngle2 = calculateInterceptAngle(bestModels[i + 1], bestModels[i + 2]);
                    bool ang1 = false;
                    bool ang2 = true;

                    if (interAngle1 < PI / 2 + angleThreshold && interAngle1 > PI / 2 - angleThreshold) ang1 = true;
                    if (interAngle2 < PI / 2 + angleThreshold && interAngle2 > PI / 2 - angleThreshold) ang2 = false;

                    if (ang1 && ang2) {
                        VectorXf interceptPoint = calculateInterceptPoint(bestModels[i], bestModels[i + 2]);
                        corners.push_back(interceptPoint);
                    }
                }
            }
        }
        return corners;
    }

    vector<VectorXf> filterCorners(vector<VectorXf>& corners, vector<float>& xCoords, vector<float>& yCoords, float duplicateThreshold, float closenessThreshold) {
        vector<VectorXf> cleanCorners;

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

        vector<VectorXf> closeCorners;

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