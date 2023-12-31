#include "robot.h"

using namespace CSV_Functions;
using namespace Data_Functions;


namespace Landmark_Functions{

    vector<CarPoint> getCorners(){
        cout<<"LM: In GETCORNERS"<<endl;

        // vector<CarPoint> dataPoints;
        // readCarFromCSVTest(dataPoints);
        // readCornersFromCSV(corners);

        // cout<<"A dataLen = "<<dataPoints.size()<<endl;

       



        int ret;
        ret = system("python3 cornerDetector.py ok go");
        //ret = system("/usr/bin/python3 cornerDetector.py ok go");
        cout << "ret/cpp = " << ret << endl;

        vector<CarPoint> corners;
        readCornersFromCSV(corners);

        //cout<<"LM: Number of Corners = "<<corners.size();

        return corners;
    }

    vector<CarPoint> getRANSACCorners(vector<CarPoint> dataPoints){
        vector<float> y;
        vector<float> x;

        for(int i =0;i<dataPoints.size();i++){
            x.push_back(dataPoints[i].x);
            y.push_back(dataPoints[i].y);
        }

        //cout<<"B x size = "<<x.size()<<endl;
        int sample_size = 80;
        int max_iters= 900;
        float inlier_thresh=0.002; //0.0005
        int min_inlier = 5; // 5
        float angleThreshold = 20.0 * M_PI / 180.0;
        float distanceThreshold = 120;
        float closenessThreshold = 40;
        float doubleLineThreshold = 300;
        
        
        //vector<VectorXf> bestModels = manager(x, y, sample_size, max_iters, inlier_thresh, min_inlier);
        vector<VectorXd> bestModels = manager2(x, y, sample_size, max_iters, inlier_thresh, min_inlier);
        vector<Vector2d> corners = findCorners2(bestModels, angleThreshold);
        vector<Vector2d> filteredCorners = filterCorners2(corners, x, y, distanceThreshold, closenessThreshold);
        vector<Vector2d> trueCorners = filterCorners2_2(filteredCorners, x, y, doubleLineThreshold);


        vector<CarPoint> final_corners;
        //cout<<"LM: Corners = ";
        for (int i =0;i<trueCorners.size();i++) {
            // Access the values inside the VectorXf
            CarPoint lm;
            lm.x = trueCorners[i][0];
            lm.y = trueCorners[i][1];
            final_corners.push_back(lm);
            //cout<<lm<<",";
        }
        //cout<<endl;

        return final_corners;
    }

    bool getInlierCheck(vector<CarPoint> dataPoints,CarPoint corner){
        float distance = -1000;
        float angleThreshold = 40.0 * M_PI / 180.0;
        CarPoint A;
        CarPoint B;
        // for(int i=0;i<dataPoints.size();i++){
        //     cout<<"dataPoints["<<i<<"] = "<<dataPoints[i];
        // }

        //Get the points witht the max distance from one another
        for(int i=0;i<dataPoints.size();i++){
            for(int j=0;j<dataPoints.size();j++){
                float d = pointDistance(dataPoints[i],dataPoints[j]);
                if(d>distance){
                    distance = d;
                    A = dataPoints[i];
                    B = dataPoints[j];
                }
            }
        }

        //Now calculate the line between corner and points
        float mA = (A.y - corner.y)/(A.x - corner.x);
        float mB = (B.y - corner.y)/(B.x - corner.x);
        float cA = A.y-A.x*mA;
        float cB = B.y-B.x*mB;

        double interAngle = M_PI / 2;
        if (mA * mB == -1) {
            interAngle = M_PI / 2;
        } else {
            interAngle = abs(atan((mB - mA) / (1 + mA * mB)));
        }

        // cout<<"\nPoints: "<<A<<", "<<corner<<", "<<B<<endl;
        // cout<<"distance = "<<distance<<endl;
        // cout<<"Inter Angle = "<<interAngle*180/M_PI<<endl;  
        // cout<<"mA= "<<mA<<", cA= "<<cA<<", mB= "<<mB<<", cB= "<<cB<<endl;

    
        

        
        if (interAngle < M_PI / 2 + angleThreshold && interAngle > M_PI / 2 - angleThreshold) {
            //cout<<"TRUE CORNER"<<endl;
            return true; //corner detected
        }
        //cout<<"FALSE CORNER"<<endl;
        return false; //corner not detected


    }


    vector<CarPoint> getMiniRANSACCorners(vector<CarPoint> dataPoints){
        vector<float> y;
        vector<float> x;

        for(int i =0;i<dataPoints.size();i++){
            x.push_back(dataPoints[i].x);
            y.push_back(dataPoints[i].y);
        }

        int sample_size = dataPoints.size();
        int max_iters= 400;
        float inlier_thresh=0.007; //0.0005
        int min_inlier = 5; // 5
        float angleThreshold = 40.0 * M_PI / 180.0;
        float distanceThreshold = 120;
        float closenessThreshold = 40;
        
        
        
        //vector<VectorXf> bestModels = manager(x, y, sample_size, max_iters, inlier_thresh, min_inlier);
        vector<VectorXd> bestModels = manager2(x, y, sample_size, max_iters, inlier_thresh, min_inlier);
        cout<<"mini: bestModel = "<<bestModels.size()<<endl;
        vector<Vector2d> corners = findCorners2(bestModels, angleThreshold);
        cout<<"mini: Corners = "<<corners.size()<<endl;
        vector<Vector2d> filteredCorners = filterCorners2(corners, x, y, distanceThreshold, closenessThreshold);
        

        

        vector<CarPoint> final_corners;
        cout<<"LM: Corners = ";
        for (int i =0;i<filteredCorners.size();i++) {
            // Access the values inside the VectorXf
            CarPoint lm;
            lm.x = filteredCorners[i][0];
            lm.y = filteredCorners[i][1];
            final_corners.push_back(lm);
            cout<<lm<<",";
        }
        cout<<endl;

        return final_corners;

    }


    VectorXf fitWithLeastSquares(VectorXf& x, VectorXf& y) {
        int numRows = x.rows();
        int numCols = x.cols();

        // Augment the feature matrix X with a column of ones for the bias term
        MatrixXf A(numRows, numCols + 1);
        A << x, MatrixXf::Ones(numRows, 1);

        // Compute the transpose of A
        MatrixXf A_transpose = A.transpose();

        // Compute A_transpose * A and A_transpose * y
        MatrixXf A_transpose_A = A_transpose * A;
        VectorXf A_transpose_y = A_transpose * y;

        // Initialize a vector for the solution theta
        VectorXf theta(numCols + 1);

        // Solve the linear system using Gauss elimination
        // Your Gauss elimination implementation here

        // For demonstration purposes, we'll use Eigen's built-in solver
        // Replace this part with your own Gauss elimination method
        theta = A_transpose_A.fullPivLu().solve(A_transpose_y);

        return theta;
    }

    int getSampleIndex(int numSamples){
        random_device rd; // obtain a random number from hardware
        mt19937 gen(rd()); // seed the generator
        uniform_int_distribution<> distr(0, numSamples); // define the range
        return distr(gen);
    }
  
    int evaluateModel(VectorXf& x, VectorXf& y, VectorXf& theta, float inlierThreshold) {
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

    VectorXf ransac(VectorXf& X, VectorXf& y, int maxIters, float inlierThreshold, int minInliers, int samplesToFit) {
        VectorXf bestModel = VectorXf::Zero(3);
        int bestModelPerformance = 0;

        int numSamples = X.rows();

        for (int i = 0; i < maxIters; i++) {
            vector<int> sampleIndices;

            //Fetch random samples indexes
            for (int j = 0; j < samplesToFit; ++j) {
                sampleIndices.push_back(getSampleIndex(numSamples));
            }

            //Create sample vectors
            VectorXf sampledX(samplesToFit);
            VectorXf sampledy(samplesToFit);

            //Assign samples to vectors
            for (int j = 0; j < samplesToFit; j++) {
                sampledX(j) = X(sampleIndices[j]);
                sampledy(j) = y(sampleIndices[j]);
            }

            VectorXf modelParams = fitWithLeastSquares(sampledX, sampledy);
            cout <<sampleIndices[0]<<" & "<<sampleIndices[1] <<"Model parameters: ";
            for (size_t i = 0; i < modelParams.size(); ++i) {
                cout << modelParams(i)<<", ";
            }

            int modelPerformance = evaluateModel(X, y, modelParams, inlierThreshold);
            cout<<" No Inliers = "<<modelPerformance <<endl;

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

    vector<VectorXf> manager(vector<float>& xCoords, vector<float>& yCoords, int sampleSize, int maxIters, float inlierThreshold, int minInliers) {
        int numSamples = (int)(xCoords.size() / sampleSize);
        vector<VectorXf> bestModels;

        cout<<"X numSamples = "<<xCoords.size()<<endl;
        cout<<"sampleSize = "<<sampleSize<<endl;

        cout<<"\nmaxIters = "<<maxIters<<endl;
        cout<<"InlierThresh = "<<inlierThreshold<<endl;
        cout<<"MinInliers = "<<minInliers<<endl;


        cout<<"\nnumSamples = "<<numSamples<<endl;

        for (int i = 0; i < numSamples; i++){
            vector<float> x;
            vector<float> y;

            VectorXf xVector(sampleSize);
            VectorXf yVector(sampleSize);

            for (int j = 0; j < sampleSize; j++) {
                x.push_back(xCoords[i * sampleSize + j]);
                y.push_back(yCoords[i * sampleSize + j]);
                xVector<<xCoords[i * sampleSize + j];
                yVector<<yCoords[i * sampleSize + j];


            }

            // VectorXf xVector = Map<VectorXf>(x.data(), x.size());
            // VectorXf yVector = Map<VectorXf>(y.data(), y.size());

            //VectorXf result;
            VectorXf result = ransac(xVector, yVector, maxIters, inlierThreshold, minInliers,2);
            bestModels.push_back(result);
        }

        return bestModels;
    }





    VectorXd fitWithLeastSquares2(const MatrixXd& X, const VectorXd& y) {
        MatrixXd A(X.rows(), X.cols() + 1);
        A << X, MatrixXd::Ones(X.rows(), 1);
        
        VectorXd theta = A.fullPivLu().solve(y);
        
        return theta;
    }

    int evaluateModel2(const MatrixXd& X, const VectorXd& y, const VectorXd& theta, double inlierThreshold) {
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

    VectorXd ransac2(const MatrixXd& X, const VectorXd& y, int maxIters, double inlierThreshold, int minInliers, int samplesToFit) {
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
            
            VectorXd modelParams = fitWithLeastSquares2(sampledX, sampledy);
            // cout <<sampleIndices[0]<<" & "<<sampleIndices[1] <<"Model parameters: ";
            // for (size_t i = 0; i < modelParams.size(); ++i) {
            //     cout << modelParams(i)<<", ";
            // }

            int modelPerformance = evaluateModel2(X, y, modelParams, inlierThreshold);

            //cout<<" No Inliers = "<<modelPerformance <<endl;
            
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

    vector<VectorXd> manager2(const vector<float>& xCoords, const vector<float>& yCoords, int sampleSize, int maxIters, double inlierThreshold, int minInliers) {
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
            
            VectorXd result = ransac2(xMatrix, yVector, maxIters, inlierThreshold, minInliers,2);
            bestModels.push_back(result);
        }
        
        return bestModels;
    }




    double calculateInterceptAngle2(const Vector2d& line1, const Vector2d& line2) {
        double interAngle = M_PI / 2;
        double m1 = line1(0);
        double m2 = line2(0);
        double b1 = line1(1);
        double b2 = line2(1);

        if (m1 * m2 == -1) {
            return interAngle;
        } else {
            interAngle = abs(atan((m2 - m1) / (1 + m1 * m2)));
            return interAngle;
        }
    }

    Vector2d calculateInterceptPoint2(const Vector2d& line1, const Vector2d& line2) {
        double m1 = line1(0);
        double m2 = line2(0);
        double b1 = line1(1);
        double b2 = line2(1);

        double x = (b2 - b1) / (m1 - m2);
        double y = x * m1 + b1;

        return Vector2d(x, y);
    }

    vector<Vector2d> findCorners2(const vector<VectorXd>& bestModels, double angleThreshold) {
        vector<Vector2d> corners;

        for (size_t i = 0; i < bestModels.size(); ++i) {
            if (i < bestModels.size() - 1) {
                double interAngle = calculateInterceptAngle2(bestModels[i], bestModels[i + 1]);
                Vector2d interceptPoint = calculateInterceptPoint2(bestModels[i], bestModels[i + 1]);

                if (interAngle < M_PI / 2 + angleThreshold && interAngle > M_PI / 2 - angleThreshold) {
                    corners.push_back(interceptPoint);
                } else if (i < bestModels.size() - 2) {
                    double interAngle1 = calculateInterceptAngle2(bestModels[i], bestModels[i + 2]);
                    double interAngle2 = calculateInterceptAngle2(bestModels[i + 1], bestModels[i + 2]);
                    bool ang1 = false;
                    bool ang2 = true;

                    if (interAngle1 < M_PI / 2 + angleThreshold && interAngle1 > M_PI / 2 - angleThreshold) ang1 = true;
                    if (interAngle2 < M_PI / 2 + angleThreshold && interAngle2 > M_PI / 2 - angleThreshold) ang2 = false;

                    if (ang1 && ang2) {
                        Vector2d interceptPoint = calculateInterceptPoint2(bestModels[i], bestModels[i + 2]);
                        corners.push_back(interceptPoint);
                    }
                }
            }
        }

        return corners;
    }

    vector<Vector2d> filterCorners2(const vector<Vector2d>& corners, vector<float>& xCoords, vector<float>& yCoords, double duplicateThreshold, double closenessThreshold) {
        //Filter cornerns of which there are duplicates
        vector<Vector2d> cleanCorners;
        for (size_t i = 0; i < corners.size(); ++i) {
            bool duplicate = false;

            for (size_t j = 0; j < cleanCorners.size(); ++j) {
                double dist = sqrt(pow(corners[i](0) - cleanCorners[j](0), 2) + pow(corners[i](1) - cleanCorners[j](1), 2));

                if (dist < duplicateThreshold) {
                    duplicate = true;
                    break;
                }
            }

            if (!duplicate) {
                cleanCorners.push_back(corners[i]);
            }
        }

        //Filter corners far away from LiDAR data
        vector<Vector2d> closeCorners;
        for (size_t i = 0; i < cleanCorners.size(); ++i) {
            double distMin = 10000000;

            for (size_t j = 0; j < xCoords.size(); ++j) {
                double distTemp = sqrt(pow(xCoords[j] - cleanCorners[i](0), 2) + pow(yCoords[j] - cleanCorners[i](1), 2));

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

    vector<Vector2d> filterCorners2_2(const vector<Vector2d>& closeCorners, vector<float>& xCoords, vector<float>& yCoords, double doubleLineThreshold){
        //Filter corners that were made by two disconjoined lines
        //Do mini-RANSACs on each groups
        vector<Vector2d> trueCorners;
        vector<CarPoint> group;
        bool detected = false;
        
        for(int i=0;i<closeCorners.size();i++){
            group.clear();
            //cout<<"\nBEGIN FILTER CORNER"<<endl;
            for(int j=0;j<xCoords.size();j++){
                //cout<<"["<<xCoords[j]<<", "<<yCoords[j]<<"],";
                double distTemp2 = sqrt(pow(xCoords[j] - closeCorners[i](0), 2) + pow(yCoords[j] - closeCorners[i](1), 2));
                if(distTemp2 < doubleLineThreshold){
                    CarPoint point(xCoords[j],yCoords[j]);
                    //cout<<","<<point;
                    group.push_back(point);
                } 
            }
            //cout<<"Group size = "<<group.size()<<endl;
            //vector<CarPoint> corners2 =  getMiniRANSACCorners(group);
            CarPoint myCorner(closeCorners[i](0),closeCorners[i](1));
            detected = getInlierCheck(group,myCorner);
            //cout<<"Corner: ("<<closeCorners[i](0)<<", "<<closeCorners[i](1)<<"): ";
            if(detected == true){
                // cout<<" corner len = "<<corners2.size()<<endl;
                trueCorners.push_back(closeCorners[i]);
            }else{
                // cout<<"corner len = 0"<<endl;
            } 
            detected = false; 
        }

        // cout<<"close corner len = "<<closeCorners.size()<<endl;
        // cout<<"true corner len = "<<trueCorners.size()<<endl;
        // for(int i =0;i<trueCorners.size();i++){
        //     cout<<trueCorners[i]<<",";
        // }

        return trueCorners;
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