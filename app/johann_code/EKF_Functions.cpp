#include "robot.h"

using namespace CSV_Functions;
using namespace Data_Functions;
using namespace Landmark_Functions;
using namespace Simulation_Functions;


ExtendedKalmanFilter::ExtendedKalmanFilter() {
    State.setZero();
    Motion_Jacobian.setIdentity();

    Motion_Noise.setZero();
    // Motion_Noise(0,0) = 0.25;
    // Motion_Noise(1,1) = 0.25;
    // Motion_Noise(2,2) = 0.27415568;

    // Motion_Noise(0,0) = 0.0001;
    // Motion_Noise(1,1) = 0.0001;
    // Motion_Noise(2,2) = 0.0001;

    //Covariance.setIdentity();
    Covariance.setZero();
    for (int n = 3; n < dim; n++) {
        Covariance(n, n) = 1;
    }


    Coordinate_Uncertainty << sigma_r*sigma_r, 0,
                                0,               sigma_theta*sigma_theta;

    F.setZero();
    F(0,0) = 1;
    F(1,1) = 1;
    F(2,2) = 1;

    z.setZero();
    z_cap.setZero();
    Observation_Jacobian.setZero();
    Gain.setZero();
    
}
//This function will update the EKF State Space Model with any changes in the robot's
//velocity[v], rotation[2] over time[t]
void ExtendedKalmanFilter::updateMotion() {
    float w2 = w;
    if(w2 == 0){
        w2 = 1;
    }
    float theta = State(2);
    float d_x = -1*(v/w2)*sin(theta) + (v/w2)*sin(theta + w*t);
    float d_y = (v/w2)*cos(theta) - (v/w2)*cos(theta + w*t);
    float d_theta = w*t;

    // d_x = 0.03336572;
    // d_y=0;
    // d_theta=0.02702237;

    // d_x = 0;
    // d_y = 0;
    // d_theta = 0;

    Motion_Jacobian(0,2) = -t*v*sin(State(2));
    Motion_Jacobian(1,2) = t*v*cos(State(2));;
    
    State(0) = State(0) + d_x;
    State(1) = State(1) + d_y;
    State(2) = State(2) + d_theta;

}

// Update covariance of robot using Gt and Rt
void ExtendedKalmanFilter::updateCovarianceOfRobot() {
    Covariance = Motion_Jacobian*Covariance*(Motion_Jacobian.transpose()) + Motion_Noise; //Textbook
    //Covariance = Motion_Jacobian.transpose()*Covariance*Motion_Jacobian + Motion_Noise;//Example
}

// Perform lidar observation and return landmarks (should be replace with function in Data_Functions)
vector<CarPoint> ExtendedKalmanFilter::observeEnvironment() {
    // vector<PolPoint> dataPoints = generateLidarData();
    // vector<CarPoint> carPoints = convertCartesian(dataPoints);
    // saveCarToCSV(carPoints);

    // vector<Line> detected_lines = houghTransform(carPoints);
    // writeLinesToCSV(detected_lines);

    // vector<CarPoint> corners = findLazyCorners(detected_lines);
    // writeCornersToCSV(corners);

    vector<CarPoint> corners;
    readCornersFromCSV(corners);

    return corners;
}

// Check if a new landmark is observed
void ExtendedKalmanFilter::isNewLandmark() {
    float distThresh = 0.8;

    //z is the Range-Bearing of the actual landmark we have just found from Sensors
    float deltaX = ObservedLandmark.x - State(0);
    float deltaY = ObservedLandmark.y - State(1);
    float q = deltaX*deltaX + deltaY*deltaY;
    z(0) = sqrt(q);
    z(1) = atan2(deltaY,deltaX) - State(2);


    vector<double> Distances;
    vector<int> Indexes;
    for(int i =3;i<dim;i=i+2){
        CarPoint StoredLandmark;
        StoredLandmark.x = State(i);
        StoredLandmark.y = State(i+1);

        Distances.push_back(pointDistance(StoredLandmark,ObservedLandmark));
        Indexes.push_back(i);
    }

    double smallestDistance = *min_element(Distances.begin(), Distances.end());
    int smallestDistanceIndex = Indexes[getIndex(Distances,smallestDistance)];
    cout<<"smallestDistance"<<smallestDistance<<" @"<<smallestDistanceIndex<<endl;
    


    
    if(smallestDistance<=distThresh){
        //This is a found landmark
        EstimatedLandmark.x = State(smallestDistanceIndex);
        EstimatedLandmark.y = State(smallestDistanceIndex+1);
        LandmarkIndex = smallestDistanceIndex;

        cout<<"Observed: ("<<ObservedLandmark.x<<","<<ObservedLandmark.y<<") "<<endl;
        cout<<"I have seen this one Before! Estimated: ("<<EstimatedLandmark.x<<","<<EstimatedLandmark.y<<") "<<endl;
    }else{
        //This is a new landmark
        cout<< "\nNewLandmark"<<endl;

        //Calculate landmark position
        EstimatedLandmark.x = State(0) + z(0)*cos(z(1) + State(2));
        EstimatedLandmark.y = State(1) + z(0)*sin(z(1) + State(2));

        NoLandmarksFound += 1;
        LandmarkIndex = 1+NoLandmarksFound*2;
        LandmarkIsNew = true;
        
        //Update State with landmark position
        State(LandmarkIndex) = EstimatedLandmark.x;
        State(LandmarkIndex+1) = EstimatedLandmark.y;  
    }


    
}

// Calculate estimated observation
void ExtendedKalmanFilter::getEstimatedObservation(float deltaX, float deltaY, float q) {
    z_cap(0) = sqrt(q);
    z_cap(1) = atan2(deltaY,deltaX) - State[2];
}

void ExtendedKalmanFilter::getEstimatedObservationJacobian(float deltaX, float deltaY, float q) {
    if(q==0){
        cout<<"q == 0!!!! (in getEstimatedObservationJacobian)";
        q = 1;
    }

    //Get the lower Observation Matrix
    Matrix <float, 2,5> Observation_Jacobian_low;
    float sq = sqrt(q);
    Observation_Jacobian_low << -1*sq*deltaX, -1*sq*deltaY,    0, sq*deltaX,  sq*deltaY, 
                                deltaY,       -1*deltaX,    -1*q, -1*deltaY,  deltaX;

    Observation_Jacobian_low = (1/q)*Observation_Jacobian_low;

    //Use F matrix to map H to higher space
    F(3,LandmarkIndex) = 1;
    F(4,LandmarkIndex+1) = 1;

    Observation_Jacobian = Observation_Jacobian_low*F;
    F(3,LandmarkIndex) = 0;
    F(4,LandmarkIndex+1) = 0;

    //cout<<"\nEstObJac: H = \n"<<Observation_Jacobian<<endl;
}

// Calculate gain matrix
void ExtendedKalmanFilter::getGainMatrix() {
    // cout<<"\n GAIN STUFF\n"<<endl;
    // cout<<"\n H = \n"<<Observation_Jacobian<<endl;
    // cout<<"\n P = \n"<<Covariance<<endl;
    // cout<<"\n C = \n"<<Coordinate_Uncertainty<<endl;

    Matrix<float,2,2> Gainx = Observation_Jacobian*Covariance*(Observation_Jacobian.transpose()) + Coordinate_Uncertainty;  
    // cout<<"\n Gainx_H*P*H.T = \n"<<Observation_Jacobian*Covariance*(Observation_Jacobian.transpose())<<endl;
    // cout<<"\n Gainx = \n"<<Gainx<<endl;
    Gain = Covariance*Observation_Jacobian.transpose() * Gainx.inverse();
    //cout<<"\n Gain = \n"<<Gain<<endl;
}

// Update State Matrix with new landmark
void ExtendedKalmanFilter::updateStateOfLandmark() {
    cout<<"\nGain = \n"<<Gain<<endl;
    cout<<"\nz-zcap = \n"<<z-z_cap<<endl;
    cout<<"\ngain* (z-zcap) = \n"<<(z-z_cap)<<endl;
    State = State + Gain*(z-z_cap);
    }

// Update Covariance Matrix with new landmark
void ExtendedKalmanFilter::updateCovarianceOfLandmark() {
    Matrix<float,dim,dim> I;
    I.setIdentity();
    Covariance = (I - Gain*Observation_Jacobian)*Covariance;
}

void ExtendedKalmanFilter::runEKF() {

    // Prediction step
    updateMotion();
    //cout << "LINE 2\nMu/State =\n" << State << "\nSigma/Covariance =\n" << Covariance << "\n";

    // cout << "\nLINE 3\nGt =\n" << Motion_Jacobian << "\nGt.T =\n" << Motion_Jacobian.transpose() << "\nSigma =\n" << Covariance << "\n";
    // cout << "\nLINE 3\nG*P*G.T =\n" << Motion_Jacobian*Covariance*Motion_Jacobian.transpose()<< "\nmotion_Noise =\n" << Motion_Noise << "\n";
    updateCovarianceOfRobot();
    //cout << "\nLINE 3\nGt =\n" << Motion_Jacobian << "\nmotion_Noise =\n" << Motion_Noise << "\nSigma =\n" << Covariance << "\n";

    // Correction step
    vector<CarPoint> landmarks = observeEnvironment();
    // vector<CarPoint> landmarks = TestValues;
    // landmarks = Simulation_Functions::landmarkNoise(landmarks);

    //cout<<"LANDMARKS = "<<fixed<<setprecision(10)<<landmarks[0]<<landmarks[1]<<landmarks[2]<<landmarks[3]<<endl;

    for (int i = 0; i < landmarks.size(); i++) {
        cout<<"\n\nXXXXX N E X T   P O I N T "<<i<<" XXXXX"<<endl;

        //Round Landmark
        //Round to 4 decimal
        // landmarks[i].x = round(landmarks[i].x * 10000.0) / 10000.0;
        // landmarks[i].y = round(landmarks[i].y * 10000.0) / 10000.0;


        ObservedLandmark = landmarks[i];


        isNewLandmark();

        float deltaX = EstimatedLandmark.x - State(0);
        float deltaY = EstimatedLandmark.y - State(1);
        float q = deltaX * deltaX + deltaY * deltaY;
        z_cap(0) = sqrt(q);
        z_cap(1) = atan2(deltaY, deltaX) - State(2);

        // cout<<"\nObsLM.x = "<<ObservedLandmark.x<<"ObsLM.y = "<<ObservedLandmark.y<<endl;
        // cout<<"EstLM.x = "<<EstimatedLandmark.x<<"EstLM.y = "<<EstimatedLandmark.y<<endl;
        // cout<<"deltaX = "<<deltaX<<" deltaY = "<<deltaY<<" q = "<<q<<endl;
        // cout<<"z.r = "<<z(0)<<"z.theta = "<<z(1)<<endl;
        // cout<<"z_cap.r = "<<z_cap(0)<<"z_cap.theta = "<<z_cap(1)<<endl;

        getEstimatedObservation(deltaX, deltaY, q);

        getEstimatedObservationJacobian(deltaX, deltaY, q);
        //cout << "\nLINE 4\nz_cap =\n" << z_cap << "\nH =\n" << Observation_Jacobian << "\nH_T =\n" << Observation_Jacobian.transpose() << "\n";
        //cout<< "\nState = \n"<<State<<endl;

        getGainMatrix();
        //cout << "\nLINE 5\nK =\n" << Gain << "\n";

        updateStateOfLandmark();
        cout << "\nLINE 6\nmu =\n" << State << "\n";

        updateCovarianceOfLandmark();
        cout << "\nLINE 7\nsigma =\n" << Covariance << "\n";
    }
}
