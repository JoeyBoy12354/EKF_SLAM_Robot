#include "robot.h"

using namespace CSV_Functions;
using namespace Data_Functions;
using namespace Simulation_Functions;
using namespace Navigation_Functions;


ExtendedKalmanFilter::ExtendedKalmanFilter() {
    State.setZero();
    Motion_Jacobian.setIdentity();

    // float test_sigma_r = 0.001;//*
    // float test_sigma_theta = 0.001;//* 

    // float test_sigma_r = 0.35;//*
    // float test_sigma_theta = 4.37;//* 
    // float test_sigma_odo_x = 0.01;
    // float test_sigma_odo_y = 0.01;
    // float test_sigma_odo_theta = 0.017;

    

    // float test_sigma_r = 0.3;
    // float test_sigma_theta = 0.3;
    // float test_sigma_odo_x = 0.01;
    // float test_sigma_odo_y = 0.01;
    // float test_sigma_odo_theta = 0.017;

    // float test_sigma_r = 0.5;
    // float test_sigma_theta = 0.5;

    // float test_sigma_odo_x = 0.5;
    // float test_sigma_odo_y = 0.5;
    // float test_sigma_odo_theta = 30*PI/180;

    // float test_sigma_r = 0.3;
    // float test_sigma_theta = 0.3;

    float test_sigma_r = 0.25;
    float test_sigma_theta = 0.25;
    
    float test_sigma_odo_x = 0.5;
    float test_sigma_odo_y = 0.5;
    float test_sigma_odo_theta = 30*PI/180;


    cout<<"INIT_sigma_r = "<<test_sigma_r<<endl;
    cout<<"INIT_sigma_theta = "<<test_sigma_theta<<endl;
    cout<<"INIT_sigma_odo_x = "<<test_sigma_odo_x<<endl;
    cout<<"INIT_sigma_odo_y = "<<test_sigma_odo_y<<endl;
    cout<<"INIT_sigma_odo_theta = "<<test_sigma_odo_theta<<endl;



    Motion_Noise.setZero();
    Motion_Noise(0,0) = test_sigma_odo_x*test_sigma_odo_x;
    Motion_Noise(1,1) = test_sigma_odo_y*test_sigma_odo_y;
    Motion_Noise(2,2) = test_sigma_theta*test_sigma_odo_theta;

    // Motion_Noise(0,0) = sigma_odo_x*sigma_odo_x;
    // Motion_Noise(1,1) = sigma_odo_y*sigma_odo_y;
    // Motion_Noise(2,2) = sigma_theta*sigma_odo_theta;

    // Motion_Noise(0,0) = 0.0001;
    // Motion_Noise(1,1) = 0.0001;
    // Motion_Noise(2,2) = 0.0001;

    Covariance.setIdentity();

    //Use initialLandmarkCovariance_AtSi;
    // float test = 1;
    // Covariance.setZero();
    // for (int n = 0; n < dim; n++) {
    //     Covariance(n, n) = test;
    // }
    // cout<<"initialLandmarkCovariance = "<<test<<endl;


    Coordinate_Uncertainty << test_sigma_r*test_sigma_r, 0,
                                0,               test_sigma_theta*test_sigma_theta;

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
//x[0], y[1] and angle[2]
void ExtendedKalmanFilter::updateMotion() {
 
    cout<<"EKF UPDATE MOTION FOR "<<distance<<"mm and "<<w*180/PI<<endl;

    //Johann code
    CarPoint C = triangularRepositioning(State, w);// Get lidar point with odomotorey angle reading.

    float CAngle = w + State(2);//Get angle from positive x to line between (0,0) and (C.x,C.y)
    float Cd_x = distance*cos(CAngle); //DeltaX change from forward movement
    float Cd_y = distance*sin(CAngle); //DeltaY change from forward movement

    CarPoint robot;
    robot.x = State(0);
    robot.y = State(1);
    float dist2 = pointDistance(robot,C);

    //I believe this Jacobian's value can be estimated with v = distance/time
    //In essecence I think this should be the rate of change but yeah they cakculated it originally as just the distance in x and y
    Motion_Jacobian(0,2) = -(distance+dist2)*sin(State(2));
    Motion_Jacobian(1,2) = (distance+dist2)*cos(State(2));

    //State is now updated with C(x,y) calculated from State when performing triangular repositioning.
    //The d_x and d_y from the distance moved forward is then added 
    State(0) = C.x + Cd_x;
    State(1) = C.y + Cd_y;
    State(2) = State(2) + w;//This is added since we are calculating the new state which is the old + change to get the new.
    State(2) = pi_2_pi(State(2));


    //Atsi sims
    // float theta = State(2);
    // float d_x = distance*cos(theta);
    // float d_y = distance*sin(theta);
    // float d_theta = w; //this might be angular velocity

    // Motion_Jacobian(0,2) = -distance*sin(State(2));
    // Motion_Jacobian(1,2) = distance*cos(State(2));

    // State(0) = State(0) + d_x;
    // State(1) = State(1) + d_y;
    // State(2) = State(2) + d_theta;

}

// Update covariance of robot using Gt and Rt
void ExtendedKalmanFilter::updateCovarianceOfRobot() {
    //Covariance = Motion_Jacobian*Covariance*(Motion_Jacobian.transpose()) + Motion_Noise; //Textbook
    Covariance = Motion_Jacobian.transpose()*Covariance*Motion_Jacobian + Motion_Noise;//Example
}

// Perform lidar observation and return landmarks (should be replace with function in Data_Functions)
vector<CarPoint> ExtendedKalmanFilter::observeEnvironment() {

    vector<CarPoint> corners;
    readCornersFromCSV(corners);

    return corners;
}

float ExtendedKalmanFilter::mahalanobisDistance(CarPoint StoredPoint,int LMindex){
    // cout<<"z = "<<z(0)<<",  "<<z(1)<<endl;
    // cout<<"x = "<<State(0)<<",  y = "<<State(1)<<",  z = "<<State(2)*180/PI<<endl;
    // cout<<"In Maha Distance Point = "<<StoredPoint<<"Index = "<<LMindex<<endl;
    Matrix<float, 2, 1> z_cap_m;

    Matrix<double,1,2> delta;
    float deltaX = StoredPoint.x - State(0);
    float deltaY = StoredPoint.y - State(1);
    //double q = deltaX * deltaX + deltaY * deltaY;
    double q = pow(deltaX,2) + pow(deltaY,2);

    

    //cout<<"q = "<<q<<", sqrt(q) = "<<sqrt(q)<<"   ";
    z_cap_m(0) = sqrt(q);
   // cout<<"z_q = "<<z_cap_m<<endl;
    z_cap_m(1) = (atan2(deltaY, deltaX)) - State(2);
    
    //cout<<"z_angle = "<<z_cap_m(1)*180/PI<<" -> "<<pi_2_pi(z_cap_m(1))*180/PI<<endl;
    z_cap_m(1) = pi_2_pi(z_cap_m(1));

    // cout<<"DeltaX = "<<deltaX<<" deltaY = "<<deltaY<<endl;
    // cout<<"z_cap = "<<z_cap_m(0)<<", "<<z_cap_m(1)<<endl;
    Matrix<float, 2, 1> delta_z = z-z_cap_m;
    delta_z(1) = pi_2_pi(delta_z(1));

    // cout<<"Stored Point Shifted = \n"<<z_cap_m<<endl;
    // cout<<"Observed Point= \n"<<z<<endl;
    // cout<<"O.x = "<<z(0)*cos(z(1))<<" O.y = "<<z(0)*sin(z(1))<<endl;
    // cout<<"shifted S.x = "<<z_cap_m(0)*cos(z_cap_m(1))<<" S.y = "<<z_cap_m(0)*sin(z_cap_m(1))<<endl;

    //cout<<"Delta_z = \n"<<delta_z<<endl;

    //OBSERVATION JACOBIAN
    //Get the lower Observation Matrix
    Matrix <float, 2,5> Observation_Jacobian_low;
    float sq = sqrt(q);
    Observation_Jacobian_low << -1*sq*deltaX, -1*sq*deltaY,    0, sq*deltaX,  sq*deltaY, 
                                deltaY,       -1*deltaX,    -1*q, -1*deltaY,  deltaX;
    Observation_Jacobian_low = (1/q)*Observation_Jacobian_low;

    //Use F matrix to map H to higher space
    F(3,LMindex) = 1;
    F(4,LMindex+1) = 1;

    Matrix<float, 2, dim> Observation_Jacobian_m;
    Observation_Jacobian_m = Observation_Jacobian_low*F;
    F(3,LMindex) = 0;
    F(4,LMindex+1) = 0;

    Matrix<float,2,2> Gainx = Observation_Jacobian_m*Covariance*Observation_Jacobian_m.transpose() + Coordinate_Uncertainty; 

    float maha_distance = delta_z.transpose()*Gainx.inverse()*delta_z;

    return maha_distance;
}


float ExtendedKalmanFilter::directDistance(CarPoint StoredPoint){
    
    float deltaX = StoredPoint.x - State(0);
    float deltaY = StoredPoint.y - State(1);
    double q = pow(deltaX,2) + pow(deltaY,2);

    Matrix<float, 2, 1> z_cap_m;
    z_cap_m(0) = sqrt(q);
    z_cap_m(1) = (atan2(deltaY, deltaX)) - State(2);
    z_cap_m(1) = pi_2_pi(z_cap_m(1));

    CarPoint Stored = {z_cap_m(0)*cos(z_cap_m(1)),z_cap_m(0)*sin(z_cap_m(1))};
    CarPoint Observed = {z(0)*cos(z(1)),z(0)*sin(z(1))};
    // cout<<"Stored = "<<Stored<<endl;
    // cout<<"Observed = "<<Observed<<endl;

    return pointDistance(Stored,Observed);

}

void ExtendedKalmanFilter::isNewLandmark2(){
    //double distThresh = 1000;
    //double distThresh = 270;//Was 600
    //double distThresh = 420;//Was 600
    double distThresh = 500;//Was 600
    //double distThresh = 600;//Was 600
    //cout<<"In NewLandmark2"<<endl;

    vector<double> minDistances;
    vector<int> indexes;
    for(int i =3;i<3+NoLandmarksFound*2;i=i+2){
        CarPoint StoredPoint;
        StoredPoint.x = State(i);
        StoredPoint.y = State(i+1);

        float dirDistance = directDistance(StoredPoint);
        minDistances.push_back(dirDistance);

        indexes.push_back(i);
    }
    minDistances.push_back(distThresh);
    indexes.push_back(NoLandmarksFound);

    double smallestDistance = 1000000000;
    for(int i =0;i<minDistances.size();i++){
        if(minDistances[i]<smallestDistance){
            smallestDistance = minDistances[i];
        }
    }

    int minDistanceVectorIndex = getIndex(minDistances,smallestDistance);
    int smallestDistanceIndex = indexes[minDistanceVectorIndex];

    if(minDistanceVectorIndex == NoLandmarksFound){
        //This is a new Landmark
        //cout<<"New LM = "<<ObservedLandmark<<" @"<<smallestDistance<<endl;
        
        //Calculate landmark position (why do this? We do this cause z has not yet been shifted)
        EstimatedLandmark.x = State(0) + z(0)*cos(z(1) + State(2));
        EstimatedLandmark.y = State(1) + z(0)*sin(z(1) + State(2));

        if(NoLandmarksFound >= N){
            cout<<"\n EKF:LANDMARK OVERFLOW !!!!!! \n"<<endl;
            return;
        }

        NoLandmarksFound += 1;
        LandmarkIndex = 1+NoLandmarksFound*2;
        //cout<<"Landmark Index = "<<LandmarkIndex<<endl;
        LandmarkIsNew = true;
        
        //Update State with landmark position
        State(LandmarkIndex) = EstimatedLandmark.x;
        State(LandmarkIndex+1) = EstimatedLandmark.y;  
        

    }else{
        //This is a found landmark
        //cout<<"Found Obs = "<<ObservedLandmark<<" Stored =  ("<<State(smallestDistanceIndex)<<", "
        //<<State(smallestDistanceIndex+1)<<")"<<" @"<<smallestDistance<<endl;

        EstimatedLandmark.x = State(smallestDistanceIndex);
        EstimatedLandmark.y = State(smallestDistanceIndex+1);
        LandmarkIndex = smallestDistanceIndex;
    }
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
    Matrix<float,2,2> Gainx = Observation_Jacobian*Covariance*Observation_Jacobian.transpose() + Coordinate_Uncertainty;  
    Gain = Covariance*Observation_Jacobian.transpose() * Gainx.inverse();
}

void ExtendedKalmanFilter::calculateNoise(){
    float random1;
    float random2;

    float min_error_x = 10000;
    float min_error_y = 10000;
    float min_error_theta = 10000;

    float best_r1=1.23456;
    float best_r2=1.23456;

    Matrix<float, 2, 1> delta_z = z-z_cap;
    for(int i =0;i<10000000;i++){
        // Generate random float values between 0 and 10 with up to 2 decimal places
        srand(static_cast<unsigned>(time(nullptr)));
        random1 = static_cast<float>(rand() % 1001) / 100.0;
        srand(static_cast<unsigned>(time(nullptr)+PI));
        random2 = static_cast<float>(rand() % 1001) / 100.0;

        Coordinate_Uncertainty << random1*random1, 0,
                                    0, random2*random2;

        

        Matrix<float,2,2> Gainx = Observation_Jacobian*Covariance*(Observation_Jacobian.transpose()) + Coordinate_Uncertainty;  
        Gain = Covariance*Observation_Jacobian.transpose() * Gainx.inverse();
        Matrix<float, dim, 1> Gain2 = Gain*delta_z;


        if( (min_error_x > abs(Gain2(0) +State(0))) && (min_error_y > abs(Gain2(1) +State(1))) && (min_error_theta > abs(Gain2(2) +State(2))) ) {
            min_error_x = abs(Gain2(0) +State(0));
            min_error_y = abs(Gain2(1) +State(1));
            min_error_theta = abs(Gain2(2) +State(2));

            best_r1 = random1;
            best_r2 = random2;

        }
        
    }

    cout<<"\nx = "<<State(0)<<endl;
    cout<<"y = "<<State(1)<<endl;
    cout<<"theta = "<<State(2)*180/PI<<endl;
    cout<<"best_r1 = "<<best_r1<<"  best_r2 = "<<best_r2<<endl;
    cout<<"error_x = "<<min_error_x<<endl;
    cout<<"error_y = "<<min_error_y<<endl;
    cout<<"error_theta = "<<min_error_theta<<endl;

    Coordinate_Uncertainty << best_r1*best_r1, 0,
                                0, best_r2*best_r2;

    Matrix<float,2,2> Gainx = Observation_Jacobian*Covariance*(Observation_Jacobian.transpose()) + Coordinate_Uncertainty;  
    Gain = Covariance*Observation_Jacobian.transpose() * Gainx.inverse();
    Matrix<float, dim, 1> Gain2 = Gain*delta_z;

    cout<<"poss x = "<<State(0)+Gain2(0)<<endl;
    cout<<"poss y = "<<State(1)+Gain2(1)<<endl;
    cout<<"poss theta = "<<(State(2)+Gain2(2))*180/PI<<endl;
    
}

// Update State Matrix with new landmark
void ExtendedKalmanFilter::updateStateOfLandmark() {

    Matrix<float, 2, 1> delta_z = z-z_cap;
    delta_z(1) = pi_2_pi(delta_z(1));
    State = State + Gain*(delta_z);

    //cout<<"Using z: r =  "<<z[0]<<", a = "<<z[1]*180/PI<<" || cap: r "<<z_cap[0]<<", a = "<<z_cap[1]*180/PI<<" || D: r = "<<delta_z[0]<<", a = "<<delta_z[1]*180/PI<<endl;
    cout<<"\nD: r = "<<delta_z[0]<<", a = "<<delta_z[1]*180/PI<<endl;
    cout<<"G: r = "<<sqrt(pow((Gain*delta_z)[0],2) + pow((Gain*delta_z)[1],2) )<<", a = "<<State[2]*180/PI<<endl;
    cout<<"used%: r = "<<(abs(sqrt(pow((Gain*delta_z)[0],2) + pow((Gain*delta_z)[1],2) ))/delta_z[0])*100<<", a = "<<((Gain*delta_z)[2]/delta_z[1])*100<<endl;
    //cout<<"D: x = "<<delta_z[0]*cos(delta_z[1])<<" || y = "<<delta_z[0]*cos(delta_z[1])<<" || a = "<<delta_z[1]*180/PI<<endl;
    //cout<<"Gain*z: x = "<<(Gain*delta_z)[0]<<" || y = "<<(Gain*delta_z)[1]<<" || a = "<<(Gain*delta_z)[2]*180/PI<<endl;
    cout<<"EKF: after_ekf State: x="<<State[0]<<", y="<<State[1]<<", w="<<State[2]*180/PI<<" deg"<<endl;
    }

// Update Covariance Matrix with new landmark
void ExtendedKalmanFilter::updateCovarianceOfLandmark() {
    Matrix<float,dim,dim> I;
    I.setIdentity();
    Covariance = (I - Gain*Observation_Jacobian)*Covariance;
}


void ExtendedKalmanFilter::runEKF() {

    // Prediction step
    updateCovarianceOfRobot();

    // Correction step
    vector<PolPoint> landmarks = TestPolValues;



    for (int i = 0; i < landmarks.size(); i++) {
        //cout<<"\n\nXXXXX N E X T   P O I N T "<<i<<" XXXXX"<<endl;
        ObservedPolarLandmark = landmarks[i];
        z(0) = ObservedPolarLandmark.distance;
        z(1) = ObservedPolarLandmark.angle;
        z(1) = pi_2_pi(z(1));

        isNewLandmark2();

        float deltaX = EstimatedLandmark.x - State(0);
        float deltaY = EstimatedLandmark.y - State(1);
        float q = deltaX * deltaX + deltaY * deltaY;
        z_cap(0) = sqrt(q);
        z_cap(1) = (atan2(deltaY, deltaX)) - State(2);
        z_cap(1) = pi_2_pi(z_cap(1));

        // cout<<"\n i = "<<i<<endl;
        //cout<<"ObsLM.x = "<<ObservedLandmark.x<<"ObsLM.y = "<<ObservedLandmark.y<<endl;
        // cout<<"EstLM.x = "<<EstimatedLandmark.x<<"EstLM.y = "<<EstimatedLandmark.y<<endl;
        // cout<<"deltaX = "<<deltaX<<" deltaY = "<<deltaY<<" q = "<<q<<endl;
        // cout<<"z.r = "<<z(0)<<"z.theta = "<<z(1)*180/PI<<endl;
        // cout<<"z_cap.r = "<<z_cap(0)<<"z_cap.theta = "<<z_cap(1)*180/PI<<endl;

        getEstimatedObservationJacobian(deltaX, deltaY, q);
        //cout << "\nLINE 4\nz_cap =\n" << z_cap << "\nH =\n" << Observation_Jacobian << "\nH_T =\n" << Observation_Jacobian.transpose() << "\n";
        //cout<< "\nState = \n"<<State<<endl;

        getGainMatrix();
        //cout << "\nLINE 5\nK =\n" << Gain << "\n";

        updateStateOfLandmark();
        //cout << "\nEKF("<<i<<")_LINE 6\nmu =\n" << State << "\n";

        updateCovarianceOfLandmark();
        //cout << "\nLINE 7\nsigma =\n" << Covariance << "\n";
    }
    //cout<<"Prior "<<State(2)*180/PI<<endl;
    State(2) = pi_2_pi(State(2));
    //cout<<"Motion Noise = "<<Motion_Noise(0,0)<<" ,"<<Motion_Noise(1,1)<<" ,"<<Motion_Noise(2,2)<<endl;
    //cout<<"Obs Noise = \n"<<Coordinate_Uncertainty<<endl;
    //cout<<"Prior "<<State(2)*180/PI<<endl;
}
