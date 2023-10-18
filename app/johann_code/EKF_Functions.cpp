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
    float test_sigma_r = 100;
    float test_sigma_theta = 0.1;
    float test_sigma_odo_x = 0.01;
    float test_sigma_odo_y = 0.01;
    float test_sigma_odo_theta = 0.017;


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

    //Covariance.setIdentity();

    //Use initialLandmarkCovariance_AtSi;
    float test = 1;
    Covariance.setZero();
    for (int n = 3; n < dim; n++) {
        Covariance(n, n) = test;
    }
    cout<<"initialLandmarkCovariance = "<<test<<endl;


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
    //Old Method
    // float w2 = w;
    // if(w2 == 0){
    //     w2 = 1;
    // }
    // float theta = State(2);
    // float d_x = -1*(v/w2)*sin(theta) + (v/w2)*sin(theta + w*t);
    // float d_y = (v/w2)*cos(theta) - (v/w2)*cos(theta + w*t);
    // float d_theta = w*t; this might be angular velocity

    // Motion_Jacobian(0,2) = -t*v*sin(State(2));
    // Motion_Jacobian(1,2) = t*v*cos(State(2));

    // float d_x = distance*cos(w);
    // float d_y = distance*sin(w);

    //flip d_x and d_y
    //The distance is set to negative due to forward motions being in the negative x-direction
    //It is assumed that lidar_x is already calculated with this in mind
    
    
    // float d_x = -distance*cos(w) + lidar_x;
    // float d_y = distance*sin(w)+ lidar_y;
    // float d_theta = w;

    CarPoint C = triangularRepositioning(State, w);// Get lidar point with odomotorey angle reading.

    float CAngle = State(2) + w;//Get angle from positive x to line between (0,0) and (C.x,C.y)
    float Cd_x = -distance*cos(CAngle); //DeltaX change from forward movement
    float Cd_y = distance*sin(CAngle); //DeltaY change from forward movement

    // cout<<"EKF: ++C = "<<C<<endl;
    // cout<<"EKF: ++Cangle = "<<CAngle<<endl;
    // cout<<"EKF: ++Cd_x = "<<Cd_x<<endl;
    // cout<<"EKF: ++Cd_y = "<<Cd_y<<endl;
    // cout<<"EKF: ++X = "<<C.x + Cd_x<<endl;
    // cout<<"EKF: ++Y = "<<C.y + Cd_y<<endl<<endl;

    

    // cout<<"EKF: distance = "<<distance<<"mm   |  Angle"<<w*180/PI<<endl;
    // cout<<"EKF: d_x: "<<Cd_x<<endl;
    // cout<<"EKF: d_y: "<<Cd_y<<endl;
    // cout<<"EKF: d_theta: "<<w*180/PI<<endl;
    

    // cout<<"EKF: distance = "<<distance<<"mm   |  Angle"<<w*180/PI<<endl;
    // cout<<"EKF: d_x: "<<-distance*cos(w)<<" + "<<lidar_x<<" = "<<d_x<<endl;
    // cout<<"EKF: d_y: "<<distance*sin(w)<<" + "<<lidar_y<<" = "<<d_x<<endl;
    // cout<<"EKF: d_theta: "<<d_theta<<endl;

    // Motion_Jacobian(0,2) = -distance*sin(State(2));
    // Motion_Jacobian(1,2) = distance*cos(State(2));
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

    //cout<<"\nEKF: dist = "<<distance<<"\nEKF: w = "<<w<<"\nEKF: d_x = "<<d_x<<"\nEKF: d_y = "<<d_y<<"\nEKF: d_theta = "<<d_theta<<endl;
    
    // State(0) = State(0) + d_x;
    // State(1) = State(1) + d_y;
    // State(2) = State(2) + d_theta;

    angleInBounds(State(2));

}

// Update covariance of robot using Gt and Rt
void ExtendedKalmanFilter::updateCovarianceOfRobot() {
    Covariance = Motion_Jacobian*Covariance*(Motion_Jacobian.transpose()) + Motion_Noise; //Textbook
    //Covariance = Motion_Jacobian.transpose()*Covariance*Motion_Jacobian + Motion_Noise;//Example
}

// Perform lidar observation and return landmarks (should be replace with function in Data_Functions)
vector<CarPoint> ExtendedKalmanFilter::observeEnvironment() {

    vector<CarPoint> corners;
    readCornersFromCSV(corners);

    return corners;
}

// Check if a new landmark is observed
void ExtendedKalmanFilter::isNewLandmark() {
    float distThresh = 200; //editing this effects the localization heavily (10)

    //z is the Range-Bearing of the actual landmark we have just found from Sensors (Observed)
    float deltaX = ObservedLandmark.x - State(0);
    float deltaY = ObservedLandmark.y - State(1);
    float q = deltaX*deltaX + deltaY*deltaY;
    z(0) = sqrt(q);
    z(1) = (PI - atan2(deltaY, deltaX)) - State(2);


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
    //cout<<"smallestDistance"<<smallestDistance<<" @"<<smallestDistanceIndex<<endl;
    

    if(smallestDistance<=distThresh){
        //This is a found landmark
        EstimatedLandmark.x = State(smallestDistanceIndex);
        EstimatedLandmark.y = State(smallestDistanceIndex+1);
        LandmarkIndex = smallestDistanceIndex;

        // cout<<"Re-Observed: ("<<ObservedLandmark.x<<","<<ObservedLandmark.y<<") "<<
        // " Stored:("<<EstimatedLandmark.x<<","<<EstimatedLandmark.y<<") "<<endl;
        //cout<<"I have seen this one Before! Estimated: ("<<EstimatedLandmark.x<<","<<EstimatedLandmark.y<<") "<<endl;
    }else{
        //This is a new landmark
        //cout<<"NewLandmark : "<<ObservedLandmark<<" smallDist = "<<smallestDistance<<" to ("<<State(smallestDistanceIndex)<<","<<State(smallestDistanceIndex+1)<<")"<<endl;

        //Calculate landmark position (why do this?)
        // EstimatedLandmark.x = State(0) + z(0)*cos(z(1) + State(2));
        // EstimatedLandmark.y = State(1) + z(0)*sin(z(1) + State(2));

        EstimatedLandmark.x = ObservedLandmark.x;
        EstimatedLandmark.y = ObservedLandmark.y;

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
    // float deltaX = EstimatedLandmark.x - State(0);
    // float deltaY = EstimatedLandmark.y - State(1);
    // float q = deltaX * deltaX + deltaY * deltaY;
    z_cap(0) = sqrt(q);
    z_cap(1) = (PI - atan2(deltaY, deltaX)) - State[2];
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


    // Observation_Jacobian_low << -1*sq*deltaX, sq*deltaY,    0, sq*deltaX,  -1*sq*deltaY, 
    //                             deltaY,       deltaX,    q, -1*deltaY,  -1*deltaX;

    // Observation_Jacobian_low << -1*sq*deltaX, sq*deltaY,    0, sq*deltaX,  sq*deltaY, 
    //                             deltaY,       -1*deltaX,    q, -1*deltaY,  deltaX;

    // Observation_Jacobian_low << -1*sq*deltaX, sq*deltaY,    0, sq*deltaX,  -1*sq*deltaY, 
    //                             -1*deltaY,       deltaX,    q, deltaY,  -1*deltaX;

    // Observation_Jacobian_low << -1*sq*deltaX, -1*sq*deltaY,    0, sq*deltaX,  -1*sq*deltaY, 
    //                             deltaY,       -1*deltaX,    q, deltaY,  -1*deltaX;




    

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
    cout<<"\nGain = "<<endl;
    for(int i =0;i<dim;i++){
        if(Gain(i,0) != 0 && Gain(i,0) != 0){
            cout<<Gain(i,0)<<" | "<<Gain(i,1)<<endl;
        }
    }

    Matrix<float, dim, 2> Gain_x = Gain;

    // cout<<"Gain(2,0)"<<Gain(2,0)<<endl;
    // cout<<"Gain(2,1)"<<Gain(2,1)<<endl;

    Gain_x(2,0) = -1*Gain_x(2,0);
    Gain_x(2,1) = -1*Gain_x(2,1);

    // cout<<"Gain(2,0)"<<Gain(2,0)<<endl;
    // cout<<"Gain(2,1)"<<Gain(2,1)<<endl;

    cout<<"\nGain = "<<endl;
    for(int i =0;i<dim;i++){
        if(Gain(i,0) != 0 && Gain(i,0) != 0){
            cout<<Gain_x(i,0)<<" | "<<Gain_x(i,1)<<endl;
        }
    }



    

    // //cout<<"z-zcap = \n"<<z-z_cap<<endl;

    cout<<"(z-z_cap).r = "<<(z-z_cap)(0)<<"(z-z_cap).theta = "<<(z-z_cap)(1)*180/PI<<endl;
    
    Matrix<float, 2, 1> delta_z = z-z_cap;
    Matrix<float, dim, 1> Gain2 = Gain_x*(z-z_cap);
    
    //delta_z(1) = delta_z(1)*-1;
    

    // if(z(1)-z_cap(1)!=0){
    //     cout<<"testing values"<<endl;
    //     calculateNoise();
    // }

    // cout<<"\n EKF: State1: x="<<State[0]<<", y="<<State[1]<<", w="<<State[2]*180/PI<<" deg"<<endl;
    // for(int i =3;i<dim;i=i+2){
    //     if(State[i] != 0 && State[i+1] != 0){
    //         cout<<"("<<State[i]<<","<<State[i+1]<<") | ";
    //     }
    // }
    // cout<<endl;

    // if(abs(Gain2(0))>abs(delta_z(0)*cos(delta_z(1))) ){
    //     cout<<"ForceChange Gain to 1 x_value: "<<delta_z(0)*sin(delta_z(1))/landmarks.size()<<endl;
    //     Gain2(0) = (-delta_z(0)*cos(delta_z(1)));


    // } 
    // if( abs(Gain2(1))>abs(delta_z(0)*sin(delta_z(1))) ){
    //     cout<<"ForceChange Gain to 1 y_value: "<<delta_z(0)*sin(delta_z(1))/landmarks.size()<<endl;
    //     Gain2(1) = delta_z(0)*sin(delta_z(1));
    // } 
    // if(abs(Gain2(2))>delta_z(1)){
    //     cout<<"ForceChange Gain to 1 theta_value: "<<delta_z(1)/landmarks.size()<<endl;
        
    //     Gain2(2) = delta_z(1);
    // }


    Gain2(2) = -Gain(2);
    Gain2(0) = -Gain(0);    
    Gain2(1) = -Gain(1);
    if(delta_z(1)*Gain2[2]<0 ){
        Gain2(2) = -Gain(2);
        cout<<"angle update opposite"<<endl;
    }
    if((ObservedLandmark.x-EstimatedLandmark.x)*Gain2[0] < 0){
        Gain2(0) = -Gain(0);
        cout<<"x update opposite"<<endl;
    }
    if((ObservedLandmark.y-EstimatedLandmark.y)*Gain2[1] < 0){
        //
        cout<<"y update opposite"<<endl;
    }



    cout<<"gain* (z-zcap) = \n"<<endl;
    cout<<Gain2[0];
    for(int i =0;i<dim;i++){
        if(Gain2[i] != 0){
            cout<<Gain2[i]<<endl;
        }
    }

    for(int i =3;i<dim;i=i+2){
        //This seems to lower landmark error and keep everything together
        // Gain2[i+1] = -1*Gain2[i+1];
        // Gain2[i] = -1*Gain2[i];
    }




    //State = State + Gain*(z-z_cap);
    State = State + Gain2;
    //State = State + Gain*(delta_z);
    //State = State + Gain*(z-z_cap);

    cout<<"LANDMARK ERROR: "<<pointDistance(EstimatedLandmark,ObservedLandmark)<<endl;
    
    cout<<"\n EKF: State2: x="<<State[0]<<", y="<<State[1]<<", w="<<State[2]*180/PI<<" deg"<<endl;
    for(int i =3;i<dim;i=i+2){
        if(State[i] != 0 && State[i+1] != 0){
            cout<<"("<<State[i]<<","<<State[i+1]<<") | ";
        }
    }
    cout<<endl;
    angleInBounds(State(2));

    // vector<float> stat{State(0),State(1),State(2)*180/PI,(z-z_cap)(0),(z-z_cap)(1)*180/PI,Gain2(0),Gain2(1)*180/PI};
    // stats.push_back(stat);

    }

// Update Covariance Matrix with new landmark
void ExtendedKalmanFilter::updateCovarianceOfLandmark() {
    Matrix<float,dim,dim> I;
    I.setIdentity();
    Covariance = (I - Gain*Observation_Jacobian)*Covariance;
}

void ExtendedKalmanFilter::runEKF() {

    // Prediction step
    //updateMotion(); (This is done in main.cpp)
    //cout << "LINE 2\nMu/State =\n" << State << "\nSigma/Covariance =\n" << Covariance << "\n";

    // cout << "\nLINE 3\nGt =\n" << Motion_Jacobian << "\nGt.T =\n" << Motion_Jacobian.transpose() << "\nSigma =\n" << Covariance << "\n";
    // cout << "\nLINE 3\nG*P*G.T =\n" << Motion_Jacobian*Covariance*Motion_Jacobian.transpose()<< "\nmotion_Noise =\n" << Motion_Noise << "\n";
    updateCovarianceOfRobot();
    //cout << "\nLINE 3\nGt =\n" << Motion_Jacobian << "\nmotion_Noise =\n" << Motion_Noise << "\nSigma =\n" << Covariance << "\n";

    // Correction step
    //landmarks = observeEnvironment();
    vector<CarPoint> landmarks = TestValues;
    // cout<<"size = "<<landmarks.size()<<endl;
    // cout<<"landmarks = "<<endl;
    // for(int i =0;i<landmarks.size();i++)
    //     cout<<landmarks[i];
    // cout<<endl;
    // landmarks = Simulation_Functions::landmarkNoise(landmarks);

    //cout<<"LANDMARKS = "<<fixed<<setprecision(10)<<landmarks[0]<<landmarks[1]<<landmarks[2]<<landmarks[3]<<endl;

    for (int i = 0; i < landmarks.size(); i++) {
        //cout<<"\n\nXXXXX N E X T   P O I N T "<<i<<" XXXXX"<<endl;

        
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
        z_cap(1) = (PI - atan2(deltaY, deltaX)) - State(2);

        cout<<"\n i = "<<i<<endl;
        cout<<"ObsLM.x = "<<ObservedLandmark.x<<"ObsLM.y = "<<ObservedLandmark.y<<endl;
        cout<<"EstLM.x = "<<EstimatedLandmark.x<<"EstLM.y = "<<EstimatedLandmark.y<<endl;
        // cout<<"deltaX = "<<deltaX<<" deltaY = "<<deltaY<<" q = "<<q<<endl;
        cout<<"z.r = "<<z(0)<<"z.theta = "<<z(1)*180/PI<<endl;
        cout<<"z_cap.r = "<<z_cap(0)<<"z_cap.theta = "<<z_cap(1)*180/PI<<endl;

        //getEstimatedObservation(deltaX, deltaY, q);

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
}
