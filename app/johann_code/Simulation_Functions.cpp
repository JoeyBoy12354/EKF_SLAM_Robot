#include "robot.h"

namespace Simulation_Functions{
    
    vector<PolPoint> generateLidarData(){
        vector<PolPoint> dataPoints;
        double r;
        for (int i = 0; i < 360; i++) {
            double angle = i % 90; // Reduce angle to the range [0, 90) degrees

            if (angle == 0) {
                angle = 0.1;
            }

            if (angle < 45 || angle >= 90) {
                r = abs(2 / cos(angle * PI / 180));
            } else {
                r = abs(2 / sin(angle * PI / 180));
            }

            dataPoints.push_back({static_cast<double>(i), r});
            //cout << "angle: " << i << " dist: " << r << endl;
        }

        return dataPoints;
    }

    double generateGaussian(double mean, double stddev) {
        static random_device rd;
        static mt19937 gen(rd());
        normal_distribution<double> distribution(mean, stddev);

        double randomValue = distribution(gen);
        
        // Ensure the generated value is between -1 and 1
        randomValue = min(1.0, max(-1.0, randomValue));

        return randomValue;
    }

    vector<CarPoint> landmarkNoise(vector<CarPoint> Landmarks){
        double mean = 0;
        double stddev = 0.01;
        for(int i =0;i<Landmarks.size();i++){
            Landmarks[i].x += generateGaussian(mean,stddev);
            Landmarks[i].y += generateGaussian(mean,stddev);
        }

        return Landmarks;
    }
}