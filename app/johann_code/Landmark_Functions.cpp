#include "robot.h"

#define CONV PI/180;

using namespace CSV_Functions;
using namespace Data_Functions;


namespace Landmark_Functions{

    // Function to calculate the perpendicular distance from a point to a line
    double perpendicularDistance(const CarPoint& point, Line& line) {
        return fabs((line.gradient * point.x - point.y + line.intercept)) / sqrt(1 + line.gradient * line.gradient);
    }

    // Function to find the point that corresponds best to a specific line
    void findInliers(vector<Line>& lines, vector<CarPoint>& points) {
        double distThresh = 0.01;

        for(Line& line : lines){
            for (const CarPoint& point : points) {
                double d_i = perpendicularDistance(point, line);
                if (d_i < distThresh) {
                    if(point.x > line.domain_max) line.domain_max = point.x;
                    if(point.x < line.domain_min) line.domain_min = point.x;

                    if(point.y > line.range_max) line.range_max = point.y;
                    if(point.y < line.range_min) line.range_min = point.y;
                }
            }
            //cout<<endl;
        }
    }

    // Function to Perform hough Transform
    vector<Line> houghTransform(vector<CarPoint>& points, int num_theta_bins, int num_rho_bins) {
        // Create Hough accumulator with specified number of bins for theta and rho
        vector<vector<int>> accumulator(num_theta_bins, vector<int>(num_rho_bins, 0));

        vector<CarPoint> mypoints;

        // Find maximum rho value (max distance from origin to a point)
        double max_rho = 0;
        for (const CarPoint& p : points) {
            double rho = sqrt(p.x * p.x + p.y * p.y);
            if (rho > max_rho) {
                max_rho = rho;
            }
        }

        // Compute theta and rho increments
        double d_theta = PI / num_theta_bins;
        double d_rho = (2 * max_rho) / num_rho_bins;

        // Perform Hough Transform
        for (const CarPoint& p : points) {
            for (int theta_bin = 0; theta_bin < num_theta_bins; ++theta_bin) {
                double theta = theta_bin * d_theta;
                double rho = p.x * cos(theta) + p.y * sin(theta);
                //cout<<"rho1: "<<p.x<<", "<<p.y<<endl;
                int rho_bin = static_cast<int>((rho + max_rho) / d_rho);
                accumulator[theta_bin][rho_bin]++;
            }
        }


        // Find the peaks in the accumulator
        vector<Line> detected_lines;
        int threshold = 600; // Adjust this threshold to control the number of detected lines
        for (int theta_bin = 0; theta_bin < num_theta_bins; ++theta_bin) {
            for (int rho_bin = 0; rho_bin < num_rho_bins; ++rho_bin) {
                if (accumulator[theta_bin][rho_bin] > threshold) {
                    double theta = theta_bin * d_theta;
                    double rho = (rho_bin * d_rho) - max_rho;

                    if(theta==0) theta = 0.0000001;
                    
                    // Calculate the gradient and intercept of the line
                    double gradient = -1.0 / tan(theta);
                    double intercept = rho / sin(theta);
                    
                    //cout<<"rho = "<<rho<<" theta = "<<theta<<" gradient  = "<<gradient<<endl;
                    Line newLine;
                    newLine.gradient=gradient;
                    newLine.intercept=intercept;
                    detected_lines.push_back(newLine);
                    
                }
            }
        }


        findInliers(detected_lines,points);
        return detected_lines;
    }

    // Function to calculate the perpendicular distance from a point to a point
    double pointDistance(CarPoint pointA, CarPoint pointB) {
        return sqrt( pow((pointA.x - pointB.x),2) + pow((pointA.y - pointB.y),2));
    }

    // Function to calculate the perpendicular distance from a Cornerpoint to a Cornerpoint
    double pointDistanceCorner(CornerPoint pointA, CornerPoint pointB) {
        return sqrt( pow((pointA.x - pointB.x),2) + pow((pointA.y - pointB.y),2));
    }

    vector<CarPoint> findCorners(vector<Line> lines){
        double distThresh = 0.1;
        vector<CarPoint> corners;

        for(int i =0;i<lines.size();i++){
            CarPoint Amin = {lines[i].domain_min,lines[i].range_min};
            CarPoint Amax = {lines[i].domain_max,lines[i].range_max};
            int count=0;//This will force a max of two points found for each line

            for(int j =0;j<lines.size();j++){
                CarPoint Bmin = {lines[j].domain_min,lines[j].range_min};
                CarPoint Bmax = {lines[j].domain_max,lines[j].range_max};

                //Do not compare line to itself
                if(i!=j && count < 2){
                    count++;
                    //push back one similar point per line if similar point is found
                    if(pointDistance(Amin,Bmin) < distThresh) lines[i].corner1 = Amin;
                    else if(pointDistance(Amin,Bmax) < distThresh) lines[i].corner1 = Amin;
                    else if(pointDistance(Amax,Bmax) < distThresh) lines[i].corner2 = Amax;
                    else if(pointDistance(Amax,Bmin) < distThresh) lines[i].corner2 = Amax;
                    else count--;
                }

            }
        }

        return corners;


    }

    //This algorithm assums the max and min values of each line are corners
    //Thus it will be true for a closed space
    vector<CarPoint> findLazyCorners(vector<Line> lines){
        vector<CarPoint> corners;
        double distThresh = 0.001;

        for(Line& line : lines){
            //Create two new points
            
            CarPoint newPoint1 = {line.domain_min,line.range_min};
            CarPoint newPoint2 = {line.domain_max,line.range_max};

            bool foundP1 = false;
            bool foundP2 = false;

            //Check if we don't already have the point
            for(CarPoint& corner : corners){
                if(pointDistance(corner,newPoint1) < 0.001) foundP1 = true;
                if(pointDistance(corner,newPoint2) < 0.001) foundP2 = true;
            }
            //cout<<endl;

            if(foundP1 == false) corners.push_back(newPoint1);
            if(foundP2 == false) corners.push_back(newPoint2);
        }

        return corners;
    }


    vector<CarPoint> findFancyCorners(vector<Line> lines){
        //Find Intercepts
        float angleThresh = 60*PI/180;
        float distThresh = 30;//Should be same or greater than RANSAC Tolerance
        vector<CarPoint> corners;
        bool angleGood = false;
        bool boundGood = false;

        for(int i =0; i<lines.size();i++){
            for(int j=0;j<lines.size();j++){
                CarPoint point;
                //Find x-coordinate
                point.x = (lines[j].intercept - lines[i].intercept)/(lines[i].gradient - lines[j].gradient);
                //Find y-coordinate
                point.y = lines[i].gradient*point.x + lines[i].intercept;
        

                //Is point creating a reasonable angle
                angleGood = false;
                //Is angle 90 degrees
                if(lines[i].gradient*lines[j].gradient==-1){
                    angleGood = true;
                }else{
                    //Absolute value to counter -90 being thrown out
                    float interAngle = abs(atan((lines[j].gradient - lines[i].gradient)/(1 + lines[i].gradient*lines[j].gradient)));
                    //Is angle within allowed bounds
                    if(PI/2 - angleThresh <= interAngle && interAngle <= PI/2 + angleThresh){
                        angleGood = true;
                    }
                }


                //Is point within reasonable bounds (we only check 1 bound cause more than that seems unneccessary)
                //Do NOTE we are using consensus limits not map limits (if corners are struggling we should use map limits)
                //Using consensus solves the problem of wrong intercepts between walls and objects in the map
                boundGood = false;
                //Check Domain
                if(point.x >= lines[i].domain_min - distThresh && lines[i].domain_max + distThresh >= point.x){
                    //Check Range
                    if(point.y >= lines[i].range_min - distThresh && lines[i].range_max + distThresh >= point.y){
                        boundGood = true;
                    }
                }

                if(boundGood == true && angleGood == true){
                    corners.push_back(point);
                }
                
            }
        }

        //maybe check if we have seen this corner before?

        

        return corners;
    }


    vector<CarPoint> findNearestPoint(vector<Line> lines){
        
        vector<CarPoint> closestPoints;
        CarPoint Ref;
        Ref.x =0;
        Ref.y =0;
        //For each line, calculate the point closest to 0,0
        for(int i =0;i<lines.size();i++){
            CarPoint temp = lines[i].ConsensusPoints[0];
            double smallD = pointDistance(Ref,lines[i].ConsensusPoints[0]);
            for(int j=1; j<lines[i].ConsensusPoints.size(); j++){
                double newD = pointDistance(Ref,lines[i].ConsensusPoints[j]);

                if(newD<smallD){
                    smallD = newD;
                    temp = lines[i].ConsensusPoints[j];
                }


            }
            closestPoints.push_back(temp);
        }

        return closestPoints;

    }

    //Assume points have been shifted with respect to robot
    void LeastSquaresLineEstimate(vector<CarPoint> SelectedPoints, double& c, double& m){
        
        int Size = SelectedPoints.size();
        double sumX;
        double sumY;
        double sumXY;
        double sumXX;
        
        for(int i = 0;i<Size;i++){
            sumX += SelectedPoints[i].x;
            sumY += SelectedPoints[i].y;
            sumXY += SelectedPoints[i].x*SelectedPoints[i].y;
            sumXX += SelectedPoints[i].x*SelectedPoints[i].x;
        }


        m = (Size*sumXY - sumX*sumY)/(Size*sumXX - pow(sumX,2));
        c = (sumY - m*sumX)/Size;
        
    }

    


    vector<Line> RANSAC(vector<CarPoint> laserdata){
        //two arrays corresponding to found lines
        vector<Line> lines;
        int totalLines = 0;

        //array of laser data points corresponding to the seen lines
        vector<CarPoint> linepoints = laserdata;
        int totalLinepoints = laserdata.size();
        
        //const int MAXTRIALS = 50000;//Max Times to run algorithm (pre-physical runs)
        const int MAXTRIALS = 8000;//Max Times to run algorithm
        const int MAXSAMPLE = 10;//Randomly selects X points
        //const int MINLINEPOINTS = 30;//If less than 40 points left don't bother trying to find consensous
        //const double RANSAC_TOLERANCE = 0.05; //RANSAC: if point is within x distance of line its part of line
        //const int RANSAC_CONSENSUS = 30; //RANSAC: at least X points in consensus required to determine if a line will be deemed valid 

        //As of right now 24 August we are in mm and doing about 10000 samples
        const double RANSAC_TOLERANCE = 10; //RANSAC: if point is within x distance of line its part of line [mm]
        const int RANSAC_CONSENSUS = 200; //RANSAC: at least X points in consensus required to determine if a line will be deemed valid 
        const int MINLINEPOINTS = 30;//If less than X points left don't bother trying to find consensous

        
        //RANSAC ALGORITHM
        int noTrials = 0;
        while(noTrials<MAXTRIALS && totalLinepoints > MINLINEPOINTS){
            //SAMPLING PHASE

            vector<CarPoint> rndSelectedPoints; //This will store our samples around the random point
            CarPoint temp;
            bool newpoint;

            //– Randomly select a subset S1 of n data points and
            //compute the model M1
            //Initial version chooses entirely randomly. Now choose
            //one point randomly and then sample from neighbours within some defined
            //radius

            //Grab a random centerPoint from all unassigned points
            int centerPoint = (rand() % totalLinepoints); //random value between 0 and totalLinePoints=laserData.size() -1
            rndSelectedPoints.push_back(linepoints[centerPoint]); 

             //Select MAXSAMPLE points randomly around centerPoint
            for(int i = 1; i<MAXSAMPLE; i++){
                newpoint = false;
                while(!newpoint){
                    //Get random point nearby centerpoint
                    int random = rand()%(MAXSAMPLE*2) - 10; //randomn value 0-10
                    //int neighbourPoint = centerPoint +  ((rand()%2) - 1) * (rand()%MAXSAMPLE);
                    int neighbourPoint = centerPoint +  random; 
                    
                    if(0<=neighbourPoint && neighbourPoint<linepoints.size()){
                        temp = linepoints[neighbourPoint];
                        for(int j = 0; j<i; j++){
                            if(rndSelectedPoints[j] == temp)
                                break; //point has already been selected
                            if(j>=i-1)
                                newpoint = true; //point has not already been selected
                        }
                    }

                }
                rndSelectedPoints.push_back(temp);
            }

            //compute model M1
            double c=0;
            double m=0;
            //y = c+ mx 


            //COMPUTE PHASE
            LeastSquaresLineEstimate(rndSelectedPoints, c, m);
            Line newLine;
            newLine.gradient=m;
            newLine.intercept=c;


            //– Determine the consensus set S1* of points is P
            //compatible with M1 (within some error tolerance)
            vector<CarPoint> consensusPoints;//assigned points
            int totalConsensusPoints = 0;

            vector<CarPoint> newLinePoints; //unassigned points
            int totalNewLinePoints = 0;

            
            //Line Plotting Variables
            double maxDomain = -1000000;
            double minDomain = 1000000;
            double maxRange = -1000000;
            double minRange = 1000000;


            double d = 0; 
            //Go through all points that are unassigned
            for(int i=0; i<totalLinepoints; i++){

                d = perpendicularDistance(linepoints[i], newLine);
                
                if (d<RANSAC_TOLERANCE){
                    //count all assigned points
                    //add points which are close to line
                    consensusPoints.push_back(linepoints[i]);
                    totalConsensusPoints++;

                    //Line Plotting Settings
                    if(linepoints[i].x>maxDomain){
                        maxDomain = linepoints[i].x;
                        newLine.domain_max = maxDomain;
                    }
                    if(linepoints[i].x<minDomain){
                        minDomain = linepoints[i].x;
                        newLine.domain_min = minDomain;
                    }
                    if(linepoints[i].y>maxRange){
                        maxRange = linepoints[i].y;
                        newLine.range_max = maxRange;
                    }
                    if(linepoints[i].y<minRange){
                        minRange = linepoints[i].y;
                        newLine.range_min = minRange;
                    }

                }
                else{
                    //count all points that are unassigned
                    //add points which are not close to line
                    newLinePoints.push_back(linepoints[i]);
                    totalNewLinePoints++;
                } 
            }

            //– If #(S1*) > t, use S1* to compute (maybe using least
            //squares) a new model M1*
            if(totalConsensusPoints>RANSAC_CONSENSUS){
                //cout<<"totalConsensusPoints"<<totalConsensusPoints<<endl;
                //Calculate updated line equation based on consensus points
                LeastSquaresLineEstimate(consensusPoints, newLine.intercept, newLine.gradient);

                //Remove points that have now been associated to this line
                linepoints = newLinePoints;
                totalLinepoints = totalNewLinePoints;

                //add line to found lines
                newLine.ConsensusPoints = consensusPoints;
                lines.push_back(newLine);

                totalLines++;
                //restart search since we found a line
                //noTrials = MAXTRIALS; //when maxtrials = debugging
                noTrials = 0;
            }

            else{
                //cout<<"totalConsensusPoints= "<<totalConsensusPoints<<endl;
                //DEBUG add point that we chose as middle value
                //tempLandmarks[centerPoint] = GetLandmark(laserdata[centerPoint], centerPoint, robotPosition);
                //– If #(S1*) < t, randomly select another subset S2 and
                //repeat
                //– If, after some predetermined number of trials there is
                //no consensus set with t points, return with failure
                noTrials++; 
            } 
        }
    
        return lines;
    }


    vector<Line> RANSAC_Manager(vector<CarPoint> laserdata){
        //Create Sections
        
        // const int DIST_RATIO = 30; //If the ratio between previous distance and current distance is greater than X there is a hole

        // int laserdataSize;
        // vector<Line> sectionLines;

        // vector<vector<CarPoint>> sectionVector;
        // vector<CarPoint> section;
        // section.push_back(laserdata[0]);
        // for(int i = 1; i<laserdata.size()-1;i++){
        //     float dist =  pointDistance(laserdata[i],laserdata[i+1]);
        //     float prevDist = pointDistance(laserdata[i-1],laserdata[i]);

        //     //New sample set
        //     if(dist/prevDist > DIST_RATIO){
        //         //Hole detected make new section
        //         vector<Line> lines = RANSAC2(section);
        //         sectionVector.push_back(section);

        //         cout<<"SectionSize = "<<section.size()<<" NoLine = "<<lines.size()<<" distRatio = "<<dist/prevDist<<endl;;
        //         section.clear();

        //         for(int j = 0; j < lines.size(); j++){
        //             sectionLines.push_back(lines[j]);
        //         }
        //     }else{
        //         //Add to current section
        //         section.push_back(laserdata[i]);
        //     }
            
        // }

        // sectionVector.push_back(section);

        // saveSectionsToCSV(sectionVector);

        const int NoSections = 1;
        const int sectionSize = int(laserdata.size()/NoSections);
        vector<CarPoint> section;
        vector<vector<CarPoint>> sectionVector;
        vector<Line> sectionLines;


        for(int i =0;i<NoSections;i++){
            section.clear();
            for(int j=0;j<sectionSize;j++){
                section.push_back(laserdata[i*sectionSize + j]);
            }

            
            sectionVector.push_back(section);
            vector<Line> lines = RANSAC2(section);

            cout<<"SectionSize = "<<sectionSize<<" True Size = "<<section.size()<<"NoLine = "<<lines.size();
            cout<<" ["<<section[0]<<section[section.size()-1]<<"]"<<endl;

            for(int j=0;j<lines.size();j++){
                sectionLines.push_back(lines[j]);
            }

        }
        saveSectionsToCSV(sectionVector);


        //We now have all the lines
        return sectionLines;
    }

    vector<Line> RANSAC2(vector<CarPoint> laserdata){
        cout<<"IN RANSAC2"<<endl;
        //two arrays corresponding to found lines
        vector<Line> lines;
        int totalLines = 0;

        //array of laser data points corresponding to the seen lines
        vector<CarPoint> linepoints = laserdata;
        int totalLinepoints = laserdata.size();
        
        //const int MAXTRIALS = 50000;//Max Times to run algorithm (pre-physical runs)
        const int MAXTRIALS = 5000;//Max Times to run algorithm
        const int MAXSAMPLE = 10;//Randomly selects X points
        //const int MINLINEPOINTS = 30;//If less than 40 points left don't bother trying to find consensous
        //const double RANSAC_TOLERANCE = 0.05; //RANSAC: if point is within x distance of line its part of line
        //const int RANSAC_CONSENSUS = 30; //RANSAC: at least X points in consensus required to determine if a line will be deemed valid 

        //As of right now 24 August we are in mm and doing about 10000 samples
        const double RANSAC_TOLERANCE = 10; //RANSAC: if point is within x distance of line its part of line [mm]
        const int RANSAC_CONSENSUS = 100; //RANSAC: at least X points in consensus required to determine if a line will be deemed valid 
        const int MINLINEPOINTS = 10;//If less than X points left don't bother trying to find consensous

        
        //RANSAC ALGORITHM
        int noTrials = 0;
        while(noTrials<MAXTRIALS && totalLinepoints > MINLINEPOINTS){
            //SAMPLING PHASE

            vector<CarPoint> rndSelectedPoints; //This will store our samples around the random point
            CarPoint temp;
            bool newpoint;

            //– Randomly select a subset S1 of n data points and
            //compute the model M1
            //Initial version chooses entirely randomly. Now choose
            //one point randomly and then sample from neighbours within some defined
            //radius

            //Grab a random centerPoint from all unassigned points
            int centerPoint = (rand() % totalLinepoints); //random value between 0 and totalLinePoints=laserData.size() -1
            rndSelectedPoints.push_back(linepoints[centerPoint]); 

             //Select MAXSAMPLE points randomly around centerPoint
             //cout<<"ENTER FOR LOOP"<<endl;
            for(int i = 1; i<MAXSAMPLE; i++){
                newpoint = false;
                //cout<<"ENTER WHILE LOOP"<<endl;
                while(!newpoint){
                    //Get random point nearby centerpoint
                    int random = rand()%(MAXSAMPLE*4) - 10; //randomn value 0-10
                    int neighbourPoint = centerPoint +  random; 
                    
                    if(0<=neighbourPoint && neighbourPoint<linepoints.size()){
                        temp = linepoints[neighbourPoint];
                        for(int j = 0; j<i; j++){
                            if(rndSelectedPoints[j] == temp){
                                //cout<<"been here"<<endl;
                                break; //point has already been selected
                            }  
                            if(j>=i-1){
                               newpoint = true; //point has not already been selected 
                            }
                                
                        }
                    }

                }
                //cout<<"LEFT WHILE LOOP"<<endl;
                rndSelectedPoints.push_back(temp);
            }

            //Non-random sample (sample sequentially)
            // int counter = 0;
            // int index = 0;
            // while(counter<=MAXSAMPLE){
            //     if(centerPoint - index >=0){
            //         rndSelectedPoints.push_back(linepoints[centerPoint - index]); 
            //         counter+=1;
            //     }
            //     if(centerPoint + index <= linepoints.size()){
            //         rndSelectedPoints.push_back(linepoints[centerPoint - index]); 
            //         counter+=1;
            //     }
            //     index+=1;
                

            // }

            //compute model M1
            double c=0;
            double m=0;
            //y = c+ mx 


            //COMPUTE PHASE
            LeastSquaresLineEstimate(rndSelectedPoints, c, m);
            Line newLine;
            newLine.gradient=m;
            newLine.intercept=c;


            //– Determine the consensus set S1* of points is P
            //compatible with M1 (within some error tolerance)
            vector<CarPoint> consensusPoints;//assigned points
            int totalConsensusPoints = 0;

            vector<CarPoint> newLinePoints; //unassigned points
            int totalNewLinePoints = 0;

            
            //Line Plotting Variables
            double maxDomain = -1000000;
            double minDomain = 1000000;
            double maxRange = -1000000;
            double minRange = 1000000;


            double d = 0; 
            //Go through all points that are unassigned
            for(int i=0; i<totalLinepoints; i++){

                d = perpendicularDistance(linepoints[i], newLine);
                
                if (d<RANSAC_TOLERANCE){
                    //count all assigned points
                    //add points which are close to line
                    consensusPoints.push_back(linepoints[i]);
                    totalConsensusPoints++;

                    //Line Plotting Settings
                    if(linepoints[i].x>maxDomain){
                        maxDomain = linepoints[i].x;
                        newLine.domain_max = maxDomain;
                    }
                    if(linepoints[i].x<minDomain){
                        minDomain = linepoints[i].x;
                        newLine.domain_min = minDomain;
                    }
                    if(linepoints[i].y>maxRange){
                        maxRange = linepoints[i].y;
                        newLine.range_max = maxRange;
                    }
                    if(linepoints[i].y<minRange){
                        minRange = linepoints[i].y;
                        newLine.range_min = minRange;
                    }

                }
                else{
                    //count all points that are unassigned
                    //add points which are not close to line
                    newLinePoints.push_back(linepoints[i]);
                    totalNewLinePoints++;
                } 
            }

            //– If #(S1*) > t, use S1* to compute (maybe using least
            //squares) a new model M1*
            if(totalConsensusPoints>RANSAC_CONSENSUS){
                //cout<<"totalConsensusPoints"<<totalConsensusPoints<<endl;
                //Calculate updated line equation based on consensus points
                LeastSquaresLineEstimate(consensusPoints, newLine.intercept, newLine.gradient);

                //Remove points that have now been associated to this line
                linepoints = newLinePoints;
                totalLinepoints = totalNewLinePoints;

                //add line to found lines
                newLine.ConsensusPoints = consensusPoints;
                lines.push_back(newLine);

                totalLines++;
                //restart search since we found a line
                //noTrials = MAXTRIALS; //when maxtrials = debugging
                noTrials = 0;
            }

            else{
                //cout<<"totalConsensusPoints= "<<totalConsensusPoints<<endl;
                //DEBUG add point that we chose as middle value
                //tempLandmarks[centerPoint] = GetLandmark(laserdata[centerPoint], centerPoint, robotPosition);
                //– If #(S1*) < t, randomly select another subset S2 and
                //repeat
                //– If, after some predetermined number of trials there is
                //no consensus set with t points, return with failure
                noTrials++; 
            } 
        }
    
        return lines;
    }


    

    CarPoint getIntercept(Line line1, Line line2){
        CarPoint interceptPoint;
        //Find x-coordinate
        interceptPoint.x = (line2.intercept - line1.intercept)/(line1.gradient - line2.gradient);
        //Find y-coordinate
        interceptPoint.y = line1.gradient*interceptPoint.x + line1.intercept;
        return interceptPoint;
    }

    float getAngle(Line line1, Line line2){
        //Is point creating a reasonable angle
        float interAngle = PI/2;
        //Is angle 90 degrees
        if(line1.gradient*line2.gradient==-1){
            return interAngle;
        }else{
            //Absolute value to counter -90 being thrown out
            interAngle = abs(atan((line2.gradient - line1.gradient)/(1 + line1.gradient*line2.gradient)));
            return interAngle;
        }
    }

    bool checkForHoles(vector<CarPoint> points) {
        float avg = 0;
        float max = -1;
        float temp_max;
        vector<float> distances;

        // Calculate distances and store them in the 'distances' vector
        for (int i = 0; i < points.size() - 1; i++) {
            distances.push_back(pointDistance(points[i], points[i + 1]));
        }

        // Bubble sort the 'distances' vector from smallest to largest
        int n = distances.size();
        for (int i = 0; i < n - 1; i++) {
            for (int j = 0; j < n - i - 1; j++) {
                if (distances[j] > distances[j + 1]) {
                    // Swap the elements if they are in the wrong order
                    float temp = distances[j];
                    distances[j] = distances[j + 1];
                    distances[j + 1] = temp;
                }
            }
        }

        // Calculate the mean of the 'distances' vector
        for (float distance : distances) {
            avg += distance;
        }
        avg /= distances.size();

        // Define an outlier threshold (3 times the mean)
        float outlierThreshold = 3 * avg;

        bool noHole = true;
        // Now you can look for outliers by comparing each distance to the threshold
        for (float distance : distances) {
            if (distance > outlierThreshold) {
                // Handle the outlier here (e.g., print it or store it in another vector)
                // You can also update 'max' if the outlier distance is the largest encountered so far
                noHole = false;
            
            }
        }

        return noHole;
    }


    vector<CarPoint> ANSAC_CORNER(vector<CarPoint> laserdata){
        cout<<"\n !!!!!!!!!!! IN ANSAC !!!!!!!!!!!!!!!\n"<<endl;
        //two arrays corresponding to found corners
        vector<CornerPoint> corners;
        int totalLines = 0;

        vector<Line> lines;
        //array of laser data points corresponding to the seen lines
        vector<CarPoint> linepoints = laserdata;
        int totalLinepoints = laserdata.size();

        //checks
        bool tolCheck = true;
        bool angleGood = false;
        bool replace = false;
        
        const int MAXSAMPLE = 20;//Selects X points in window

        //const double ANSAC_TOLERANCE = 23; //If point is within x distance of neighbour its part of a corner
        const float ANGLE_THRESHOLD_LOW = 60*PI/180; //If angle made by intercepts is within PI/2 +- X then keep corner
        const float ANGLE_THRESHOLD_HIGH = 120*PI/180; //If angle made by intercepts is within PI/2 +- X then keep corner
        const float DIST_THRESHOLD = 50; //If intercept point is within X of a corner we have then toss or replace

        const int INDEX_STEP= 1;//If no angle found in sample shift window by X points onwards.
        const float PLOT_LINE_LEN = 50; //Plot line domain is intercept.x +- X
        
        //RANSAC ALGORITHM
        int currIndex = 0;
        int count = 0;
        while(currIndex<totalLinepoints - MAXSAMPLE){
            //SAMPLING PHASE
            vector<CarPoint> selectedPoints; //This will store our samples around the next point
            for(int i =0;i<MAXSAMPLE;i++){
                selectedPoints.push_back(linepoints[i+currIndex]);
            }
            CarPoint centerPoint = selectedPoints[int(selectedPoints.size()/2)];

            //Check Tolerance
            // tolCheck = true;
            // for(int i = 0;i<selectedPoints.size()-1;i++){
            //     //If this is triggered then points are too far from neighbours and we should stop
            //     if(pointDistance(selectedPoints[i],selectedPoints[i+1]) > ANSAC_TOLERANCE){
            //         count++;
            //         tolCheck = false;
            //     }
            // }

            tolCheck = checkForHoles(selectedPoints);
            

            if(tolCheck == true){
                
                //COMPUTE PHASE
                //compute model M1
                float x_min = 1000000; //Only used for plotting
                float x_max = -1000000;
                double c=0;
                double m=0;
                vector<CarPoint> line1Points; //This will store our samples around the next point
                for(int i =0;i<int(MAXSAMPLE/2);i++){
                    line1Points.push_back(selectedPoints[i]);
                    if(selectedPoints[i].x > x_max){
                        x_max = selectedPoints[i].x;
                    }
                    if(selectedPoints[i].x < x_min){
                        x_min = selectedPoints[i].x;
                    }
                }
                LeastSquaresLineEstimate(line1Points, c, m);
                Line line1;
                line1.gradient=m;
                line1.intercept=c;
                line1.ConsensusPoints = line1Points;
                line1.domain_max = x_max;
                line1.domain_min = x_min;

                x_min = 1000000;
                x_max = -1000000;
                c=0;
                m=0;
                vector<CarPoint> line2Points; //This will store our samples around the next point
                for(int i =int(MAXSAMPLE/2); i<MAXSAMPLE; i++){
                    line2Points.push_back(selectedPoints[i]);
                    if(selectedPoints[i].x > x_max){
                        x_max = selectedPoints[i].x;
                    }
                    if(selectedPoints[i].x < x_min){
                        x_min = selectedPoints[i].x;
                    }
                }
                LeastSquaresLineEstimate(line2Points, c, m);
                Line line2;
                line2.gradient=m;
                line2.intercept=c;
                line2.ConsensusPoints = line2Points;
                line2.domain_max = x_max;
                line2.domain_min = x_min;


                //Is point creating a reasonable angle
                angleGood = false;
                float interAngle = getAngle(line1,line2);
                if(ANGLE_THRESHOLD_LOW <= interAngle && interAngle <= ANGLE_THRESHOLD_HIGH){
                    angleGood == true;
                }
               

                
                if(angleGood == true){
                    
                    CornerPoint interceptPoint;
                    //Find x-coordinate
                    interceptPoint.x = (line2.intercept - line1.intercept)/(line1.gradient - line2.gradient);
                    //Find y-coordinate
                    interceptPoint.y = line1.gradient*interceptPoint.x + line1.intercept;
                    interceptPoint.angle = interAngle;
                    cout<<"angle = "<<interAngle*180/PI<<" low="<<ANGLE_THRESHOLD_LOW*180/PI<<" hi="<<ANGLE_THRESHOLD_HIGH*180/PI<<endl;
                    cout<<"POINT = "<<interceptPoint;

                    //check if intercept point is basically a point we already have
                    CornerPoint replaceMe;
                    replaceMe.angle = 1000000;
                    int replaceMeIndex = 0;
                    double dist = 1000000000;
                    for(int i =0;i<corners.size();i++){
                        double dist_temp = pointDistanceCorner(interceptPoint,corners[i]);
                        if(dist>dist_temp){
                            dist = dist_temp;
                            replaceMe = corners[i];
                            replaceMeIndex = i;
                        }
                    }
                    // cout<<"CenterPoint = "<<centerPoint<<endl;
                    // cout<<"InterceptPoint = "<<interceptPoint<<endl;
                    // cout<<"Dist = "<<dist<<endl;
                    // cout<<"Max = "<<selectedPoints[selectedPoints.size()]<<"Min = "<<selectedPoints[0]<<endl;
                    // cout<<"line1 m= "<<line1.gradient<<" c="<<line1.intercept<<endl;
                    // cout<<"line2 m= "<<line2.gradient<<" c="<<line2.intercept<<endl;
                    // cout<<endl;

                    replace = false;
                    //If similar point
                    if(dist < DIST_THRESHOLD){
                        //If intercept angle is closer to 90 degrees replace similar angle
                        if(abs(PI/2 - replaceMe.angle) > abs(PI/2 - interceptPoint.angle) ){
                            replace = true;
                        }

                        //check if intercept is close to midpoint
                        if(replace == true){
                            //Add corner to corner list
                            corners[replaceMeIndex] = interceptPoint;
                            //Remove samples from list
                            currIndex = currIndex + MAXSAMPLE;

                            line1.domain_max = interceptPoint.x;
                            line1.domain_min = interceptPoint.x;
                            line1.range_max = interceptPoint.y;
                            line1.range_min = interceptPoint.y;
                            
                            line2.domain_max = interceptPoint.x;
                            line2.domain_min = interceptPoint.x;
                            line2.range_max = interceptPoint.y;
                            line2.range_min = interceptPoint.y;

                            lines[2*replaceMeIndex] = line1;
                            lines[2*replaceMeIndex + 1] = line2;
                        }else{
                            currIndex = currIndex + INDEX_STEP;
                        }

                    }else{
                        //Add corner to corner list
                        corners.push_back(interceptPoint);
                        //Remove samples from list
                        currIndex = currIndex + MAXSAMPLE;

                        line1.domain_max = interceptPoint.x;
                        line1.domain_min = interceptPoint.x;
                        line1.range_max = interceptPoint.y;
                        line1.range_min = interceptPoint.y;
                        
                        line2.domain_max = interceptPoint.x;
                        line2.domain_min = interceptPoint.x;
                        line2.range_max = interceptPoint.y;
                        line2.range_min = interceptPoint.y;

                        lines.push_back(line1);
                        lines.push_back(line2);
                    }

                }else{
                    currIndex = currIndex + INDEX_STEP;
                }
            }else{
                currIndex = currIndex + INDEX_STEP;
            }

            
            
        }

        writeLinesToCSV(lines);
        writeConsensusToCSV(lines);

        //Convert Corners to Cartesian
        vector<CarPoint> carCorners;
        for(int i =0;i<corners.size();i++){
            CarPoint point;
            point.x = corners[i].x;
            point.y = corners[i].y;
            carCorners.push_back(point);
        }

        cout<<"lenTotalfix = "<<linepoints.size()<<" count = "<< count<<endl;

        cout<<"\n\n!!!!!!!!!!!  LEAVING ANSAC !!!!!!!!!!!\n\n";
        return carCorners;
    }

    

}