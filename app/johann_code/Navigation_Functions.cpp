#include "robot.h"

using namespace Landmark_Functions;
using namespace CSV_Functions;
using namespace Data_Functions;
using namespace Mapping_Functions;


int noNavTrials = 0;
vector<bool> LandmarksExplored;
double foundRadius = 250;//Robot must be within Xmm of LM for LM to be considered Explored
double closeness = 150;//Robot should travel distance: (Robot->LM) - closeness mm
float stuck_threshold_fixed = 3; //If the robot is still within 100mm of the same posistion after X attempted movements we assume it is stuck 
float stuck_threshold = stuck_threshold_fixed;
vector<float> prevState;

float theta;
float dist;


namespace Navigation_Functions{

    //Set motors and start them
    void motorControlGrid(float angle, float distance){

        // cout<<"!!!!!!!!!!!!!! !!!!!!!!! TESTING CONST theta & DIST"<<endl;

        cout<<"NAVI: Set angle = "<<angle*180/(PI)<<" deg Set Distance = "<<distance<<"mm"<<endl;
        // angle = 0;
        // distance = 0;


        //Send to motors
        writeMotorToCSV(angle,distance);

        cout<<"NAVI: Run python"<<endl;
    
        int ret;
        ret = system("python3 motorControl.py ok go");
        cout << "ret/cpp = " << ret << endl;

        

    }

    //rotate triangle around (0,0) for angle amount
    vector<CarPoint> rotateTriangle(const vector<CarPoint> triangle,double angleRad) {        
        // Create a rotation matrix
        // double cosA = cos(angleRad);
        // double sinA = sin(angleRad);

        double cosA = cos(angleRad);
        double sinA = sin(angleRad);

        // Initialize the rotation matrix
        double rotationMatrix[2][2] = {
            {cosA, -sinA},
            {sinA, cosA}
        };

        // Rotate each vertex of the triangle
        vector<CarPoint> rotatedTriangle;
        for (const CarPoint& vertex : triangle) {
            CarPoint rotatedVertex;
            rotatedVertex.x = vertex.x * rotationMatrix[0][0] + vertex.y * rotationMatrix[0][1];
            rotatedVertex.y = vertex.x * rotationMatrix[1][0] + vertex.y * rotationMatrix[1][1];
            rotatedTriangle.push_back(rotatedVertex);
        }

        return rotatedTriangle;
    }

    //Translate triangle towards reference point
    vector<CarPoint> translateTriangle(vector<CarPoint> triangle,CarPoint reference){
        triangle[0] = {reference.x,reference.y};
        triangle[1] = {triangle[1].x + reference.x, triangle[1].y + reference.y};
        triangle[2] = {triangle[2].x + reference.x, triangle[2].y + reference.y};

        return triangle;
    }

    //Perform rotation and translation of triangle state
    CarPoint triangularRepositioning(MatrixXf State, float angle){
        // cout<<"IN triangFunction "<<endl;
        // cout<<"State x = "<<State(0)<<" State y = "<<State(1)<<"State theta = "<<State(2)*180/PI<<endl;
        // cout<<"Angle = "<<angle*180/PI<<endl;
        vector<CarPoint> triangle_init = {{0, 0}, {-81, 71}, {-81, -71}};
        vector<CarPoint> triangle_rot = rotateTriangle(triangle_init,State(2));
        vector<CarPoint> triangle_shift = translateTriangle(triangle_rot,{State(0),State(1)});
        saveTriangleToCSV(triangle_shift);

        CarPoint C; //Will hold the value of the tip/front of triangle after rotation
        CarPoint B = {triangle_shift[0].x,triangle_shift[0].y}; //Will hold the value of the current tip/front of triangle
        CarPoint A; //This point will hold the coordinate of the corner around which we will rotate

        CarPoint T1 = A = {triangle_shift[1].x,triangle_shift[1].y};
        CarPoint T2 = A = {triangle_shift[2].x,triangle_shift[2].y};
        //cout<<"NAVI,GRID: Wheels = "<<T1<<","<<T2<<endl;
        if(angle>0){
            //Rotate around right corner
            A = {triangle_shift[1].x,triangle_shift[1].y};
        }else{
            //Rotate around left corner
            A = {triangle_shift[2].x,triangle_shift[2].y};
        } 

        //Rotate around A
        C.x=A.x+(B.x-A.x)*cos(angle) - (B.y-A.y)*sin(angle);
        C.y=A.y+(B.x-A.x)*sin(angle) + (B.y-A.y)*cos(angle);

        // cout<<"NAVI,TRIANGLE: tri_rot = "<<triangle_rot[0]<<","<<triangle_rot[1]<<","<<triangle_rot[2]<<endl;
        // cout<<"NAVI,TRIANGLE: tri_shift = "<<triangle_shift[0]<<","<<triangle_shift[1]<<","<<triangle_shift[2]<<endl;
        // cout<<"NAVI,TRIANGLE around A = "<<A<<" from B = "<<B<<" resulting C "<<C<<endl;

        return C;

    }


    GridPoint stateToGridDot(MatrixXf State, vector<vector<GridPoint>> gridMap){
        GridPoint myRobot;
        myRobot.x=State(0);
        myRobot.y=State(1);
        myRobot.trav=false;
        float min_distance = 10000000;
        GridPoint current;

        //Transform current position to a gridPoint
        for(int i =0;i<gridMap.size();i++){
            for(int j =0;j<gridMap[i].size();j++){
                float dist = gridPointDistance(gridMap[i][j],myRobot);
                if(min_distance>dist){
                    current = gridMap[i][j];
                    min_distance = dist;
                }
            }
        }

        return current;
    }


    CarPoint findNextPoint(MatrixXf State,vector<vector<GridPoint>> gridMap, bool& mapped){
        float smallDistance = 10000000;
        CarPoint closestPoint;
        CarPoint robotPoint;
        robotPoint.x = State(0);
        robotPoint.y = State(1);

        for(int i =0;i<gridMap.size();i++){
            for(int j=0;j<gridMap[i].size();j++){
                CarPoint tempPoint;
                tempPoint.x = gridMap[i][j].x;
                tempPoint.y = gridMap[i][j].y;

                //Only check points that have not been traversed
                if(gridMap[i][j].trav == false){
                    mapped = false;
                    float temp_dist = pointDistance(tempPoint,robotPoint);
                    if(temp_dist<smallDistance){
                        smallDistance = temp_dist;
                        closestPoint = tempPoint;
                    }
                }  
            }
        }

        return closestPoint;
    }

    CarPoint findNextPointRadius(MatrixXf State, vector<vector<GridPoint>> gridMap, bool& mapped){
        int scalar = 3;

        //The problem with the algorithm is that it encourages bouncing where it will jump between a high y value then a low value
        //Up and down over and over again
        
        CarPoint nextPoint;
        GridPoint current = stateToGridDot(State,gridMap);
        float case1;
        float case2;
        float case3;

        //The goal of this algorithm is to eliminate doing SLAM on intermdiary dots

        for(int k =0;k<scalar;k++){
            //This will mean that as we do not find traversable grid points we decrease our search space
            float radius = grid_xstep*(scalar-k);

            //if point's x-value, y-value or both differ by radius then we should select it
            for(int i =0;i<gridMap.size();i++){
                for(int j=0;j<gridMap[i].size();j++){
                    //Only check points that have not been traversed
                    if(gridMap[i][j].trav == false){
                        mapped = false;
                        
                        //Calculate the 3 cases
                        case1 = abs(gridMap[i][j].x-current.x);
                        case2 = abs(gridMap[i][j].y-current.y);
                        case3 = abs(gridMap[i][j].x-current.x) + abs(gridMap[i][j].y-current.y);


                        //This does allow for diagonal movements
                        if( case1  == radius || case2  == radius || case3  == 2*radius ){
                            nextPoint.x = gridMap[i][j].x;
                            nextPoint.y = gridMap[i][j].y;
                            return nextPoint;
                        }
                    }
                }
            }
        }

        //If we made it here the grid dot is further away than our search space or all dots have been traversed
        nextPoint = findNextPoint(State,gridMap,mapped);

        return nextPoint;
    }

    // CarPoint findNextPointStraight(MatrixXf State, vector<vector<GridPoint>> gridMap, bool& mapped){
    //     cout<<"!!!\nNAVI: Find Point Straight"<<endl;
    //     //I want the algorithm to maximize covering unknown dots
    //     //Or just have it cover maximum straights

    //     //Take current position and look up,down,left and right
    //     int scalar = 5;
    //     GridPoint current = stateToGridDot(State,gridMap);
    //     float orientation = State(2)*180/PI; // Get the orientation

    //     cout<<"This is us = ("<<current.x<<", "<<current.y<<") orientation = "<<orientation<<endl;

    //     CarPoint case1Point;
    //     CarPoint case2Point;
    //     CarPoint case3Point;

    //     bool case1Bool = false;
    //     bool case2Bool = false;
    //     bool case3Bool = false;

    //     float case1;
    //     float case2;
    //     float case3;

    //     for(int k =0;k<scalar;k++){
    //         //This will mean that as we do not find traversable grid points we decrease our search space
    //         float radius = grid_xstep*(scalar-k);

    //         //if point's x-value, y-value or both differ by radius then we should select it
    //         for(int i =0;i<gridMap.size();i++){
    //             for(int j=0;j<gridMap[i].size();j++){
    //                 //Only check points that have not been traversed
    //                 if(gridMap[i][j].trav == false){
    //                     mapped = false;
                        
    //                     //Calculate the 3 cases
    //                     case1 = abs(gridMap[i][j].x-current.x);
    //                     case2 = abs(gridMap[i][j].y-current.y);
    //                     case3 = abs(gridMap[i][j].x-current.x) + abs(gridMap[i][j].y-current.y);

    //                     cout<<"G: ("<<gridMap[i][j].x<<","<<gridMap[i][j].y<<") -> xd = "<<abs(gridMap[i][j].x-current.x)<<", yd = "<<abs(gridMap[i][j].y-current.y)<<endl;

    //                     if( case1  == radius && case1Bool == false){
                            
    //                         case1Point.x = gridMap[i][j].x;
    //                         case1Point.y = gridMap[i][j].y;
    //                         cout<<"case1 = "<<case1<<"case1Point = "<<case1Point<<endl;
    //                         case1Bool = true;
    //                     }

    //                     if( case2  == radius && case2Bool == false ){
    //                         case2Point.x = gridMap[i][j].x;
    //                         case2Point.y = gridMap[i][j].y;
    //                         cout<<"case2 = "<<case2<<"case2Point = "<<case2Point<<endl;
    //                         case2Bool = true;
    //                     }

    //                     if( case3  == 2*radius && case3Bool == false ){
    //                         case3Point.x = gridMap[i][j].x;
    //                         case3Point.y = gridMap[i][j].y;
    //                         cout<<"case3 = "<<case3<<"case3Point = "<<case3Point<<endl;
    //                         case3Bool = true;
    //                     }
    //                 }
    //             }
    //         }
    //     }


    //     //Here we have access to all 3 of them
    //     //We should give preference to same direction
    //     //We should give preference to majority unknown if possible
    //     //We should give preference to far distances

    //     //We must first decide on an orientation 0/+-180 or +-90

    //     if( (abs(orientation) - 180 || abs(orientation)) < (abs(orientation) - 90) && (abs(orientation) - 180 || abs(orientation))  < (abs(orientation) - 45 || abs(orientation) - 135 ) && case1Bool == true){
    //         cout<<"NAVI: nextPoint: We on X-axis and point on x exists"<<endl;
    //         //Assume we on the x-axis so case1
    //         return case1Point;
    //     }else if( (abs(orientation) - 90) < (abs(orientation) - 45 || abs(orientation) - 135 ) && case2Bool == true){
    //         cout<<"NAVI: nextPoint: We on Y-axis and point on y exists"<<endl;
    //         //Assume we closer to the y-axis so case2
    //         return case2Point;
    //     }else if(case1Bool == true && case2Bool == true && case3Bool == true ){
    //         cout<<"NAVI: nextPoint: We on some-axis and point on all exists"<<endl;
    //         //Assume we closer to the diagonal-axis so case3 and all directions are available
            
    //         //We choose the shortest distance
    //         CarPoint point(current.x,current.y);
    //         float d1 = pointDistance(point,case1Point);
    //         float d2 = pointDistance(point,case2Point);
    //         float d3 = pointDistance(point,case3Point);
    //         cout<<"d1 = "<<d1<<", d2 = "<<d2<<", d3 = "<<d3<<endl;

    //         if(d1<d2 && d1<d3){
    //             return case1Point;
    //         }else if(d2<d1 && d2<d3){
    //             return case2Point;
    //         }else if(d3<d1 && d2<d3){
    //             return case3Point;
    //         }            
    //     }else if(case3Bool == true){
    //         cout<<"NAVI: nextPoint: We on some-axis and point on diag exists"<<endl;
    //         //Assume we are closer to diagonal and only diagonal available
    //         return case3Point;
    //     }

    //     cout<<"NAVI: nextPoint: We did not know what to do"<<endl;
    //     //If we made it here the grid dot is further away than our search space or all dots have been traversed
    //     return findNextPoint(State,gridMap,mapped);

    // }


    CarPoint findNextPointStraight(MatrixXf State, vector<vector<GridPoint>> gridMap, bool& mapped){
        cout<<"!!!\nNAVI: Find Point Straight"<<endl;
        //I want the algorithm to maximize covering unknown dots
        //Or just have it cover maximum straights

        //Take current position and look up,down,left and right
        int scalar = 5;
        GridPoint current = stateToGridDot(State,gridMap);
        float orientation = State(2)*180/PI; // Get the orientation
        int closestDirection = 0;

        if ((orientation >= -45 && orientation <= 45)) {
            closestDirection = 0;
        } else if (orientation >= 45 && orientation <= 135)  {
            closestDirection = 90;
        } else if (orientation >= -135 && orientation < -45) {
            closestDirection = -90;
        } else if ((orientation >= 135 && orientation <= 180) || (orientation <= -135 && orientation >= -180)) {
            closestDirection = 180;
        }

        cout<<"This is us = ("<<current.x<<", "<<current.y<<") orientation = "<<orientation<<" closestDirection = "<<closestDirection<<endl;

        CarPoint xpPoint; //On + x-axis
        CarPoint ypPoint; //On + y-axis
        CarPoint xnPoint;
        CarPoint ynPoint; 

        bool xpBool = false;
        bool ypBool = false;
        bool xnBool = false;
        bool ynBool = false;

        float xDelta;
        float yDelta;

        for(int k =0;k<scalar;k++){
            //This will mean that as we do not find traversable grid points we decrease our search space
            float radius = grid_xstep*(scalar-k);

            //if point's x-value, y-value or both differ by radius then we should select it
            for(int i =0;i<gridMap.size();i++){
                for(int j=0;j<gridMap[i].size();j++){
                    //Only check points that have not been traversed
                    if(gridMap[i][j].trav == false){
                        mapped = false;
                        
                        //Calculate the 3 cases
                        xDelta = gridMap[i][j].x-current.x;
                        yDelta = gridMap[i][j].y-current.y;

                        cout<<"radius = "<<radius<<endl;
                        cout<<"G: ("<<gridMap[i][j].x<<","<<gridMap[i][j].y<<") -> xd = "<<abs(gridMap[i][j].x-current.x)<<", yd = "<<abs(gridMap[i][j].y-current.y)<<endl;

                        //Positive x
                        if( abs(xDelta)  == radius && gridMap[i][j].x>current.x && yDelta == 0 && xpBool == false){
                            
                            xpPoint.x = gridMap[i][j].x;
                            xpPoint.y = gridMap[i][j].y;
                            cout<<"xpPoint = "<<xpPoint<<endl;
                            xpBool = true;

                            if(closestDirection == 0){
                                cout<<"Solution is on our axis"<<endl;
                                return xpPoint;
                            }
                        }

                        //Negative x
                        if( abs(xDelta)  == radius && gridMap[i][j].x<current.x && yDelta == 0 && xnBool == false){
                            
                            xnPoint.x = gridMap[i][j].x;
                            xnPoint.y = gridMap[i][j].y;
                            cout<<"xnPoint = "<<xnPoint<<endl;
                            xnBool = true;

                            if(closestDirection == 180){
                                cout<<"Solution is on our axis"<<endl;
                                return xnPoint;
                            }
                        }

                        //Positive y
                        if( abs(yDelta)  == radius && gridMap[i][j].y>current.y && xDelta == 0 && ypBool == false){
                            
                            ypPoint.x = gridMap[i][j].x;
                            ypPoint.y = gridMap[i][j].y;
                            cout<<"ypPoint = "<<ypPoint<<endl;
                            ypBool = true;

                            if(closestDirection == 90){
                                cout<<"Solution is on our axis"<<endl;
                                return ypPoint;
                            }
                        }

                        //Negative y
                        if( abs(yDelta)  == radius && gridMap[i][j].y<current.y && xDelta == 0 && ynBool == false){
                            
                            ynPoint.x = gridMap[i][j].x;
                            ynPoint.y = gridMap[i][j].y;
                            cout<<"ynPoint = "<<ynPoint<<endl;
                            ynBool = true;

                            if(closestDirection == -90){
                                cout<<"Solution is on our axis"<<endl;
                                return ynPoint;
                            }
                        }
                        
                    }
                }
            }
        }


        //We are here if we did not find a solution on our axis
        if(closestDirection == 0){
            if(ypBool == true){
                return ypPoint;
            }else if(ynBool == true){
                return ynPoint;
            }else if(xnBool == true){
                return xnPoint;
            }
        }else if(closestDirection == 90){
            if(xpBool == true){
                return xpPoint;
            }else if(xnBool == true){
                return xnPoint;
            }else if(ynBool == true){
                return ynPoint;
            }
        }else if(closestDirection == -90){
            if(xpBool == true){
                return xpPoint;
            }else if(xnBool == true){
                return xnPoint;
            }else if(ypBool == true){
                return ypPoint;
            }
        }else if(closestDirection == 180){
            if(ypBool == true){
                return ypPoint;
            }else if(ynBool == true){
                return ynPoint;
            }else if(xpBool == true){
                return xpPoint;
            }
        }

        

        cout<<"NAVI: nextPoint: We did not know what to do"<<endl;
        //If we made it here the grid dot is further away than our search space or all dots have been traversed
        return findNextPoint(State,gridMap,mapped);

    }



    bool mapMovement(MatrixXf State, vector<vector<GridPoint>> gridMap, vector<CarPoint>& path){
        cout<<"In pre-map movement"<<endl;
        //take grid map
        bool mapped = true;
        CarPoint closestPoint;


        if(path.size() == 0){
            //find closest non-traversed point
            //closestPoint = findNextPoint(State,gridMap,mapped);

            closestPoint = findNextPointStraight(State,gridMap,mapped);

            cout<<"ClosestPoint = "<<closestPoint<<endl;

            //Update Path to get There
            path = pathFinder(State,closestPoint);
            closestPoint = path[0];

            //Remove this element from the path
            path.erase(path.begin());
        }
        //Path still exists
        else{
            mapped = false;
            closestPoint = path[0];
            cout<<"Follow Path"<<closestPoint<<endl;
            //Remove this element from the path
            path.erase(path.begin());
        }

        cout<<"mapped = "<<mapped;
        
        if(mapped == false){
            cout<<"Go to -> "<<closestPoint<<endl;
            updateMovement(closestPoint,State);
        }   
        

        return mapped;


    }


    bool postMapMovement(MatrixXf State, vector<CarPoint>& path, bool home){
        CarPoint closestPoint(0,0);
        //No Path Then generate a path Home
        if(path.size() == 0){
            //Update Path to get There
            path = pathFinder(State,closestPoint);
            closestPoint = path[0];

            //Remove this element from the path
            path.erase(path.begin());

        }else{
            closestPoint = path[0];
            //Remove this element from the path
            path.erase(path.begin());

            if(closestPoint.x ==0 && closestPoint.y ==0){
                home = true;
            }
        }


        updateMovement(closestPoint,State);

        return home;
    }


    float wallAvoidance(MatrixXf State, float angle){
        float turnableDistance = 300; //Total distance from object required to make a turn (REMEMBER SAME AS IN MC Python)
        float turnableDistance2 = 150;
        vector<CarPoint> map;
        CarPoint bot(State(0),State(1));
        readCarFromCSV(map); //Read latest scan

        vector<CarPoint> collisionLeft;
        vector<CarPoint> collisionRight;//unused

        CarPoint minL;
        CarPoint minR;
        float mindistL = 1000;
        float mindistR = 1000;
        
        float d=0;
        int noCollisionPoints = 0;

        cout<<"NAVI: wallAvoidance collision Points = ";
        for(int i =0;i<map.size();i++){
            if(pointDistance(map[i],bot) < turnableDistance){
                //In here we have collision points

                noCollisionPoints+=1;
                //S
                float deltaX = map[i].x - State(0);
                float deltaY = map[i].y - State(1);
                float angle_coll = atan2(deltaY,deltaX) - State(2);//Get collision angle
                angle_coll = pi_2_pi(angle_coll);
                
                CarPoint C = triangularRepositioning(State,angle_coll);

                if(angle_coll>0 && abs(angle_coll) < (abs(angle) +10*PI/180 )){
                    collisionLeft.push_back(C);
                    d = pointDistance(C,bot);
                    //cout<<", L:"<<C<<" | d:"<<d;
                    if(d<mindistL){
                        mindistL = d;
                        minL = C;
                    }
                }else if(angle_coll<0 && abs(angle_coll) < (abs(angle) +10*PI/180 )){
                    collisionRight.push_back(C);
                    d = pointDistance(C,bot);
                    //cout<<", R:"<<C<<" | d:"<<d;
                    if(d<mindistR){
                        mindistR = d;
                        minR = C;
                    }
                }                
            }
        }
        cout<<endl;

        cout<<"No of collision points = "<<noCollisionPoints<<endl;
        cout<<"mindistL: "<<mindistL<<" mindistR: "<<mindistR<<endl;
        cout<<"minL: "<<minL<<" minR: "<<minR<<endl;

        if(mindistL<turnableDistance2 && mindistR<turnableDistance2){
            cout<<"NAVI: AVOID!! BOTH ANGLES ARE BAD"<<endl;
            return 0;
            
        }

        if(mindistL>mindistR){
            if(angle<0){
                //cout<<"Angle change from: "<<angle*180/PI<<" -> "<<(angle - PI/2)*180/PI<<endl;
                cout<<"Angle change from: "<<angle*180/PI<<" -> "<<(angle + 2*PI)*180/PI<<endl;
                return angle + 2*PI;           
            }
        }else if(mindistR<mindistL){
            if(angle>0){
                //cout<<"Angle change from: "<<angle*180/PI<<" -> "<<(angle + PI/2)*180/PI<<endl;
                cout<<"Angle change from: "<<angle*180/PI<<" -> "<<(angle - 2*PI)*180/PI<<endl;
                return angle - 2*PI;
            }
        }

        return angle;
    }


    float wallAvoidanceForward(MatrixXf State, float angle, CarPoint postPoint, CarPoint goalPoint){
        cout<<"\n WALL AVOIDANCE FORWARD "<<endl;
        float tooClose_thresh = 130;
        vector<CarPoint> map;
        //readCarFromFullMapCSV(map);
        readCarFromCSV(map); //Read latest scan


        float m = (goalPoint.y - postPoint.y)/(goalPoint.x - postPoint.x);
        float c = goalPoint.y - m*goalPoint.x;

        float xmax;
        float ymax;
        float xmin;
        float ymin;
        
        //Set line bounds
        if(postPoint.x>goalPoint.x){
            xmax = postPoint.x + tooClose_thresh;
            xmin = goalPoint.x - tooClose_thresh;
        }else{
            xmin = postPoint.x - tooClose_thresh;;
            xmax = goalPoint.x + tooClose_thresh;;
        }

        if(postPoint.y>goalPoint.y){
            ymax = postPoint.y + tooClose_thresh;;
            ymin = goalPoint.y - tooClose_thresh;;
        }else{
            ymin = postPoint.y - tooClose_thresh;;
            ymax = goalPoint.y + tooClose_thresh;;
        }

        float d;
        float d_avoid = 100000;
        CarPoint avoidPoint;
        //Find points near Line (take all over to y side)
        cout<<"xmax = "<<xmax<<endl;
        cout<<"xmin = "<<xmin<<endl;
        cout<<"ymax = "<<ymax<<endl;
        cout<<"ymin = "<<ymin<<endl;
        
        // cout<<"NAVI: avoidFor Points: ";
        for(int i =0;i<map.size();i++){

            d = abs(-m*map[i].x + 1*map[i].y -c)/sqrt(pow(m,2) + 1);//Distance between point and line
            if(d<tooClose_thresh && map[i].x<xmax && map[i].x>xmin && map[i].y<ymax && map[i].y>ymin){
                    // cout<<map[i];
                float dist = pointDistance(postPoint,map[i]);
                if(dist<d_avoid){
                    d_avoid = dist;
                    avoidPoint = map[i];
                }

            }
        }
        if(d_avoid == 100000){
            cout<<" Nothing to AVOID "<<endl;
            d_avoid = pointDistance(postPoint,goalPoint)+450;
        }

        saveSonarIndicationToCSV(d_avoid);
        cout<<"NAVI: AvoidFor,: Avoid: "<<avoidPoint<<", From: "<<postPoint<<" To : "<<goalPoint<<endl;
        cout<<"NAVI: m: "<<m<<", c = "<<c<<endl;
        cout<<"NAVI: Predicted Dist = "<<d_avoid<<endl;

        return d_avoid;

    }

    float wallAvoidanceTurn(MatrixXf State){
        float turnableDistance = 100;
        float tooClose_thresh = 100;
         // Distance to move in the x and y directions based on theta and 20cm distance
        double deltaX = turnableDistance * cos(State(2));
        double deltaY = turnableDistance * sin(State(2));

        CarPoint goalPoint;
        goalPoint.x = State(0) + deltaX;
        goalPoint.y = State(1) + deltaY;

        CarPoint postPoint;
        postPoint.x = State(0);
        postPoint.y = State(1);

        vector<CarPoint> map;
        //readCarFromFullMapCSV(map);
        readCarFromCSV(map); //Read latest scan


        float m = (goalPoint.y - postPoint.y)/(goalPoint.x - postPoint.x);
        float c = goalPoint.y - m*goalPoint.x;

        float xmax;
        float ymax;
        float xmin;
        float ymin;
        
        //Set line bounds
        if(postPoint.x>goalPoint.x){
            xmax = postPoint.x + tooClose_thresh;
            xmin = goalPoint.x - tooClose_thresh;
        }else{
            xmin = postPoint.x - tooClose_thresh;;
            xmax = goalPoint.x + tooClose_thresh;;
        }

        if(postPoint.y>goalPoint.y){
            ymax = postPoint.y + tooClose_thresh;;
            ymin = goalPoint.y - tooClose_thresh;;
        }else{
            ymin = postPoint.y - tooClose_thresh;;
            ymax = goalPoint.y + tooClose_thresh;;
        }

        float d;
        float d_avoid = 100000;
        CarPoint avoidPoint;
        //Find points near Line (take all over to y side)
        cout<<"xmax = "<<xmax<<endl;
        cout<<"xmin = "<<xmin<<endl;
        cout<<"ymax = "<<ymax<<endl;
        cout<<"ymin = "<<ymin<<endl;
        
        // cout<<"NAVI: avoidFor Points: ";
        for(int i =0;i<map.size();i++){

            d = abs(-m*map[i].x + 1*map[i].y -c)/sqrt(pow(m,2) + 1);//Distance between point and line
            if(d<tooClose_thresh && map[i].x<xmax && map[i].x>xmin && map[i].y<ymax && map[i].y>ymin){
                    // cout<<map[i];
                float dist = pointDistance(postPoint,map[i]);
                if(dist<d_avoid){
                    d_avoid = dist;
                    avoidPoint = map[i];
                }

            }
        }
        if(d_avoid == 100000){
            d_avoid = pointDistance(postPoint,goalPoint)+250;
        }

        saveSonarIndicationTurnToCSV(d_avoid);
        cout<<"NAVI turn: AvoidFor,: Avoid: "<<avoidPoint<<", From: "<<postPoint<<" To : "<<goalPoint<<endl;
        cout<<"NAVI turn: m: "<<m<<", c = "<<c<<endl;
        cout<<"NAVI turn: Predicted Dist = "<<d_avoid<<endl;
        

        return d_avoid;

    }

    void escape(CarPoint closestPoint,MatrixXf State,float avoid_dist){
        cout<<"Entered escapism"<<endl;
        //Prefer solutions that get us closer to closest point
        //Prefer solutions which have the largest open space
        

        CarPoint leftEscape;
        CarPoint bot(State(0),State(1));
        float escape_check = 15*PI/180;
        float escape_ang = escape_check;
        float dist = pointDistance(closestPoint,bot);
        float angleL;
        float distanceL;
        float deltaY;
        float deltaX;

        bool escapeLeft = false;
        while(escapeLeft == false){
            leftEscape.x = closestPoint.x + dist*cos(escape_ang);
            leftEscape.y = closestPoint.y + dist*sin(escape_ang);
            escape_ang+=escape_check;

            //Set destination
            deltaX = leftEscape.x - State(0);
            deltaY = leftEscape.y - State(1);
            angleL = atan2(deltaY,deltaX) - State(2);
            angleL = pi_2_pi(angleL);
            
            CarPoint C = triangularRepositioning(State,angleL);
            deltaX = leftEscape.x - C.x;
            deltaY = leftEscape.y - C.y;
            distanceL = deltaX*deltaX + deltaY*deltaY;
            distanceL = sqrt(distanceL);

            //Wall Avoidance
            angleL = wallAvoidance(State,angleL);
            float dist = wallAvoidanceForward(State, angleL, C, leftEscape);
            cout<<dist<<" > "<<avoid_dist*2<<endl;


            if(dist>avoid_dist*2 && dist>200){
                escapeLeft = true;
            }

            if( pi_2_pi(escape_ang) == escape_check){
                cout<<"\nNo ESCAPE Route found L"<<endl;
                motorControlGrid(0,0);
            }
        }

        CarPoint rightEscape;
        escape_check = -escape_check;
        escape_ang = escape_check;
        bool escapeRight = false;
        float angleR;
        float distanceR;
        while(escapeRight == false){
            rightEscape.x = closestPoint.x + dist*cos(escape_ang);
            rightEscape.y = closestPoint.y + dist*sin(escape_ang);
            escape_ang+=escape_check;

            //Set destination
            deltaX = rightEscape.x - State(0);
            deltaY = rightEscape.y - State(1);
            angleR = atan2(deltaY,deltaX) - State(2);
            angleR = pi_2_pi(angleR);
            
            CarPoint C = triangularRepositioning(State,angleR);
            deltaX = rightEscape.x - C.x;
            deltaY = rightEscape.y - C.y;
            distanceR = deltaX*deltaX + deltaY*deltaY;
            distanceR = sqrt(distanceR);

            //Wall Avoidance
            angleR = wallAvoidance(State,angleR);
            float dist = wallAvoidanceForward(State, angleR, C, rightEscape);

            if(dist>avoid_dist*2 && dist>300){
                escapeRight = true;
            }
            if( pi_2_pi(escape_ang) == escape_check){
                cout<<"\nNo ESCAPE Route found R"<<endl;
                motorControlGrid(0,0);
            }
        }


        //We now have two viable escape points we compare them to decide
        float dR = pointDistance(rightEscape,closestPoint);
        float dL = pointDistance(leftEscape,closestPoint);

        if(dR>dL){
            cout<<"Selected LeftEscape as "<<leftEscape<<endl;
            deltaX = leftEscape.x - State(0);
            deltaY = leftEscape.y - State(1);
            angleL = atan2(deltaY,deltaX) - State(2);
            angleL = pi_2_pi(angleL);
            
            CarPoint C = triangularRepositioning(State,angleL);
            deltaX = leftEscape.x - C.x;
            deltaY = leftEscape.y - C.y;
            distanceL = deltaX*deltaX + deltaY*deltaY;
            distanceL = sqrt(distanceL);

            motorControlGrid(angleL,distanceL);
        }else{
            cout<<"Selected RightEscape as "<<rightEscape<<endl;
            deltaX = rightEscape.x - State(0);
            deltaY = rightEscape.y - State(1);
            angleR = atan2(deltaY,deltaX) - State(2);
            angleR = pi_2_pi(angleR);

            CarPoint C = triangularRepositioning(State,angleR);
            deltaX = rightEscape.x - C.x;
            deltaY = rightEscape.y - C.y;
            distanceR = deltaX*deltaX + deltaY*deltaY;
            distanceR = sqrt(distanceR);

            motorControlGrid(angleR,distanceR);
        }

        return;

    }


    //Set distance and angle to go to given grid point
    void updateMovement(CarPoint closestPoint,MatrixXf State){

        //Determine if stuck
        if(prevState.size() == 0){
            prevState.push_back(State(0));
            prevState.push_back(State(1));
            prevState.push_back(State(2));
        }else{
            CarPoint botPrev(prevState[0],prevState[1]);
            CarPoint botCurr(State(0),State(1));
            if(pointDistance(botPrev,botCurr) < 100){
                stuck_threshold = stuck_threshold-1;
            }else{
                stuck_threshold = stuck_threshold_fixed;
            }
            prevState[0] = State(0);
            prevState[1] = State(1);
            prevState[2] = State(2);
        }

 
        cout<<"ClosestPoint = "<<closestPoint<<endl;
        
        

        //Set destination
        float deltaX = closestPoint.x - State(0);
        float deltaY = closestPoint.y - State(1);
        float angle = atan2(deltaY,deltaX) - State(2);
        angle = pi_2_pi(angle);
        //angle = PI-angle;

        //angle = -160*PI/180;
        //cout<<"Set angle force to ="<<angle*180/PI<<endl;
        //angle = 0;

        


        // cout<<"NAVI,GRID: deltaX = "<<deltaX<<" = "<<-1*closestPoint.x<<" - "<<robotPoint.x<<endl;
        // cout<<"NAVI,GRID: deltaY = "<<deltaY<<" = "<<-1*closestPoint.y<<" - "<<robotPoint.y<<endl;
        // cout<<"NAVI,GRID angle = "<<angle*180/PI<<" = "<<atan2(deltaY,deltaX)*180/PI<<" - "<<State(2)*180/PI<<endl;
        // cout<<endl;
        
        CarPoint C = triangularRepositioning(State,angle);

        deltaX = closestPoint.x - C.x;
        //cout<<"NAVI,GRID: deltaX = "<<deltaX<<" = "<<closestPoint.x<<" - "<<C.x<<endl;
        deltaY = closestPoint.y - C.y;
        //cout<<"NAVI,GRID: deltaY = "<<deltaY<<" = "<<closestPoint.y<<" - "<<C.y<<endl;
        float distance = deltaX*deltaX + deltaY*deltaY;
        //cout<<"NAVI,GRID: dist1 = "<<distance<<" = "<<deltaX*deltaX<<" - "<<deltaY*deltaY<<endl;
        distance = sqrt(distance);
        //distance = 100;
        //distance = 100;

        //distance =0;

        cout<<"NAVI,GRID rotate angle = "<<angle*180/PI<<endl;
        cout<<"NAVI,GRID then distance = "<<distance<<endl;
        cout<<"NAVI,GRID to visit: "<<closestPoint<<endl;
        
        //cout<<"NAVI,GRID: movement: "<<distance<<"mm "<<angle*180/PI<<" deg"<<endl;


        //Wall Avoidance
        angle = wallAvoidance(State,angle);
        cout<<"NAVI,GRID rotate angle after avoidance = "<<angle*180/PI<<endl;
        float avoid_dist = wallAvoidanceForward(State, angle, C, closestPoint);
        wallAvoidanceTurn(State);
        //End Wall avoidance

        // angle = 0;
        // distance = 0;

        if(stuck_threshold == 0){
            cout<<"\n WE ARE STUCK"<<endl;
            escape(closestPoint,State,avoid_dist);
        }else{
            //Set motors
            motorControlGrid(angle,distance);
        }


        
        
        return;
    }


    vector<GridNode*> findNeighboursBFS2(GridNode& point,vector<vector<GridNode>>& gridMap){
            float defaultVal = 15;
            
            //Initialize Neighbours
            GridNode UP(defaultVal,defaultVal);
            UP.traversable = false;
            GridNode DOWN(defaultVal,defaultVal);
            DOWN.traversable = false;
            GridNode LEFT(defaultVal,defaultVal);
            LEFT.traversable = false;
            GridNode RIGHT(defaultVal,defaultVal);
            RIGHT.traversable = false;
            vector<GridNode*> neighbours;
            neighbours.push_back(&UP);
            neighbours.push_back(&DOWN);
            neighbours.push_back(&LEFT);
            neighbours.push_back(&RIGHT);

            //Initialize indexes
            

            for(int i =0;i<gridMap.size();i++){
                for(int j =0;j<gridMap[i].size();j++){
                    
                    // cout<<"x: "<<gridMap[i][j].x<<", "<<point.x<<" = "<<gridMap[i][j].x - point.x<<endl;
                    // cout<<"y: "<<gridMap[i][j].y<<", "<<point.y<<" = "<<gridMap[i][j].y - point.y<<endl;
                    //left from point
                    if(gridMap[i][j].x - point.x == grid_xstep && gridMap[i][j].y - point.y == 0){
                        neighbours[2] = &gridMap[i][j];
                    }//right from point
                    else if(gridMap[i][j].x - point.x == -grid_xstep && gridMap[i][j].y - point.y == 0){
                        neighbours[3] = &gridMap[i][j];
                    }//up from point
                    else if(gridMap[i][j].x - point.x == 0 && gridMap[i][j].y - point.y == grid_ystep){
                        neighbours[0] = &gridMap[i][j];
                    }//down from point
                    else if(gridMap[i][j].x - point.x == 0 && gridMap[i][j].y - point.y == -grid_ystep){
                        neighbours[1] = &gridMap[i][j];
                    }

                }
            }

            return neighbours;
        }

    // Breadth-first search
    vector<GridNode*> bfs(std::vector<std::vector<GridNode>>& gridMap, GridNode& start, GridNode& goal) {
        std::queue<GridNode*> toVisit;
        std::vector<GridNode*> path;

        // Start with the initial node
        toVisit.push(&start);

        while (!toVisit.empty()) {
            GridNode* current = toVisit.front();
            toVisit.pop();
            //cout<<"Current = "<<current->x<<" and "<<current->y<<endl;
            // Check if the current node is the goal
            if (current->x == goal.x && current->y == goal.y) {
                // Reconstruct the path
                while (current->parent != nullptr) {
                    path.push_back(current);
                    current = current->parent;
                }
                path.push_back(&start); // Add the start node to the path
                reverse(path.begin(), path.end()); // Reverse to get the path from start to goal
                return path;
            }

            // Mark the current node as visited
            current->traversable = false;

            // Find the neighbors of the current node
            std::vector<GridNode*> neighbors = findNeighboursBFS2(*current, gridMap);

            //cout<<"Neighbor size = "<<neighbors.size()<<endl;

            // Enqueue unvisited neighbors
            for (GridNode* neighbor : neighbors) {
                //cout<<"NeighBor = "<<neighbor->x<<" and "<<neighbor->y<<endl;
                if (neighbor->traversable) {
                    neighbor->parent = current;
                    toVisit.push(neighbor);
                }
            }
        }

        // No path found
        return path;
    }


    //This will remove straight long sections and also the inital value in a long path
    vector<CarPoint> pathShortening(vector<CarPoint> path){
        vector<CarPoint> shortenedPath;
        vector<CarPoint> dumpedPoints;

        shortenedPath.push_back(path[0]); // Add the first point.

        for (int i = 1; i < path.size() - 1; ++i) {
            int dx1 = path[i].x - path[i - 1].x;
            int dy1 = path[i].y - path[i - 1].y;
            int dx2 = path[i + 1].x - path[i].x;
            int dy2 = path[i + 1].y - path[i].y;

            if (dx1 != dx2 || dy1 != dy2) {
                shortenedPath.push_back(path[i]); // Add the point when there's a change in direction.
            }else{
                dumpedPoints.push_back(path[i]);
            }
        }

        shortenedPath.push_back(path.back()); // Add the last point.

        //Remove the starting point because it is our current position
        shortenedPath.erase(shortenedPath.begin());


        //Set dumped points to traversed
        vector<vector<GridPoint>> gridOld;
        readGridFromCSV(gridOld);
        gridDataAssosciationPath(gridOld,dumpedPoints);
        saveGridToCSV(gridOld);

        return shortenedPath;

    }


    vector<CarPoint> pathFinder(MatrixXf State,CarPoint Goal){
        cout<<"In PathFinder"<<endl;
        vector<vector<GridPoint>> gridMap;
        readGridFromCSV(gridMap);

        GridPoint current = stateToGridDot(State, gridMap);

        GridNode start(current.x, current.y);
        GridNode goal(Goal.x, Goal.y);

        cout<<"Start = "<<current.x<<", "<<current.y<<endl;
        cout<<"Goal = "<<goal.x<<", "<<goal.y<<endl;
        
        vector<vector<GridNode>> gridNew;
        mapConverter(gridMap, gridNew);
        cout<<"GridMap size = "<<gridNew.size()<<endl;
        vector<GridNode*> path  = bfs(gridNew,start,goal);

        if(path.empty()) {
            cout << "No path found." << endl;
        } else {
            cout << "Path: ";
            for (GridNode* point : path) {
                cout << "(" << point->x << ", " << point->y << ") ";
            }
            cout << endl;
        }


        //Convert Path to Cartesian Coordinates
        vector<CarPoint> pathCartesian;
        for(int i =0;i<path.size();i++){
            CarPoint newPoint(path[i]->x,path[i]->y);
            pathCartesian.push_back(newPoint);
        }
        
        if(path.size()>1){

            if(pathCartesian[0].x != current.x && pathCartesian[1].y != current.y ){
                CarPoint newPoint;
                newPoint.x = current.x;
                newPoint.y = current.y;
                pathCartesian.insert(pathCartesian.begin(), newPoint);
            }
            pathCartesian = pathShortening(pathCartesian);
            
        }
        

        if(pathCartesian.empty()) {
            cout << "No Shortened path found." << endl;
            if(goal.x!=current.x || goal.y!=current.y){
                CarPoint goali;
                goali.x = goal.x;
                goali.y = goal.y;
                pathCartesian.push_back(goali);
                cout<<"Adding goali to path :"<<goali<<endl;
            }
        } else {
            cout << "Shortened Path: ";
            for (int i =0;i<pathCartesian.size();i++) {
                cout << pathCartesian[i];
            }
            cout << endl;
        }

        


        return pathCartesian;
    }

}


    