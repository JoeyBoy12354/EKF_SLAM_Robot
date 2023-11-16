#include "robot.h"

using namespace CSV_Functions;
using namespace Data_Functions;


namespace Mapping_Functions{
    //extern int noCorners; // Declaration, not a definition
    
    

    void storeMapPointsCleaning(vector<CarPoint> lidardata,Matrix<float, dim, 1> State){
        //Fit new scan
        fitCartesian(lidardata,State(0),State(1),State(2));
        //Write new Scan
        saveCarToCSV(lidardata);

       
        storeStatePoints(State);
        
    }

    //Build Full map, place it in csv
    //Points have to be translated
    void storeMapPoints(vector<CarPoint> lidardata,Matrix<float, dim, 1> State){
        float distance_threshold = 150;//If two points are greater than Xmm then keep this point.
        float distance_threshold2 = 5;//If two points are greater than Xmm then keep this point. (ACCURACY test)

        vector<CarPoint> oldmap;
        readCarFromFullMapCSV(oldmap);//Fetch all current points

        //Fit new scan
        fitCartesian(lidardata,State(0),State(1),State(2));
        //Write new Scan
        saveCarToCSV(lidardata);

        vector<CarPoint> temp;

        //Compare with new points (only update if points are different)
        //If a point is less than Xmm away from oldmap point then remove it from circulation
        // for(int i=0; i<oldmap.size();i++){
        //     for(int j=0; j<lidardata.size();j++){
        //             vector<CarPoint> temp;
        //             if(pointDistance(lidardata[j],oldmap[i]) > distance_threshold){
        //                 temp.push_back(lidardata[j]);
        //             }
        //             lidardata = temp;
        //     }
        // }

        float accuracy = 0;
        float accuracy_dist = 15;
        bool isAccurate=false;

        //Append new points to current oldmap
        //This is okay because we do not use fullmap data for anything
        bool isNew = true;
        for(int i =0;i<lidardata.size();i++){
            isNew = true;
            isAccurate = false;
            for(int j=0;j<oldmap.size();j++){
                if(pointDistance(lidardata[i],oldmap[j]) < distance_threshold){
                    isNew = false;
                }

                if(pointDistance(lidardata[i],oldmap[j]) < accuracy_dist){
                    isAccurate = true;
                }
            }

            if(isNew == true){
                temp.push_back(lidardata[i]);
                //oldmap.push_back(lidardata[i]);
            }

            if(isAccurate == true){
                accuracy=accuracy+ 1;
                //oldmap.push_back(lidardata[i]);
            }
        }



        //A very accurate scan should be fully added
        vector<CarPoint> temp2;
        isNew = true;
        float acc_percentage = (accuracy/lidardata.size())*100;

        if(noCorners>1){
                if( acc_percentage> 70){
                cout<<"\n\n VERY ACCURATE SCAN @ "<< acc_percentage <<" %\n\n";
                temp.clear();
                for(int i = 0;i<lidardata.size();i++){
                    for(int j =0;j<oldmap.size();j++){
                        if(pointDistance(lidardata[i],oldmap[j]) < distance_threshold2){
                            isNew = false;
                        }
                            
                            //temp.push_back(lidardata[i];)
                    } 
                    if(isNew == true){
                        oldmap.push_back(lidardata[i]);
                    }
                    isNew=true;

                }
                cout<<"YAHA"<<endl;


            }else if(acc_percentage>10){
                cout<<"\n\n NOT VERY ACCURATE SCAN addNewPoints @ "<< acc_percentage <<" %\n\n";
                for(int i = 0;i<temp.size();i++){
                    oldmap.push_back(temp[i]);
                }

            }else{
                cout<<"\n\n NOT ACCURATE SCAN ADD NOTHING @ "<< acc_percentage <<" %\n\n";
            }

        }else{
            cout<<"NO SCAN ADDED DUE TO ONLY 1 CORNER"<<endl;
        }
        


        

        cout<<"No of New Points = "<<temp.size()<<endl;
    

        //Update full map
        //appendCarToFullMapCSV(lidardata);
        //saveCarToFullMapCSV(lidardata);
        saveCarToFullMapCSV(oldmap);

        storeStatePoints(State);
        
    }

    void storeStatePoints(Matrix<float, dim, 1> State){
        vector<CarPoint> landmarks;
        vector<float> position;

        //Move Position into vector
        position.push_back(State[0]);
        position.push_back(State[1]);
        position.push_back(State[2]);
        

        //Convert State Landmarks to points
        for(int i = 3;i<State.size();i=i+2){
            CarPoint LM;
            LM.x = State[i];
            LM.y = State[i+1];
            if(LM.x != 0 && LM.y !=0 ){
                landmarks.push_back(LM);
            }
            
        }

        saveLandmarkToCSV(landmarks);
        savePositionToCSV(position);

    }

    void getMapBounds(vector<CarPoint> map, vector<float>& bounds){
        float yMax = -10000;
        float yMin = 10000;
        float xMax = -10000;
        float xMin = 10000;

        for(int i =0;i<map.size();i++){
            if(map[i].y>yMax){
                yMax = map[i].y;
            }
            if(map[i].y<yMin){
                yMin = map[i].y;
            }
            if(map[i].x>xMax){
                xMax = map[i].x;
            }
            if(map[i].x<xMin){
                xMin = map[i].x;
            }
        }

        bounds.push_back(xMax);
        bounds.push_back(xMin);
        bounds.push_back(yMax);
        bounds.push_back(yMin);
    }

    void getMapBoundsSpecific(vector<CarPoint> map, vector<float>& bounds, GridPoint dot){
        float distThresh = 100;
        float yMax = -10000;
        float yMin = 10000;
        float xMax = -10000;
        float xMin = 10000;

        for(int i =0;i<map.size();i++){
            if(map[i].y>yMax && abs(map[i].x - dot.x) <= distThresh){
                yMax = map[i].y;
            }
            if(map[i].y<yMin && abs(map[i].x - dot.x) <= distThresh){
                yMin = map[i].y;
            }
            if(map[i].x>xMax && abs(map[i].y - dot.y) <= distThresh){
                xMax = map[i].x;
            }
            if(map[i].x<xMin && abs(map[i].y - dot.y) <= distThresh){
                xMin = map[i].x;
            }

            // if(abs(map[i].x - dot.x) <= distThresh && dot.x>1100 && dot.x<1300){
            //     cout<<"dot = ("<<dot.x<<", "<<dot.y<<") abs(map[i].x - dot.x) =  "<<abs(map[i].x - dot.x)<<" map.x = "<<map[i].x<<endl;
            // }
            // if(abs(map[i].y - dot.y) <= distThresh && dot.x>1100 && dot.x<1300){
            //     cout<<"dot = ("<<dot.x<<", "<<dot.y<<") abs(map[i].x - dot.x) = "<<abs(map[i].y - dot.y)<<" map.y = "<<map[i].y<<endl;
            // }
        }

        bounds.push_back(xMax);
        bounds.push_back(xMin);
        bounds.push_back(yMax);
        bounds.push_back(yMin);
    }

    void gridDataProcess(vector<vector<GridPoint>>& gridNew,Matrix<float, dim, 1> State, bool firstRun){

        cout<<"GRID: In grid data process"<<endl;
        vector<CarPoint> map;
        readCarFromFullMapCSV(map);
        //gridMakeDots(map,gridNew);
        gridMakeDots2(map,gridNew);

        if(firstRun == false){
            vector<vector<GridPoint>> gridOld;
            readGridFromCSV(gridOld);
            gridDataAssosciationMap(gridOld,gridNew);
            gridDataAssosciationMoveSimple(gridNew,State);
        }else{
            gridDataAssosciationMoveSimple(gridNew,State);
        }

        // cout<<"INITIAL MAP SHOW ONLY DOTS THAT HAVE BEEN TRAVERSED"<<endl;
        //     for(int i =0;i<gridNew.size();i++){
        //         for(int j=0;j<gridNew[i].size();j++){
        //             if(gridNew[i][j].trav == true){
        //                 cout<<gridNew[i][j]<<"++,";
        //             }else{
        //                 cout<<gridNew[i][j]<<"--,";
        //             }
        //         }
        //     }
        //     cout<<endl;


        saveGridToCSV(gridNew);
        cout<<"GRID:SAVED TO CSV"<<endl;

        

        
        return;
        

    }   

    void gridDataAssosciationPath(vector<vector<GridPoint>>& gridOld, vector<CarPoint> dumpedPath){

        cout<<"Dumped Traversed = ";
        for(int i = 0;i<gridOld.size();i++){
            for(int j =0;j<gridOld[i].size();j++){
                for(int k=0;k<dumpedPath.size();k++){
                    if(gridOld[i][j].x == dumpedPath[k].x && gridOld[i][j].y == dumpedPath[k].y){
                        gridOld[i][j].trav = true;
                        cout<<gridOld[i][j];
                    }
                }
            }
        }
        cout<<endl;
        return;
    } 

    void gridDataAssosciationMoveSimple(vector<vector<GridPoint>>& gridNew, Matrix<float, dim, 1> State){
        //int count = 0;
        float dist_thresh= grid_xstep/2;//If points are less than Xmm from position  then set trav=true.
        GridPoint Robot;
        Robot.x = State(0);
        Robot.y = State(1);
        for(int i = 0;i<gridNew.size();i++){
            for(int j =0;j<gridNew[i].size();j++){
                if(gridPointDistance(gridNew[i][j],Robot)<=dist_thresh){
                    gridNew[i][j].trav = true;
                    cout<<" Traversed = "<<gridNew[i][j]<<endl;
                }
            }
        }

        
        return;

    }

    void gridDataAssosciationMap(vector<vector<GridPoint>> gridOld, vector<vector<GridPoint>>& gridNew){
        int y_size;
        int x_size = (gridNew.size() > gridOld.size()) ? gridOld.size() : gridNew.size();

        
        //Update Traversal points of old
        //Compare new Points with old points (this assumes that grid old and gridNew have the same y- and x-axis)
        for(int i=0; i<x_size;i++){
            y_size = (gridNew[i].size() > gridOld[i].size()) ? gridOld[i].size() : gridNew[i].size();
            for(int j=0; j<y_size; j++){
                    if(gridNew[i][j].x == gridOld[i][j].x and gridNew[i][j].y == gridOld[i][j].y and gridOld[i][j].trav == true){
                        gridNew[i][j].trav = true; 
                    }
            }
        }

        return;
        
    }

    bool gridDotBoundCheck(vector<CarPoint> searchMap,GridPoint point){
        // if(point.x > 500 && point.x < 1500){
        //     cout<<"MapPoint: ("<<point.x<<", "<<point.y<<")"<<endl;
        // }

        vector<float> bounds;
        getMapBoundsSpecific(searchMap, bounds, point);
        // cout<<"MAP BOUNDS: xmax = "<<bounds[0]<<" xmin = "<<bounds[1]<<" ymax = "<<bounds[2]<<" ymin = "<<bounds[3]<<endl;


        //check if point is within max and min of lidardata bounds=[Xmax,Xmin,Ymax,Ymin]
        if(point.x>bounds[0] || point.x<bounds[1] || point.y>bounds[2] || point.y<bounds[3] ){
            return false;
        }

        // if(point.x > 500 && point.x < 1500){
        //     cout<<"MapPoint passed bounds check"<<endl;
        // }

        return true;
    }

    bool gridDotBigBoundCheck(GridPoint point,vector<float> bounds){
        // if(point.x > 500 && point.x < 1500){
        //     cout<<"MapPoint: ("<<point.x<<", "<<point.y<<")"<<endl;
        // }



        //check if point is within max and min of lidardata bounds=[Xmax,Xmin,Ymax,Ymin]
        if(point.x>bounds[0] || point.x<bounds[1] || point.y>bounds[2] || point.y<bounds[3] ){
            return false;
        }

        // if(point.x > 500 && point.x < 1500){
        //     cout<<"MapPoint passed bounds check"<<endl;
        // }

        return true;
    }

    bool gridDotLidarCheck(vector<CarPoint> searchMap, GridPoint point,float distThresh){
        if(searchMap.size()==0){
            return false;
        }

        // if(point.x > 500 && point.x < 1500){
        //     cout<<"MapPoint: ("<<point.x<<", "<<point.y<<")"<<endl;
        // }


        //Get Cartesian
        CarPoint point2;
        point2.x = point.x;
        point2.y = point.y;            
        float dist = 10000000;
        CarPoint smallPoint;
        //check if point is far away enough from lidarPoints
        for(int i = 0; i<searchMap.size(); i++){
            
            float temp_dist = pointDistance(searchMap[i],point2);
            if(dist > temp_dist){
                dist = temp_dist;
                smallPoint = searchMap[i];
            }
        
        }

        if(dist < distThresh){
            // if(point.x > 500 && point.x < 1500){
            //     cout<<"MapPoint failed lidar distance check"<<endl;
            // }
            return false;

        }else{
            // if(point.x > 500 && point.x < 1500){
            //     cout<<"MapPoint passed lidar distance check"<<endl;
            // }
            return true;
        } 
    }
   
    void gridMakeDots(vector<CarPoint> mapdata, vector<vector<GridPoint>>& points){
        

        //We need to create the vertical lines
        //Might be good to calculate this with max size of current lidar scan
        float vLimit = 20; //Max number of vertical points from x-axis in 1 direction
        float hLimit = 20; //Max Number of horistontal points from v-axis in 1 direction

        // float yStep = 450;//y-distance between points on same x-coordinate
        // float xStep = 450;//x-distance between points on same y-coordinate
        float yStep = grid_ystep;//y-distance between points on same x-coordinate
        float xStep = grid_ystep;//x-distance between points on same y-coordinate
        
        
        float boundThresh = 340;//If distance between gridPoint and lidarPoint <= Xmm then return false 

        float xPos; //holds current x-coordinate
        float yPos; //holds current y-coordinate

        int maxNoRuns = 20;
        int noRuns = 0;
        bool dotCheck = true;
        bool lidarCheck = true;

        vector<float> bounds; //Xmax,Xmin,Ymax,Ymin
        getMapBounds(mapdata,bounds);

        
        cout<<"Map Bounds = xmin: "<<bounds[0]<<", xmax"<<bounds[1]<<", ymin"<<bounds[2]<<", ymax"<<bounds[3]<<endl;
    



        vector<GridPoint> yPoints;
        GridPoint newPoint;

        bool pushed;
        int pushedPoints = 0;

        //Select
        //Positive x-axis
        // -Postivive y-axis
        // -Negative y-axis
        //Negative x-axis
        // -Postivive y-axis
        // -Negative y-axis

        //Positive X-axis
        xPos = 0;
        while(points.size()<=hLimit && noRuns<maxNoRuns){
            pushed = false;

            //Do Positive Y-Axis
            dotCheck = true;
            yPoints.clear();
            yPos = 0;
            newPoint.x = xPos;
            newPoint.y = yPos;
            while(yPoints.size()<=vLimit && dotCheck == true){
                dotCheck = gridDotBoundCheck(mapdata,newPoint);
                lidarCheck = gridDotLidarCheck(mapdata,newPoint,boundThresh);

                if(dotCheck == true && lidarCheck == true){
                    yPoints.push_back(newPoint);
                    yPos += yStep;
                    newPoint.x = xPos;
                    newPoint.y = yPos; 
                    pushed = true;
                }else if(dotCheck == true && lidarCheck == false){
                    //This is to ensure that we still keep checking for dots even if there is an obstacle
                    //However this means that as long as a point is within max bounds it can be plotted, so corridors and rooms
                    // Will have points between them
                    yPos += yStep;
                    newPoint.x = xPos;
                    newPoint.y = yPos; 
                }
            }
            if(yPoints.size()>0){
                points.push_back(yPoints);
            }
            

            //Do Negative Y-axis
            dotCheck = true;
            yPoints.clear();
            yStep = -1*yStep;
            yPos = yStep;
            newPoint.x = xPos;
            newPoint.y = yPos;
            while(yPoints.size()<=vLimit && dotCheck == true){
                dotCheck = gridDotBoundCheck(mapdata,newPoint);
                lidarCheck = gridDotLidarCheck(mapdata,newPoint,boundThresh);

                if(dotCheck == true && lidarCheck == true){
                    yPoints.push_back(newPoint);
                    yPos += yStep;
                    newPoint.x = xPos;
                    newPoint.y = yPos; 
                    pushed = true;
                }else if(dotCheck == true && lidarCheck == false){
                    yPos += yStep;
                    newPoint.x = xPos;
                    newPoint.y = yPos; 
                }
            }
            if(yPoints.size()>0){
                points.push_back(yPoints);
            }
            
            yStep = -1*yStep;//Change Back to positive
            xPos += xStep;    
            noRuns+=1;
            if(pushed == true){
                pushedPoints =+1;
            }
                

        }

        cout<<"Points Pushed = "<<pushedPoints<<endl;
        
        //Negative X-axis
        noRuns = 0;
        xStep = -1*xStep;
        xPos = xStep;
        while(points.size()<=hLimit*2 && noRuns<maxNoRuns){
            //Do Positive Y-Axis
            dotCheck = true;
            yPoints.clear();
            yPos = 0;
            newPoint.x = xPos;
            newPoint.y = yPos;
            while(yPoints.size()<=vLimit && dotCheck == true){
                dotCheck = gridDotBoundCheck(mapdata,newPoint);
                lidarCheck = gridDotLidarCheck(mapdata,newPoint,boundThresh);

                if(dotCheck == true && lidarCheck == true){
                    yPoints.push_back(newPoint);
                    yPos += yStep;
                    newPoint.x = xPos;
                    newPoint.y = yPos; 
                }else if(dotCheck == true && lidarCheck == false){
                    yPos += yStep;
                    newPoint.x = xPos;
                    newPoint.y = yPos; 
                }

                // dotCheck = gridDotBoundCheck(mapdata,newPoint,boundThresh,bounds);
                // if(dotCheck == true){
                //     yPoints.push_back(newPoint);
                //     yPos += yStep;
                //     newPoint.x = xPos;
                //     newPoint.y = yPos;
                // }
            }
            if(yPoints.size()>0){
                points.push_back(yPoints);
            }

            //Do Negative Y-axis
            dotCheck = true;
            yPoints.clear();
            yStep = -1*yStep;
            yPos = yStep;
            newPoint.x = xPos;
            newPoint.y = yPos;
            while(yPoints.size()<=vLimit && dotCheck == true){
                dotCheck = gridDotBoundCheck(mapdata,newPoint);
                lidarCheck = gridDotLidarCheck(mapdata,newPoint,boundThresh);

                if(dotCheck == true && lidarCheck == true){
                    yPoints.push_back(newPoint);
                    yPos += yStep;
                    newPoint.x = xPos;
                    newPoint.y = yPos; 
                }else if(dotCheck == true && lidarCheck == false){
                    yPos += yStep;
                    newPoint.x = xPos;
                    newPoint.y = yPos; 
                }
                // dotCheck = gridDotBoundCheck(mapdata,newPoint,boundThresh,bounds);
                // if(dotCheck == true){
                //     yPoints.push_back(newPoint);
                //     yPos += yStep;
                //     newPoint.x = xPos;
                //     newPoint.y = yPos;
                // }
            }
            if(yPoints.size()>0){
                points.push_back(yPoints);
            }
            

            yStep = -1*yStep;//Change Back to positive
            xPos += xStep;    
            noRuns+=1;
            }
    



        // //Do search over y-axix
        // //Positive y-axis
        // vector<GridPoint> xPoints;
        // vector<vector<GridPoint>> points2;
        // yPos = 0;
        // while(points2.size()<=vLimit && noRuns<maxNoRuns){
        //     pushed = false;

        //     //Do Positive X-Axis
        //     dotCheck = true;
        //     xPoints.clear();
        //     xPos = 0;
        //     newPoint.x = xPos;
        //     newPoint.y = yPos;
        //     while(xPoints.size()<=hLimit && dotCheck == true){
        //         dotCheck = gridDotBoundCheck(mapdata,newPoint);
        //         lidarCheck = gridDotLidarCheck(mapdata,newPoint,boundThresh);

        //         if(dotCheck == true && lidarCheck == true){
        //             xPoints.push_back(newPoint);
        //             xPos += xStep;
        //             newPoint.x = xPos;
        //             newPoint.y = yPos; 
        //             pushed = true;
        //         }else if(dotCheck == true && lidarCheck == false){
        //             //This is to ensure that we still keep checking for dots even if there is an obstacle
        //             //However this means that as long as a point is within max bounds it can be plotted, so corridors and rooms
        //             // Will have points between them
        //             xPos += xStep;
        //             newPoint.x = xPos;
        //             newPoint.y = yPos; 
        //         }
        //     }
        //     if(xPoints.size()>0){
        //         points2.push_back(yPoints);
        //     }
            

        //     //Do Negative Y-axis
        //     dotCheck = true;
        //     xPoints.clear();
        //     xStep = -1*xStep;
        //     xPos = xStep;
        //     newPoint.x = xPos;
        //     newPoint.y = yPos;
        //     while(xPoints.size()<=hLimit && dotCheck == true){
        //         dotCheck = gridDotBoundCheck(mapdata,newPoint);
        //         lidarCheck = gridDotLidarCheck(mapdata,newPoint,boundThresh);

        //         if(dotCheck == true && lidarCheck == true){
        //             xPoints.push_back(newPoint);
        //             xPos += xStep;
        //             newPoint.x = xPos;
        //             newPoint.y = yPos; 
        //             pushed = true;
        //         }else if(dotCheck == true && lidarCheck == false){
        //             xPos += xStep;
        //             newPoint.x = xPos;
        //             newPoint.y = yPos; 
        //         }
        //     }
        //     if(yPoints.size()>0){
        //         points2.push_back(yPoints);
        //     }
            
        //     xStep = -1*xStep;//Change Back to positive
        //     yPos += yStep;    
        //     noRuns+=1;
        //     if(pushed == true){
        //         pushedPoints =+1;
        //     }

        // }




        // //Negative y-axis
        // noRuns = 0;
        // yStep = -1*yStep;
        // yPos = yStep;
        // while(points2.size()<=vLimit && noRuns<maxNoRuns){
        //     pushed = false;

        //     //Do Positive X-Axis
        //     dotCheck = true;
        //     xPoints.clear();
        //     xPos = 0;
        //     newPoint.x = xPos;
        //     newPoint.y = yPos;
        //     while(xPoints.size()<=hLimit && dotCheck == true){
        //         dotCheck = gridDotBoundCheck(mapdata,newPoint);
        //         lidarCheck = gridDotLidarCheck(mapdata,newPoint,boundThresh);

        //         if(dotCheck == true && lidarCheck == true){
        //             xPoints.push_back(newPoint);
        //             xPos += xStep;
        //             newPoint.x = xPos;
        //             newPoint.y = yPos; 
        //             pushed = true;
        //         }else if(dotCheck == true && lidarCheck == false){
        //             //This is to ensure that we still keep checking for dots even if there is an obstacle
        //             //However this means that as long as a point is within max bounds it can be plotted, so corridors and rooms
        //             // Will have points between them
        //             xPos += xStep;
        //             newPoint.x = xPos;
        //             newPoint.y = yPos; 
        //         }
        //     }
        //     if(xPoints.size()>0){
        //         points2.push_back(yPoints);
        //     }
            

        //     //Do Negative Y-axis
        //     dotCheck = true;
        //     xPoints.clear();
        //     xStep = -1*xStep;
        //     xPos = xStep;
        //     newPoint.x = xPos;
        //     newPoint.y = yPos;
        //     while(yPoints.size()<=hLimit && dotCheck == true){
        //         dotCheck = gridDotBoundCheck(mapdata,newPoint);
        //         lidarCheck = gridDotLidarCheck(mapdata,newPoint,boundThresh);

        //         if(dotCheck == true && lidarCheck == true){
        //             xPoints.push_back(newPoint);
        //             xPos += xStep;
        //             newPoint.x = xPos;
        //             newPoint.y = yPos; 
        //             pushed = true;
        //         }else if(dotCheck == true && lidarCheck == false){
        //             xPos += xStep;
        //             newPoint.x = xPos;
        //             newPoint.y = yPos; 
        //         }
        //     }
        //     if(yPoints.size()>0){
        //         points2.push_back(yPoints);
        //     }
            
        //     xStep = -1*xStep;//Change Back to positive
        //     yPos += yStep;    
        //     noRuns+=1;
        //     if(pushed == true){
        //         pushedPoints =+1;
        //     }

        // }

        // bool found=false;
        // for(int i =0;i<points2.size();i++){
        //     for(int j=0;j<points2[i].size();j++){
        //         for(int k=0;k<points.size();k++){
        //             for(int z=0;z<points[k].size();k++){
        //                 if(points[k][z].x==points2[i][j].x && points[k][z].y==points2[i][j].y){
        //                     found=true;
        //                 }
        //             }
        //             if(found==false){
        //                 points.push_back(points2[i][j]);
        //             }
                
        //         }
                
        //     found = false;
        //     }
        // }

    }




     void gridMakeDots2(vector<CarPoint> mapdata, vector<vector<GridPoint>>& points){
        

        //We need to create the vertical lines
        //Might be good to calculate this with max size of current lidar scan
        float vLimit = 20; //Max number of vertical points from x-axis in 1 direction
        float hLimit = 20; //Max Number of horistontal points from v-axis in 1 direction

        // float yStep = 450;//y-distance between points on same x-coordinate
        // float xStep = 450;//x-distance between points on same y-coordinate
        float yStep = grid_ystep;//y-distance between points on same x-coordinate
        float xStep = grid_ystep;//x-distance between points on same y-coordinate
        
        
        

        float xPos; //holds current x-coordinate
        float yPos; //holds current y-coordinate

        int maxNoRuns = 20;
        int noRuns = 0;
        bool dotCheck = true;
        bool lidarCheck = true;
        bool bigDotCheck = true;

        vector<float> bounds; //Xmax,Xmin,Ymax,Ymin
        getMapBounds(mapdata,bounds);

        
        cout<<"Map Bounds = xmax: "<<bounds[0]<<", xmin"<<bounds[1]<<", ymax"<<bounds[2]<<", ymin"<<bounds[3]<<endl;
    



        vector<GridPoint> yPoints;
        GridPoint newPoint;

        bool pushed;
        int pushedPoints = 0;

        //Select
        //Positive x-axis
        // -Postivive y-axis
        // -Negative y-axis
        //Negative x-axis
        // -Postivive y-axis
        // -Negative y-axis

        //Positive X-axis
        xPos = 0;
        while(points.size()<=hLimit && noRuns<maxNoRuns){
            pushed = false;

            //Do Positive Y-Axis
            dotCheck = true;
            bigDotCheck = true;
            yPoints.clear();
            yPos = 0;
            newPoint.x = xPos;
            newPoint.y = yPos;
            while(yPoints.size()<=vLimit && bigDotCheck == true){
                dotCheck = gridDotBoundCheck(mapdata,newPoint);
                bigDotCheck = gridDotBigBoundCheck(newPoint,bounds);
                // cout<<"1,1BIg CHECL = "<<bigDotCheck<<endl;
                lidarCheck = gridDotLidarCheck(mapdata,newPoint,boundThresh);

                if(dotCheck == true && lidarCheck == true){
                    yPoints.push_back(newPoint);
                    yPos += yStep;
                    newPoint.x = xPos;
                    newPoint.y = yPos; 
                    pushed = true;
                }else if(dotCheck == true && lidarCheck == false || bigDotCheck == true){
                    //This is to ensure that we still keep checking for dots even if there is an obstacle
                    //However this means that as long as a point is within max bounds it can be plotted, so corridors and rooms
                    // Will have points between them
                    yPos += yStep;
                    newPoint.x = xPos;
                    newPoint.y = yPos; 
                }
            }
            if(yPoints.size()>0){
                points.push_back(yPoints);
            }
            

            //Do Negative Y-axis
            dotCheck = true;
            bigDotCheck = true;
            yPoints.clear();
            yStep = -1*yStep;
            yPos = yStep;
            newPoint.x = xPos;
            newPoint.y = yPos;
            while(yPoints.size()<=vLimit && bigDotCheck == true){
                dotCheck = gridDotBoundCheck(mapdata,newPoint);
                bigDotCheck = gridDotBigBoundCheck(newPoint,bounds);
                // cout<<"1.2BIg CHECL = "<<bigDotCheck<<endl;
                lidarCheck = gridDotLidarCheck(mapdata,newPoint,boundThresh);

                if(dotCheck == true && lidarCheck == true){
                    yPoints.push_back(newPoint);
                    yPos += yStep;
                    newPoint.x = xPos;
                    newPoint.y = yPos; 
                    pushed = true;
                }else if(dotCheck == true && lidarCheck == false || bigDotCheck == true ){
                    yPos += yStep;
                    newPoint.x = xPos;
                    newPoint.y = yPos; 
                }
            }
            if(yPoints.size()>0){
                points.push_back(yPoints);
            }
            
            yStep = -1*yStep;//Change Back to positive
            xPos += xStep;    
            noRuns+=1;
            if(pushed == true){
                pushedPoints =+1;
            }
                

        }

        cout<<"Points Pushed = "<<pushedPoints<<endl;
        
        //Negative X-axis
        noRuns = 0;
        xStep = -1*xStep;
        xPos = xStep;
        while(points.size()<=hLimit*2 && noRuns<maxNoRuns){
            //Do Positive Y-Axis
            dotCheck = true;
            bigDotCheck = true;
            yPoints.clear();
            yPos = 0;
            newPoint.x = xPos;
            newPoint.y = yPos;
            while(yPoints.size()<=vLimit && bigDotCheck == true){
                dotCheck = gridDotBoundCheck(mapdata,newPoint);
                bigDotCheck = gridDotBigBoundCheck(newPoint,bounds);
                // cout<<"2.1BIg CHECL = "<<bigDotCheck<<endl;
                lidarCheck = gridDotLidarCheck(mapdata,newPoint,boundThresh);

                if(dotCheck == true && lidarCheck == true){
                    yPoints.push_back(newPoint);
                    yPos += yStep;
                    newPoint.x = xPos;
                    newPoint.y = yPos; 
                }else if(dotCheck == true && lidarCheck == false || bigDotCheck == true){
                    yPos += yStep;
                    newPoint.x = xPos;
                    newPoint.y = yPos; 
                }

                // dotCheck = gridDotBoundCheck(mapdata,newPoint,boundThresh,bounds);
                // if(dotCheck == true){
                //     yPoints.push_back(newPoint);
                //     yPos += yStep;
                //     newPoint.x = xPos;
                //     newPoint.y = yPos;
                // }
            }
            if(yPoints.size()>0){
                points.push_back(yPoints);
            }

            //Do Negative Y-axis
            dotCheck = true;
            bigDotCheck = true;
            yPoints.clear();
            yStep = -1*yStep;
            yPos = yStep;
            newPoint.x = xPos;
            newPoint.y = yPos;
            while(yPoints.size()<=vLimit && bigDotCheck == true){
                dotCheck = gridDotBoundCheck(mapdata,newPoint);
                bigDotCheck = gridDotBigBoundCheck(newPoint,bounds);
                //cout<<"2.2BIg CHECL = "<<bigDotCheck<<endl;
                lidarCheck = gridDotLidarCheck(mapdata,newPoint,boundThresh);

                if(dotCheck == true && lidarCheck == true){
                    yPoints.push_back(newPoint);
                    yPos += yStep;
                    newPoint.x = xPos;
                    newPoint.y = yPos; 
                }else if(dotCheck == true && lidarCheck == false || bigDotCheck == true){
                    yPos += yStep;
                    newPoint.x = xPos;
                    newPoint.y = yPos; 
                }
                // dotCheck = gridDotBoundCheck(mapdata,newPoint,boundThresh,bounds);
                // if(dotCheck == true){
                //     yPoints.push_back(newPoint);
                //     yPos += yStep;
                //     newPoint.x = xPos;
                //     newPoint.y = yPos;
                // }
            }
            if(yPoints.size()>0){
                points.push_back(yPoints);
            }
            

            yStep = -1*yStep;//Change Back to positive
            xPos += xStep;    
            noRuns+=1;
            }
    



        // //Do search over y-axix
        // //Positive y-axis
        // vector<GridPoint> xPoints;
        // vector<vector<GridPoint>> points2;
        // yPos = 0;
        // while(points2.size()<=vLimit && noRuns<maxNoRuns){
        //     pushed = false;

        //     //Do Positive X-Axis
        //     dotCheck = true;
        //     xPoints.clear();
        //     xPos = 0;
        //     newPoint.x = xPos;
        //     newPoint.y = yPos;
        //     while(xPoints.size()<=hLimit && dotCheck == true){
        //         dotCheck = gridDotBoundCheck(mapdata,newPoint);
        //         lidarCheck = gridDotLidarCheck(mapdata,newPoint,boundThresh);

        //         if(dotCheck == true && lidarCheck == true){
        //             xPoints.push_back(newPoint);
        //             xPos += xStep;
        //             newPoint.x = xPos;
        //             newPoint.y = yPos; 
        //             pushed = true;
        //         }else if(dotCheck == true && lidarCheck == false){
        //             //This is to ensure that we still keep checking for dots even if there is an obstacle
        //             //However this means that as long as a point is within max bounds it can be plotted, so corridors and rooms
        //             // Will have points between them
        //             xPos += xStep;
        //             newPoint.x = xPos;
        //             newPoint.y = yPos; 
        //         }
        //     }
        //     if(xPoints.size()>0){
        //         points2.push_back(yPoints);
        //     }
            

        //     //Do Negative Y-axis
        //     dotCheck = true;
        //     xPoints.clear();
        //     xStep = -1*xStep;
        //     xPos = xStep;
        //     newPoint.x = xPos;
        //     newPoint.y = yPos;
        //     while(xPoints.size()<=hLimit && dotCheck == true){
        //         dotCheck = gridDotBoundCheck(mapdata,newPoint);
        //         lidarCheck = gridDotLidarCheck(mapdata,newPoint,boundThresh);

        //         if(dotCheck == true && lidarCheck == true){
        //             xPoints.push_back(newPoint);
        //             xPos += xStep;
        //             newPoint.x = xPos;
        //             newPoint.y = yPos; 
        //             pushed = true;
        //         }else if(dotCheck == true && lidarCheck == false){
        //             xPos += xStep;
        //             newPoint.x = xPos;
        //             newPoint.y = yPos; 
        //         }
        //     }
        //     if(yPoints.size()>0){
        //         points2.push_back(yPoints);
        //     }
            
        //     xStep = -1*xStep;//Change Back to positive
        //     yPos += yStep;    
        //     noRuns+=1;
        //     if(pushed == true){
        //         pushedPoints =+1;
        //     }

        // }


        // bool found=false;
        // for(int i =0;i<points2.size();i++){
        //     for(int j=0;j<points2[i].size();j++){
        //         for(int k=0;k<points.size();k++){
        //             for(int z=0;z<points[k].size();k++){
        //                 if(points[k][z].x==points2[i][j].x && points[k][z].y==points2[i][j].y){
        //                     found=true;
        //                 }
        //             }
        //             if(found==false){
        //                 points.push_back(points2[i][j]);
        //             }
                
        //         }
                
        //     found = false;
        //     }
        // }

    }









    //This function will fill the map with non-traversable points
    void mapConverter(vector<vector<GridPoint>> gridOld, vector<vector<GridNode>>& gridNew){
        for(int i =0;i<gridOld.size();i++){
            vector<GridNode> nodeVector;
            for(int j =0;j<gridOld[i].size();j++){
                GridNode node(gridOld[i][j].x, gridOld[i][j].y);
                nodeVector.push_back(node);
            }
            gridNew.push_back(nodeVector);
        }
    }

}