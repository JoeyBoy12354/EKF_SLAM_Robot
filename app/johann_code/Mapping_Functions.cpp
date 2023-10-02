#include "robot.h"

using namespace CSV_Functions;
using namespace Landmark_Functions;
using namespace Data_Functions;


namespace Mapping_Functions{

    //Build Full map, place it in csv
    //Points have to be translated
    void storeMapPoints(vector<CarPoint> lidardata){
        float distance_threshold = 20;//If two points are greater than Xmm then keep this point.

        vector<CarPoint> oldmap;
        readCarFromFullMapCSV(oldmap);//Fetch all current points
        

        //Compare with new points (only update if points are different)
        //If a point is less than Xmm away from oldmap point then remove it from circulation
        for(int i=0; i<oldmap.size();i++){
            for(int j=0; j<lidardata.size();j++){
                    vector<CarPoint> temp;
                    if(pointDistance(lidardata[j],oldmap[i]) > distance_threshold){
                        temp.push_back(lidardata[j]);
                    }
                    lidardata = temp;
            }
        }

        cout<<"No of New Points = "<<lidardata.size()<<endl;
    

        //Update full map
        appendCarToFullMapCSV(lidardata);
        
    }

    void storeStatePoints(Matrix<float, dim, 1> State){
        cout<<"\nIN STORE STATE POINTS"<<endl;
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
            landmarks.push_back(LM);
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


    void gridDataProcess(vector<vector<GridPoint>>& gridNew,Matrix<float, dim, 1> State, bool firstRun){

        cout<<"\n\nGRID: In grid data process"<<endl;
        vector<CarPoint> map;
        readCarFromFullMapCSV(map);
        gridMakeDots(map,gridNew);

        if(firstRun == false){
            vector<vector<GridPoint>> gridOld;
            readGridFromCSV(gridOld);
            gridDataAssosciationMap(gridOld,gridNew);
            gridDataAssosciationMove(gridNew,State);
        }

        saveGridToCSV(gridNew);
        cout<<"GRID:SAVED TO CSV \n\n"<<endl;

        

        
        return;
        

    }    

    void gridDataAssosciationMove(vector<vector<GridPoint>>& gridNew, Matrix<float, dim, 1> State){
        float thresh= 200;//If points are less than Xmm from travelled line bounds then allow point to be considered.
        float dist_thresh= 290;//If points are less than Xmm from travelled line then set trav=true.
        
        //Update traversal from movement points,state
        float angle;
        float distance;
        readMotorFromCSV(angle, distance);
        float x_new = distance*cos(angle);
        float y_new = distance*sin(angle);

        float x_old = State[0];
        float y_old = State[1];

        float x_max = (x_new > x_old) ? x_new : x_old;
        float y_max = (y_new > y_old) ? y_new : y_old;

        Line tLine;
        tLine.gradient = (y_new-y_old)/(x_new-x_old);
        tLine.intercept = y_old-(tLine.gradient*x_old);
        tLine.domain_max = (x_new > x_old) ? x_new : x_old;
        tLine.domain_min = (x_new < x_old) ? x_new : x_old;
        tLine.range_max = (y_new > y_old) ? y_new : y_old;
        tLine.range_min = (y_new > y_old) ? y_new : y_old;



        for(int i = 0;i<gridNew.size();i++){
            for(int j =0;j<gridNew[i].size();j++){
                CarPoint point;
                point.x = gridNew[i][j].x;
                point.y = gridNew[i][j].y;

                //Ensure point is within bounds of where line should exist
                if(point.x > tLine.domain_min-thresh && point.x < tLine.domain_max+thresh &&
                   point.y > tLine.range_min-thresh  && point.y < tLine.range_max+thresh){

                    if(perpendicularDistance(point,tLine) <= dist_thresh){
                        gridNew[i][j].trav = true;
                    }
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

    bool gridDotBoundCheck(vector<CarPoint> searchMap, GridPoint point,float distThresh, vector<float> bounds){
        if(searchMap.size()==0){
            cout<<"Bound Check SearchMap==0"<<endl;
            return false;
        }

        

        //check if point is within max and min of lidardata bounds=[Xmax,Xmin,Ymax,Ymin]
        if(point.x>bounds[0] || point.x<bounds[1] || point.y>bounds[2] || point.y<bounds[3] ){
            return false;
        }

        //cout<<"GRID Point(inbound) x="<<point.x<<" y="<<point.y<<endl;

        //Get Cartesian
        CarPoint point2;
        point2.x = point.x;
        point2.y = point.y;
        // cout<<" "<<endl;

        cout<<"GRID3 MAPDATA"<<endl;
        cout<<"point: ("<<point.x<<","<<point.y<<") SM size = "<<searchMap.size()<<endl;
        for(int i = 0;i<searchMap.size();i++){
            if(searchMap[i].x>190 && searchMap[i].x < -210 && searchMap[i].y<-400){
                cout<<"("<<searchMap[i].x<<","<<searchMap[i].y<<")";
            }
        }
        cout<<endl;
            
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
            cout<<"Returning ("<<point2.x<<","<<point2.y<<") dist = "<<temp_dist<<" at ("<<smallPoint.x<<","<<smallPoint.y<<")"<<endl;
            return false;

        }else{
            cout<<"Passing ("<<point2.x<<","<<point2.y<<") dist = "<<temp_dist<<" at ("<<smallPoint.x<<","<<smallPoint.y<<")"<<endl;
            return true;
        }



        

    }
   
    void gridMakeDots(vector<CarPoint> mapdata, vector<vector<GridPoint>>& points){
        

            
        
        //We need to create the vertical lines
        //Might be good to calculate this with max size of current lidar scan
        float vLimit = 10; //Max number of vertical points from x-axis in 1 direction
        float hLimit = 10; //Max Number of horistontal points from v-axis in 1 direction

        float yStep = 200;//y-distance between points on same x-coordinate
        float xStep = 200;//x-distance between points on same y-coordinate
        
        
        float boundThresh = 180;//If distance between gridPoint and lidarPoint <= Xmm then return false 

        float xPos; //holds current x-coordinate
        float yPos; //holds current y-coordinate

        int maxNoRuns = 20;
        int noRuns = 0;

        vector<float> bounds; //Xmax,Xmin,Ymax,Ymin
        getMapBounds(mapdata,bounds);

        // cout<<endl;
        // cout<<"GRID2 MAPDATA"<<endl;
        // for(int i = 0;i<mapdata.size();i++){
        //     if(mapdata[i].x>-200 && mapdata[i].x < -150 && mapdata[i].y<-400){
        //         cout<<"("<<mapdata[i].x<<","<<mapdata[i].y<<")";
        //     }
        // }
        //cout<<endl;

        vector<GridPoint> yPoints;
        GridPoint newPoint;


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
            // cout<<"GRID: POSTIVE X-Axis WHile Loop NoRuns = "<<noRuns<<endl;

            //Do Positive Y-Axis
            yPoints.clear();
            yPos = 0;
            newPoint.x = xPos;
            newPoint.y = yPos;
            while(yPoints.size()<=vLimit && gridDotBoundCheck(mapdata,newPoint,boundThresh,bounds)){
                newPoint.x = xPos;
                newPoint.y = yPos;
                yPos += yStep;

                yPoints.push_back(newPoint);
            }
            if(yPoints.size()>0){
                points.push_back(yPoints);
            }
            

            //Do Negative Y-axis
            yPoints.clear();
            yStep = -1*yStep;
            yPos = yStep;
            newPoint.x = xPos;
            newPoint.y = yPos;
            while(yPoints.size()<=vLimit && gridDotBoundCheck(mapdata,newPoint,boundThresh,bounds)){
                newPoint.x = xPos;
                newPoint.y = yPos;
                yPos += yStep;

                cout<<"PosX, NegY PNT:("<<newPoint.x<<","<<newPoint.y<<")"<<endl;

                yPoints.push_back(newPoint);
            }
            if(yPoints.size()>0){
                
                points.push_back(yPoints);
            }
            

            yStep = -1*yStep;//Change Back to positive
            xPos += xStep;    
            noRuns+=1;
        }
        
        //Negative X-axis
        noRuns = 0;
        xStep = -1*xStep;
        xPos = xStep;
        while(points.size()<=hLimit*2 && noRuns<maxNoRuns){
            // cout<<"GRID: NEGATIVE X-Axis WHile Loop"<<endl;

            //Do Positive Y-Axis
            yPoints.clear();
            yPos = 0;
            newPoint.x = xPos;
            newPoint.y = yPos;
            while(yPoints.size()<=vLimit && gridDotBoundCheck(mapdata,newPoint,boundThresh,bounds)){
                newPoint.x = xPos;
                newPoint.y = yPos;
                yPos += yStep;

                yPoints.push_back(newPoint);
            }
            if(yPoints.size()>0){
                points.push_back(yPoints);
            }

            //Do Negative Y-axis
            yPoints.clear();
            yStep = -1*yStep;
            yPos = yStep;
            newPoint.x = xPos;
            newPoint.y = yPos;
            while(yPoints.size()<=vLimit && gridDotBoundCheck(mapdata,newPoint,boundThresh,bounds)){
                newPoint.x = xPos;
                newPoint.y = yPos;
                yPos += yStep;

                yPoints.push_back(newPoint);
            }
            if(yPoints.size()>0){
                points.push_back(yPoints);
            }
            

            yStep = -1*yStep;//Change Back to positive
            xPos += xStep;    
            noRuns+=1;
            }
            
        
    }

    

}