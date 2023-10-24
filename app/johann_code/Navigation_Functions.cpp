#include "robot.h"

using namespace Landmark_Functions;
using namespace CSV_Functions;
using namespace Data_Functions;
using namespace Mapping_Functions;


int noNavTrials = 0;
vector<bool> LandmarksExplored;
double foundRadius = 250;//Robot must be within Xmm of LM for LM to be considered Explored
double closeness = 150;//Robot should travel distance: (Robot->LM) - closeness mm

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
        int scalar = 4;
        
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

    bool mapMovement(MatrixXf State, vector<vector<GridPoint>> gridMap, vector<CarPoint>& path){
        cout<<"In pre-map movement"<<endl;
        //take grid map
        bool mapped = true;
        CarPoint closestPoint;


        if(path.size() == 0){
            //find closest non-traversed point
            closestPoint = findNextPoint(State,gridMap,mapped);

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


    //Set distance and angle to go to given grid point
    void updateMovement(CarPoint closestPoint,MatrixXf State){
        cout<<"ClosestPoint = "<<closestPoint<<endl;

        //Set destination
        float deltaX = closestPoint.x - State(0);
        float deltaY = closestPoint.y - State(1);
        float angle = atan2(deltaY,deltaX) - State(2);
        angle = pi_2_pi(angle);
        //angle = PI-angle;

        //angle = PI/2;
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

        //Set motors
        motorControlGrid(angle,distance);
        
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
                std::reverse(path.begin(), path.end()); // Reverse to get the path from start to goal
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
            pathCartesian = pathShortening(pathCartesian);
            
        }
        

        if(pathCartesian.empty()) {
            cout << "No Shortened path found." << endl;
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


    