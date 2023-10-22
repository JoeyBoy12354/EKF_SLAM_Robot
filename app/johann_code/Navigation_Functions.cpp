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
        cout<<"IN triangFunction "<<endl;
        cout<<"State x = "<<State(0)<<" State y = "<<State(1)<<"State theta = "<<State(2)*180/PI<<endl;
        cout<<"Angle = "<<angle*180/PI<<endl;
        vector<CarPoint> triangle_init = {{0, 0}, {-81, 71}, {-81, -71}};
        vector<CarPoint> triangle_rot = rotateTriangle(triangle_init,State(2));
        vector<CarPoint> triangle_shift = translateTriangle(triangle_rot,{State(0),State(1)});
        saveTriangleToCSV(triangle_shift);

        CarPoint C; //Will hold the value of the tip/front of triangle after rotation
        CarPoint B = {triangle_shift[0].x,triangle_shift[0].y}; //Will hold the value of the tip/front of triangle
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

        cout<<"NAVI,TRIANGLE: tri_rot = "<<triangle_rot[0]<<","<<triangle_rot[1]<<","<<triangle_rot[2]<<endl;
        cout<<"NAVI,TRIANGLE: tri_shift = "<<triangle_shift[0]<<","<<triangle_shift[1]<<","<<triangle_shift[2]<<endl;

         cout<<"NAVI,TRIANGLE around A = "<<A<<" from B = "<<B<<" resulting C "<<C<<endl;

        return C;

    }

    bool preMapMovement(MatrixXf State, vector<vector<GridPoint>> gridMap){
        cout<<"In pre-map movement"<<endl;
        //take grid map
        bool mapped = true;
        float smallDistance = 10000000;
        CarPoint closestPoint;
        CarPoint robotPoint;
        robotPoint.x = State(0);
        robotPoint.y = State(1);

        //find closest non-traversed point
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
        
        if(mapped == false){
            updateMovement(closestPoint,State);
        }
        

        return mapped;


    }

    //Set distance and angle to go to nearest unexplored grid point
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

    vector<GridPoint> findNeighbours(vector<vector<GridPoint>> gridMap, GridPoint point, float defaultVal,vector<CarPoint>& indexes){
        //Initialize Neighbours
        GridPoint UP;
        UP.x = defaultVal;
        UP.y = defaultVal;
        UP.trav = false;
        GridPoint DOWN;
        DOWN.x = defaultVal;
        DOWN.y = defaultVal;
        DOWN.trav = false;
        GridPoint LEFT;
        LEFT.x = defaultVal;
        LEFT.y = defaultVal;
        LEFT.trav = false;
        GridPoint RIGHT;
        RIGHT.x = defaultVal;
        RIGHT.y = defaultVal;
        RIGHT.trav = false;
        vector<GridPoint> neighbours;
        neighbours.push_back(UP);
        neighbours.push_back(DOWN);
        neighbours.push_back(LEFT);
        neighbours.push_back(RIGHT);

        //Initialize indexes
        

        for(int i =0;i<gridMap.size();i++){
            for(int j =0;j<gridMap[i].size();j++){
                CarPoint index;
                index.x = i;
                index.y = j;
                
                // cout<<"x: "<<gridMap[i][j].x<<", "<<point.x<<" = "<<gridMap[i][j].x - point.x<<endl;
                // cout<<"y: "<<gridMap[i][j].y<<", "<<point.y<<" = "<<gridMap[i][j].y - point.y<<endl;
                //left from point
                if(gridMap[i][j].x - point.x == grid_xstep && gridMap[i][j].y - point.y == 0){
                    neighbours[2] = gridMap[i][j];
                    indexes[2] = index;
                }//right from point
                else if(gridMap[i][j].x - point.x == -grid_xstep && gridMap[i][j].y - point.y == 0){
                    neighbours[3] = gridMap[i][j];
                    indexes[3] = index;
                }//up from point
                else if(gridMap[i][j].x - point.x == 0 && gridMap[i][j].y - point.y == grid_ystep){
                    neighbours[0] = gridMap[i][j];
                    indexes[0] = index;
                }//down from point
                else if(gridMap[i][j].x - point.x == 0 && gridMap[i][j].y - point.y == -grid_ystep){
                    neighbours[1] = gridMap[i][j];
                    indexes[1] = index;
                }

            }
        }

        return neighbours;
    }

    // //Find array of gridPoints to traverse through
    vector<GridPoint> pathFinder(vector<vector<GridPoint>> gridMap, MatrixXf State,GridPoint goal){
        //Assume there is a grid point at (0,0)
        cout<<"In PathFinder"<<endl;

        GridPoint myRobot;
        myRobot.x=State(0);
        myRobot.y=State(1);
        myRobot.trav=false;

        GridPoint current;
        float min_distance = 10000000;

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


        float defaultVal = 15;

        vector<GridPoint> path;
        bool search = true;
        //Get the start node as the gridpoint nearest to myRobot

        path.push_back(current);

        int count = 0;
        while(search){
            
            cout<<"\nCurrent = "<<current<<endl;
            vector<CarPoint> indexes{{0,0},{0,0},{0,0},{0,0}};
            vector<GridPoint> neighbours = findNeighbours(gridMap,current,defaultVal,indexes);
            //Work on minimizing delta first
            cout<<"neigh = ";
            for(int i =0;i<4;i++){
                cout<<neighbours[i]<<", ";
            }
            cout<<endl;
            
             //Check if point is goal
            if(path[path.size()-1].x == goal.x && path[path.size()-1].y == goal.y){
                search = false;
            }
            else{
                if(goal.y - current.y != 0 ){
                    //Should I go up?
                    if(goal.y - neighbours[0].y >=0 && neighbours[0].x != defaultVal && neighbours[0].y != defaultVal){
                        path.push_back(neighbours[0]);
                        gridMap[indexes[0].x][indexes[0].y].trav = true;
                    }
                    //Should I go down?
                    else if(goal.y - neighbours[1].y >=0 && neighbours[1].x != defaultVal && neighbours[1].y != defaultVal){
                        path.push_back(neighbours[1]);
                        gridMap[indexes[1].x][indexes[1].y].trav = true;
                    }
                }
                else if(goal.x - current.x != 0 ){
                    //Should I go left?
                    if(goal.x - neighbours[2].x >=0 && neighbours[2].x != defaultVal && neighbours[2].y != defaultVal){
                        path.push_back(neighbours[2]);
                        gridMap[indexes[2].x][indexes[2].y].trav = true;
                    }
                    //Should I go right?
                    else if(goal.x - neighbours[3].x >=0 && neighbours[3].x != defaultVal && neighbours[3].y != defaultVal){
                        path.push_back(neighbours[3]);
                        gridMap[indexes[3].x][indexes[3].y].trav = true;
                    }
                }
                else{
                    cout<<"FAILURE TO ADD POINT PLEASE HELP"<<endl;
                }

                cout<<"Point added: "<<path[path.size() - 1].x<<", "<<path[path.size() - 1].y<<endl;
                current = path[path.size()-1];
            }
            
        }

        cout<<"Path Found = ";
        for(int i =0;i<path.size();i++){
            cout<<path[i]<<"->";
        }
        cout<<endl;

        //Maybe add some filtering like filtering duplicate points etc
        //Lowering Path Points given that their x-values or their y-valaues are the same
        vector<GridPoint> shortenedPath;
        shortenedPath.push_back(path[0]);
        for(int i =1;i<path.size()-1;i++){
            if(path[i].x != path[i+1].x && path[i].y == path[i+1].y){
                shortenedPath.push_back(path[i]);
            }
        }
        shortenedPath.push_back(path[path.size() - 1]);

        cout<<"ShortPath Found = ";
        for(int i =0;i<shortenedPath.size();i++){
            cout<<shortenedPath[i]<<"->";
        }
        cout<<endl;

        return shortenedPath;
    }

    void postMapMovement(MatrixXf State){
        GridPoint Goal;
        Goal.x = 0;
        Goal.y = 0;
        Goal.trav = false;
        vector<vector<GridPoint>> gridMap;
        readGridFromCSV(gridMap);

        vector<GridPoint> path = pathFinder(gridMap,State,Goal);

        for(int i =0;i<path.size();i++){
            CarPoint pathPoint;
            pathPoint.x=path[i].x;
            pathPoint.y=path[i].y;
            updateMovement(pathPoint,State);

        }






    }

    

    vector<GridNode*> findNeighboursBFS2(vector<vector<Node>>& gridMap, GridNode& point, float defaultVal){
            //Initialize Neighbours
            GridNode UP;
            UP.x = defaultVal;
            UP.y = defaultVal;
            UP.trav = false;
            RIGHT.traversable = false;
            GridNode DOWN;
            DOWN.x = defaultVal;
            DOWN.y = defaultVal;
            DOWN.trav = false;
            RIGHT.traversable = false;
            GridNode LEFT;
            LEFT.x = defaultVal;
            LEFT.y = defaultVal;
            LEFT.trav = false;
            RIGHT.traversable = false;
            GridNode RIGHT;
            RIGHT.x = defaultVal;
            RIGHT.y = defaultVal;
            RIGHT.trav = false;
            RIGHT.traversable = false;
            vector<GridNode*> neighbours;
            neighbours.push_back(UP);
            neighbours.push_back(DOWN);
            neighbours.push_back(LEFT);
            neighbours.push_back(RIGHT);

            //Initialize indexes
            

            for(int i =0;i<gridMap.size();i++){
                for(int j =0;j<gridMap[i].size();j++){
                    
                    // cout<<"x: "<<gridMap[i][j].x<<", "<<point.x<<" = "<<gridMap[i][j].x - point.x<<endl;
                    // cout<<"y: "<<gridMap[i][j].y<<", "<<point.y<<" = "<<gridMap[i][j].y - point.y<<endl;
                    //left from point
                    if(gridMap[i][j].x - point.x == grid_xstep && gridMap[i][j].y - point.y == 0){
                        neighbours[2] = gridMap[i][j];
                    }//right from point
                    else if(gridMap[i][j].x - point.x == -grid_xstep && gridMap[i][j].y - point.y == 0){
                        neighbours[3] = gridMap[i][j];
                    }//up from point
                    else if(gridMap[i][j].x - point.x == 0 && gridMap[i][j].y - point.y == grid_ystep){
                        neighbours[0] = gridMap[i][j];
                    }//down from point
                    else if(gridMap[i][j].x - point.x == 0 && gridMap[i][j].y - point.y == -grid_ystep){
                        neighbours[1] = gridMap[i][j];
                    }

                }
            }

            return neighbours;
        }


    

    // Define a function to find the neighbors of a grid point
    std::vector<GridNode*> findNeighboursBFS(GridNode& node, std::vector<std::vector<GridNode>>& gridMap) {
        // Define neighbor offsets (up, down, left, right)
        int dx[] = {0, 0, -1, 1};
        int dy[] = {-1, 1, 0, 0};

        std::vector<GridNode*> neighbors;

        for (int i = 0; i < 4; ++i) {
            int newX = node.x + dx[i];
            int newY = node.y + dy[i];

            // Check if the neighbor is within bounds and is traversable
            if (newX >= 0 && newY >= 0 && newX < gridMap.size() && newY < gridMap[0].size() && gridMap[newX][newY].traversable) {
                neighbors.push_back(&gridMap[newX][newY]);
            }
        }

        return neighbors;
    }

    // Breadth-first search
    std::vector<GridNode*> bfs(std::vector<std::vector<GridNode>>& gridMap, GridNode& start, GridNode& goal) {
        std::queue<GridNode*> toVisit;
        std::vector<GridNode*> path;

        // Start with the initial node
        toVisit.push(&start);

        while (!toVisit.empty()) {
            GridNode* current = toVisit.front();
            toVisit.pop();
            cout<<"Current = "<<current->x<<" and "<<current->y<<endl;
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

            cout<<"Neighbor size = "<<neighbors.size()<<endl;

            // Enqueue unvisited neighbors
            for (GridNode* neighbor : neighbors) {
                cout<<"NeighBor = "<<neighbor->x<<" and "<<neighbor->y<<endl;
                if (neighbor->traversable) {
                    neighbor->parent = current;
                    toVisit.push(neighbor);
                }
            }
        }

        // No path found
        return path;
    }

    void postMapMovement2(MatrixXf State){

        cout<<"A1"<<endl;
        GridPoint myRobot;
        myRobot.x=State(0);
        myRobot.y=State(1);
        myRobot.trav=false;

        GridPoint current;
        float min_distance = 10000000;

        vector<vector<GridPoint>> gridMap;
        readGridFromCSV(gridMap);

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

        GridNode start(current.x, current.y);
        GridNode goal(0, 0);
        

        vector<vector<GridNode>> gridNew;
        mapConverter(gridMap, gridNew);
        cout<<"Grid Size: "<<gridNew.size()<<"x"<<gridNew[0].size()<<endl;

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



    }

}