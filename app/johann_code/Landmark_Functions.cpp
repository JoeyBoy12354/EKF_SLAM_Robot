#include "robot.h"

using namespace CSV_Functions;



namespace Landmark_Functions{

    vector<CarPoint> getCorners(){
        int ret;
        ret = system("python cornerDetector.py ok go");
        //ret = system("/usr/bin/python3 cornerDetector.py ok go");
        cout << "ret/cpp = " << ret << endl;

        vector<CarPoint> corners;
        readCornersFromCSV(corners);

        cout<<"LM: Number of Corners = "<<corners.size();

        return corners;
    }

}