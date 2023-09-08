#include "robot.h"

string cornersCSV = "cornersCSV.csv";
string linesCSV = "linesCSV.csv";
string mapCSV = "mapCSV.csv";
string fullMapCSV = "fullMapCSV.csv";
string motorCSV = "motorCSV.csv";
string landmarkCSV = "landmarkCSV.csv";
string positionCSV = "positionCSV.csv";


namespace CSV_Functions{

    void savePolToCSV(const vector<PolPoint>& points, const string& filename) {
        ofstream outputFile(filename);
        if (!outputFile.is_open()) {
            cerr << "Error opening the file: " << filename << endl;
            return;
        }

        // Write the data to the CSV file
        for (const PolPoint& point : points) {
            outputFile << point.angle << "," << point.distance << "\n";
        }

        outputFile.close();
    }

    void appendPolToCSV(const vector<PolPoint>& points, const string& filename) {
        ofstream outputFile(filename, ios::app); //open file in append mode
        if (!outputFile.is_open()) {
            cerr << "Error opening the file: " << filename << endl;
            return;
        }

        // Write the data to the CSV file
        for (const PolPoint& point : points) {
            outputFile << point.angle << "," << point.distance << "\n";
        }

        outputFile.close();
    }

    //New Map Functions
    void saveCarToCSV(const vector<CarPoint>& points) {
        cout<<"\n save Car to CSV\n"<<endl;
        cout<<"\nnumber of points = "<<points.size()<<endl;
        ofstream outputFile(mapCSV);
        if (!outputFile.is_open()) {
            cerr << "Error opening the file: " << mapCSV << endl;
            return;
        }

        // Write the data to the CSV file
        for (const CarPoint& point : points) {
            outputFile << point.x << "," << point.y << "\n";
        }

        outputFile.close();
    }

    void appendCarToCSV(const vector<CarPoint>& points) {
        ofstream outputFile(mapCSV, ios::app);
        if (!outputFile.is_open()) {
            cerr << "Error opening the file: " << mapCSV << endl;
            return;
        }

        // Write the data to the CSV file
        for (const CarPoint& point : points) {
            outputFile << point.x << "," << point.y << "\n";
        }

        outputFile.close();
    }

    void readCarFromCSV(vector<CarPoint>& points){
        ifstream file(mapCSV);

        if (!file.is_open()) {
            cerr << "Error opening file: " << mapCSV << endl;
            return;
        }

        string line;
        while (getline(file, line)) {
            istringstream iss(line);
            string x_str, y_str;

            if (getline(iss, x_str, ',') && getline(iss, y_str)) {
                CarPoint point;
                point.x = stod(x_str);
                point.y = stod(y_str);
                points.push_back(point);
            }
        }

        file.close();
    }

    //Full Map Functions
    void saveCarToFullMapCSV(const vector<CarPoint>& points) {
        ofstream outputFile(fullMapCSV);
        if (!outputFile.is_open()) {
            cerr << "Error opening the file: " << fullMapCSV << endl;
            return;
        }

        // Write the data to the CSV file
        for (const CarPoint& point : points) {
            outputFile << point.x << "," << point.y << "\n";
        }

        outputFile.close();
    }

    void appendCarToFullMapCSV(const vector<CarPoint>& points) {
        ofstream outputFile(fullMapCSV, ios::app);
        if (!outputFile.is_open()) {
            cerr << "Error opening the file: " << fullMapCSV << endl;
            return;
        }

        // Write the data to the CSV file
        for (const CarPoint& point : points) {
            outputFile << point.x << "," << point.y << "\n";
        }

        outputFile.close();
    }

    void readCarFromFullMapCSV(vector<CarPoint>& points){
        ifstream file(fullMapCSV);

        if (!file.is_open()) {
            cerr << "Error opening file: " << fullMapCSV << endl;
            return;
        }

        string line;
        while (getline(file, line)) {
            istringstream iss(line);
            string x_str, y_str;

            if (getline(iss, x_str, ',') && getline(iss, y_str)) {
                CarPoint point;
                point.x = stod(x_str);
                point.y = stod(y_str);
                points.push_back(point);
            }
        }

        file.close();
    }

    //State Functions
    void saveLandmarkToCSV(const vector<CarPoint>& landmarks){
        ofstream outputFile(landmarkCSV);
        if (!outputFile.is_open()) {
            cerr << "Error opening the file: " << landmarkCSV << endl;
            return;
        }

        // Write the data to the CSV file
        for (const CarPoint& point : landmarks) {
            outputFile << point.x << "," << point.y << "\n";
        }

        outputFile.close();
    }

    void savePositionToCSV(const vector<float>& position){
        ofstream outputFile(positionCSV);
        if (!outputFile.is_open()) {
            cerr << "Error opening the file: " << positionCSV << endl;
            return;
        }

        // Write the data to the CSV file
        outputFile << position[0] << "\n" << position[1] << "\n" << position[2] << "\n";
        outputFile.close();
    }


    //Line and Corner Functions
    void writeLinesToCSV(const vector<Line>& lines) {
        ofstream csvFile(linesCSV);
        if (!csvFile) {
            cerr << "Error opening file: " << linesCSV << endl;
            return;
        }

        // Write the CSV header
        csvFile << "Line,Gradient,Intercept,Domain_Min,Domain_Max,Range_Min,Range_Max" << endl;

        // Write the lines' data to the CSV file
        for (int i = 0; i < lines.size(); ++i) {
            const Line& line = lines[i];
            csvFile << i + 1 << ","
                    << line.gradient << ","
                    << line.intercept << ","
                    << line.domain_min << ","
                    << line.domain_max << ","
                    << line.range_min << ","
                    << line.range_max << endl;
        }

        csvFile.close();
    }

    void writeCornersToCSV(const vector<CarPoint>& corners){
        ofstream csvFile(cornersCSV);
        if (!csvFile) {
            cerr << "Error opening file: " << cornersCSV << endl;
            return;
        }

        // Write the lines' data to the CSV file
        for (int i = 0; i < corners.size(); ++i) {
            const CarPoint& corner = corners[i];
            csvFile << corner.x << ","
                    << corner.y << endl;
        }

        csvFile.close();
    }

    void readCornersFromCSV(vector<CarPoint>& corners){
        ifstream file(cornersCSV);

        if (!file.is_open()) {
            cerr << "Error opening file: " << cornersCSV << endl;
            return;
        }

        string line;
        while (getline(file, line)) {
            istringstream iss(line);
            string x_str, y_str;

            if (getline(iss, x_str, ',') && getline(iss, y_str)) {
                CarPoint point;
                point.x = stod(x_str);
                point.y = stod(y_str);
                corners.push_back(point);
            }
        }

        file.close();
    }

    //Motor
    void writeMotorToCSV(float angle, float distance){
        ofstream csvFile(motorCSV);
        if (!csvFile) {
            cerr << "Error opening file: " << motorCSV << endl;
            return;
        }

        // Write to CSV
        csvFile << angle << endl << distance <<endl;

        csvFile.close();

    }

    void readMotorFromCSV(float& angle, float& distance, float& time){
        ifstream file(motorCSV);

        if (!file.is_open()) {
            cerr << "Error opening file: " << motorCSV << endl;
            return;
        }

        //skip first two lines
        

        string line;
        getline(file, line);
        getline(file, line);

        while (getline(file, line)) {
            angle = stod(line);

            if (getline(file, line)){
                distance = stod(line);
            }

            if (getline(file, line)){
                time = stod(line);
            }
        }

        file.close();

    }

}