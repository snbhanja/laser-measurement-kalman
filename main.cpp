#include <iostream>
#include <sstream>
#include <vector>
#include "Dense"
#include "measurement_package.h"
#include "tracking.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::ifstream;
using std::istringstream;
using std::string;
using std::vector;


int main() {

  /**
   * Set Measurements
   */
  vector<MeasurementPackage> measurement_pack_list;

  // hardcoded input file with laser and radar measurements
  string in_file_name_ = "obj_pose-laser-radar-synthetic-input.txt";
  ifstream in_file(in_file_name_.c_str(), ifstream::in);

  if (!in_file.is_open()) {
    cout << "Cannot open input file: " << in_file_name_ << endl;
  }

  string line;
  // set i to get only first 3 measurments
  int i = 0;
  while (getline(in_file, line) && (i<=3)) {

    MeasurementPackage meas_package;

    istringstream iss(line);
    string sensor_type;
    iss >> sensor_type; // reads first element from the current line
    int64_t timestamp;
    if (sensor_type.compare("L") == 0) {  // laser measurement
      // read measurements
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float x;
      float y;
      iss >> x;
      iss >> y;
      meas_package.raw_measurements_ << x,y;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);

    } else if (sensor_type.compare("R") == 0) {
      // Skip Radar measurements
      continue;
    }
    ++i;
  }

  // Create a Tracking instance
  Tracking tracking;

  // call the ProcessingMeasurement() function for each measurement
  size_t N = measurement_pack_list.size();
  // start filtering from the second frame 
  // (the speed is unknown in the first frame)
  for (size_t k = 0; k < N; ++k) {
    tracking.ProcessMeasurement(measurement_pack_list[k]);
  }

  if (in_file.is_open()) {
    in_file.close();
  }
  return 0;
}


/*
x_=  0.96749
0.405862
 4.58427
-1.83232
P_= 0.0224541         0  0.204131         0
        0 0.0224541         0  0.204131
 0.204131         0   92.7797         0
        0  0.204131         0   92.7797
x_= 0.958365
0.627631
0.110368
 2.04304
P_= 0.0220006         0  0.210519         0
        0 0.0220006         0  0.210519
 0.210519         0   4.08801         0
        0  0.210519         0   4.08801
x_=   1.34291
 0.364408
  2.32002
-0.722813
P_= 0.0185328         0  0.109639         0
        0 0.0185328         0  0.109639
 0.109639         0   1.10798         0
        0  0.109639         0   1.10798

*/
