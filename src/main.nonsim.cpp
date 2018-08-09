#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include "Eigen/Dense"
#include "json.hpp"
#include <math.h>
#include "ukf.h"
#include "tools.h"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using json = nlohmann::json;

int main() {

	/*******************************************************************************
	 *  Set Measurements															 *
	 *******************************************************************************/
//	vector<MeasurementPackage> measurement_pack_list;

	// hardcoded input file with laser and radar measurements
	string in_file_name_ = "../EKF_DATA/obj_pose-laser-radar-synthetic-input02.txt";
	//string in_file_name_ = "../EKF_DATA/obj_pose-laser-radar-synthetic-input.txt";
	ifstream in_file(in_file_name_.c_str(),std::ifstream::in);

	if (!in_file.is_open()) {
		cout << "Cannot open input file: " << in_file_name_ << endl;
	}

	string line;

        UKF ukf;
        Tools tools;
        vector<VectorXd> estimations;
        vector<VectorXd> ground_truth;
 
	// set i to get only first 3 measurments
	int i = 0;
	while(getline(in_file, line) && (i<=30)){

//	    cout << line << endl;




            string sensor_measurment = line;
  
            MeasurementPackage meas_package;
            istringstream iss(sensor_measurment);
            long long timestamp;
  
            // reads first element from the current line
            string sensor_type;
            iss >> sensor_type;
  
            if (sensor_type.compare("L") == 0) {
                          meas_package.sensor_type_ = MeasurementPackage::LASER;
                          meas_package.raw_measurements_ = VectorXd(2);
                          float px;
                          float py;
                          iss >> px;
                          iss >> py;
                          meas_package.raw_measurements_ << px, py;
                          iss >> timestamp;
                          meas_package.timestamp_ = timestamp;
            } else if (sensor_type.compare("R") == 0) {
  
                          meas_package.sensor_type_ = MeasurementPackage::RADAR;
                          meas_package.raw_measurements_ = VectorXd(3);
                          float ro;
                          float theta;
                          float ro_dot;
                          iss >> ro;
                          iss >> theta;
                          iss >> ro_dot;
                          meas_package.raw_measurements_ << ro,theta, ro_dot;
                          iss >> timestamp;
                          meas_package.timestamp_ = timestamp;
            }
            float x_gt;
            float y_gt;
            float vx_gt;
            float vy_gt;
            iss >> x_gt;
            iss >> y_gt;
            iss >> vx_gt;
            iss >> vy_gt;
            VectorXd gt_values(4);
            gt_values(0) = x_gt;
            gt_values(1) = y_gt;
            gt_values(2) = vx_gt;
            gt_values(3) = vy_gt;
            ground_truth.push_back(gt_values);
  
            //Call ProcessMeasurment(meas_package) for Kalman filter
            ukf.ProcessMeasurement(meas_package);
  
            //Push the current estimated x,y positon from the Kalman filter's state vector
  
            VectorXd estimate(4);
  

            //std::cout<< "ukf.x_" << ukf.x_ << std::endl;


            double p_x = ukf.x_(0);
            double p_y = ukf.x_(1);
            double v  = ukf.x_(2);
            double yaw = ukf.x_(3);
  
            double v1 = cos(yaw)*v;
            double v2 = sin(yaw)*v;
  
            estimate(0) = p_x;
            estimate(1) = p_y;
            estimate(2) = v1;
            estimate(3) = v2;
  
            estimations.push_back(estimate);
  
            VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);
  
            json msgJson;
            msgJson["estimate_x"] = p_x;
            msgJson["estimate_y"] = p_y;
            msgJson["rmse_x"] =  RMSE(0);
            msgJson["rmse_y"] =  RMSE(1);
            msgJson["rmse_vx"] = RMSE(2);
            msgJson["rmse_vy"] = RMSE(3);
            auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
//            std::cout << "msg:" <<msg << std::endl;
            //std::cout << "" << "" << std::endl;
            //std::cout << "estimations" << p_x << "    "<< p_y << std::endl;
            //std::cout << "truth:" << x_gt << "    "<< y_gt << std::endl;
            //std::cout << "" << "" << std::endl;
            //std::cout << "#######################################################" << std::endl;
//            std::cout << "#######################################################" << std::endl;







 
		sleep(0.3);
		
	}


//        std::cout<< "ukf.P_" << ukf.P_ << std::endl;

	if(in_file.is_open()){
		in_file.close();
	}
	return 0;
}

