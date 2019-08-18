/***********************************************************************************************
 * Copyright (c) 2018, Youlian Long, Robotic Lab, Sun Yat-sen University, Guangzhou, China.
 * You can contact the author with <longylian@foxmail.com>
 **********************************************************************************************/

#include <ros/ros.h>
#include <iostream>
#include "gps/gps.h"
using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gpa_tran_node");
	ros::NodeHandle nh;

	double init_longitude = 113.3847256315;
	double init_latitude  = 23.0685365248;
	double init_altitude  = 30.0;

	cout.flags(ios::fixed);
	cout.precision(9); 
	cout << "initial GPS is:" << endl;
	cout << "init_longitude \t= " << init_longitude << endl;
	cout << "init_latitude \t= " << init_latitude << endl;
	cout << "init_altitude \t= " << init_altitude << endl; 

	GpsTran gps_tran(init_longitude, init_latitude, init_altitude);

	GpsDataType gps;
	NedDataType ned;
	gps.longitude = 113.384984656;
	gps.latitude  = 23.068384355;
	gps.altitude  = 26.743499738;
	gps_tran.fromGpsToNed(ned, gps);

	cout << "\nNED of current GPS is:" << endl;
	cout << "x_north \t= " << ned.x_north << endl;
	cout << "y_east \t= " << ned.y_east << endl;
	cout << "z_down \t= " << ned.z_down << endl;
	
	return 0;
	
}
