// test file

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cmath>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h> 
#include <vector>


void test_bb(double x,double y){
    
    /*std::string name {};
    ros::param::get("robot_name",name);
    ROS_INFO_STREAM(name);*/
    
    double radius{2.0};
    
    // If coords are within the circle --> distance from center < r
    // If coords are on the circle line --> distance from center = r
    // if coords are outside the circle --> distance from center > r
    // Distance from center formula: √((0 - x_coord)² + (0 - y_coord)²)

    double dist_from_center{};
    double total{};

    x = pow(x,2);
    y= pow(y,2);
    total = x + y;

    dist_from_center = sqrt(total);

    if (dist_from_center <= radius){
        ROS_INFO("WARNING!!! Object in safety box");
    }
}