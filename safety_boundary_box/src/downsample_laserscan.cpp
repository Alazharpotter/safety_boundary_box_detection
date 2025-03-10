#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cmath>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <vector> 
#include "downsample_laserscan.h"


sensor_msgs::LaserScan downsample_laserscan(sensor_msgs::LaserScan scan){

    // how nth point do you remove
    int n = 5;

    std::vector<float>ranges_ds{};        
        
    for(size_t i =0; i<scan.ranges.size(); i += n){

        // adds every ith value in scan's range value to new range vector
        ranges_ds.push_back(scan.ranges[i]);

    }
    
    // Have to change angle_increment accordingly
    scan.angle_increment = (scan.angle_increment)*n;

    // Have to change range values accordingly
    scan.ranges = ranges_ds;


    return scan;
}