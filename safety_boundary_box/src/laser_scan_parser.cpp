#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cmath>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h> 
#include <vector>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include "two_d_boundary_box.h"
#include "three_d_boundary_box.h"
#include "test.h"
#include "std_msgs/Bool.h"
#include "downsample_laserscan.h"

// ### README ###
// This is subscription node that subscribes
// to laserscan/pointcloud message
// Based on user input on choice of lidar data,
// laser points are converted to coordinates
// either (x,y) or (x,y,z) --> for 2D and 3D respectively


// Global counter to track messages
// Use this to manually ensure not every scan message is being processed
int message_counter = 0;
const int N = 20; // Process every 20th message

// function to convert polar coordinates to cartesian
// can be used by both 2D and 3D
// if 2D it will ignore the z-coord

std::tuple<float,float> polar_to_cartesian_coords(float angle_one, float range){

    float x_coord{};
    float y_coord{};
    
    // to get x coordinate
    x_coord = range*cos(angle_one);

    // get y coordinate
    y_coord = range*sin(angle_one);

    
    std::tuple<double,double> coordinates{x_coord,y_coord};
    
    return coordinates;
}


// Function to parse through LaserScan Message

void two_d_laserscan_clbk(const sensor_msgs::LaserScan::ConstPtr& laser_msg){

    // safety flag which will be true if object within safety area dictated by user in params file
    bool safety_flag_n(false);

    // Increment the counter
    message_counter++;

    // create downsample scan message
    sensor_msgs::LaserScan downsample_scan{};  
    
    
    // To only process every Nth message
    if(message_counter==N)
    {
    
        // initialise point_dist with laser range values, ie each value in the vector
        // is the distance from lidar to that point
        // ### If not downsampling can do this instead
        /*std::vector<float> point_dist = laser_msg -> ranges;
        float angle_min = laser_msg -> angle_min;
        float angle_increment = laser_msg -> angle_increment;*/

        // Downsample data for faster processing --> since precision not important, downsampling can help improve reactivity of system to obstacles
        downsample_scan = *laser_msg;
        downsample_scan = downsample_laserscan(downsample_scan);

        std::vector<float> point_dist = downsample_scan.ranges;
        float angle_min = downsample_scan.angle_min;
        float angle_increment = downsample_scan.angle_increment;
        
        // angle variable to store angle of each laser beam
        float angle{};

        // coordinates vector
        std::tuple<double,double> coordinates{};

        //ROS_INFO("Receiving Scan Data!");
        
        
        for(int i=0;i<point_dist.size();i++){

            angle = angle_min + (i*angle_increment);

            // igrnore if range == inf
            if(!(std::isinf(point_dist[i]))){
            
                // use angle value + range value to get coordinate of point
                // wrt to lidar
                // x,y coordinates
                // ROS_INFO_STREAM(point_dist[i]);
                coordinates = polar_to_cartesian_coords(angle,point_dist[i]);        

                // pass rosnode handle too for other function
                // to access params
                bb_chooser(std::get<0>(coordinates),std::get<1>(coordinates));            
            }
        }  
        
        // Reset it
        message_counter = 0;
    }

}

// Function to parse through Pointcloud2 message
void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg){

    // create PCL Pointcloud object
    // Use smart pointers instead
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert Pointcloud 2 to PCL object format
    pcl::fromROSMsg(*pointcloud_msg,*cloud);

    // Set Passthrough filter to ignore floor

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);  // Set the input point cloud
    pass.setFilterFieldName("z");  // Set the field to filter on (z-axis)
    double z_filter{};
    ros::param::get("min_height",z_filter);
    pass.setFilterLimits(z_filter, std::numeric_limits<float>::max());  // Set the filter limits for height
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter(*cloud_filtered);  // Apply the filter

    // downsample for faster processing

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_filtered);  // Set the input point cloud
    vg.setLeafSize(0.1f, 0.1f, 0.1f);  // Set the size of the voxel grid

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    vg.filter(*cloud_downsampled);  // Apply the voxel grid filter

    float x{};
    float y{};
    float z{};

    for(size_t i=0;i<cloud_downsampled->points.size();++i){
        x = cloud_downsampled->points[i].x;
        y = cloud_downsampled->points[i].y;
        z = cloud_downsampled->points[i].z;

        // pass into function to check
        three_d_bb_chooser(x,y,z);
    }

    
}

int main(int argc, char **argv){

    // Initialisations
    ros::init(argc,argv,"laser_data_parser");
    ros::NodeHandle n;

    ros::Subscriber sub;  

    // Load user input from config --> params.yaml --> lidar_type
    int lidar_type{0};    

    // User input for type of Ops
    ros::param::get("lidar_type",lidar_type);   
    
    if(lidar_type ==0){
        
        // Sub to scan topic (2D)
                      
        sub = n.subscribe("scan",500,two_d_laserscan_clbk); 
        
    }
    else{
        
        // Sub to scan topic (3D)
        
        sub = n.subscribe("velodyne_points",500,pointcloud_callback);
        
    }
      
    //ros::spin();   

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;

}