#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cmath>
#include <iostream>
#include <vector>
#include "three_d_boundary_box.h"


void sphere_bb(double x,double y,double z){

    // If coordinate distance to origin is more than radius than its outside
    
    x = x*x;
    y = y*y;
    z = z*z;

    double total{};
    total = x+y+z;
    total = sqrt(total);

    double radius{};
    ros::param::get("sphere/radius",radius);

    if (total<= radius){
        ROS_INFO("WARNING!!! Object in safety box");
    }    
}

void cuboid_bb(double x,double y,double z){

    double side_length {};
    ros::param::get("cuboid/side_length",side_length);

    bool y_flag{false};
    bool x_flag{false};
    bool z_flag{false};

    // Check if x and y are outside the range and set flags

    if (x >= (-side_length) && x <= side_length){
        x_flag = true;
    }
    
    if (y >= (-side_length) && y <= side_length){
        y_flag = true;
    } 

    if (z >= (-side_length) && z <= side_length){
        z_flag = true;
    }
    
    if( x_flag==true && y_flag==true && z_flag==true){
        ROS_INFO("WARNING!!! Object in safety box");
        
    }
    
}

void ellipsoid_bb(double x,double y,double z){

    // same logic for ellipse here

    double major_axis{};
    double semi_major_axis{};
    double minor_axis{};

    // ellipse has major and minor axis to consider
    ros::param::get("ellipsoid/major_axis",major_axis);
    ros::param::get("ellipsoid/semi_major_axis",semi_major_axis);
    ros::param::get("ellipsoid/minor_axis",minor_axis);

    x = (x/minor_axis)*(x/minor_axis);
    y = (y/major_axis)*(y/major_axis);
    z = (z/semi_major_axis)*(z/semi_major_axis);

    x = x + y + z;

    if(x<=1){
        ROS_INFO("WARNING!!! Object in safety box");
    }
}

// starting point of this function which will be used
// to choose which boundary box will be used
void three_d_bb_chooser(double x_coord,double y_coord,double z_coord){
 
    // User choice for what 3D boundary box to use
    // See params.yaml
    int choice{};
    ros::param::get("boundary_box",choice);

    if(choice == 5){
        sphere_bb(x_coord,y_coord,z_coord);
    }
    else if(choice == 6){
        cuboid_bb(x_coord,y_coord,z_coord);
    }
    else{
        ellipsoid_bb(x_coord,y_coord,z_coord);
    }    
}