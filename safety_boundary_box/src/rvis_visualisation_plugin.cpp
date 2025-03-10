#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>


bool safety_flag{false};

// function to fill up the message based on user params etc
visualization_msgs::Marker message_filler(visualization_msgs::Marker marker,int bb_box_type){
    
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "visual_plugin";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.a = 0.4; // transparency
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.scale.z = 0.1;

    // change colour when something enters the safety boundary box
    /*if(safety_flag ==true){
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
    else{
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;        
    }*/
    
    // in 3D one can change z for min height
    marker.pose.position.z = 0;

    // get min height param for 3D
    double min_height{};
    ros::param::get("min_height",min_height);    
     
    
    switch ((bb_box_type+1))
    {
        // triangle
        case 1:
        {
            
            marker.type = visualization_msgs::Marker::TRIANGLE_LIST;  // Triangle list
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
           
            // Define triangle vertices
            geometry_msgs::Point p1, p2, p3;

            double height{};
            double breath{};

            ros::param::get("triangle/height",height);
            ros::param::get("triangle/breath",breath);

            height = height/2;
            breath = breath/2;
                      
            // this is because front of robot is facing +ve x-axis
            
            p1.x = height;         // Apex
            p1.y = 0.0;
            p1.z = 0.0;

            p2.x = -height;  // Bottom left
            p2.y = -breath;
            p2.z = 0.0;
            
            p3.x = -height;   // Bottom right
            p3.y = breath;
            p3.z = 0.0;
            
            marker.points.push_back(p1);
            marker.points.push_back(p3);
            marker.points.push_back(p2);
            
            /*
            Can also do this way
            
            p1.x = 0.0;         // Apex
            p1.z = 0.0;
            p1.y = height;

            p2.x = -breath;  // Bottom left
            p2.y = -height;
            p2.z = 0.0;
            
            p3.x = breath;   // Bottom right
            p3.y = -height;
            p3.z = 0.0;
            

            // Add Two Triangles
            marker.points.push_back(p1);
            marker.points.push_back(p2);
            marker.points.push_back(p3);
            

            // Set orientation to align the triangle correctly
            // For a 90-degree rotation around the Z-axis:
            double angle = M_PI/2 ;  // 90 degrees in radians
            
            marker.pose.orientation.z = sin(1.5*angle);  // sin(π/4) = √2/2
            marker.pose.orientation.w = cos(1.5*angle);  // cos(π/4) = √2/2*/

            
            break;
        }

        // circle
        case 2:
        {

            double radius{};
            ros::param::get("circle/radius",radius);

            // becase x and y are the length along the x and y axix
            marker.scale.x = 2*radius;
            marker.scale.y = 2*radius;
            marker.type = visualization_msgs::Marker::SPHERE;
    
            break;
        }
        
        // square
        case 3:
        {
            double square_side{};
            ros::param::get("square/sq_side",square_side);
            
            marker.scale.x = square_side;
            marker.scale.y = square_side;
            marker.type = visualization_msgs::Marker::CUBE;
            
            // to make sphere into circle
            marker.scale.z = 0.0;    

            break;
        }

        // ellipse
        case 4:
        {
            double major_axis{};
            double minor_axis{};
        
            // ellipse has major and minor axis to consider
            ros::param::get("ellipse/major_axis",major_axis);
            ros::param::get("ellipse/minor_axis",minor_axis);

            marker.scale.x = major_axis;
            marker.scale.y = minor_axis;            
            marker.type = visualization_msgs::Marker::SPHERE;
    
            break;
        }

        // pentagon
        case 5:
        {
            double L{}; 
            ros::param::get("pentagon/len_from_mid_to_vertice",L);
            
            marker.type = visualization_msgs::Marker::TRIANGLE_LIST;  // Triangle list
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            
            // Define the vertices of the pentagon
            geometry_msgs::Point p1, p2, p3, p4, p5, p6;

            
            // x and y's have been swapped as front of robot is facing +ve x-axis
            p1.y = 0.0;  // Top vertex
            p1.x = L;
            p1.z = 0.0;

            p2.y = -L*cos(M_PI / 10);  // Top-left vertex
            p2.x = L*sin(M_PI / 10);
            p2.z = 0.0;

            p3.y = -L*cos(3 * M_PI / 10) ; // Bottom-rleft vertex
            p3.x = -L*sin(3 * M_PI / 10);
            p3.z = 0.0;

            p4.y = L * cos(3 * M_PI / 10); // Bottom-right vertex
            p4.x = -L * sin(3 * M_PI / 10);
            p4.z = 0.0;

            p5.y = L * cos(M_PI / 10); // Top-right vertex
            p5.x = L * sin(M_PI / 10);
            p5.z = 0.0;


            p6.x = 0.0; // Top-left vertex
            p6.y = 0.0;
            p6.z = 0.0;


            // Add vertices to the marker in order
            marker.points.push_back(p6); 
            marker.points.push_back(p2); 
            marker.points.push_back(p1);

            marker.points.push_back(p6); 
            marker.points.push_back(p3); 
            marker.points.push_back(p2); 

            marker.points.push_back(p6); 
            marker.points.push_back(p4); 
            marker.points.push_back(p3); 

            marker.points.push_back(p6); 
            marker.points.push_back(p5); 
            marker.points.push_back(p4);

            marker.points.push_back(p6); 
            marker.points.push_back(p1); 
            marker.points.push_back(p5);                    
            
            
            break;
        }

        // sphere
        case 6:
        {
            marker.type = visualization_msgs::Marker::SPHERE;

            double sphere_radius {};
            ros::param::get("sphere/radius",sphere_radius);

            marker.scale.x = 2*sphere_radius;
            marker.scale.y = 2*sphere_radius;
            marker.scale.z = 2*sphere_radius;
            marker.pose.position.z = sphere_radius + min_height;            
    
            break;
        }
        // cuboid
        case 7:
        {
            marker.type = visualization_msgs::Marker::CUBE;

            double side_length {};
            ros::param::get("cuboid/side_length",side_length);

            marker.scale.x = side_length;
            marker.scale.y = side_length;
            marker.scale.z = side_length;
            marker.pose.position.z = min_height;
    
            break;
        }
        // ellipsoid
        case 8:
        {
            marker.type = visualization_msgs::Marker::SPHERE;

            double major_axis{};
            double semi_major_axis{};
            double minor_axis{};

            // ellipse has major and minor axis to consider
            ros::param::get("ellipsoid/major_axis",major_axis);
            ros::param::get("ellipsoid/semi_major_axis",semi_major_axis);
            ros::param::get("ellipsoid/minor_axis",minor_axis);

            marker.scale.x = major_axis;
            marker.scale.y = minor_axis;
            marker.scale.z = semi_major_axis;
            marker.pose.position.z = min_height;
    
            break;
        }
        default:
            break;
    }
 
    return marker;    

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "visual_plugin");
    ros::NodeHandle nh_t;

    int bb_box_type{};
    nh_t.getParam("boundary_box",bb_box_type);

    ros::Publisher vis_pub = nh_t.advertise<visualization_msgs::Marker>( "visualization_marker", 1000 );
    ros::Rate loop_rate(10);

        
    while (ros::ok())
    {
        visualization_msgs::Marker marker;
        marker = message_filler(marker,bb_box_type);
        
        ROS_INFO("Publishing visuals");
        vis_pub.publish(marker);

        ros::spinOnce();

        loop_rate.sleep();
        
    }    
    
    return 0;

}