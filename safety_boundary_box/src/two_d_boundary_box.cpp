#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cmath>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h> 
#include <vector>
#include "two_d_boundary_box.h"

// ### README ###
// Purpose of this function is to see if an object is inside
// the bounding box and to alert 

bool circle_bb(double x,double y){

    // get params

    double radius{};
    ros::param::get("circle/radius",radius);

    // If coords are within the circle --> distance from center < r
    // If coords are on the circle line --> distance from center = r
    // if coords are outside the circle --> distance from center > r
    // Distance from center formula: √((0 - x_coord)² + (0 - y_coord)²)

    double dist_from_center{};
    double total{};

    x = x*x;
    y= y*y;
    total = x + y;

    dist_from_center = sqrt(total);

    //ROS_INFO_STREAM(dist_from_center);

    if(dist_from_center <= radius){
        ROS_INFO("WARNING!!! Object in safety box");
        return true;
    }
    else{
        //ROS_INFO("No object in safety box");
        return false;
    }
}

bool triangle_bb(double x,double y){

    double height{};
    double breath{};

    ros::param::get("triangle/height",height);
    ros::param::get("triangle/breath",breath);

    std::tuple<double,double> A {0,height/2};
    std::tuple<double,double> B {-(breath/2),-(height/2)};
    std::tuple<double,double> C {(breath/2),-(height/2)};
    std::tuple<double,double> P {x,y};

    // use cross product to determine if point is within/on/outside triangle
    // within triangle --> cross product of point with AB,BC,CA --> ALL signs will be same
    // within triangle --> cross product of point with AB,BC,CA --> sum will be 0
    // outside triangle --> cross product of point with AB,BC,CA --> signs will be mixed
    
    // ### LOGIC ###
    // if signs are negative(going clockwise) or positive (going anti_clockwise,
    // then it means point is on the left side or right side of the
    // lines AB/BC/CA ie they are inside the triangle
    // If signs are mixed, it means for some lines point is on the
    // left side and for some its on the right --> ie outside triangle

    /*double temp_x{};
    double temp_y{};

    // Variables to hold vector AB/BC/CA
    std::tuple<double,double> AB {};
    std::tuple<double,double> BC {};
    std::tuple<double,double> CA {};
    
    temp_x = std::get<0>(B) - std::get<0>(A);
    temp_y = std::get<1>(B) - std::get<1>(A);

    // Vector AB
    std::get<0>(AB) = temp_x;
    std::get<1>(AB) = temp_y;

    temp_x = std::get<0>(C) - std::get<0>(B);
    temp_y = std::get<1>(C) - std::get<1>(B);

    // Vector BC
    std::get<0>(BC) = temp_x;
    std::get<1>(BC) = temp_y;

    temp_x = std::get<0>(A) - std::get<0>(C);
    temp_y = std::get<1>(A) - std::get<1>(C);

    // Vector CA
    std::get<0>(CA) = temp_x;
    std::get<1>(CA) = temp_y;

    // Variables to hold vectors AP/BP/CP
    std::tuple<double,double> AP {};
    std::tuple<double,double> BP {};
    std::tuple<double,double> CP {}; 

    // Vector AP
    std::get<0>(AP) = x - std::get<0>(A);
    std::get<1>(AP) = y - std::get<1>(A);

    // Vector BP
    std::get<0>(BP) = x - std::get<0>(B);
    std::get<1>(BP) = y - std::get<1>(B);

    // Vector CP
    std::get<0>(CP) = x - std::get<0>(C);
    std::get<1>(CP) = y - std::get<1>(C);


    double cross_pdt_one{};
    double cross_pdt_two{};
    double cross_pdt_three{};

    // cross product with AB and AP

    // Formula --> Ax*By - Ay*Bx
    
    cross_pdt_one = std::get<0>(AP)*std::get<1>(AB) - std::get<1>(AP)*std::get<0>(AB);

    // cross product with BC and BP
    cross_pdt_two = std::get<0>(BP)*std::get<1>(BC) - std::get<1>(BP)*std::get<0>(BC);

    // cross product with CA and CP
    cross_pdt_three = std::get<0>(CP)*std::get<1>(CA) - std::get<1>(CP)*std::get<0>(CA);*/


    // Variables to hold vector AB/BC/CA
    std::tuple<double,double> AB {};
    std::tuple<double,double> BC {};
    std::tuple<double,double> CA {};

    auto create_vectors = [](std::tuple<double,double> pt_a, std::tuple<double,double> pt_b) -> std::tuple<double,double> {
        
        double temp_x{};
        double temp_y{};

        temp_x = std::get<0>(pt_b) - std::get<0>(pt_a);
        temp_y = std::get<1>(pt_b) - std::get<1>(pt_a);

        std::tuple<double,double> return_var{temp_x,temp_y};    
        
        
        return return_var;         

    };

    AB = create_vectors(A,B);
    BC = create_vectors(B,C);
    CA = create_vectors(C,A);

    // Variables to hold vectors AP/BP/CP
    std::tuple<double,double> AP {};
    std::tuple<double,double> BP {};
    std::tuple<double,double> CP {}; 

    AP = create_vectors(A,P);
    BP = create_vectors(B,P);
    CP = create_vectors(C,P);
    
    auto cross_products = [](std::tuple<double,double> pt_a, std::tuple<double,double> pt_b) -> double {
        
        double return_var{};

        return_var = std::get<0>(pt_a)*std::get<1>(pt_b) - std::get<1>(pt_a)*std::get<0>(pt_b);                
        
        return return_var;         

    };

    double cross_pdt_one{};
    double cross_pdt_two{};
    double cross_pdt_three{};

    cross_pdt_one = cross_products(AP,AB);
    cross_pdt_two = cross_products(BP,BC);
    cross_pdt_three = cross_products(CP,CA);


    // i only check for negative because i have been going anti clockwise for all my vector ops above
    if((cross_pdt_one <= 0 && cross_pdt_two <= 0 && cross_pdt_three <= 0) || (cross_pdt_one > 0 && cross_pdt_two > 0 && cross_pdt_three > 0)){
        ROS_INFO("WARNING!!! Object in safety box");
        return true;
    }
    else{
        return false;
    }
   
}

bool square_bb(double x,double y){

    double square_side{};
    ros::param::get("square/sq_side",square_side);

    square_side = square_side/2;

    bool y_flag{false};
    bool x_flag{false};

    // Check if x and y are outside the range and set flags

    if (x >= (-square_side) && x <= square_side){
        x_flag = true;
    }
    
    if (y >= (-square_side) && y <= square_side){
        y_flag = true;
    } 
    
    if( x_flag==true && y_flag==true){
        ROS_INFO("WARNING!!! Object in safety box");
        return true;
    }
    else{
        return false;
    }
    
}

bool ellipse_bb(double x,double y){

    double major_axis{};
    double minor_axis{};

    // ellipse has major and minor axis to consider
    ros::param::get("ellipse/major_axis",major_axis);
    ros::param::get("ellipse/minor_axis",minor_axis);


    // For vertical ellipse, you will need to plug in the coordinates into the standard
    // formula --> (x^2 / a^2) + (y^2 / b^2)
    // a --> minor and b --> major
    // If point inside the ellipse --> result < 1
    // If point on the ellipse --> result = 1
    // If point outside the ellipse --> result > 1

    double total{};

    total = (pow(x,2)/pow(minor_axis,2))+(pow(y,2)/pow(major_axis,2));

    if(total <= 1){
        ROS_INFO("WARNING!!! Object in safety box");
        return true;
    }
    else{
        return false;
    }    
}

// Raycasting Algorithm
// if number of intersects is odd --> point inside the pentagon

struct Point{
    double x;
    double y;
};

bool pentagon_bb(double x,double y){

    double L{};
    ros::param::get("pentagon/len_from_mid_to_vertice",L);

    Point coordinate = {x,y};

    // 5 vertices
    // A is the point infront of the robot and the the rest are going anti-clockwise
    Point A = {0, L};
    Point B = {-L * cos(M_PI / 10), L * sin(M_PI / 10)};    // 72° from A (18° from x-axis)
    Point C ={-L * cos(3 * M_PI / 10), -L * sin(3 * M_PI / 10)};  // 144°
    Point D ={L * cos(3 * M_PI / 10), -L * sin(3 * M_PI / 10)};   // 216°
    Point E ={L * cos(M_PI / 10), L * sin(M_PI / 10)};     // 288°

    std::vector<Point> vertices{A,B,C,D,E};

    int counter{0};

    const double EPSILON = 1e-6; // Small precision threshold

       
    for(size_t i=0;i<5;++i){

        Point pt_one = vertices[i];
        // we do this so that when vertice i refers to point E, vertice i+1 can loop back to point A
        Point pt_two = vertices[(i+1)%5];

        double max_y(std::max(pt_one.y,pt_two.y));
        double min_y(std::min(pt_one.y,pt_two.y));

        double max_x(std::max(pt_one.x,pt_two.x));
        double min_x(std::min(pt_one.x,pt_two.x));

        // check if the y is within range and x is within range
        if((coordinate.y >= (min_y - EPSILON)) && (coordinate.y <= (max_y + EPSILON))){
        
            // eddge case were the vertice form a horizontal line --> in this case
            // point can be on the left,on the line, right of line
            // we only care for on the line case
            // with the above 2 conditions we know the point is within ranges of line --> therefore only thing we need to check
            // is if line is horizontal --> if its counter can +1
            if(std::abs(pt_one.y - pt_two.y) < EPSILON ){
                if((coordinate.x <= (max_x + EPSILON)) && (coordinate.x >= (min_x - EPSILON))){
                    ROS_INFO("here");
                    counter++;
                }
            }
            else{
                // now we need to see coordinate's X-INTERCEPT with the verticex
                // x-intercept = (y-intercept - c)/((y2-y1)/(x2-x1))
                // c = y1 - x1(((y2-y1)/(x2-x1)))

                double gradient = (pt_two.y - pt_one.y)/(pt_two.x - pt_one.x);
                //double c = pt_one.y - pt_one.x*(gradient);
                double x_intercept = ((coordinate.y - pt_one.y)/(1/gradient)) + pt_one.x;

                // to visualise why this condition, do the calculations on paper to get an intuitve understanding
                // Essentially if the point is on the right handside of line and inside polygon, its x-coordinate will be more than its x-intercept or
                // if point is inside polygon and on the left of line, its x-coordinate will be less than its x-intercept --> this is the condition we are testing
                // for as we only care for ray to go towards the right, first case is ignored
                if(coordinate.x <= (x_intercept + EPSILON)){
                    ROS_INFO_STREAM(i);
                    ROS_INFO_STREAM((i+1)%5);
                    counter++;
                }
            }               
            
        }   
       
    }

    //ROS_INFO_STREAM(counter);
    //ROS_INFO_STREAM(counter);
    if((counter%2) == 1){
        //ROS_INFO("WARNING!!! Object in safety box");
        return true;
    }
    else{
        return false;
    } 
    
}


// starting point of this function which will be used
// to choose which boundary box will be used
void bb_chooser(double x_coord,double y_coord){

    
    // User choice for what boundary box to use
    // See params.yaml
    int choice{};
    ros::param::get("boundary_box",choice);

    bool flag{};

    // EXPLANATION --> for triangle you need to 
    // swap coordinates because front of robot is facing +ve x-axis

    if(choice == 0){
        flag = triangle_bb(y_coord,x_coord);
    }
    else if(choice == 1){
        flag = circle_bb(x_coord,y_coord);
    }
    else if(choice == 2){
        flag = square_bb(x_coord,y_coord);
    }
    else if(choice == 3){
        flag = ellipse_bb(x_coord,y_coord);
    }
    else{
        flag = pentagon_bb(y_coord,x_coord);
    }

    //ROS_INFO_STREAM(flag);

    //return flag;

}



