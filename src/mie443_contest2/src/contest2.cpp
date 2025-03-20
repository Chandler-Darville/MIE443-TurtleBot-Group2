#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <math.h>

#include <fstream> //for file operation
#include <iostream>
#include <string>
#include <cstdlib>


#define RAD2DEG(rad)(rad*180./M_PI)
#define DEG2RAD(deg)(deg*M_PI/180.)


float odomX = 0.0, odomY = 0.0, odomYaw = 0.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    odomX = msg->pose.pose.position.x;
    odomY = msg->pose.pose.position.y;
    odomYaw = tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("Position: (%f, %f) Orientation: %f rad", odomX, odomY, odomYaw);
}


// Function to write tag information to a file
void writeTagInfoToFile(int tagID, float x, float y, float yaw, const std::string& tagName, int counter) {
    std::ofstream outFile("tag_info.txt", std::ios::app); // Open file in append mode
    if (outFile.is_open()) {
        outFile << "Tag ID: " << tagID << ", Position: (" << x << ", " << y << "), Yaw: " << yaw << ", Name: " << tagName << ", Appeared" << counter << " time(s)" << std::endl;
        outFile.close();
    } else {
        ROS_ERROR("Unable to open file for writing tag information.");
    }
}

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);

    // Odometry subscriber
    ros::Subscriber odom = n.subscribe("odom", 1, &odomCallback);

    //Wheel command publisher
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel;

    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }
    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);
    std::vector<int> recognizedTemplates;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    //Initializing variables
    int i =0;
    float startingYaw, currentX, currentY, currentYaw, originX, originY, originYaw;
    float offsetAngle = 20;
    int attempt = 0;
    bool goalSucess;

    int template_blank_counter=0;
    int template_0_counter=0;
    int template_1_counter=0;
    int template_2_counter=0;

    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300) {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        //ROS_INFO("Odom: (%f, %f, %f)", odomX, odomY, odomYaw);
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        currentX = robotPose.x;
        currentY=robotPose.y;
        currentYaw = robotPose.phi;

        //360 spin at the start
        if (i==0){
          startingYaw= odomYaw;
          i++;
          continue;
        }

        if(i==1)
        {
            if (not (odomYaw>startingYaw-0.3 && odomYaw<startingYaw-0.01)){
                ROS_INFO("Odom: (%f) starting: %f", odomYaw, startingYaw);
                vel.angular.z = M_PI/2;
                vel_pub.publish(vel);
            }
            else{
                originX = currentX;
                originY = currentY;
                originYaw = currentYaw;
                i++;
                ROS_INFO("Origin Coordinates: (%f, %f, %f)", originX, originY, originYaw);
            }
        }
        else{
            for (int u = 0; u<5;u++){
                float xGoal, yGoal, boxYaw, yawGoal;
                do{ 
                    if(attempt==0){
                        boxYaw= boxes.coords[u][2];
                        xGoal= boxes.coords[u][0] + cos(DEG2RAD(boxYaw))*0.4;
                        yGoal= boxes.coords[u][1] +sin(DEG2RAD(boxYaw))*0.4;
                        yawGoal= DEG2RAD(boxYaw) + M_PI;    
                    }                                // getting valid target coordinate by changing offset angle
                    if(attempt==1){
                        boxYaw= boxes.coords[u][2]-offsetAngle;
                        xGoal= boxes.coords[u][0] + cos(DEG2RAD(boxYaw))*0.4;
                        yGoal= boxes.coords[u][1] +sin(DEG2RAD(boxYaw))*0.4;
                        yawGoal= DEG2RAD(boxYaw) + M_PI;
                    }
                    if(attempt==2){
                        boxYaw= boxes.coords[u][2]+offsetAngle;
                        xGoal= boxes.coords[u][0] + cos(DEG2RAD(boxYaw))*0.4;
                        yGoal= boxes.coords[u][1] +sin(DEG2RAD(boxYaw))*0.4;
                        yawGoal= DEG2RAD(boxYaw) + M_PI;
                    }
                    ROS_INFO("Goal %d: (%f, %f, %f)", u, xGoal, yGoal, yawGoal);
                    goalSucess = Navigation::moveToGoal(xGoal, yGoal, yawGoal);
                    if (goalSucess){
                        attempt=0;
                        break;
                    }
                    attempt ++;
                    
                    if(attempt >=3){
                        attempt=0;
                        break;
                    }    
                }
                while (!goalSucess);

                ros::spinOnce();
                int templateID = imagePipeline.getTemplateID(boxes);

                std::cout<< "Arrived"<<std::endl;


                std::cout << "c2 Final Template ID: " << templateID << std::endl;
                ros::Duration(0.01).sleep();           


                // write goal coordinates to file every loop

                // std::string tagName = "template" + std::to_string(templateID);
                std::string tagName = "";
                if(templateID==0)
                {
                    tagName = "raison bran";
                    template_0_counter=template_0_counter+1;
                    writeTagInfoToFile(templateID, xGoal, yGoal, yawGoal, tagName,template_0_counter);
                }
                else if(templateID==1)
                {
                    tagName = "cinnamon toast crunch";
                    template_1_counter=template_1_counter+1;
                    writeTagInfoToFile(templateID, xGoal, yGoal, yawGoal, tagName,template_1_counter);
                }
                else if(templateID==2)
                {
                    tagName = "rice krispies";
                    template_2_counter=template_2_counter+1;
                    writeTagInfoToFile(templateID, xGoal, yGoal, yawGoal, tagName,template_2_counter);
                }
                else
                {
                    tagName = "BLANK";
                    template_blank_counter=template_blank_counter+1;
                    writeTagInfoToFile(templateID, xGoal, yGoal, yawGoal, tagName,template_blank_counter);
                }

                std::cout << "To write to file:-----------------------" << std::endl;
                std::cout << "      templateID: " << templateID << std::endl;
                std::cout << "      xGoal: " << xGoal << std::endl;
                std::cout << "      yGoal: " << yGoal << std::endl;
                std::cout << "      yawGoal: " << yawGoal << std::endl;
                std::cout << "      tagName: " << tagName << std::endl;

                // writeTagInfoToFile(templateID, xGoal, yGoal, yawGoal, tagName);

               
            }
            bool success;
            success = Navigation::moveToGoal(originX, originY, originYaw);
            
            int end_tout = 0;

            while (!success && end_tout < 10){

                float newx, newy;   // vars for new x and y coords
                float ran_val = ros::Time::now().toSec() - floor(ros::Time::now().toSec()); // rand [0,1]
                // random offset to new coords
                newx = originX + 0.2*cos(DEG2RAD(360*ran_val));
                newy= originY + 0.2*sin(DEG2RAD(360*ran_val));

                success = Navigation::moveToGoal(originX, originY, originYaw);

                end_tout++;
            }

            if (success) {

                for (int i = 0; i < 4; i++) {

                    vel.angular.z = 1*pow(-1,i);    // changes rotational speed cw/ccw every iter
                    vel_pub.publish(vel);
                    ros::Duration(2).sleep();       // rotates for 2 seconds
                }
                vel.angular.z = 0;    // stop
                vel_pub.publish(vel);

            } else if (end_tout >= 10) {
                std::cout << "10 tries timeout" << std::endl;
            }




            break;
        }

    }
    return 0;
}
