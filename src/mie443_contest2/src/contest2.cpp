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

#define RAD2DEG(rad)(rad*180./M_PI)
#define DEG2RAD(deg)(deg*M_PI/180.)


float odomX = 0.0, odomY = 0.0, odomYaw = 0.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    odomX = msg->pose.pose.position.x;
    odomY = msg->pose.pose.position.y;
    odomYaw = tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("Position: (%f, %f) Orientation: %f rad", odomX, odomY, odomYaw);
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
    float offset = 0.4;

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
                boxYaw= boxes.coords[u][2];
                xGoal= boxes.coords[u][0] + cos(DEG2RAD(boxYaw))*offset;
                yGoal= boxes.coords[u][1] +sin(DEG2RAD(boxYaw))*offset;
                yawGoal= DEG2RAD(boxYaw) + M_PI;
                ROS_INFO("Goal %d: (%f, %f, %f)", u, xGoal, yGoal, yawGoal);
                Navigation::moveToGoal(xGoal, yGoal, yawGoal);
                ros::Duration(1).sleep();

                //-------------calling image detection------------
                // for (int g=0; g<2; ++g)
                // {
                    std:: vector<int> templateIDs(5,-1);
                    for (int i=0; i<5; ++i)
                    {
                        templateIDs[i]=imagePipeline.getTemplateID(boxes);
                    }

                    //find the most common ids
                    std::sort(templateIDs.begin(),templateIDs.end());
                    int mostCommonID=templateIDs[0];
                    int maxCount=1,currentCount=1;

                    for (size_t i=1; i<templateIDs.size();++i)
                    {
                        if (templateIDs[i]==templateIDs[i -1])
                        {
                            currentCount++;
                        }
                        else
                        {
                            currentCount=1;
                        }
                        if(currentCount>maxCount)
                        {
                            maxCount=currentCount;
                            mostCommonID=templateIDs[i];
                        }
                    }

                    recognizedTemplates.push_back(mostCommonID);
                    std::cout<<"Most common template ID: " << mostCommonID<<std::endl;
                    std::cout<<"Templates Vector: [";
                    for (int id:recognizedTemplates)
                    {
                        std::cout<<id<<"";
                    }
                    std::cout<<"]"<<std::endl;
                // }
                //------------------------------------------------
            }
            Navigation::moveToGoal(originX,originY, originYaw);
            break;
        }

        
        //imagePipeline.getTemplateID(boxes);
        ros::Duration(0.01).sleep();
    }
    return 0;
}
