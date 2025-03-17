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
#include <cstdlib> // For system() to display the file

#define RAD2DEG(rad)(rad*180./M_PI)
#define DEG2RAD(deg)(deg*M_PI/180.)


float odomX = 0.0, odomY = 0.0, odomYaw = 0.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    odomX = msg->pose.pose.position.x;
    odomY = msg->pose.pose.position.y;
    odomYaw = tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("Position: (%f, %f) Orientation: %f rad", odomX, odomY, odomYaw);
}

// int tagIDs[5];
// float xCoords[5];
// float yCoords[5];
// float yawCoords[5];
// std::string tagNames[5];
// int tag_idx = 0;

// // Function to write tag information to a file
// void writeTagInfoToFile(int tagID, float x, float y, float yaw, const std::string& tagName) {
//     std::ofstream outFile("tag_info.txt", std::ios::app); // Open file in append mode
//     if (outFile.is_open()) {
//         outFile << "Tag ID: " << tagID << ", Position: (" << x << ", " << y << "), Yaw: " << yaw << ", Name: " << tagName << std::endl;
//         outFile.close();
//     } else {
//         ROS_ERROR("Unable to open file for writing tag information.");
//     }
// }

// Structure to store tag information
struct TagInfo {
    int tagID;
    float x, y, yaw;
    std::string tagName;
};

// Function to write tag information to a file
void writeTagInfoToFile(const std::vector<TagInfo>& tagInfoList, const std::string& filename) {
    std::ofstream outFile(filename);
    if (outFile.is_open()) {
        for (const auto& tagInfo : tagInfoList) {
            if (tagInfo.tagID == -1) {
                outFile << "Template ID: -1 (Empty Image or No Match)" << std::endl;
            } else {
                outFile << "Tag ID: " << tagInfo.tagID << ", Position: (" << tagInfo.x << ", " << tagInfo.y 
                        << "), Yaw: " << tagInfo.yaw << ", Name: " << tagInfo.tagName << std::endl;
            }
        }
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

                ros::spinOnce();
                int templateID = imagePipeline.getTemplateID(boxes);

                std::cout<< "Arrived"<<std::endl;


                // if (templateID == -1) {
                //     templateID = imagePipeline.getTemplateID(boxes);
                // }

                std::cout << "c2 Final Template ID: " << templateID << std::endl;
                ros::Duration(0.01).sleep();

                //////////////////////////////////////////
                std::string tagName = (templateID == -1) ? "No Match" : "Tag_" + std::to_string(templateID);

                // Store tag information
                TagInfo tagInfo;
                tagInfo.tagID = templateID;
                tagInfo.x = xGoal;
                tagInfo.y = yGoal;
                tagInfo.yaw = yawGoal;
                tagInfo.tagName = tagName;
                tagInfoList.push_back(tagInfo);
        
                std::cout << "c2 Final Template ID: " << templateID << std::endl;
                ros::Duration(0.01).sleep();
            }
        
            // Write tag information to file
            std::string filename = "tag_info.txt";
            writeTagInfoToFile(tagInfoList, filename);
        
            // Display the file at program exit
            std::cout << "Displaying tag information from file:" << std::endl;
        #ifdef _WIN32
            system("type tag_info.txt"); // For Windows
        #else
            system("cat tag_info.txt"); // For Linux/Mac
        #endif                

                //  // Write tag information to file
                //  if (templateID >= -1) {
                //     tagIDs[tag_idx] = templateID;
                //     xCoords[tag_idx] = xGoal;
                //     yCoords[tag_idx] = yGoal;
                //     yawCoords[tag_idx] = yawGoal;
                //     std::string tagName = "Tag_" + std::to_string(templateID); // Modify as needed
                //     tagNames[tag_idx] = tagName;
                //     tag_idx++;
                //     //writeTagInfoToFile(templateID, xGoal, yGoal, yawGoal, tagName);
                // }

               
            }
            Navigation::moveToGoal(originX,originY, originYaw);

            // for (int i=0; i<5; i++) {
            //     writeTagInfoToFile(tagIDs[i], xCoords[i], yCoords[i], yawCoords[i], tagNames[i]);
            // }


            break;
        }

    }
    return 0;
}
