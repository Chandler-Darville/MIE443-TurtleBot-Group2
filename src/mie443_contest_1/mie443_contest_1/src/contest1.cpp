#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

//Odometry header files
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <cmath>

#include <chrono>

float angular = 0.0;
float linear = 0.2;

double posX=0.0, posY=0.0, yaw=0.0;

#define N_BUMPER (3)
#define RAD2DEG(rad)((rad)*180./M_PI)
#define DEG2RAD(deg)((deg)*M_PI/180.)

float minLaserDist = std::numeric_limits <float> ::infinity();
int32_t nLasers = 0, desiredNLasers = 0, desiredAngle = 5;

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED,kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
}

void bumperMovement(geometry_msgs::Twist vel, ros::Publisher vel_pub)
{
    bool leftState = (bumper[kobuki_msgs::BumperEvent::LEFT] == kobuki_msgs::BumperEvent::PRESSED);
    bool centerState = (bumper[kobuki_msgs::BumperEvent::CENTER] == kobuki_msgs::BumperEvent::PRESSED);
    bool rightState = (bumper[kobuki_msgs::BumperEvent::RIGHT] == kobuki_msgs::BumperEvent::PRESSED);

    if (leftState) {
        ROS_WARN("Left bumper hit! Moving backward...");
        vel.angular.z = 0.0;
        vel.linear.x = -0.1;
        vel_pub.publish(vel);
        ros::Duration(1.5).sleep(); //robot move backward for 1 second

        ROS_WARN("Turning Right to find open space...");
        vel.angular.z = -M_PI/3;
        vel.linear.x = 0.0;
        vel_pub.publish(vel);
        ros::Duration(1.5).sleep(); 
    }

    else if (centerState) {
        ROS_WARN("Center bumper hit! Moving backward...");
        vel.angular.z = 0.0;
        vel.linear.x = -0.1;
        vel_pub.publish(vel);
        ros::Duration(1.5).sleep(); //robot move backward for 1 second

        ROS_WARN("Turning Left to find open space...");
        angular = M_PI/2;
        linear = 0.0;
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        ros::Duration(1.5).sleep(); 
    }

    else if (rightState) {
        ROS_WARN("Right bumper hit! Moving backward...");
        vel.angular.z = 0.0;
        vel.linear.x = -0.1; //move backwward
        vel_pub.publish(vel);
        ros::Duration(1.5).sleep(); //robot move backward for 1 second

        ROS_WARN("Turning Left to find open space...");
        vel.angular.z = M_PI/3;  //turn 60 degrees right
        vel.linear.x = 0.0;
        vel_pub.publish(vel);
        ros::Duration(1.5).sleep(); 
    }
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	minLaserDist=std::numeric_limits<float>::infinity();
    
    nLasers=(msg->angle_max-msg->angle_min)/msg->angle_increment; // total number of lasers
    desiredNLasers=DEG2RAD(desiredAngle)/msg->angle_increment;// index of the desired laser
    //ROS_INFO("Size of laser scan array: %i and size of offset: %i",nLasers,desiredNLasers); // offset is the number of sectors from the center sector

    if(DEG2RAD(desiredAngle)<msg->angle_max && DEG2RAD(-desiredAngle)>msg->angle_min) //if the desired angle is within the size of the cone
    {
        for(uint32_t laser_idx = nLasers/2-desiredNLasers; laser_idx < nLasers/2 + desiredNLasers;++laser_idx)
        {
            //ROS_INFO("Min laser dist: %f, msg->ranges[laser_idx]: %f", minLaserDist, msg->ranges[laser_idx]);

            minLaserDist=std::min(minLaserDist, msg->ranges[laser_idx]);
        }

        if(minLaserDist==std::numeric_limits <float> ::infinity())
        {
            minLaserDist=0;
        }
        ROS_INFO("Minimum Laser Dist: %f",minLaserDist);
    }
    else // if the desired angle is not in the cone (probably not useful)
    {
        for(uint32_t laser_idx = 0; laser_idx<nLasers; ++laser_idx)
        {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
        // ROS_INFO("Minimum Laser Dist: %i",minLaserDist);
        
        if(minLaserDist==std::numeric_limits <float> ::infinity())
        {
            minLaserDist=0;
        }
    }
    
    ros::Duration(0.5).sleep();
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}

// Random Exploration - Moves and Turns Randomly
void explore(geometry_msgs::Twist &vel, ros::Publisher &vel_pub) {
    if (minLaserDist < 0.8) {  // If an obstacle is close
        ROS_WARN("Obstacle detected! Turning...");
        vel.linear.x = 0.0;
        vel.angular.z = (rand() % 2 == 0) ? M_PI / 6 : -M_PI / 6;  // Random turn direction
    } else {
        vel.linear.x = 0.2;  // Move forward
        vel.angular.z = ((rand() % 10) == 0) ? ((rand() % 2 == 0) ? M_PI / 6 : -M_PI / 6) : 0.0;  // Occasional random turn
    }

    vel_pub.publish(vel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "maze_explorer");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Subscriber odom = nh.subscribe("odom", 1, odomCallback);

    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.2;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        
        // //Check if any of the bumpers were pressed
        // bool any_bumper_pressed = false;
        // for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx){
        //     any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        // }

        bumperMovement(vel, vel_pub);
        explore(vel, vel_pub);

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}
