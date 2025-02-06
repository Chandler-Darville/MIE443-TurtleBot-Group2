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
float linear = 1.0;

double posX=0.0, posY=0.0, yaw=0.0;

#define N_BUMPER (3)
#define RAD2DEG(rad)((rad)*180./M_PI)
#define DEG2RAD(deg)((deg)*M_PI/180.)


uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED,kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
//     bool leftState = bumper[kobuki_msgs::BumperEvent::LEFT];
//     bool centerState = bumper[kobuki_msgs::BumperEvent::CENTER];
//     bool rightState = bumper[kobuki_msgs::BumperEvent::RIGHT];
}

float minLaserDist = std::numeric_limits <float> ::infinity();
int32_t nLasers = 0, desiredNLasers = 0, desiredAngle = 10;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	minLaserDist=std::numeric_limits<float>::infinity();
    
    nLasers=(msg->angle_max-msg->angle_min)/msg->angle_increment; // total number of lasers
    desiredNLasers=DEG2RAD(desiredAngle)/msg->angle_increment;// index of the desired laser
    ROS_INFO("Size of laser scan array: %i and size of offset: %i",nLasers,desiredNLasers); // offset is the number of sectors from the center sector

    if(DEG2RAD(desiredAngle)<msg->angle_max && DEG2RAD(-desiredAngle)>msg->angle_min) //if the desired angle is within the size of the cone
    {
        for(uint32_t laser_idx = nLasers/2-desiredNLasers; laser_idx < nLasers/2 + desiredNLasers;++laser_idx)
        {

            ROS_INFO("Min laser dist: %f, msg->ranges[laser_idx]: %f", minLaserDist, msg->ranges[laser_idx]);

            minLaserDist=std::min(minLaserDist, msg->ranges[laser_idx]);
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
    }
    
    ros::Duration(1.5).sleep();
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
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
    float linear = 2.0;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        
        //Check if any of the bumpers were pressed
        bool any_bumper_pressed = false;
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx){
            any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        }

        //Bumper Avoidance
        if (any_bumper_pressed) {
            //if bumper is hit, move backward
            ROS_WARN("Bumper hit! Moving backward...");
            angular = 0.0;
            linear = -1.0; //move backward
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);
            ros::Duration(1.5).sleep(); //robot move backward for 1 second

            //turn find open space
            ROS_WARN("Turning to find open space...");
            angular = M_PI/3;
            linear = 0.0;
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);
            ros::Duration(1.5).sleep(); 

            //reset bumper state
            for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx){
                any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
            }
        }

        else if (minLaserDist > 0.5){ //if space is open
            angular = 0.0;
            linear = 1.0;
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);
        }

        else { //if too close to the wall or object, turn
            angular = M_PI/2;
            linear = 0.0;
            vel.angular.z = angular;
            vel.linear.x = linear;
            vel_pub.publish(vel);
        }

        // //Control  logic after bumpers were pressed
        // if (posX<0.5 && yaw < M_PI/12 && !any_bumper_pressed){
        //     angular = 0.0;
        //     linear = 0.2;
        // }

        // else if (yaw < M_PI/2 && posX > 0.5 && !any_bumper_pressed) {
        //     angular = M_PI/6;
        //     linear = 0.0;
        // }

        // else{
        //     angular = 0.0;
        //     linear = 2.0;
        //     break;
        // }

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}