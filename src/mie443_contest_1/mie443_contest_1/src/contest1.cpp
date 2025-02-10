#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>

// Odometry header files
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <cmath>

#include <chrono>

float angular = 0.0;
float linear = 0.2;

// Global Variables to stores robot position
double posX = 0.0, posY = 0.0, yaw = 0.0;
std::vector<std::pair<double, double>> visited_positions;

// Threshold for detecting revisited locations
const double revisit_threshold = 0.5; // meters

#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad) * 180. / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.)

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg)
{
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
}

void bumperMovement(geometry_msgs::Twist vel, ros::Publisher vel_pub)
{
    bool leftState = (bumper[kobuki_msgs::BumperEvent::LEFT] == kobuki_msgs::BumperEvent::PRESSED);
    bool centerState = (bumper[kobuki_msgs::BumperEvent::CENTER] == kobuki_msgs::BumperEvent::PRESSED);
    bool rightState = (bumper[kobuki_msgs::BumperEvent::RIGHT] == kobuki_msgs::BumperEvent::PRESSED);

    if (leftState)
    {
        ROS_WARN("Left bumper hit! Moving backward...");
        vel.angular.z = 0.0;
        vel.linear.x = -0.1;
        vel_pub.publish(vel);
        ros::Duration(1.5).sleep(); // robot move backward for 1.5 second

        ROS_WARN("Turning Right to find open space...");
        vel.angular.z = -M_PI / 3;
        vel.linear.x = 0.0;
        vel_pub.publish(vel);
        ros::Duration(1.5).sleep();
    }

    else if (centerState)
    {
        ROS_WARN("Center bumper hit! Moving backward...");
        vel.angular.z = 0.0;
        vel.linear.x = -0.1;
        vel_pub.publish(vel);
        ros::Duration(1.5).sleep(); // robot move backward for 1.5 second

        ROS_WARN("Turning Left to find open space...");
        angular = M_PI / 2;
        linear = 0.0;
        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);
        ros::Duration(1.5).sleep();
    }

    else if (rightState)
    {
        ROS_WARN("Right bumper hit! Moving backward...");
        vel.angular.z = 0.0;
        vel.linear.x = -0.1; // move backwward
        vel_pub.publish(vel);
        ros::Duration(1.5).sleep(); // robot move backward for 1 second

        ROS_WARN("Turning Left to find open space...");
        vel.angular.z = M_PI / 3; // turn 60 degrees right
        vel.linear.x = 0.0;
        vel_pub.publish(vel);
        ros::Duration(1.5).sleep();
    }
}

float minLaserDistCenter = std::numeric_limits <float> ::infinity();
float minLaserDistLeft = std::numeric_limits <float> ::infinity();
float minLaserDistRight = std::numeric_limits <float> ::infinity();
int32_t nLasers = 0, desiredNLasersCenter = 0, desiredNLasersLeft = 0, desiredNLasersRight = 0, desiredAngleCenter = 10,desiredAngleRight = 10,desiredAngleLeft = 10;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	minLaserDistCenter=std::numeric_limits<float>::infinity();
    minLaserDistLeft=std::numeric_limits<float>::infinity();
    minLaserDistRight=std::numeric_limits<float>::infinity();

    nLasers=(msg->angle_max-msg->angle_min)/msg->angle_increment; // total number of laser indices
    desiredNLasersCenter=DEG2RAD(desiredAngleCenter)/msg->angle_increment;// number of indices to search in center
    desiredNLasersLeft=DEG2RAD(desiredAngleLeft)/msg->angle_increment;// number of indices to search on the left
    desiredNLasersRight=DEG2RAD(desiredAngleRight)/msg->angle_increment;// number of indices to search on the right
    //ROS_INFO("Size of laser scan array: %i and size of offset: %i",nLasers,desiredNLasersCenter); // offset is the number of sectors from the center sector

//Minimum center distance
    if(DEG2RAD(desiredAngleCenter)<msg->angle_max && DEG2RAD(-desiredAngleCenter)>msg->angle_min) //if the desired angle is within the size of the cone
    {
        for(uint32_t laser_idx = nLasers/2-desiredNLasersCenter; laser_idx < nLasers/2 + desiredNLasersCenter;++laser_idx) // Center sector
        {
            //ROS_INFO("Min laser dist: %f, msg->ranges[laser_idx]: %f", minLaserDistCenter, msg->ranges[laser_idx]);

            minLaserDistCenter=std::min(minLaserDistCenter, msg->ranges[laser_idx]);
        }

        if(minLaserDistCenter==std::numeric_limits <float> ::infinity())
        {
            minLaserDistCenter=0;
        }
        ROS_INFO("Minimum Laser Dist CENTER: %f",minLaserDistCenter);
    }

    else // if the desired angle is not in the cone (probably not useful)
    {
        for(uint32_t laser_idx = 0; laser_idx<nLasers; ++laser_idx)
        {
            minLaserDistCenter = std::min(minLaserDistCenter, msg->ranges[laser_idx]);
        }
        // ROS_INFO("Minimum Laser Dist: %i",minLaserDistCenter);
        
        if(minLaserDistCenter==std::numeric_limits <float> ::infinity())
        {
            minLaserDistCenter=0;
        }
    }

//Minimum distance on the right
    if(DEG2RAD(desiredAngleRight)<((msg->angle_max)-(msg->angle_min)))
    {
        for(uint32_t laser_idx=0; laser_idx<desiredNLasersRight; ++laser_idx)
        {
            minLaserDistRight=std::min(minLaserDistRight, msg->ranges[laser_idx]);
        }
        if(minLaserDistRight==std::numeric_limits <float> ::infinity())
        {
            minLaserDistRight=0;
        }
        ROS_INFO("Minimum Laser Dist RIGHT: %f",minLaserDistRight);
    }
    else{
        for(uint32_t laser_idx = 0; laser_idx<nLasers; ++laser_idx)
        {
            minLaserDistRight = std::min(minLaserDistRight, msg->ranges[laser_idx]);
        }       
        if(minLaserDistRight==std::numeric_limits <float> ::infinity())
        {
            minLaserDistRight=0;
        }
        ROS_INFO("Minimum Laser Dist RIGHT: %f",minLaserDistRight);
    }

//Minimum distance on the left
    if(DEG2RAD(desiredAngleLeft)<((msg->angle_max)-(msg->angle_min)))
    {
        for(uint32_t laser_idx=nLasers-1; laser_idx>(nLasers-1)-desiredNLasersLeft; --laser_idx)
        {
            minLaserDistLeft=std::min(minLaserDistLeft, msg->ranges[laser_idx]);
        }
        if(minLaserDistLeft==std::numeric_limits <float> ::infinity())
        {
                minLaserDistLeft=0;
        }

        ROS_INFO("Minimum Laser Dist LEFT: %f",minLaserDistLeft);
    }
    else
    {
        for(uint32_t laser_idx = 0; laser_idx<nLasers; ++laser_idx)
        {
            minLaserDistLeft = std::min(minLaserDistLeft, msg->ranges[laser_idx]);
        }

        if(minLaserDistLeft==std::numeric_limits <float> ::infinity())
        {
            minLaserDistLeft=0;
        }

        ROS_INFO("Minimum Laser Dist LEFT: %f",minLaserDistLeft);
    }

    
    // ros::Duration(0.5).sleep();
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
}

// // Random Exploration - Moves and Turns Randomly
// void explore(geometry_msgs::Twist &vel, ros::Publisher &vel_pub) {
//     if (minLaserDist < 0.8) {  // If an obstacle is close
//         ROS_WARN("Obstacle detected! Turning...");
//         vel.linear.x = 0.0;
//         vel.angular.z = (rand() % 2 == 0) ? M_PI / 6 : -M_PI / 6;  // Random turn direction
//         vel_pub.publish(vel);
//         ros::Duration(1.5).sleep();
//     } else {
//         vel.linear.x = 0.2;  // Move forward
//         vel.angular.z = ((rand() % 10) == 0) ? ((rand() % 2 == 0) ? M_PI / 6 : -M_PI / 6) : 0.0;  // Occasional random turn
//         vel_pub.publish(vel);
//         ros::Duration(1.5).sleep();
//     }
// }

// Function to check if the robot is revisiting an area
bool isRevisiting()
{
    for (const auto &pos : visited_positions)
    {
        double distance = std::hypot(posX - pos.first, posY - pos.second);
        if (distance < revisit_threshold)
        {
            ROS_WARN("Revisiting detected! Adjusting path...");
            return true;
        }
    }
    return false;
}

// To be run before explore, delete if not working
void avoidRevisiting(geometry_msgs::Twist &vel, ros::Publisher &vel_pub)
{
    if (isRevisiting())
    {
        vel.linear.x = 0.0;
        vel.angular.z = M_PI / 2; // Turn left to find a new path
        vel_pub.publish(vel);
        ros::Duration(0.5).sleep();
    }
}

void explore(geometry_msgs::Twist &vel, ros::Publisher &vel_pub)
{
    // Target distance from the wall
    float wall_distance = 0.5;  // Ideal distance to maintain
    float front_threshold = 0.5; // Obstacle avoidance threshold
    float deadend_threshold = 0.5; // Dead end detection threshold

    // Read precomputed minimum distances from the laser scan
    float front_dist = minLaserDistCenter;
    float left_dist = minLaserDistLeft;
    float right_dist = minLaserDistRight;

    // Detect dead-end (obstacles in front, left, and right)
    if (front_dist < deadend_threshold && left_dist < deadend_threshold && right_dist < deadend_threshold)
    {
        ROS_WARN("Dead-end detected! Turning around...");
        vel.linear.x = 0.0;
        vel.angular.z = M_PI; // Rotate 180 degrees
        vel_pub.publish(vel);
        ros::Duration(1.5).sleep(); // Allow time to turn
        return; // Exit function after turning

        while (minLaserDistCenter > wall_distance)
        {
            vel.linear.x = 0.2;
            vel.angular.z = 0.0; // Rotate 180 degrees
            vel_pub.publish(vel);
            ros::Duration(0.1).sleep(); // Allow time to turn
            return; // Exit function after turning 
        }

        ROS_WARN("Wall detected! Turning right...");
        vel.linear.x = 0.0;
        vel.angular.z = -M_PI/1.5; // Rotate 180 degrees
        vel_pub.publish(vel);
        ros::Duration(1.5).sleep(); // Allow time to turn
        return;
    }   

    // if (isRevisiting())
    // {
    //     vel.linear.x = 0.0;
    //     vel.angular.z = M_PI / 2; // Turn left to avoid the area
    //     vel_pub.publish(vel);
    //     ros::Duration(0.5).sleep();
    //     return;
    // }

    // // If an obstacle is directly in front, turn away
    // if (front_dist < front_threshold)
    // {
    //     ROS_WARN("Obstacle ahead! Turning...");
    //     vel.linear.x = 0.0;
    //     vel.angular.z = (left_dist > right_dist) ? M_PI / 2 : -M_PI / 2; // Turn toward the more open direction
    //     vel_pub.publish(vel);
    //     ros::Duration(0.5).sleep();
    // }

    // If an obstacle is directly in front, turn away
    if (front_dist < front_threshold)
    {
        ROS_WARN("Obstacle ahead! Turning...");
        vel.linear.x = 0.0;

        if (rand()% 2 == 0)
        {
            vel.angular.z = M_PI/2;
        }
        else {
            vel.angular.z = -M_PI/2;
        }
        
        vel_pub.publish(vel);
        ros::Duration(0.5).sleep();
    }


    else
    {
        // Wall-following behavior (try to maintain wall_distance)
        float error = 0.0;
        float kP = 2.0; // Proportional gain for wall following

        if (left_dist < wall_distance && left_dist > 0.1)  
        {
            // Too close to the left wall → turn right
            error = wall_distance - left_dist;
            vel.angular.z = -kP * error;
        }
        else if (right_dist < wall_distance && right_dist > 0.1)
        {
            // Too close to the right wall → turn left
            error = wall_distance - right_dist;
            vel.angular.z = kP * error;
        }
        else
        {
            // No wall detected → go straight
            vel.angular.z = 0.0;
        }

        vel.linear.x = 0.15;  // Move forward
        vel_pub.publish(vel);
        ros::Duration(0.1).sleep();  // Small sleep for smooth movement

        // // Store new position in visited locations
        // visited_positions.push_back({posX, posY});
    }
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

    while (ros::ok() && secondsElapsed <= 480)
    {
        ros::spinOnce();

        // //Check if any of the bumpers were pressed
        // bool any_bumper_pressed = false;
        // for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx){
        //     any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        // }

        bumperMovement(vel, vel_pub);
        // avoidRevisiting(vel, vel_pub);
        explore(vel, vel_pub);

        vel.angular.z = angular;
        vel.linear.x = linear;
        vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
        loop_rate.sleep();
    }

    return 0;
}
