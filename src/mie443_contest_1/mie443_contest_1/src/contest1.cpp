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

float angular;
float linear;
float minDistRight, minDistLeft, minDistCenter;
double posX, posY, yaw;
int numberOfConsecutiveTurns=0;

#define N_BUMPER (3)
#define RAD2DEG(rad)((rad)*180./M_PI)
#define DEG2RAD(deg)((deg)*M_PI/180.)
bool leftState, centerState, rightState;


uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED,kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	//Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state;
    leftState = bumper[kobuki_msgs::BumperEvent::LEFT];
    centerState = bumper[kobuki_msgs::BumperEvent::CENTER];
    rightState = bumper[kobuki_msgs::BumperEvent::RIGHT];

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
        //ROS_INFO("Minimum Laser Dist CENTER: %f",minLaserDistCenter);
        minDistCenter= minLaserDistCenter;
    }
    else // if the desired angle is not in the cone (probably not useful)
    {
        for(uint32_t laser_idx = 0; laser_idx<nLasers; ++laser_idx)
        {
            minLaserDistCenter = std::min(minLaserDistCenter, msg->ranges[laser_idx]);
        }
        //ROS_INFO("Minimum Laser Dist: %i",minLaserDistCenter);
        
        if(minLaserDistCenter==std::numeric_limits <float> ::infinity())
        {
            minLaserDistCenter=0;
        }
        minDistCenter=minLaserDistCenter;
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
        //ROS_INFO("Minimum Laser Dist RIGHT: %f",minLaserDistRight);
        minDistRight= minLaserDistRight;
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
        //ROS_INFO("Minimum Laser Dist RIGHT: %f",minLaserDistRight);
        minDistRight=minLaserDistRight;
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

        //ROS_INFO("Minimum Laser Dist LEFT: %f",minLaserDistLeft);
        minDistLeft=minLaserDistLeft;
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

        //ROS_INFO("Minimum Laser Dist LEFT: %f",minLaserDistLeft);
        minDistLeft=minLaserDistLeft;
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    //ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw));
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
    std::chrono::time_point<std::chrono::system_clock> beginning_time;

    std::chrono::time_point<std::chrono::system_clock> cornerTimes[3];
    cornerTimes[0]=std::chrono::system_clock::now();
    cornerTimes[1]=std::chrono::system_clock::now();
    cornerTimes[2] =std::chrono::system_clock::now();
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    float angular = 0.0;
    float linear = 0.0;
    double lastYaw;
    int corner =0, direction = 1;
    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();
        beginning_time = std::chrono::system_clock::now();

        
        // //Check if any of the bumpers were pressed
        // bool any_bumper_pressed = false;
        // for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx){
        //     any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        // }

        // //Bumper Avoidance
        // if (any_bumper_pressed) {
        //     //if bumper is hit, move backward
        //     ROS_WARN("Bumper hit! Moving backward...");
        //     angular = 0.0;
        //     linear = -1.0; //move backward
        //     vel.angular.z = angular;
        //     vel.linear.x = linear;
        //     vel_pub.publish(vel);
        //     ros::Duration(1.5).sleep(); //robot move backward for 1 second

        //     //turn find open space
        //     ROS_WARN("Turning to find open space...");
        //     angular = M_PI/3;
        //     linear = 0.0;
        //     vel.angular.z = angular;
        //     vel.linear.x = linear;
        //     vel_pub.publish(vel);
        //     ros::Duration(1.5).sleep(); 

        //     //reset bumper state
        //     for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx){
        //         any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
        //     }
        // }

        // else if (minLaserDistCenter > 0.5){ //if space is open
        //     angular = 0.0;
        //     linear = 0.0;
        //     vel.angular.z = angular;
        //     vel.linear.x = linear;
        //     vel_pub.publish(vel);
        // }

        // else { //if too close to the wall or object, turn
        //     angular = M_PI/2;
        //     linear = 0.0;
        //     vel.angular.z = angular;
        //     vel.linear.x = linear;
        //     vel_pub.publish(vel);
        // }

        //Control  logic after bumpers were pressed
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

        // vel.angular.z = angular;
        // vel.linear.x = linear;
        // vel_pub.publish(vel);

        // The last thing to do is to update the timer.
        
        

        if (centerState)
        {   
            linear = -0.25;
            angular =0;
            vel.linear.x= linear;
            vel.angular.z = angular;
            while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-beginning_time).count()<1)
            {
                vel_pub.publish(vel);
            }
            
        }
        else if(leftState)
        {
            linear = -0.25;
            angular =M_PI/2;
            vel.linear.x= linear;
            vel.angular.z = angular;
            while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-beginning_time).count()<1)
            {
                vel_pub.publish(vel);
            }
            
        }
        else if(rightState)
        {
            linear = -0.25;
            angular =-M_PI/2;
            vel.linear.x= linear;
            vel.angular.z = angular;
            while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-beginning_time).count()<1)
            {
                vel_pub.publish(vel);
            }
            
        }
        else{
            

            if (minDistCenter<0.5)
            {
                
                linear=0;
                                                        
                if (corner >=2 && (cornerTimes[0]-cornerTimes[1]).count()<15)     // odd direction turn left, even turn right when stuck
                {                                          // if stuck in 2 corners within 15 secs, switch turning direction
                    ++direction;
                }
                if(numberOfConsecutiveTurns>5 && direction%2!=0)      // keep turning to left if robot has tried to adjust itself for 6 times while standing still
                {
                    angular=M_PI/2;
                    vel.linear.x= linear;
                    vel.angular.z = angular;
                    vel_pub.publish(vel);
                    for(int i = 0; i<1; ++i)
                    {
                        cornerTimes[i]=cornerTimes[i+1];
                    }
                    cornerTimes[1]=std::chrono::system_clock::now();
                    ++corner;
                }
                else if (numberOfConsecutiveTurns>5 && direction%2==0)
                {
                    angular=-M_PI/2;
                    vel.linear.x= linear;
                    vel.angular.z = angular;
                    vel_pub.publish(vel);
                    for(int i = 0; i<1; ++i)
                    {
                        cornerTimes[i]=cornerTimes[i+1];
                    }
                    cornerTimes[1]=std::chrono::system_clock::now();
                    ++corner;
                }
                else
                {
                    //turn to whichever side has more space
                    if (minDistRight<minDistLeft)      
                    {
                        angular = M_PI/2;
                        vel.angular.z = angular;
                        vel.linear.x = linear;
                        vel_pub.publish(vel);
                        ++numberOfConsecutiveTurns;
                    }
                    else
                    {
                        angular= -M_PI/2;
                        vel.angular.z = angular;
                        vel.linear.x = linear;
                        vel_pub.publish(vel);
                        ++numberOfConsecutiveTurns;
                    }
                }
            }
            else if (minDistLeft<1||minDistRight<1)
            {
                if (minDistRight<minDistLeft)
                {
                    numberOfConsecutiveTurns=0;
                    linear = 0.25;
                    angular = M_PI/6;
                    vel.angular.z = angular;
                    vel.linear.x = linear;
                    vel_pub.publish(vel);
                }
                
                else
                {
                    numberOfConsecutiveTurns=0;
                    linear=0.25;
                    angular = -M_PI/6;
                    vel.angular.z = angular;
                    vel.linear.x = linear;
                    vel_pub.publish(vel);
                }
            }
            else
            {
                numberOfConsecutiveTurns=0;
                linear=0.25;
                angular = 0;
                vel.angular.z = angular;
                vel.linear.x = linear;
                vel_pub.publish(vel);
            }    
        
        }
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}