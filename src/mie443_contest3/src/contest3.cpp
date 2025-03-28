#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>

#include <opencv2/opencv.hpp>

#include <kobuki_msgs/CliffEvent.h>

#include <kobuki_msgs/BumperEvent.h>
#include <ros/time.h>

using namespace std;

#define Nbumpers 3
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
ros::Time bumper_press_time;  // Stores the time when both bumpers are first pressed
bool waiting_for_confirmation = false;  // Flag to track if we're waiting for confirmation

ros::Time left_pressed_time;
ros::Time right_pressed_time;
ros::Time center_pressed_time;
ros::Time press_start_time;
bool left_pressed = false;
bool right_pressed = false;
bool center_pressed = false;
bool play_sound=true;
const double GRACE_PERIOD = 0.3;   // 0.3 seconds grace period for simultaneous press


geometry_msgs::Twist follow_cmd;
int world_state = 0; // 0: start up, 1: Surprise - Pet; 2: Rage - obstacle; 3: Fear - Lift; 4: Sadness - lost user; 5 - following



void followerCB(const geometry_msgs::Twist msg){
    follow_cmd = msg;
}

void bumperCB(const kobuki_msgs::BumperEvent::ConstPtr& msg){
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	string path_to_pics = ros::package::getPath("mie443_contest3") + "/Pics/";
	bumper[msg->bumper] = msg->state;

	bool leftBumper = (bumper[kobuki_msgs::BumperEvent::LEFT] == kobuki_msgs::BumperEvent::PRESSED);
	bool centerBumper = (bumper[kobuki_msgs::BumperEvent::CENTER] == kobuki_msgs::BumperEvent::PRESSED);
    bool rightBumper = (bumper[kobuki_msgs::BumperEvent::RIGHT] == kobuki_msgs::BumperEvent::PRESSED);

	// std::cout <<"Bumper Left: " << leftBumper << std::endl;
	// std::cout <<"Bumper Right: "<< rightBumper << std::endl;
	// std::cout << std::endl;
	// std::cout << std::endl;
	// std::cout << std::endl;

    if (msg->state == kobuki_msgs::BumperEvent::PRESSED) 
	{
        if (msg->bumper == kobuki_msgs::BumperEvent::LEFT) 
		{
            left_pressed = true;
            left_pressed_time = ros::Time::now();
			press_start_time = ros::Time::now();
        } 
		else if (msg->bumper == kobuki_msgs::BumperEvent::RIGHT) 
		{
            right_pressed = true;
            right_pressed_time = ros::Time::now();
			press_start_time = ros::Time::now();
        } 
		else if (msg->bumper == kobuki_msgs::BumperEvent::CENTER) 
		{
            center_pressed = true;
            center_pressed_time = ros::Time::now();
			press_start_time = ros::Time::now();
        }
    } 
	else if (msg->state == kobuki_msgs::BumperEvent::RELEASED) 
	{
        if (msg->bumper == kobuki_msgs::BumperEvent::LEFT) 
		{
            left_pressed = false;
        } 
		else if (msg->bumper == kobuki_msgs::BumperEvent::RIGHT) 
		{
            right_pressed = false;
        } 
		else if (msg->bumper == kobuki_msgs::BumperEvent::CENTER) 
		{
            center_pressed = false;
        }
    }

    // If center bumper is pressed, play "rage.wav" immediately
    if (center_pressed && play_sound) {
        world_state = 2; // state 2 = obstacle
        // ros::Duration(0.5).sleep();
        return;
    }

    // If only one bumper is pressed, wait for the grace period to allow the second press
    
    while ((ros::Time::now() - press_start_time).toSec() < GRACE_PERIOD) {
        // ros::spinOnce();
        if (left_pressed && right_pressed && !center_pressed) {
            world_state=1;	// state 1 = hug
            // ros::Duration(0.5).sleep();
            return;
        }
    }

    // If only one bumper remains pressed after the grace period, play "rage.wav"
    if ((left_pressed || right_pressed) && play_sound) {
        world_state=2; //stage 2 = obstacle
        // ros::Duration(0.5).sleep();
    }

	if (!left_pressed && !right_pressed && !center_pressed)
	{
		play_sound=true;
	}
}


// Callback function for cliff detection (Fear state)
void cliffCB(const kobuki_msgs::CliffEvent::ConstPtr& msg) {
    if (msg->state == kobuki_msgs::CliffEvent::CLIFF) {  
        ROS_WARN("Cliff detected! Switching world_state to 3.");
        world_state = 3; 
    } else {
        world_state = 0;  // Resume normal movement
    }
}

//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = ros::package::getPath("mie443_contest3") + "/sounds/";
	string path_to_pics = ros::package::getPath("mie443_contest3") + "/Pics/";
	teleController eStop;

	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
	ros::Subscriber cliff_sub = nh.subscribe("mobile_base/events/cliff", 10, &cliffCB);

    // contest count down timer
	ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	// int world_state = 0;

	double angular = 0.2;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	// sc.playWave(path_to_sounds + "sound.wav");
	// ros::Duration(0.5).sleep();

	while(ros::ok() && secondsElapsed <= 480){		
		ros::spinOnce();

		if(world_state == 0){
			//fill with your code
			//vel_pub.publish(vel);
			vel_pub.publish(follow_cmd);
			ROS_WARN("State 0: Following");

		}else if(world_state == 1){
			sc.playWave(path_to_sounds + "surprise.wav");
			play_sound=false;
			// Load and display an image
			cv::Mat img = cv::imread(path_to_pics+"surprise.jpg");
			if (!img.empty()) 
			{
				cv::imshow("surprise!", img);
				cv::waitKey(2000);  // Show for 2 seconds
				cv::destroyWindow("surprise!");
			} 
			// ros::Duration(2).sleep();
			world_state=0;
			ROS_WARN("State 1: Excitement!");
		}
		else if (world_state==2)
		{
			vel.angular.z = 0.0;
			vel.linear.x = 0.0;
			vel_pub.publish(vel);
			ros::Duration(1).sleep();
			
			sc.playWave(path_to_sounds + "rage.wav");
			play_sound=false;

			// Load and display an image
			cv::Mat img = cv::imread(path_to_pics+"rage.jpg");
			if (!img.empty()) {
				cv::imshow("rage!", img);
				cv::waitKey(2000);  // Show for 2 seconds
				cv::destroyWindow("rage!");
			} 

			// ROS_WARN("Bumper hit! Moving backward...");
			vel.angular.z = 0.0;
			vel.linear.x = -0.1;
			vel_pub.publish(vel);
			ros::Duration(1.5).sleep(); // robot move backward for 1.5 second
			world_state=0;
			ROS_WARN("State 2: Rage");
		}
		else if(world_state == 3){
			sc.playWave(path_to_sounds + "Lift_Fear.wav");
			ROS_WARN("Robot stopped due to cliff detection!");
			ros::Duration(1).sleep();
			ROS_WARN("State 3: Fear");
		}
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}
