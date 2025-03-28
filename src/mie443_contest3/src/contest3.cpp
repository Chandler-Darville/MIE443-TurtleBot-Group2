#include <header.h>
#include <ros/package.h>
#include <imageTransporter.hpp>
#include <chrono>
#include <kobuki_msgs/Led.h>	// for LED control
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

string path_to_pics = ros::package::getPath("mie443_contest3") + "/Pics/";

geometry_msgs::Twist follow_cmd;
int world_state = 0; 

//0-startup, 1-following, 2-sadt, 3-surprise, 4-rage, 5-fear

void image_display(string file_name)
{
	// Load and display an image
	cv::Mat img = cv::imread(path_to_pics + file_name);
	if (!img.empty()) 
	{
		// cv::namedWindow(file_name, cv::WINDOW_FULLSCREEN);
        // cv::setWindowProperty(file_name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

		cv::imshow(file_name, img);
		cv::waitKey(2000);  // Show for 2 seconds
		cv::destroyWindow(file_name);
	}
}

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
        world_state = 4; // state 4 = obstacle
        // ros::Duration(0.5).sleep();
        return;
    }

    // If only one bumper is pressed, wait for the grace period to allow the second press
    
    while ((ros::Time::now() - press_start_time).toSec() < GRACE_PERIOD) {
        // ros::spinOnce();
        if (left_pressed && right_pressed && !center_pressed) {
            world_state=3;	// state 3 = hug
            // ros::Duration(0.5).sleep();
            return;
        }
    }

    // If only one bumper remains pressed after the grace period, play "rage.wav"
    if ((left_pressed || right_pressed) && play_sound) {
        world_state=4; //stage 4 = obstacle
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
        ROS_WARN("Cliff detected! Switching world_state to 5.");
        world_state = 5; 
    } else {
        world_state = 1;  // Resume normal movement
    }
}

// void set_led(ros::Publisher &pub, int color) {
// 	// colors: off = 0, Green = 1, Orange = 2, Red = 3
//     kobuki_msgs::Led msg;
//     msg.value = color;
//     pub.publish(msg);
//     ROS_INFO("Set LED to color %d", color);
// }

// ros::Time prev_time = ros::Time::now();

int led1_state = 0;
int led2_state = 0;

bool change_led(ros::Time prev_time, ros::Time curr_time) {

	if ((curr_time - prev_time).toSec() >= 1.0) {
		return true;
	}
	else {
		return false;
	}
}

// void led_sequence(ros::Publisher &led1pub, ros::Publisher &led2pub) {

// 	if (world_state == 0) {		// initialized - slow alternating green blink

// 		if (change_led) {

// 			if (led1_state == 0 && led2_state == 0){

// 				set_led(led1pub, 1);
// 				led1_state = 1;
// 			}
// 		}

// 	}
// }



//-------------------------------------------------------------

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
	ros::Publisher led1_pub = nh.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1", 10);
    ros::Publisher led2_pub = nh.advertise<kobuki_msgs::Led>("/mobile_base/commands/led2", 10);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);
	ros::Subscriber cliff_sub = nh.subscribe("mobile_base/events/cliff", 10, &cliffCB);

    // contest count down timer
	ros::Rate loop_rate(10);
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
	std::chrono::time_point<std::chrono::system_clock> lost;
    uint64_t secondsElapsed = 0;

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	// int world_state = 0;

	double angular = M_PI/3;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;

	vector<float> linear_vel, angular_vel, zero;
	zero = {0.0,0.0,0.0};
	bool lostSound = false;

	// sc.playWave(path_to_sounds + "sound.wav");
	// ros::Duration(0.5).sleep();

	while(ros::ok() && secondsElapsed <= 480){	
		ros::spinOnce();
		linear_vel = {follow_cmd.linear.x, follow_cmd.linear.y,follow_cmd.linear.z};
		angular_vel = {follow_cmd.angular.x, follow_cmd.angular.y, follow_cmd.angular.z};
		if(world_state == 0){				//world state 0-startup	
			if(linear_vel == zero && angular_vel==zero){
				vel_pub.publish(vel);
				ROS_INFO("Hasn't found user to follow");
			}
			else{
				world_state = 1;
			}

		}else if(world_state == 1){		//world state 1-following
			vel_pub.publish(follow_cmd);
			ROS_INFO("Following user");
			if(linear_vel == zero && angular_vel==zero){
				world_state = 2;
				lost = std::chrono::system_clock::now();
			}
		}
		else if(world_state == 2){		//world state 2- lost, sad
			ROS_INFO("Lost track of user");
			vel.angular.z = M_PI/3;
			vel.linear.x = 0.0;
			if(((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-lost).count()/1000)%6) < 1){
				lostSound=true;
			}
			else{
				lostSound=false;
			}		
			if(lostSound){
				image_display("sadness.jpg");
				
				ROS_INFO("Playing sad sound");
				sc.playWave(path_to_sounds + "KennyCries.wav");
			}
			vel_pub.publish(vel);
			if(linear_vel != zero || angular_vel!=zero){
				world_state = 1;
				sc.stopWave(path_to_sounds + "KennyCries.wav");
			}
		}
		else if(world_state == 3){		//world state 3- surprise, pet
			sc.playWave(path_to_sounds + "surprise.wav");
			play_sound=false;
			
			image_display("surprise.jpg");

			// ros::Duration(2).sleep();
			world_state=1;
			ROS_WARN("Surprise!");
		}
		else if(world_state == 4){		//world state 4- rage, obstacle
			vel.angular.z = 0.0;
			vel.linear.x = 0.0;
			vel_pub.publish(vel);
			ros::Duration(1).sleep();

			// ROS_WARN("Bumper hit! Moving backward...");
			vel.angular.z = 0.0;
			vel.linear.x = -0.1;
			vel_pub.publish(vel);
			ros::Duration(1.5).sleep(); // robot move backward for 1.5 second
			
			sc.playWave(path_to_sounds + "rage.wav");
			play_sound=false;

			image_display("rage.jpg");

			
			world_state=1;
			ROS_WARN("Rage");
		}
		else if(world_state == 5){		//world state 5 - fear, lift
			sc.playWave(path_to_sounds + "Lift_Fear.wav");
			image_display("fear.jpg");
			ROS_WARN("Robot stopped due to cliff detection!");
			ros::Duration(1).sleep();
			ROS_WARN("Fear");
		}
		
		secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
		loop_rate.sleep();
	}

	return 0;
}
