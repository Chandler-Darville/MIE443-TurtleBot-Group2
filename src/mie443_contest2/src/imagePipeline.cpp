// #include <iostream>
// #include "opencv2/core.hpp"
// //#ifdef HAVE_OPENCV_XFEATURES2D
// #include "opencv2/calib3d.hpp"
// #include "opencv2/highgui.hpp"
// #include "opencv2/imgproc.hpp"
// #include "opencv2/features2d.hpp"
// #include "opencv2/xfeatures2d.hpp"
// using namespace cv;
// using namespace cv::xfeatures2d;
// using std::cout;
// using std::endl;


// #include <imagePipeline.h>

// #define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
// #define IMAGE_TOPIC "camera/image" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

// const char*keys=
// "{help h || Print help message.}"
// "{input1 | box.png | Path to input image 1.}"
// "{input2 | box_in_scene.png | Path to input image 2.}";
// int main(int argc, char*argv[])
// {
//     CommandLineParser parser(argc,argv,keys);
//     Mat img_object=imread(parser.get<String>("input1"),IMREAD_GRAYSCALE);
//     Mat img_scene=imread(parser.get<String>("input2"),IMREAD_GRAYSCALE);
//     if(img_object.empty()||img_scene.empty())
//     {
//         cout << "Could not open or find the image!\n" << endl;
//         parser.printMessage();
//         return -1;
//     }
//     // STEP 1: DETECT THE KEYPOINTS USING SURF DETECTOR, COMPUTE THE DESCRIPTORS
//     int minHessian=400;
//     Ptr<SURF> detector=SURF::create(minHessian);
//     std::vector<KeyPoint> keypoints_object,keypoints_scene;
//     Mat descriptors_object, descriptors_scene;
//     detector->detectAndCompute(img_object,noArray(),keypoints_object,descriptors_object);
//     detector->detectAndCompute(img_scene,noArray(),keypoints_scene,descriptors_scene);

//     Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
//     std::vector<std::vector<DMatch>> knn_matches;
//     matcher->knnMatch(descriptors_object,descriptors_scene,knn_matches,2);
//     // Filter matches using the Lowe's ratio test
//     const float ratio_thresh=0.75f;

//     std::vector<DMatch> good_matches;
//     for (size_t i=0; i<knn_matches.size();i++)
//     {
//         if (knn_matches[i][0].distance<ratio_thresh*knn_matches[i][1].distance)
//         {
//             good_matches.push_back(knn_matches[i][0]);
//         }
//     }

//     //Draw matches
//     Mat img_matches;
//     drawMatches(img_object,keypoints_object,img_scene,keypoints_scene,good_matches,img_matches,Scalar::all(-1),Scalar::all(-1),std::vector<char>(),DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

//     //Localize the object
//     std::vector<Point2f> obj;
//     std::vector<Point2f> scene;
//     for(size_t i=0;i<good_matches.size();i++)
//     {
//         //Get the keypoints from the good matches
//         obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
//         scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
//     }

//     Mat H=findHomography(obj,scene,RANSAC);
//     //Get the corners from image_1 (the object to be "detected")
//     std::vector<Point2f> obj_corners(4);
//     obj_corners[0]=Point2f(0,0);
//     obj_corners[1]=Point2f((float)img_object.cols,0);
//     obj_corners[2]=Point2f((float)img_object.cols,(float)img_object.rows);
//     obj_corners[3]=Point2f(0,(float)img_object.rows);
//     std::vector<Point2f> scene_corners(4);
//     perspectiveTransform(obj_corners,scene_corners,H);

//     //Draw lines between the corners (the mapped object in the scene - image_2)
//     line(img_matches,scene_corners[0]+Point2f((float)img_object.cols,0),scene_corners[0]+Point2f((float)img_object.cols,0),Scalar(0,255,0),4);
//     line(img_matches,scene_corners[1]+Point2f((float)img_object.cols,0),scene_corners[1]+Point2f((float)img_object.cols,0),Scalar(0,255,0),4);
//     line(img_matches,scene_corners[2]+Point2f((float)img_object.cols,0),scene_corners[2]+Point2f((float)img_object.cols,0),Scalar(0,255,0),4);
//     line(img_matches,scene_corners[3]+Point2f((float)img_object.cols,0),scene_corners[3]+Point2f((float)img_object.cols,0),Scalar(0,255,0),4);
//     //Show detected matches
//     imshow("Good Matches & Object detection", img_matches);
//     waitKey();
//     return 0;
// }

// ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
//     image_transport::ImageTransport it(n);
//     sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
//     isValid = false;
// }

// void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
//     try {
//         if(isValid) {
//             img.release();
//         }
//         img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
//         isValid = true;
//     } catch (cv_bridge::Exception& e) {
//         std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
//                   << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
//         isValid = false;
//     }    
// }

// int ImagePipeline::getTemplateID(Boxes& boxes) {
//     int template_id = -1;
//     if(!isValid) {
//         std::cout << "ERROR: INVALID IMAGE!" << std::endl;
//     } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
//         std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
//         std::cout << "img.empty():" << img.empty() << std::endl;
//         std::cout << "img.rows:" << img.rows << std::endl;
//         std::cout << "img.cols:" << img.cols << std::endl;
//     } else {
//         /***YOUR CODE HERE***/
//         // Use: boxes.templates
//         cv::imshow("view", img);
//         cv::waitKey(10);
//     }  
//     return template_id;
// }

////////////////////////////////////////////////////////////////////////////////////////////

#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/flann.hpp>

#include <imagePipeline.h>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        // Initialize SURF detector
        cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(300);  // 500 is the Hessian threshold
        cv::Ptr<cv::xfeatures2d::SURF> extractor = cv::xfeatures2d::SURF::create();

        // Detect keypoints and descriptors in the current image
        std::vector<cv::KeyPoint> keypoints_img;
        cv::Mat descriptors_img;
        detector->detectAndCompute(img, cv::noArray(), keypoints_img, descriptors_img);

        // Iterate through the templates in boxes.templates (you will need to modify the 'Boxes' class to store template images)
        for (int i = 0; i < boxes.templates.size(); i++) {
            cv::Mat template_img = boxes.templates[i];

            // Detect keypoints and descriptors in the template image
            std::vector<cv::KeyPoint> keypoints_template;
            cv::Mat descriptors_template;
            detector->detectAndCompute(template_img, cv::noArray(), keypoints_template, descriptors_template);

            // Use FLANN to match the descriptors
            cv::FlannBasedMatcher matcher;
            std::vector<cv::DMatch> matches;
            matcher.match(descriptors_img, descriptors_template, matches);

            // Evaluate the matches (you can apply a distance threshold to filter out bad matches)
            double max_dist = 0;
            double min_dist = 100;

            for (int i = 0; i < descriptors_img.rows; i++) {
                double dist = matches[i].distance;
                if (dist < min_dist) min_dist = dist;
                if (dist > max_dist) max_dist = dist;
            }

            std::cout << "-- Max dist : " << max_dist << " ; Min dist : " << min_dist << std::endl;

            // If a sufficient number of good matches are found, consider this template a match
            int good_matches = 0;
            for (int i = 0; i < matches.size(); i++) {
                if (matches[i].distance <= std::max(2 * min_dist, 0.02)) {
                    good_matches++;
                }
            }

            std::cout << "Good Matches: " << good_matches << std::endl;

            // If the good matches exceed a threshold, consider it as a valid match
            if (good_matches > 250) { // This threshold can be adjusted
                template_id = i;
                break;
            }
        }

        // Display the image with keypoints for visualization
        cv::Mat img_with_keypoints;
        cv::drawKeypoints(img, keypoints_img, img_with_keypoints);
        cv::imshow("SURF Keypoints", img_with_keypoints);
        //std::cout<<"Template id:" <<template_id <<std::endl;
        cv::waitKey(1);
    }  

    std::cout<<"Template id:" <<template_id <<std::endl;
    return template_id;
}