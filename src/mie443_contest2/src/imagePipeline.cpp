#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/flann.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <imagePipeline.h>
#include <thread>

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/rgb/image_raw"

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) img.release();
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert image!" << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    if (!isValid || img.empty()) {
        std::cout << "ERROR: Invalid image!" << std::endl;
        return template_id;
    }

    int minHessian = 500;  
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(minHessian);

    std::vector<cv::KeyPoint> keypoints_img;
    cv::Mat descriptors_img;
    detector->detectAndCompute(img, cv::noArray(), keypoints_img, descriptors_img);

    std::cout << "Captured Image Keypoints: " << keypoints_img.size() << std::endl;

    if (descriptors_img.empty()) {
        std::cout << "ERROR: No descriptors found in captured image!" << std::endl;
        return template_id;
    }

    cv::FlannBasedMatcher matcher(new cv::flann::KDTreeIndexParams(), new cv::flann::SearchParams());
    int maxmatch = 0;

    for (int i = 0; i < boxes.templates.size(); i++) {
        cv::Mat template_img = boxes.templates[i];

        std::vector<cv::KeyPoint> keypoints_template;
        cv::Mat descriptors_template;
        detector->detectAndCompute(template_img, cv::noArray(), keypoints_template, descriptors_template);

        if (descriptors_template.empty()) {
            std::cout << "Skipping template " << i << " due to empty descriptors!" << std::endl;
            continue;
        }

        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher.knnMatch(descriptors_img, descriptors_template, knn_matches, 2);

        std::vector<cv::DMatch> good_matches;
        for (const auto& match : knn_matches) {
            if (match[0].distance < 0.75f * match[1].distance) {
                good_matches.push_back(match[0]);
            }
        }

        std::cout << "Template " << i << " - Good Matches: " << good_matches.size() << std::endl;

        if (good_matches.size() > maxmatch) {
            maxmatch = good_matches.size();
            template_id = i;
    
            //std::cout << "Template " << i << " is a possible match!" << std::endl;

            std::vector<cv::Point2f> obj, scene;
            for (const auto& match : good_matches) {
                obj.push_back(keypoints_template[match.trainIdx].pt);
                scene.push_back(keypoints_img[match.queryIdx].pt);
            }

            if (obj.size() >= 4) {
                cv::Mat H = cv::findHomography(obj, scene, cv::RANSAC);
                if (!H.empty()) {
                    std::vector<cv::Point2f> obj_corners = { {0,0}, {template_img.cols,0}, 
                                                            {template_img.cols,template_img.rows}, {0,template_img.rows} };
                    std::vector<cv::Point2f> scene_corners(4);
                    cv::perspectiveTransform(obj_corners, scene_corners, H);

                    cv::line(img, scene_corners[0], scene_corners[1], cv::Scalar(0, 255, 0), 4);
                    cv::line(img, scene_corners[1], scene_corners[2], cv::Scalar(0, 255, 0), 4);
                    cv::line(img, scene_corners[2], scene_corners[3], cv::Scalar(0, 255, 0), 4);
                    cv::line(img, scene_corners[3], scene_corners[0], cv::Scalar(0, 255, 0), 4);

                    cv::imshow("Detected Object", img);
                }
            }

            cv::waitKey(1);
            break;
        }
    }

    std::cout << "Final Template ID: " << template_id << std::endl;
    return template_id;
}
