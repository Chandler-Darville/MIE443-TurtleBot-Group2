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
        /***YOUR CODE HERE***/
        // Use: boxes.templates


        //////// Code for testing //////////
        cv::Mat grayImg;
        cv::cvtColor(img, grayImg, cv::COLOR_BGR2GRAY);

        double bestMatchScore = std::numeric_limits<double>::max(); // Lower is better
        int bestMatchIndex = -1;

        for (size_t i = 0; i < boxes.templates.size(); ++i) {
            if (boxes.templates[i].empty()) {
                std::cout << "Template " << i << " is empty, skipping..." << std::endl;
                continue;
            }

            cv::Mat result;
            cv::matchTemplate(grayImg, boxes.templates[i], result, cv::TM_SQDIFF_NORMED);
            double minVal, maxVal;
            cv::minMaxLoc(result, &minVal, &maxVal);

            if (minVal < bestMatchScore) {
                bestMatchScore = minVal;
                bestMatchIndex = i;
            }
        }

        if (bestMatchIndex != -1) {
            std::cout << "Best matching template ID: " << bestMatchIndex 
                      << " with score: " << bestMatchScore << std::endl;
            template_id = bestMatchIndex;
        } else {
            std::cout << "No good match found!" << std::endl;
        }



        cv::imshow("view", img);
        cv::waitKey(10);
    }  
    return template_id;
}
