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

ImagePipeline::ImagePipeline(ros::NodeHandle& n, Boxes& boxes) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;

    //Preprocess all tags and store in boxes so this only has to happen once
    for (int i=0; i<boxes.templates.size();i++)
    {
        tagPreprocess(boxes.templates[i]);
    }
}

//------------------------Savo's----------------------------------------------
void ImagePipeline::tagPreprocess(cv::Mat& tag)
{
    cv::resize(tag,tag,cv::Size(500,400)); //resize roughly to match aspect ratio on boxes
    cv::GaussianBlur(tag,tag,cv::Size(3,3),0,0); //add blur to aid feature matching
    cv::imshow("Tag as used",tagImage); //show image used in search
}
//----------------------------------------------------------------------------


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

int ImagePipeline::getTemplateID(Boxes& boxes, bool showInternals) {
    int template_id = -1; //Default to error
    if(!isValid) {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;

        return template_id;
    } else if(img.empty() || img.rows <= 0 || img.cols <= 0) {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } else {
        // Initialize SURF detector
        cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(500);  // 500 is the Hessian threshold
        cv::Ptr<cv::xfeatures2d::SURF> extractor = cv::xfeatures2d::SURF::create();

        // Detect keypoints and descriptors in the current image
        std::vector<cv::KeyPoint> keypoints_img;
        cv::Mat descriptors_img;
        detector->detectAndCompute(img, cv::noArray(), keypoints_img, descriptors_img);

        //----------------------------------------------------------------------------------
        //find the id and confidence levels of the two highest rated candidates
        float maxConfidence=0.0, secondConfidence=0.0;
        uint8_t maxID=0;
        Mat bestTag; //stores the best matched reference tag for display purpose
        //----------------------------------------------------------------------------------

        // ============================================================
        // Loop through all possible tags
        for (int template_id = 0; template_id < boxes.templates.size(); template_id++) {

            // Load reference to tag from boxes (just to help simplfy the code that follows)
            Mat &tagImage = boxes.templates[template_id];

            // See what portion of features from the reference are matched in the scene
            std::vector<DMatch> goodMatches;
            searchInScene(tagImage, descriptorsScene, keyPointsObject, goodMatches, detector);
            float confidence = (float)goodMatches.size() / (float)keyPointsObject.size();

            // ============================================================
            // Investigate futher if initial confidence is good
            float area = 0; // Area object takes up in scene (pixels)
            if (confidence > reqConfMinimum) {
                // Localize the object
                std::vector<Point2f> refPoints;
                std::vector<Point2f> scenePoints;
                for(int i = 0; i < goodMatches.size(); i++) {
                    // Get the keypoints from the good matches
                    refPoints.push_back(keyPointsObject[goodMatches[i].queryIdx].pt);
                    scenePoints.push_back(keyPointsScene[goodMatches[i].trainIdx].pt);
                }

                // Determine transformation matrix of reference to scene pixels
                Mat H = findHomography(refPoints, scenePoints, RANSAC);

                // Check if there is a possible transform
                if (H.empty()) {
                    // Failed to find a transform from reference to scene
                    ROS_WARN("Unable to transform perspective using reference %d.", template_id + 1);
                    confidence = 0; // It's a bad match
                }
                else {
                    // Tranform from refence image to scene is possible

                    // Get the corners from the reference image
                    std::vector<Point2f> cornersInReference(4);
                    cornersInReference[0] = Point2f(0, 0);
                    cornersInReference[1] = Point2f((float)tagImage.cols, 0 );
                    cornersInReference[2] = Point2f((float)tagImage.cols, (float)tagImage.rows );
                    cornersInReference[3] = Point2f(0, (float)tagImage.rows );
                    std::vector<Point2f> corInScene(4);

                    // Apply transform to reference corners to transform into scene bounds
                    cv::perspectiveTransform( cornersInReference, corInScene, H);
                    
                    // Check if the corners are not "tangled" before calculating area (forming a "bow" shape (invalid))
                    if (checkTangledBox(corInScene) == false) {
                        // Calculate area of the region
                        // (1/2) * [(x1y2 + x2y3 + x3y4 + x4y1) - (x2y1 + x3y2 + x4y3 + x1y4)]
                        area = corInScene[0].x * corInScene[1].y + corInScene[1].x * corInScene[2].y +
                            corInScene[2].x * corInScene[3].y + corInScene[3].x * corInScene[0].y; 
                        area = area - (corInScene[1].x * corInScene[0].y + corInScene[2].x * corInScene[1].y +
                            corInScene[3].x * corInScene[2].y + corInScene[0].x * corInScene[3].y);
                        area = area / 2;
                    }
                }
            }

            // ============================================================
            // Look to record this match if it's worthy

            if (area <= reqMinArea) confidence = 0; // Nullify confidence if it isn't present
            else confidence = confidence * area * areaConfidenceFactor; // Add area to confidence
            // Area is added to prefer objects that are closer to rover but might have some of their features out of
            // frame, resulting in a lower "confidence" than a fully visible, but futher object in the background

            if (showInternals) {
                printf("Template %2d - Confidence %6.2f%% - KP %4d / %4d - Area %6.0f\n", 
                    template_id + 1, confidence * 100.0, (int)goodMatches.size(), (int) keyPointsObject.size(), area);
            }

            // See how this compares to previous cases
            if (confidence > maxConfidence) {
                // New best
                secondConfidence = maxConfidence;
                maxConfidence = confidence;

                // Record values needed outside the loop
                maxID = template_id;
                bestTag = tagImage.clone();
            }
            else if (confidence > secondConfidence) {
                // Record second place confidence for ratio comparison later
                secondConfidence = confidence;
            }
        }
        
        // ============================================================
        // Process the results of the scan
        determinedId = 0; // Default (inconclusive scan, but at least no error!)

        if (maxConfidence < reqConfMinimum) {
            // If there is no satisfactory option 
            ROS_INFO("Failed to find a match.");
        }
        else if ((maxConfidence > reqConfMinimum) && ((maxConfidence / secondConfidence) > reqConfRatio)) {
            determinedId = maxID + 1; // Add one to match file names and to allow 0 to be used as a fail code

            ROS_INFO("Image contains %d, %.2f%% (%.2f) confidence", determinedId,
                maxConfidence * 100.0, (maxConfidence / secondConfidence));

            if (showInternals) {
                // Redo winning search
                std::vector<DMatch> goodMatches;
                searchInScene(bestTag, descriptorsScene, keyPointsObject, goodMatches, detector);

                // Show resulting matches
                Mat imgOfMatches = ImagePipeline::drawSceneMatches(img, bestTag, goodMatches, keyPointsObject, keyPointsScene);
                imshow("Selected match", imgOfMatches);

                cv::waitKey(250); // Wait until any key is pressed or 250ms pass
            }
        }

        return determinedId;
    }
}

void ImagePipeline::searchInScene(cv::Mat &tagImage, cv::Mat &descriptorsScene, std::vector<cv::KeyPoint> &keyPointsObject,
    std::vector<cv::DMatch> &goodMatches, cv::Ptr<cv::xfeatures2d::SURF> &detector) {

    using namespace cv;
    Mat descriptors_object;

    // Detect markers for the reference to find in the scene
    detector->detectAndCompute( tagImage, noArray(), keyPointsObject, descriptors_object );

    // Matching descriptor vectors with a FLANN based matcher
    // Since SURF is a floating-point descriptor NORM_L2 is used
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch( descriptors_object, descriptorsScene, knn_matches, 2 );

    // Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.75;
    for (size_t i = 0; i < knn_matches.size(); i++) {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
            goodMatches.push_back(knn_matches[i][0]);
        }
    }
}

void ImagePipeline::loadImage(std::string fileLocation, bool printMessage) {
    // Replace image in pipeline with something else
    img = cv::imread(fileLocation, 1);
    isValid = true;

    if (printMessage) ROS_INFO("Image loaded from into video feed.\n\t\"%s\"", fileLocation.c_str());
    }

    cv::Mat ImagePipeline::drawSceneMatches(cv::Mat &scene, cv::Mat &tagImage, std::vector<cv::DMatch> &matches, 
    std::vector<cv::KeyPoint> &keyPointsRef, std::vector<cv::KeyPoint> &keyPointsScene){

    using namespace cv;

    // Draw matches
    Mat imageOfMatches; // Image with matches illustrated
    drawMatches(tagImage, keyPointsRef, scene, keyPointsScene, matches, imageOfMatches, Scalar::all(-1),
                    Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    // Localize the object
    std::vector<Point2f> refPoints;
    std::vector<Point2f> scenePoints;
    for( size_t i = 0; i < matches.size(); i++ ) {
        // Get the keypoints from the good matches
        refPoints.push_back( keyPointsRef[ matches[i].queryIdx ].pt );
        scenePoints.push_back( keyPointsScene[ matches[i].trainIdx ].pt );
    }

    Mat H = findHomography(refPoints, scenePoints, RANSAC );

    if (!H.empty()) {
        // Can preform transform from reference to scene

        float refImageCol = (float)tagImage.cols;

        // Get the corners from the reference image (the object to be "detected")
        std::vector<Point2f> cornersInReference(4);
        cornersInReference[0] = Point2f(0, 0);
        cornersInReference[1] = Point2f( refImageCol, 0 );
        cornersInReference[2] = Point2f( refImageCol, (float)tagImage.rows );
        cornersInReference[3] = Point2f( 0, (float)tagImage.rows );
        std::vector<Point2f> corInScene(4);

        cv::perspectiveTransform( cornersInReference, corInScene, H);

        // Draw lines between the corners of the spotted object (reference) in the scene
        cv::line( imageOfMatches, corInScene[0] + Point2f(refImageCol, 0),
                corInScene[1] + Point2f(refImageCol, 0), Scalar(0, 255, 0), 4 );
        cv::line( imageOfMatches, corInScene[1] + Point2f(refImageCol, 0),
                corInScene[2] + Point2f(refImageCol, 0), Scalar( 0, 255, 0), 4 );
        cv::line( imageOfMatches, corInScene[2] + Point2f(refImageCol, 0),
                corInScene[3] + Point2f(refImageCol, 0), Scalar( 0, 255, 0), 4 );
        cv::line( imageOfMatches, corInScene[3] + Point2f(refImageCol, 0),
                corInScene[0] + Point2f(refImageCol, 0), Scalar( 0, 255, 0), 4 );
    }
    else {
        ROS_WARN("Can't draw matches. Corners cannot be transformed.");
    }

    return imageOfMatches;
}

bool ImagePipeline::checkTangledBox(std::vector<cv::Point2f> corners) {

    // Check that line between 1 and 4 does not cross 23
    bool tempA = checkAbove(corners[0],corners[1],corners[2]);
    bool tempB = checkAbove(corners[3],corners[1],corners[2]);
    bool result = tempA == tempB; // Store if both points fall on the same side (not tangled)

    // Check that line 12 does not cross 34
    tempA = checkAbove(corners[0],corners[3],corners[2]);
    tempB = checkAbove(corners[1],corners[3],corners[2]);
    result = result && (tempA == tempB); // Update result to ensure no sets of lines intersect

    return !result; // Return true if the system IS tangled
    }

    bool ImagePipeline::checkAbove(cv::Point2f test, cv::Point2f a, cv::Point2f b) {
    // Define line between a and b
    float gradient = (a.y - b.y) / (a.x - b.x);

    // Linearly extrapolate line between a and b to test point
    float dx = test.x - a.x;
    float estimateY = a.y + dx * gradient;

    // Return if the point lies above the line or not
    return test.y > estimateY;
}
//         // Iterate through the templates in boxes.templates (you will need to modify the 'Boxes' class to store template images)
//         for (int i = 0; i < boxes.templates.size(); i++) {
//             cv::Mat template_img = boxes.templates[i];

//             // Detect keypoints and descriptors in the template image
//             std::vector<cv::KeyPoint> keypoints_template;
//             cv::Mat descriptors_template;
//             detector->detectAndCompute(template_img, cv::noArray(), keypoints_template, descriptors_template);

//             // Use FLANN to match the descriptors
//             cv::FlannBasedMatcher matcher;
//             std::vector<cv::DMatch> matches;
//             matcher.match(descriptors_img, descriptors_template, matches);

//             // Evaluate the matches (you can apply a distance threshold to filter out bad matches)
//             double max_dist = 0;
//             double min_dist = 100;

//             for (int i = 0; i < descriptors_img.rows; i++) {
//                 double dist = matches[i].distance;
//                 if (dist < min_dist) min_dist = dist;
//                 if (dist > max_dist) max_dist = dist;
//             }

//             std::cout << "-- Max dist : " << max_dist << " ; Min dist : " << min_dist << std::endl;

//             // If a sufficient number of good matches are found, consider this template a match
//             int good_matches = 0;
//             for (int i = 0; i < matches.size(); i++) {
//                 if (matches[i].distance <= std::max(2 * min_dist, 0.02)) {
//                     good_matches++;
//                 }
//             }
            
//             std::cout<<"Good Matches: " << good_matches << std::endl;

//             // If the good matches exceed a threshold, consider it as a valid match
//             if (good_matches > 10) { // This threshold can be adjusted
//                 template_id = i;
//                 break;
//             }
//         }

//         // Display the image with keypoints for visualization
//         cv::Mat img_with_keypoints;
//         cv::drawKeypoints(img, keypoints_img, img_with_keypoints);
//         cv::imshow("SURF Keypoints", img_with_keypoints);
//         //std::cout<<"Template id:" <<template_id <<std::endl;
//         cv::waitKey(1);
//     }  

//     std::cout<<"Template id:" <<template_id <<std::endl;
//     return template_id;
// }