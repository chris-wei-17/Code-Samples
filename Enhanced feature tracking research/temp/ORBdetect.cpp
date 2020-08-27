///////////////////////////////////////////////////////////////////////////
// 
// 			UPDATED FROM SANDBOX 8/3/2020 WORKING PROGRAM
//
///////////////////////////////////////////////////////////////////////////

/***********************************************************************************************
 **
 **  
 ***********************************************************************************************/

 // ZED includes
#include <sl/Camera.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/stitching/detail/matchers.hpp"

#include <vector>
#include <string>
#include <iostream>

// Sample includes
#include <SaveDepth.hpp>
#include <hello.hpp>

// ZED namespace
using namespace sl;

cv::Mat slMat2cvMat(Mat& input), 
		image_ocv_L_g, image_ocv_R_g, 
		ORB_Left, ORB_Right, 
		left_descriptors, right_descriptors,
		stereo_matches;
		
std::vector<cv::KeyPoint> left_keypoints, right_keypoints;

void printHelp();

int main(int argc, char **argv) {

    // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION::HD720;
    init_params.depth_mode = DEPTH_MODE::PERFORMANCE;
    init_params.coordinate_units = UNIT::METER;
    init_params.camera_fps = 0;
    init_params.sdk_gpu_id = -1;

    if (argc > 1) init_params.input.setFromSVOFile(argv[1]);
        
    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != ERROR_CODE::SUCCESS) {
        printf("%s\n", toString(err).c_str());
        zed.close();
        return 1; // Quit if an error occurred
    }

	zed.setCameraSettings(VIDEO_SETTINGS::BRIGHTNESS, 4);
	zed.setCameraSettings(VIDEO_SETTINGS::CONTRAST, 6);
	zed.setCameraSettings(VIDEO_SETTINGS::SHARPNESS, 8);


    // Display help in console
    printHelp();

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE::STANDARD;

    // Prepare new image size to retrieve half-resolution images
    Resolution image_size = zed.getCameraInformation().camera_resolution;
    int reduction_factor = 2; // 1/rf * original dimension = new dimension
    int new_width = image_size.width/reduction_factor; 
    int new_height = image_size.height/reduction_factor;

    Resolution new_image_size(new_width, new_height);

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    Mat image_zed_L(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat image_ocv_L = slMat2cvMat(image_zed_L);
    Mat image_zed_R(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat image_ocv_R = slMat2cvMat(image_zed_R);
    Mat depth_image_zed(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);
    Mat point_cloud;

    // Loop until 'q' is pressed
    char key = ' ';
    
    // Set Harris detector parameters
    int blockSize = 4;
    int apertureSize = 3;
    double k = .04;
    int thresh = 125;
    
    
	
    while (key != 'q') {

        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {
			
			
            // Retrieve the left image, right image, depth image in half-resolution
            zed.retrieveImage(image_zed_L, VIEW::LEFT, MEM::CPU, new_image_size);
            zed.retrieveImage(image_zed_R, VIEW::RIGHT, MEM::CPU, new_image_size);
            // zed.retrieveImage(depth_image_zed, VIEW::DEPTH, MEM::CPU, new_image_size); Don't currently need depth image

            // Retrieve the RGBA point cloud in half-resolution
            // To learn how to manipulate and display point clouds, see Depth Sensing sample
            // zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA, MEM::CPU, new_image_size); Dont' currently need point cloud

            // Get camera fps for display metrics
            int current_fps = zed.getCurrentFPS();
            std::string fps = std::to_string(current_fps);
            
            // Overlay fps onto image output
            cv::putText(image_ocv_L, fps, cv::Point(15,30) , 1, 2, cv::Scalar(0,0,255), 2);
            
            // Display image and depth using cv:Mat which share sl:Mat data
            cv::imshow("Left Image", image_ocv_L);
            // cv::imshow("Depth", depth_image_ocv); Comment out, don't need to see depth image

			// Convert images to grayscale
			cv::cvtColor( image_ocv_L, image_ocv_L_g, cv::COLOR_BGR2GRAY);
			cv::cvtColor( image_ocv_R, image_ocv_R_g, cv::COLOR_BGR2GRAY);
			
			// Define ORB Mask
    		cv::Mat mask_l(cv::Mat::ones(image_ocv_L_g.size(),CV_8U));
    		cv::Mat mask_r(cv::Mat::ones(image_ocv_R_g.size(),CV_8U));
    		

			// Left frame detector
			cv::Ptr<cv::ORB> left_detector = cv::ORB::create(500, 2, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
			left_detector->detectAndCompute(image_ocv_L_g, mask_l, left_keypoints, left_descriptors, false);
			cv::drawKeypoints(image_ocv_L, left_keypoints, ORB_Left, cv::Scalar(255,0,0));
			// cv::imshow("ORB_Left", ORB_Left); keep for debug, don't display to save resources
			
			// Right frame detector
			cv::Ptr<cv::ORB> right_detector = cv::ORB::create(500, 2, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
			right_detector->detectAndCompute(image_ocv_R_g, mask_r, right_keypoints, right_descriptors, false);
			cv::drawKeypoints(image_ocv_R, right_keypoints, ORB_Right, cv::Scalar(255,0,0));
			// cv::imshow("ORB_Right", ORB_Right); keep for debug, don't display to save resources

			// Match keypoints between frames
			std::vector<cv::DMatch> matches;
			cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NormTypes::NORM_HAMMING, false);
			matcher->match(left_descriptors, right_descriptors, matches);
			
			// Show matches
			cv::drawMatches(image_ocv_L_g, left_keypoints, image_ocv_R_g, right_keypoints, matches, 
			stereo_matches, cv::Scalar(0,255,0), cv::Scalar(0,0,255), std::vector<char>(), cv::DrawMatchesFlags::DEFAULT);
			
			cv::imshow("Stereo Matches", stereo_matches);

            // Handle key event
            key = cv::waitKey(10);
            processKeyEvent(zed, key);
        }
    }
    zed.close();
    return 0;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM::CPU));
}

/**
* This function displays help in console
**/
void printHelp() {
    std::cout << " Press 's' to save Side by side images" << std::endl;
    std::cout << " Press 'p' to save Point Cloud" << std::endl;
    std::cout << " Press 'd' to save Depth image" << std::endl;
    std::cout << " Press 'm' to switch Point Cloud format" << std::endl;
    std::cout << " Press 'n' to switch Depth format" << std::endl;
}




