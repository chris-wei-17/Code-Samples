///////////////////////////////////////////////////////////////////////////
//
//
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
// #include "opencv2/cudastereo.hpp" Figure out why CUDA support not with OpenCV	

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
		disp, disp8; 
		
// std::vector<cv::KeyPoint> left_keypoints, right_keypoints;

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
    Mat image_zed_L_g(new_width, new_height, MAT_TYPE::U8_C1);
    cv::Mat image_ocv_L_g = slMat2cvMat(image_zed_L_g);
    Mat image_zed_R_g(new_width, new_height, MAT_TYPE::U8_C1);
    cv::Mat image_ocv_R_g = slMat2cvMat(image_zed_R_g);
    Mat depth_image_zed(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);
    Mat point_cloud;

    // Loop until 'q' is pressed
    char key = ' ';
        
	
    while (key != 'q') {

        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {
				
            // Retrieve the left image, right image, depth image in half-resolution
            zed.retrieveImage(image_zed_L_g, VIEW::LEFT_GRAY, MEM::CPU, new_image_size);
            zed.retrieveImage(image_zed_R_g, VIEW::RIGHT_GRAY, MEM::CPU, new_image_size);
            // zed.retrieveImage(depth_image_zed, VIEW::DEPTH, MEM::CPU, new_image_size); Don't currently need depth image

            // Retrieve the RGBA point cloud in half-resolution
            // To learn how to manipulate and display point clouds, see Depth Sensing sample
            // zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA, MEM::CPU, new_image_size); Dont' currently need point cloud

            // Get camera fps for display metrics
            int current_fps = zed.getCurrentFPS();
            std::string fps = std::to_string(current_fps);
            
            // Overlay fps onto image output
            cv::putText(image_ocv_L_g, fps, cv::Point(15,30) , 1, 2, cv::Scalar(0,0,255), 2);
            
            // Display image and depth using cv:Mat which share sl:Mat data
            cv::imshow("Left Image", image_ocv_L_g);
			
			// Create stereoBM
			cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(32,21);
			stereo->compute(image_ocv_L_g, image_ocv_R_g, disp);          
          
          	// Normalize disparity map
          	cv::normalize(disp, disp8, 0, 255, cv::NORM_MINMAX, CV_8U);
          
          	// Show normalized disparity map
          	cv::imshow("Disparity", disp8);
          
          
            
            
            
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



           
            
            
            
            
            
            
            
            
            
            
            
