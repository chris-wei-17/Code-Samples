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
#include "opencv2/calib3d.hpp"
// #include "opencv2/cudastereo.hpp" Figure out why CUDA support not with OpenCV	

#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <ctime>
#include <sstream>
#include <fstream>

// Sample includes
#include <SaveDepth.hpp>
#include <hello.hpp>

// Zhengao includes
//#include <Zhengao.hpp>
#include "feature.h"
#include "utils.h"
#include "evaluate_odometry.h"
#include "visualOdometry.h"
#include "Frame.h"

#include "camera_object.h"
#include "rgbd_standalone.h"

// Optimization includes
#include "optimizer.h"

// ZED namespace
using namespace sl;
using namespace std;


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
	zed.setCameraSettings(VIDEO_SETTINGS::CONTRAST, 3);
	zed.setCameraSettings(VIDEO_SETTINGS::SHARPNESS, 2);


    // Display help in console
    printHelp();

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE::STANDARD;

    // Prepare new image size to retrieve half-resolution images
    Resolution image_size = zed.getCameraInformation().camera_resolution;
    int reduction_factor = 1; // 1/rf * original dimension = new dimension
    int new_width = image_size.width/reduction_factor; 
    int new_height = image_size.height/reduction_factor;

    Resolution new_image_size(new_width, new_height);
    
    std::cout << "new width: " << new_width << std::endl;
    std::cout << "new height: " << new_height << std::endl;
    
    // Get camera calibration information from ZED
	CalibrationParameters cal_params = zed.getCameraInformation().calibration_parameters;
	float fxl = cal_params.left_cam.fx;
	float fyl = cal_params.left_cam.fy;
	float fxr = cal_params.right_cam.fx;
	float fyr = cal_params.right_cam.fy;
	float cxl = cal_params.left_cam.cx;
	float cyl = cal_params.left_cam.cy;
	float cxr = cal_params.right_cam.cx;
	float cyr = cal_params.right_cam.cy;
	float b = -.120;
	float bf = b*fxr;

	// Debug - confirm camera cal params
	
	std::cout << "fxl: " << fxl << std::endl;
	std::cout << "fyl: " << fyl << std::endl;
	std::cout << "fxr: " << fxr << std::endl;
	std::cout << "fyr: " << fyr << std::endl;
	std::cout << "cxl: " << cxl << std::endl;
	std::cout << "cyl: " << cyl << std::endl;
	std::cout << "cxr: " << cxr << std::endl;
	std::cout << "cyr: " << cyr << std::endl;
	std::cout << "b: " << b << std::endl;
	std::cout << "bf: " << bf << std::endl;

	// Set function config params
    bool display_ground_truth = false;
    bool use_intel_rgbd = false;

	// Create dummy ground truth pose matrix to test display
	std::vector<Matrix> pose_matrix_gt;
	//pose_matrix_gt = (0,0,0,0,0,0,0,0,0,0,0,0);

	// Create projection matrices
    cv::Mat projMatrl = (cv::Mat_<float>(3, 4) << fxl, 0., cxl, 0., 0., fyl, cyl, 0., 0,  0., 1., 0.);
    cv::Mat projMatrr = (cv::Mat_<float>(3, 4) << fxr, 0., cxr, bf, 0., fyr, cyr, 0., 0,  0., 1., 0.);
    cout << "P_left: " << endl << projMatrl << endl;
    cout << "P_right: " << endl << projMatrr << endl;
    

    // -----------------------------------------
    // Initialize variables
    // -----------------------------------------
    cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat translation = cv::Mat::zeros(3, 1, CV_64F);

    cv::Mat pose = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat Rpose = cv::Mat::eye(3, 3, CV_64F);
    
    cv::Mat frame_pose = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat frame_pose32 = cv::Mat::eye(4, 4, CV_32F);

    std::cout << "frame_pose " << frame_pose << std::endl;
    cv::Mat trajectory = cv::Mat::zeros(600, 1200, CV_8UC3);
    FeatureSet currentVOFeatures;
    cv::Mat points4D, points3D;
    int init_frame_id = 0;

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
  
   	// For images at t = 0
    Mat image_zed_L_t0(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat image_ocv_L_t0 = slMat2cvMat(image_zed_L_t0);
    Mat image_zed_R_t0(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat image_ocv_R_t0 = slMat2cvMat(image_zed_R_t0);
    Mat depth_image_zed(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);
    
    double scale;

    // Loop until 'q' is pressed
    char key = ' ';
        
        
    if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) 
    {
        
    	// ------------------------
    	// Load first images
    	// ------------------------    
    
        zed.retrieveImage(image_zed_L_t0, VIEW::LEFT, MEM::CPU, new_image_size);
        zed.retrieveImage(image_zed_R_t0, VIEW::RIGHT, MEM::CPU, new_image_size);
	}
	
	cv::Mat image_ocv_L_t0_g, image_ocv_R_t0_g;
	
	cvtColor(image_ocv_L_t0, image_ocv_L_t0_g, cv::COLOR_BGR2GRAY);
	cvtColor(image_ocv_R_t0, image_ocv_R_t0_g, cv::COLOR_BGR2GRAY);
	
	clock_t t_a, t_b;

    // -----------------------------------------
    // Run visual odometry
    // -----------------------------------------

	std::vector<FeaturePoint> oldFeaturePointsLeft;
    std::vector<FeaturePoint> currentFeaturePointsLeft;
	
	int frame_id = init_frame_id + 1;    
	
	// Initialize k features
	int kCorners = 200;

    odometry:while (key != 'q') {
		
		
		std::cout << std::endl << "frame id " << frame_id << std::endl;
		
        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {
        	
        // ------------
        // Load images
        // ------------        	
        	
        	// Establish t+ images			
            // Retrieve the left image, right image, depth image

    // For subsequent images t = t+
    Mat image_zed_L_t1(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat image_ocv_L_t1 = slMat2cvMat(image_zed_L_t1);
    Mat image_zed_R_t1(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat image_ocv_R_t1 = slMat2cvMat(image_zed_R_t1);
   
    zed.retrieveImage(image_zed_L_t1, VIEW::LEFT, MEM::CPU, new_image_size);
    zed.retrieveImage(image_zed_R_t1, VIEW::RIGHT, MEM::CPU, new_image_size);
    
    // Get camera fps for display metrics
    int current_fps = zed.getCurrentFPS();
    std::string camera_fps = std::to_string(current_fps);
        

	cv::Mat image_ocv_L_t1_g, image_ocv_R_t1_g;

	cvtColor(image_ocv_L_t1, image_ocv_L_t1_g, cv::COLOR_BGR2GRAY);
	cvtColor(image_ocv_R_t1, image_ocv_R_t1_g, cv::COLOR_BGR2GRAY);
	
        	t_a = clock();
        	std::vector<cv::Point2f> oldPointsLeft_t0 = currentVOFeatures.points;


        	std::vector<cv::Point2f> pointsLeft_t0, pointsRight_t0, pointsLeft_t1, pointsRight_t1;  
        	matchingFeatures( image_ocv_L_t0_g, image_ocv_R_t0_g,
                          	image_ocv_L_t1_g, image_ocv_R_t1_g, 
                          	currentVOFeatures,
                          	pointsLeft_t0, 
                          	pointsRight_t0, 
                          	pointsLeft_t1, 
                          	pointsRight_t1,
                          	kCorners);  


                          	
            image_ocv_L_t0_g = image_ocv_L_t1_g;
        	image_ocv_R_t0_g = image_ocv_R_t1_g;

        	std::vector<cv::Point2f>& currentPointsLeft_t0 = pointsLeft_t0;
        	std::vector<cv::Point2f>& currentPointsLeft_t1 = pointsLeft_t1;
        
        	std::vector<cv::Point2f> newPoints;
        	std::vector<bool> valid; // valid new points are true
		
		// This condition allows for points to expire after age > 10 while robot is idle
		// enabling this return to the top allows for a lower number of VOFeatures.  A
		// higher number of VOFeatures keeps enough new features to satisfy triangulate
		// functions below.  With a low number of VOFeatures, sometimes you end up with an
		// empty vector being fed to triangulatePoints which causes a cv Exception.
		
		if (currentVOFeatures.size() == 0)
		{
			goto odometry;
		}

        // ---------------------
        // Triangulate 3D Points
        // ---------------------
        cv::Mat points3D_t0, points4D_t0;
        cv::triangulatePoints( projMatrl,  projMatrr,  pointsLeft_t0,  pointsRight_t0,  points4D_t0);
        cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);

        cv::Mat points3D_t1, points4D_t1;
        cv::triangulatePoints( projMatrl,  projMatrr,  pointsLeft_t1,  pointsRight_t1,  points4D_t1);
        cv::convertPointsFromHomogeneous(points4D_t1.t(), points3D_t1);
        

        // ---------------------------
        //
        // ---------------------------

		optimizePerformance(points3D_t0, points3D_t1, rotation, translation, kCorners);
        

        // ---------------------
        // Tracking transfomation
        // ---------------------
		clock_t tic_gpu = clock();
        trackingFrame2Frame(projMatrl, projMatrr, pointsLeft_t0, pointsLeft_t1, points3D_t0, rotation, translation, false);
		clock_t toc_gpu = clock();
		std::cerr << "tracking frame 2 frame: " << float(toc_gpu - tic_gpu)/CLOCKS_PER_SEC*1000 << "ms" << std::endl;
		
        displayTracking(image_ocv_L_t1_g, pointsLeft_t0, pointsLeft_t1, camera_fps, scale);

		/*points4D = points4D_t0;
        frame_pose.convertTo(frame_pose32, CV_32F);
        points4D = frame_pose32 * points4D;
        cv::convertPointsFromHomogeneous(points4D.t(), points3D);*/

        // ------------------------------------------------
        // Intergrating and display
        // ------------------------------------------------

        cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);
		
		// Uncomment following lines to check rotation/translation calculation
		//std::cout << "rotation RAW" << rotation << std::endl;
		//std::cout << "translation RAW" << translation << std::endl;

        cv::Mat rigid_body_transformation;

        if(abs(rotation_euler[1])<0.1 && abs(rotation_euler[0])<0.1 && abs(rotation_euler[2])<0.1)
        {
            integrateOdometryStereo(frame_id, rigid_body_transformation, frame_pose, rotation, translation, scale);
            std::cout << "Return Scale: " << scale << std::endl;

        } 
        
        else 
        {

            std::cout << "Too large rotation"  << std::endl;
        }
        
		//displayTracking(image_ocv_L_t1_g, pointsLeft_t0, pointsLeft_t1, camera_fps, scale);

        t_b = clock();
        float frame_time = 1000*(double)(t_b-t_a)/CLOCKS_PER_SEC;
        float fps = 1000/frame_time;
        cout << "[Info] frame times (ms): " << frame_time << endl;
        cout << "[Info] FPS: " << fps << endl;
		std::cout << "ZED FPS: " << camera_fps << std::endl;
        std::cout << "rigid_body_transformation" << rigid_body_transformation << std::endl;
        std::cout << "rotation: " << rotation_euler << std::endl;
        std::cout << "translation: " << translation.t() << std::endl;
        std::cout << "frame_pose" << frame_pose << std::endl;
		//std::cout << "points3D" << points3D_t0 << std::endl;

        cv::Mat xyz = frame_pose.col(3).clone();
        
        //std::cout << "X: " << double(xyz.at<double>(0)) << std::endl;
    	//std::cout << "Y: " << double(xyz.at<double>(2)) << std::endl;
    	
        display(frame_id, trajectory, xyz, pose_matrix_gt, fps, display_ground_truth);
		
		frame_id = frame_id + 1;
            

            
        // Display image and depth using cv:Mat which share sl:Mat data
        //cv::imshow("Left Image", image_ocv_L_g_t1);
			
		
	    //sleep(2);	
		
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
    std::cout << " Press 'q' to quit" << std::endl;
}



           
            
            
            
            
            
            
            
            
            
            
            
