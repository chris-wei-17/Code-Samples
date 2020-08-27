#include "optimizer.h"	

// This function evaluates the difference between the estimated translation as well as the
// the observed translation of triangulated points from t0 to t1.  If the estimate is not
// within the error bounds, the number of features, k, used for VIO will be adjusted


void optimizePerformance(cv::Mat& points3D_t0,
						 cv::Mat& points3D_t1,
						 cv::Mat& rotation,
						 cv::Mat& translation,
						 int& kCorners)
{
	
	std::cout << "====== translation: " << translation << " ======" << std::endl;
	std::cout << "====== points3D size: " << points3D_t0.rows << " ======" << std::endl;
	for (int i = 0; i < points3D_t0.rows; ++i)
	
	{

		double dX = points3D_t0.at<double>(i,1) - points3D_t1.at<double>(i,1);
		double dY = points3D_t0.at<double>(i,2) - points3D_t1.at<double>(i,2);
		double dZ = points3D_t0.at<double>(i,3) - points3D_t1.at<double>(i,3);
		
		double x_t0 = points3D_t0.at<double>(i,1);
		double x_t1 = points3D_t1.at<double>(i,1);
		double y_t0 = points3D_t0.at<double>(i,2);
		double y_t1 = points3D_t1.at<double>(i,2);
		double z_t0 = points3D_t0.at<double>(i,3);
		double z_t1 = points3D_t1.at<double>(i,3);
		
		int t0_rows = points3D_t0.rows;
		int t1_rows = points3D_t1.rows;
		
		double Tx = translation.at<double>(0);
		double Ty = translation.at<double>(1);
		double Tz = translation.at<double>(2);
		
		double error_x = dX - Tx;
		double error_y = dY - Ty;
		double error_z = dZ - Tz;
		
		
		
		
		
		
		
		/////////////////////////////////////////////////
		// 
		// Output options for debug and performance check
		//
		/////////////////////////////////////////////////
		
		
		//std::cout << "====== Tx: " << Tx << " ======" << std::endl;
		//std::cout << "====== Ty: " << Ty << " ======" << std::endl;
		//std::cout << "====== Tz: " << Tz << " ======" << std::endl;
		
		//std::cout << "====== row: " << i << " ======" << "====== error_x: " << error_x << " ======" << std::endl;
		//std::cout << "====== row: " << i << " ======" << "====== error_y: " << error_y << " ======" << std::endl;
		//std::cout << "====== row: " << i << " ======" << "====== error_z: " << error_z << " ======" << std::endl;
		//std::cout<<std::endl;
		//std::cout << "====== x_t0: " << x_t0 << " ======" << std::endl;
		//std::cout << "====== x_t1: " << x_t1 << " ======" << std::endl;
		//std::cout << "=== translation: " << translation << " ======" << std::endl;
		//std::cout << "====== rows_t0: " << t0_rows << " ======" << std::endl;
		//std::cout << "====== rows_t1: " << t1_rows << " ======" << std::endl;
		//std::cout << "====== rows_T: " << T_rows << " ======" << std::endl;
		//std::cout << "====== cols_T: " << T_cols << " ======" << std::endl << std::endl;
		
		//std::cout << "====== row: " << i << " ======" << "====== diff-x: " << dX << " ======" << std::endl;


	}
	
	//std::cout << "======== k: " << k << " ========" << std::endl;
	//std::cout << "======== 3D_t0 rows: " << points3D_t0.rows << " ========" << std::endl;
	

}





