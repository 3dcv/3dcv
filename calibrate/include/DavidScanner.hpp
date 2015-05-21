#ifndef DavidScanner_HPP
#define DavidScanner_HPP

#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/affine.hpp>
#include <iostream>
#include <fstream>
#include <unistd.h>

using namespace cv;
using namespace std;

class DavidScanner
{
public:

	/**
	 * @brief 	Ctor
	 */
   DavidScanner(Mat& reference_undist, Mat& camera_matrix, Mat& dist_coeffs, Mat& rvec, Mat& tvec);

    /**
     * @brief	Dtor
     */
    ~DavidScanner();

    /**
     * @brief	Puts the scanned vertices in a ply file
     * @param string name of file
     * @return succsess
     */
    int writePLY(string ply_string);
	
	 /**
     * @brief david scanning
     */
	int scan(Mat& frame);

private:

    /**
     * @brief calculates the intersection between a line coordinate and an axis
     */
	Vec3f identifyPlaneCoord(Point3f proj);
	
	/**
     * @brief calculates an object point in world coordinates by its intersection with the laser plane 
     */
	Vec3f findObjectPos(vector<Vec3f> lines_world, Vec3f cam_world, Vec3f proj_world);
	
	/**
     * @brief average filter for laser lines
     */
	void gaussianSlopeAvg(Vec4i& avg_line, vector<Vec4i>& lines, bool is_sloppy_site_left);

    Mat reference_undist_; 
    Mat camera_matrix_; 
    Mat dist_coeffs_;
    Mat rvec_; 
    Mat tvec_;
    vector<Vec3f> verts_;
    Point3f cam_world_pos_;
    Affine3f to3D_;
};


#endif // DavidScanner_HPP
