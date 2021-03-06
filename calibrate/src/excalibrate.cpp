#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

static int excalibrate(char ** argv, Mat* image)
{
  
  vector <vector <Point2f> > corners(atoi(argv[4]));
  Size board_size = cvSize(7, 5);
  bool found = 0;
  float aspectratio = (float)image->cols/image->rows;
  Size image_size = image->size();
  Mat view_gray, rvec, tvec, camera_matrix, dist_coeffs, image_copy;
  cvtColor(*image, view_gray, CV_BGR2GRAY);
  vector <Point3f> object_points, prov_object_points;
  image_copy = image->clone();

  // Read intrinsic parameters from config file
  FileStorage fsip(argv[3], FileStorage::READ);
  if(!fsip.isOpened())
  {
    std::cerr << "Parameter file could not be opened." << std::endl;
    return -1;
  }
  fsip["Camera_Matrix"] >> camera_matrix;
  fsip["Distortion_Coefficients"] >> dist_coeffs;

  // Read 3D object points from config file
  FileStorage fsop(argv[2], FileStorage::READ);
  if(!fsop.isOpened())
  {
    std::cerr << "Object points file could not be opened." << std::endl;
    return -1;
  }
  fsop["Object_Points"] >> prov_object_points;

  // Make some distinctions depending on whether 2 or 3 patterns are being used.
  object_points.insert(object_points.begin(), prov_object_points.begin(), prov_object_points.begin()+atoi(argv[4])*(board_size.height*board_size.width));

  // Find chessboard corners for all patterns and draw them on original image
  for(int i=0; i<corners.size(); i++)
  {
    found = findChessboardCorners(*image, board_size, corners[i], CV_CALIB_CB_ADAPTIVE_THRESH);
    std::cout << (found ? "Found" : "Failed") << std::endl;
    if(!found)
    {
      std::cout << "At least one chessboard wasn't found. Adjust the camera so as to get a good view on all patterns." << std::endl;
      return -1;
    }

    cornerSubPix(view_gray, corners[i], Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
    drawChessboardCorners(*image, board_size, corners[i], found);

    while(1)
    {
      imshow("bam", *image);
      if(waitKey(30) > 0) break;
    }

    // If corners are drawn in non-desired sequence, invert points to match corresponding 3d object points
    if(corners[i][0].x > corners[i][corners[i].size()-1].x)
    {
      vector<Point2f> swap_vec;
      swap_vec.resize(corners[i].size());
      for(int j=0; j<corners[i].size(); j++)
      {
        swap_vec[j] = corners[i][corners[i].size()-1-j];
      }
      corners[i] = swap_vec;
    }
  }

  // Handle variations in order of pattern 'recognition'. Yes, this does not handle the case in which the left pattern was detected last.
  vector<Point2f> chained_corners;
  chained_corners.insert(chained_corners.end(), corners[0].begin(), corners[0].end());
  if(corners[0][0].x < corners[1][0].x)
  {
    chained_corners.insert(chained_corners.end(), corners[1].begin(), corners[1].end());
  }
  else chained_corners.insert(chained_corners.begin(), corners[1].begin(), corners[1].end());
  if(atoi(argv[4]) == 3) 
  {
    if(corners[2][0].y > corners[1][0].y) 
    {
      chained_corners.insert(chained_corners.end(), corners[2].begin(), corners[2].end());
    }
    else
    {
      chained_corners.insert(chained_corners.begin()+corners[0].size(), corners[2].begin(), corners[2].end());
    }
  }
  
  // Calculate extrinsic parameters rvec and tvec from corresponding 3D and image points
  solvePnP(object_points, chained_corners, camera_matrix, dist_coeffs, rvec, tvec, 0, CV_ITERATIVE); 
  std::cout << "rvec: " << rvec << std::endl;
  std::cout << "tvec: " << tvec << std::endl;

  // For testing purposes, project 3D points back to image points with acquired rvec and tvec
  vector<Point2f> image_points;
  projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs, image_points, noArray(), 0);

  // Calculate average error or projected corner points
  double err_x, err_y;
  for(int i=0; i<chained_corners.size(); i++)
  {
    err_x += pow(chained_corners[i].x - image_points[i].x, 2);
    err_y += pow(chained_corners[i].y - image_points[i].y, 2);
  }
  err_x /= chained_corners.size();
  err_y /= chained_corners.size();
  std::cout << "error: \n" << "x: " << sqrt(err_x) << "  y: " << sqrt(err_y) << std::endl;

  // Draw projected points
  vector<Point2f> projected_corners;
  projected_corners.resize(corners[0].size());
  for(int i=0; i<corners.size(); i++)
  {
    projected_corners.clear();
    projected_corners.insert(projected_corners.begin(), image_points.begin()+35*i, image_points.begin()+35*i+35);
    drawChessboardCorners(image_copy, board_size, projected_corners, found);
  }

  // Show
  while(1)
  {
    imshow("excalibrate", image_copy);
    if(waitKey(30) > 0) break;
  }

  FileStorage fs("../config/extrinsic_params", FileStorage::WRITE);
  fs << "RVec" << rvec;
  fs << "TVec" << tvec;
  fs.release();
}

static int record_image(char ** argv)
{
  VideoCapture cap(atoi(argv[1])); 
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 768);

  std::cout << "To calibrate, press c. To break, press ESC." << std::endl;
  
  while(1)
  {
    Mat frame;
    cap >> frame;

    imshow("frame", frame);
    int key = waitKey(30);
    
    // This requires a switch/case
    switch (key)
    {
      case 1048603: cap.release(); return -1;
      case 1048675: excalibrate(argv, &frame); break;
    }
  }
}

int main(int argc, char** argv)
{
  if (argc != 5)
  {
    std::cerr << "Error using this file. Use as: " << std::endl 
      << "./excalibrate [camera_id] [object_points_file] [intrinsic_params_file] [number of chessboard patterns]" << std::endl;
    return -1;
  }
  if (atoi(argv[4]) < 2 || atoi(argv[4]) > 3)
  {
    std::cerr << "You gotta be out of your damn mind, son!" << std::endl;
    return -1;
  }

  record_image(argv);

}

