#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/affine.hpp>
#include <iostream>
#include <fstream>
#include <unistd.h>

using namespace cv;
using namespace std;

static int write_ply(vector<Vec3f>* verts, char* ply_path)
{
  string ply_string = ply_path;
  if(ply_string.substr(ply_string.size()-4, 4) != ".ply")
  {
    ply_string.append(".ply");
  }
  cout << "Creating file " << ply_string << "." << endl;
  ofstream ofst(ply_string.c_str(), ios::out | ios::trunc);
  if(!ofst.is_open())
  {
    cout << "Could not create file at desired path." << endl;
    return -1;
  }
  
  // Write header
  ofst << "ply\nformat ascii 1.0\nelement vertex " << verts->size() << "\n"; 
  ofst << "property float x\nproperty float y\nproperty float z\nend_header\n";
  for(int i=0; i<verts->size(); i++)
  {
    ofst << verts->at(i)[0] << " " << verts->at(i)[1] << " " << verts->at(i)[2] << "\n";
  }
  ofst.close();
}

static Point3f identify_plane_coord(Point3f cam, Point3f proj)
{
  Point3f x,y,z, min, sp = cam-proj;
  double s, min_dist = 10;
  if(proj.x < 0)
  {
    s = cam.x/sp.x;
    x = Point3f(0, cam.y - s * sp.y, cam.z - s * sp.z);
    min_dist = norm(cam-x);
    min = x;
  }
  if(proj.y < 0)
  {
    s = cam.y/sp.y;
    y = Point3f(cam.x - s * sp.x, 0, cam.z - s * sp.z);
    if(norm(cam-y) < min_dist) { min_dist = norm(cam-y); min = y; }
  }
  if(proj.z < 0)
  {
    s = cam.z/sp.z;
    z = Point3f(cam.x - s * sp.x, cam.y - s * sp.y, 0);
    if(norm(cam-z) < min_dist) { min_dist = norm(cam-z); min = z; }
  }
  return min;
}

static int find_lines(Mat* frame, Mat reference, Mat camera_matrix, Mat dist_coeffs, Mat rvec, Mat tvec, vector<Point2f> corners)
{

  Mat dst, gray, clonius;
  undistort(*frame, dst, camera_matrix, dist_coeffs);
  clonius = frame->clone();
  absdiff(dst, reference, dst);

  for(int i=0; i<dst.cols; i++)
  {
    //Vec3b baer = dst.at<Vec3b>(3,i);
    //std::cout << (int)dst.at<Vec3b>(3,i).val[0] << std::endl;
    //std::cout << baer.val[0] << std::endl;
    //std::cout << (int)baer.val[0] << std::endl;
    for(int j=0; j<dst.rows; j++)
    {
      uchar rvl = 255-(dst.at<Vec3b>(j,i).val[2]);

      if(dst.at<Vec3b>(j,i).val[2]-60 > dst.at<Vec3b>(j,i).val[0] && dst.at<Vec3b>(j,i).val[2]-60 > dst.at<Vec3b>(j,i).val[1])
      {

        frame->at<Vec3b>(j,i) = Vec3b(rvl, rvl, rvl);
      }
      else
      {
        frame->at<Vec3b>(j,i) = Vec3b(255, 255, 255);
      }
      
    }

    
  }


  //GaussianBlur(*frame, *frame, Size(5,5), 0, 0, BORDER_DEFAULT );
  cvtColor(*frame, gray, CV_BGR2GRAY);


  //Laplacian(gray, dst, CV_8UC1);
  Canny(gray, dst, 70, 230, 5, 1);
  //cvtColor(dst, *frame, CV_GRAY2BGR);

  vector<Vec2f> lines;
  vector<Vec4i> linesP;
  HoughLinesP( dst, linesP, 1, CV_PI/180, 70, 40, 200 );
  //HoughLines(dst, lines, 1, CV_PI/180, 100);
  for(int i=0; i<lines.size(); i++) 
  {
    //std::cout << lines[i] << std::endl;
    float rho = lines[i][0];
    float theta = lines[i][1];
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;
    Point pt1(cvRound(x0 + 1000*(-b)),
              cvRound(y0 + 1000*(a)));
    Point pt2(cvRound(x0 - 1000*(-b)),
              cvRound(y0 - 1000*(a)));
    line(*frame, pt1, pt2, Scalar(0,0,255), 3, 8 );
  }

  vector<Point> img_points;
  // now to select the right lines. assumption for now: there're two lines, as they represent the zero-y and zero-x planes.
  // only those lines will be drawn. trying to handle the occurrence of double lines
  // TODO look forward rather than backward to be able to compare double lines and use the longer one. or rather compare norms of the respective points instead of those of the entire lines
  for(int i=0; i<linesP.size(); i++) 
  {

    Vec4i l = linesP[i];
    if((((l[0]<frame->cols*0.3 && l[2] <frame->cols*0.3) || (l[0]>frame->cols*0.7 && l[2]>frame->cols*0.7)) && l[2]-l[0] > frame->cols/7)
      || (i>0 && norm(linesP[i]-linesP[i-1]) < 800))
    {
      linesP.erase(linesP.begin()+i);
      i--;
    }
    else
    {
      //std::cout << linesP[i] << std::endl;
      //if(i>0 norm(linesP[i]-linesP[i-1]) < 50) cout << "i-(i-1): " << linesP[i]-linesP[i-1] << endl << "norm(i-(i-1)): " << norm(linesP[i]-linesP[i-1]) << endl;
      line(*frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);

    }
  }
  // if there are exactly two lines left, keep working with the frame. otherwise do nothing. there are plenty more frames in the sea

  cout << "linesP size " << linesP.size() << endl;
  if(linesP.size() == 2)
  {
    *frame = clonius.clone();
    vector<Point3f> hom_lines, hom_corners, orig;
    vector<Point2f> img_points;
    img_points.push_back(Point(linesP[0][0], linesP[0][1]));
    img_points.push_back(Point(linesP[0][2], linesP[0][3]));
    img_points.push_back(Point(linesP[1][0], linesP[1][1]));
    img_points.push_back(Point(linesP[1][2], linesP[1][3]));
    cout << linesP[0] << endl << linesP[1] << endl << endl;
    cout << "norm " << norm(linesP[0]-linesP[1]) << endl;
    convertPointsToHomogeneous(img_points, hom_lines);
    convertPointsToHomogeneous(corners, hom_corners);
    
    Matx<float,3,3> cam2(camera_matrix); 
    //circle(*frame, img_points[0], 25, 5, 1, 8, 0);
    //circle(*frame, img_points[1], 25, 5, 1, 8, 0);
    //circle(*frame, img_points[2], 25, 5, 1, 8, 0);
    //circle(*frame, img_points[3], 25, 5, 1, 8, 0);
    Affine3f rodri((Vec3f)rvec, (Vec3f)tvec);

    Affine3f to3D = rodri.inv();

    // Calculate camera position in world coordinates TODO take affine and cam_world_pos outside find_lines
    Point3f cam_world_pos = Point3f(to3D*Point3f(0, 0, 0));
    //cout << "cam_world_pos" << cam_world_pos << endl;
    //vector<Point3f> dumb_vec;
    //dumb_vec.push_back(Point3f(to3D*cam_pos));
    
    for(int i=0; i<corners.size(); i++)
    {
      Matx<float,3,1> hom2(hom_corners[i]); 
      Matx<float,3,1> bert; 
      bert = cam2.inv()*hom2;
      Point3f bert2;
      bert2.x = bert(0);
      bert2.y = bert(1);
      bert2.z = bert(2);
      //cout << bert << endl;
      //cout << hom_lines[0] << endl;


      //cout << "profit: " << to3D*bert2<< endl;
      
      vector<Point3f> bubi;
      bubi.push_back(Point3f(to3D*bert2));
      //cout << "fett normiert. alter. " << norm(bubi[0]) << endl;
      vector<Point2f> image_points;
      projectPoints(bubi, rvec, tvec, camera_matrix, dist_coeffs, image_points, noArray(), 0);
      circle(*frame, image_points[0], 75, 150, 1, 8, 0);

      bubi[0] = identify_plane_coord(cam_world_pos, bubi[0]);
      projectPoints(bubi, rvec, tvec, camera_matrix, dist_coeffs, image_points, noArray(), 0);
      cout << "yeah! " << bubi[0] << endl;
      circle(*frame, image_points[0], 30, 250, 1, 8, 0);
  //    projectPoints(bubi, rvec, tvec, camera_matrix, dist_coeffs, image_points, noArray(), 0);
  //    circle(*frame, image_points[0], 75, 250, 2, 8, 0);
  //    cout << "profit 2: " << bubi[0] << endl;
    }
    // find red points whose x is between first line's second points's x and second line's first point's x
    // approach:
    // 1: project line image points to 3D
    // calculate plane equation from two lines
    // project image points in between lines to 3D and 
    // somehow use plane equation as rotation vector on those points and vector representing the point's distance from plane as translation
  } 

}

int main(int argc, char** argv)
{
  string stiggi = string(argv[1]);
  VideoCapture cap;
  if(stiggi.length() == 1) 
  {
    cap.open(atoi(argv[1]));
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 768);
  }
  else cap.open(argv[1]);
  Mat rvec, tvec, camera_matrix, dist_coeffs, reference;
  //usleep(3000000);
  for(int i=0; i < 50; i++) cap >> reference; 
  while(1) { imshow("reference", reference); if(waitKey() > 0) break; }
  vector<Point2f> corners;
//  VideoWriter writie("../saved_images/vid.mp4", CV_FOURCC('X','2','6','4'), 20, cvSize(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT)));
  VideoWriter writie("../saved_images/vid.avi", CV_FOURCC('M','J','P','G'), 25, cvSize(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT)));

  //testing ply_writer
  vector<Vec3f> baer;
  baer.push_back(Vec3f(0.3, 4, 19.34));
  baer.push_back(Vec3f(0.7, 8, 49.34));
  baer.push_back(Vec3f(0.3, 4, 9.34));
  baer.push_back(Vec3f(0.0, 0, 19.34));
  //write_ply(&baer, (char*)"../config/shit");


  // Read intrinsic and extrinsic parameters from config files
  FileStorage fsip(argv[2], FileStorage::READ);
  if(!fsip.isOpened())
  {
    std::cerr << "Parameter file could not be opened." << std::endl;
    return -1;
  }
  fsip["Camera_Matrix"] >> camera_matrix;
  fsip["Distortion_Coefficients"] >> dist_coeffs;

  FileStorage fsep(argv[3], FileStorage::READ);
  if(!fsep.isOpened())
  {
    std::cerr << "Parameter file could not be opened." << std::endl;
    return -1;
  }
  fsep["RVec"] >> rvec;
  fsep["TVec"] >> tvec;
  fsep["corners"] >> corners;

  while(1)
  {
    Mat frame;
    cap >> frame;
    if(stiggi.length() == 1) writie << frame;
    find_lines(&frame, reference, camera_matrix, dist_coeffs, rvec, tvec, corners);
    

    imshow("frame", frame);
    int key = waitKey(30);

    if(key == 1048603)
    {
       cap.release(); if(stiggi.length() == 1) writie.release(); return -1;
    }
  }


}
