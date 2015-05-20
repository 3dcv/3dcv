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
    cout << "Could not create file at specified location." << endl;
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

static Vec3f identify_plane_coord(Point3f cam, Point3f proj)
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
  return Vec3f(min.x, min.y, min.z);
}

static Vec3f find_object_pos(vector<Vec3f> lines_world, Vec3f cam_world, Vec3f proj_world)
{
  Vec3f dir_vec1 = lines_world[0] - lines_world[1];
  Vec3f dir_vec2 = lines_world[2] - lines_world[3];
  Vec3f base_vec(lines_world[0]);
  Vec3f plane_normal = dir_vec1.cross(dir_vec2);
  
  Vec3f line_dir = cam_world - proj_world;
  float intersect_scalar = (base_vec - cam_world).dot(plane_normal)/line_dir.dot(plane_normal);
  Vec3f object_pos = cam_world + intersect_scalar * line_dir;
  return object_pos;
}

static void sloppy_slope(Vec4i& sloppy_line, vector<Vec4i>& lines, bool is_sloppy_site_left)
{
  double slope_avg;
  double slope_dev;
  vector<double> slopes;
  for(auto line : lines)
  {
    slopes.push_back(double(line[3] - line[1])/double(line[2] - line[0]));
    slope_avg += slopes.back();
  }
  slope_avg /= slopes.size();

  for(double slope : slopes)
  {
    slope_dev += pow(slope - slope_avg, 2); 
  }
  slope_dev = sqrt(slope_dev/slopes.size());
  for(int i = 0; i < slopes.size(); i++)
  {
    if(slopes[i] > slope_avg + slope_dev) 
    {
      slopes.erase(slopes.begin() + i);
      lines.erase(lines.begin() + i);
      i--;  
    }
  }
  slope_avg = 0;
  for(double slope : slopes)
  {
    slope_avg += slope;
  }
  slope_avg /= slopes.size();
  
  
  int line_xtreme;
  if(is_sloppy_site_left)
     line_xtreme = lines[0][2];
  else
     line_xtreme = lines[0][0];
  for(int i = 0; i < lines.size(); i++)
  {
    sloppy_line += lines[i];
    if(is_sloppy_site_left)
      line_xtreme = max(line_xtreme, lines[i][2]);
    else
      line_xtreme = min(line_xtreme, lines[i][0]);
  }
  cout << "lines size:" << lines.size() << endl;
  sloppy_line[0] /= lines.size();
  sloppy_line[1] /= lines.size();
  sloppy_line[2] /= lines.size();
  sloppy_line[3] /= lines.size();
  if(is_sloppy_site_left)
  {
    sloppy_line[2] = line_xtreme;
    sloppy_line[3] = sloppy_line[1] + slope_avg * (line_xtreme - sloppy_line[0]);
  }
  else
  {
     sloppy_line[0] = line_xtreme;
     sloppy_line[1] = sloppy_line[3] - slope_avg * abs(line_xtreme - sloppy_line[2]);
  }
} 
  

static int find_lines(Mat* frame, Mat reference, Mat camera_matrix, Mat dist_coeffs, Mat rvec, Mat tvec, vector<Vec3f>* verts)
{

  Mat dst, gray;
  undistort(*frame, dst, camera_matrix, dist_coeffs);

  // Subtract reference image from current frame. Better than your run-of-the-mill absdiff.
  // Then identify red pixels and make 'em grey. Everything else be white.
  for(int i=0; i<dst.cols; i++)
  {
    for(int j=0; j<dst.rows; j++)
    {
      dst.at<Vec3b>(j,i).val[0] = max(dst.at<Vec3b>(j,i).val[0] - reference.at<Vec3b>(j,i).val[0], 0);  
      dst.at<Vec3b>(j,i).val[1] = max(dst.at<Vec3b>(j,i).val[1] - reference.at<Vec3b>(j,i).val[1], 0);  
      dst.at<Vec3b>(j,i).val[2] = max(dst.at<Vec3b>(j,i).val[2] - reference.at<Vec3b>(j,i).val[2], 0);  
    
      frame->at<Vec3b>(j,i) = Vec3b(255, 255, 255);

      if(dst.at<Vec3b>(j,i).val[2]-35 > dst.at<Vec3b>(j,i).val[0] && dst.at<Vec3b>(j,i).val[2]-35 > dst.at<Vec3b>(j,i).val[1])
      {
        //frame->at<Vec3b>(j,i) = Vec3b(180, 180, 180);
        frame->at<Vec3b>(j,i) = 255-dst.at<Vec3b>(j,i).val[2];
      }
    }
    //imshow("after_absdiff", dst);
  }


  cvtColor(*frame, gray, CV_BGR2GRAY);
  Canny(gray, dst, 70, 230, 5, 1);
  imshow("Ihr Schweine!", dst);
  vector<Vec4i> linesP;
  HoughLinesP( dst, linesP, 1, CV_PI/180, 75, 300, 150 );

  vector<Vec4i> cool_lines1, cool_lines2;
  for(int i=0; i<linesP.size(); i++) 
  {
    Vec4i l = linesP[i];
    line(*frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 1, CV_AA);
    bool good = 1;
    cout << "pre found line " << i << ": " << linesP[i] << endl;
    if((l[0]<frame->cols*0.3 && l[2] <frame->cols*0.3) || (l[0]>frame->cols*0.7 && l[2]>frame->cols*0.7) || l[2]-l[0] < frame->cols/7)
    {
      continue;
    }

    if(l[0] < frame->cols*0.29) { cool_lines1.push_back(l);}
    else if(l[2] > frame->cols*0.71) { cool_lines2.push_back(l); }

  }
  if(cool_lines1.size() > 0 && cool_lines2.size() > 0)
  {
    Vec4i sloppy_line1;
    Vec4i sloppy_line2;
    sloppy_slope(sloppy_line1, cool_lines1,1);
    sloppy_slope(sloppy_line2, cool_lines2,0);
    line(*frame, Point(sloppy_line1[0],sloppy_line1[1]), Point(sloppy_line1[2], sloppy_line1[3]), Scalar(255,0,0), 1, CV_AA);
    line(*frame, Point(sloppy_line2[0],sloppy_line2[1]), Point(sloppy_line2[2], sloppy_line2[3]), Scalar(0,0,255), 1, CV_AA);

    // Project to world coordinates
    vector<Point3f> hom_lines, orig, hom_objs;
    vector<Point2f> img_points;
    img_points.push_back(Point(sloppy_line1[0], sloppy_line1[1]));
    img_points.push_back(Point(sloppy_line1[2], sloppy_line1[3]));
    img_points.push_back(Point(sloppy_line2[0], sloppy_line2[1]));
    img_points.push_back(Point(sloppy_line2[2], sloppy_line2[3]));
    cout << sloppy_line1 << endl << sloppy_line2 << endl << endl;
    convertPointsToHomogeneous(img_points, hom_lines);
    
    Matx<float,3,3> cam2(camera_matrix); 
    Affine3f rodri((Vec3f)rvec, (Vec3f)tvec);

    Affine3f to3D = rodri.inv();

    // Calculate camera position in world coordinates TODO take affine and cam_world_pos outside find_lines function
    Point3f cam_world_pos = Point3f(to3D*Point3f(0, 0, 0));
    Vec3f cam_world_vec = Vec3f(cam_world_pos.x, cam_world_pos.y, cam_world_pos.z);
    
    vector<Vec3f> bubi;
    for(int i=0; i<img_points.size(); i++)
    {
      Matx<float,3,1> hom2(hom_lines[i]); 
      Matx<float,3,1> bert; 
      bert = cam2.inv()*hom2;
      Point3f bert2;
      bert2.x = bert(0);
      bert2.y = bert(1);
      bert2.z = bert(2);

      bubi.push_back(Point3f(to3D*bert2));

      bubi[i] = identify_plane_coord(cam_world_pos, Point3f(bubi[i][0], bubi[i][1], bubi[i][2]));
      cout << "yeah! " << bubi[i] << endl;

    }
    vector<Point2f> image_points;
    projectPoints(bubi, rvec, tvec, camera_matrix, dist_coeffs, image_points, noArray(), 0);

    // Identify red image points in between the two lines and project them to world coordinates as well
    vector<Point2f> obj_img_points;
    for(int i=sloppy_line1[2]+1; i<sloppy_line2[0]; i++)
    {
      int count = 0, y_pos = 0;
      for(int j=max(0, sloppy_line1[3]-150); j<min(frame->rows, sloppy_line1[3]+150); j++)
      {
        if(frame->at<Vec3b>(j,i).val[2] < 200)
        {
          count ++; y_pos += j; 
        }
      }
      cout << "count count: " << count << endl;
      if(count > 0) 
      { 
        obj_img_points.push_back(Point2f(i,y_pos/count));  
        rectangle(*frame, Point2f(i,y_pos/count), Point2f(i,y_pos/count), 0, 1, 8, 0); 
      }
    }
    cout << "obj_img_points size: " << obj_img_points.size() << endl;
    if(obj_img_points.size() > 0) convertPointsToHomogeneous(obj_img_points, hom_objs);
    vector<Point3f> bubibert;
    vector<Vec3f> burt;
    for(int i=0; i<hom_objs.size(); i++)
    {
      Matx<float,3,1> hom2(hom_objs[i]); 
      Matx<float,3,1> bert; 
      bert = cam2.inv()*hom2;
      Point3f bert2 = Point3f(bert(0), bert(1), bert(2));
      
      bubibert.push_back(identify_plane_coord(cam_world_pos, to3D*bert2));
      burt.push_back(find_object_pos(bubi, cam_world_pos, bubibert[i]));
      // Add found points to vector containing vertices that are eventually written to the .ply file
      if (burt[i][0] > 0 && burt[i][1] > 0 && burt[i][2] > 0) verts->push_back(burt[i]);
    }
    vector<Point2f> img_ps;
    if(obj_img_points.size() > 0) projectPoints(burt, rvec, tvec, camera_matrix, dist_coeffs, img_ps, noArray(), 0);
  } 

}

int main(int argc, char** argv)
{
  if(argc < 4)
  {
    cerr << "Not enough params. Use as: \n./david [CAPTURE DEVICE (cam or video file)] [INTRINSIC PARAMS FILE] [EXTRINSIC PARAMS FILE]"
         << endl;
    return -1;
  }
  string stiggi = string(argv[1]);
  VideoCapture cap;
  vector<Vec3f> verts;
  if(stiggi.length() == 1) 
  {
    cap.open(atoi(argv[1]));
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 768);
  }
  else cap.open(argv[1]);
  Mat rvec, tvec, camera_matrix, dist_coeffs, reference, reference_undist;

  // Collect a few images to get a good reference. First ones are mostly useless for some reason.
  for(int i=0; i < 50; i++) cap >> reference; 

  // VideoWriter writie("../saved_images/vid.mp4", CV_FOURCC('X','2','6','4'), 20, cvSize(cap.get(CV_CAP_PROP_FRAME_WIDTH), 
  //      cap.get(CV_CAP_PROP_FRAME_HEIGHT)));
  VideoWriter writie("../saved_images/vid.avi", CV_FOURCC('M','J','P','G'), 25, cvSize(cap.get(CV_CAP_PROP_FRAME_WIDTH), 
          cap.get(CV_CAP_PROP_FRAME_HEIGHT)));

  // Read intrinsic and extrinsic parameters from config files
  FileStorage fsip(argv[2], FileStorage::READ);
  if(!fsip.isOpened())
  {
    cerr << "Parameter file could not be opened." << endl;
    return -1;
  }
  fsip["Camera_Matrix"] >> camera_matrix;
  fsip["Distortion_Coefficients"] >> dist_coeffs;

  FileStorage fsep(argv[3], FileStorage::READ);
  if(!fsep.isOpened())
  {
    cerr << "Parameter file could not be opened." << endl;
    return -1;
  }
  fsep["RVec"] >> rvec;
  fsep["TVec"] >> tvec;

  undistort(reference, reference_undist, camera_matrix, dist_coeffs);

  while(1)
  {
    Mat frame;
    cap >> frame;
    if(stiggi.length() == 1) writie << frame;
    int key = waitKey(30);

    //if (key == 1048608) 
    find_lines(&frame, reference_undist, camera_matrix, dist_coeffs, rvec, tvec, &verts);
    imshow("frame", frame);

    if(key == 1048603) // ESC
    {
       cap.release(); if(stiggi.length() == 1) writie.release(); write_ply(&verts, (char*)"../clouds/shit"); return 1;
    }
  }

}

