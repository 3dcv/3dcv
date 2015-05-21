#include "DavidScanner.hpp"

DavidScanner::DavidScanner(Mat& reference_undist, Mat& camera_matrix, 
						   Mat& dist_coeffs, Mat& rvec, Mat& tvec): 
						   reference_undist_(reference_undist), 
						   camera_matrix_(camera_matrix), 
						   dist_coeffs_(dist_coeffs), rvec_(rvec), tvec_(tvec)
{
	// Convert rvec and tvec to affine transformation
	 Affine3f rodri((Vec3f)rvec_, (Vec3f)tvec_);
     to3D_ = rodri.inv();
    // Calculate camera position in world coordinates
    cam_world_pos_ = Point3f(to3D_*Point3f(0, 0, 0));
}

DavidScanner::~DavidScanner()
{

}

int DavidScanner::writePLY(string ply_string)
{
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
  ofst << "ply\nformat ascii 1.0\nelement vertex " << verts_.size() << "\n"; 
  ofst << "property float x\nproperty float y\nproperty float z\nend_header\n";
  for(int i=0; i<verts_.size(); i++)
  {
    ofst << verts_.at(i)[0] << " " << verts_.at(i)[1] << " " << verts_.at(i)[2] << "\n";
  }
  ofst.close();
}

Vec3f DavidScanner::identifyPlaneCoord(Point3f proj)
{
  Point3f x,y,z, min, sp = cam_world_pos_-proj;
  double s, min_dist = 10;
  // check on which axis the point should be projected
  if(proj.x < 0)
  {
    s = cam_world_pos_.x/sp.x;
    x = Point3f(0, cam_world_pos_.y - s * sp.y, cam_world_pos_.z - s * sp.z);
    min_dist = norm(cam_world_pos_-x);
    min = x;	
  }
  if(proj.y < 0)
  {
    s = cam_world_pos_.y/sp.y;
    y = Point3f(cam_world_pos_.x - s * sp.x, 0, cam_world_pos_.z - s * sp.z);
    if(norm(cam_world_pos_-y) < min_dist) { min_dist = norm(cam_world_pos_-y); min = y; }
  }
  if(proj.z < 0)
  {
    s = cam_world_pos_.z/sp.z;
    z = Point3f(cam_world_pos_.x - s * sp.x, cam_world_pos_.y - s * sp.y, 0);
    if(norm(cam_world_pos_-z) < min_dist) { min_dist = norm(cam_world_pos_-z); min = z; }
  }
  // return point on axis plane
  return Vec3f(min.x, min.y, min.z);
}

Vec3f DavidScanner::findObjectPos(vector<Vec3f> lines_world, Vec3f cam_world, Vec3f proj_world)
{
  // calculate laser plane normal
  Vec3f dir_vec1 = lines_world[0] - lines_world[1];
  Vec3f dir_vec2 = lines_world[2] - lines_world[3];
  Vec3f base_vec(lines_world[0]);
  Vec3f plane_normal = dir_vec1.cross(dir_vec2);
  
  // calculate object laser line intersection with laser plane
  Vec3f line_dir = cam_world - proj_world;
  float intersect_scalar = (base_vec - cam_world).dot(plane_normal)/line_dir.dot(plane_normal);
  Vec3f object_pos = cam_world + intersect_scalar * line_dir;
  // return object position
  return object_pos;
}

void DavidScanner::gaussianSlopeAvg(Vec4i& avg_line, vector<Vec4i>& lines, bool is_sloppy_site_left)
{
  double slope_avg;
  double slope_dev;
  vector<double> slopes;
  // average
  for(auto line : lines)
  {
    slopes.push_back(double(line[3] - line[1])/double(line[2] - line[0]));
    slope_avg += slopes.back();
  }
  slope_avg /= slopes.size();

  // standard deviation
  for(double slope : slopes)
  {
    slope_dev += pow(slope - slope_avg, 2); 
  }
  slope_dev = sqrt(slope_dev/slopes.size());
  
  // gaussian filter
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
  
  // avg all lines
  int line_xtreme;
  if(is_sloppy_site_left)
     line_xtreme = lines[0][2];
  else
     line_xtreme = lines[0][0];
  for(int i = 0; i < lines.size(); i++)
  {
    avg_line += lines[i];
    if(is_sloppy_site_left)
      line_xtreme = max(line_xtreme, lines[i][2]);
    else
      line_xtreme = min(line_xtreme, lines[i][0]);
  }
  cout << "lines size:" << lines.size() << endl;
  avg_line[0] /= lines.size();
  avg_line[1] /= lines.size();
  avg_line[2] /= lines.size();
  avg_line[3] /= lines.size();
  if(is_sloppy_site_left)
  {
    avg_line[2] = line_xtreme;
    avg_line[3] = avg_line[1] + slope_avg * (line_xtreme - avg_line[0]);
  }
  else
  {
     avg_line[0] = line_xtreme;
     avg_line[1] = avg_line[3] - slope_avg * abs(line_xtreme - avg_line[2]);
  }
} 
  

int DavidScanner::scan(Mat& frame)
{

  Mat dst, gray;
  undistort(frame, dst, camera_matrix_, dist_coeffs_);

  // Subtract reference_undist_ image from current frame. Better than your run-of-the-mill absdiff.
  // Then identify red pixels and make 'em grey. Everything else be white.
  for(int i=0; i<dst.cols; i++)
  {
    for(int j=0; j<dst.rows; j++)
    {
      dst.at<Vec3b>(j,i).val[0] = max(dst.at<Vec3b>(j,i).val[0] - reference_undist_.at<Vec3b>(j,i).val[0], 0);  
      dst.at<Vec3b>(j,i).val[1] = max(dst.at<Vec3b>(j,i).val[1] - reference_undist_.at<Vec3b>(j,i).val[1], 0);  
      dst.at<Vec3b>(j,i).val[2] = max(dst.at<Vec3b>(j,i).val[2] - reference_undist_.at<Vec3b>(j,i).val[2], 0);  
    
      frame.at<Vec3b>(j,i) = Vec3b(255, 255, 255);

      if(dst.at<Vec3b>(j,i).val[2]-35 > dst.at<Vec3b>(j,i).val[0] && dst.at<Vec3b>(j,i).val[2]-35 > dst.at<Vec3b>(j,i).val[1])
      {
        //frame.at<Vec3b>(j,i) = Vec3b(180, 180, 180);
        frame.at<Vec3b>(j,i) = 255-dst.at<Vec3b>(j,i).val[2];
      }
    }
    //imshow("after_absdiff", dst);
  }

  // ### Use canny edge algorithm for non maximum supression
  cvtColor(frame, gray, CV_BGR2GRAY);
  Canny(gray, dst, 70, 230, 5, 1);
  imshow("Ihr Schweine!", dst);
  vector<Vec4i> linesP;
  // ### propabilistic hough line detection to identify laser lines in pic
  HoughLinesP( dst, linesP, 1, CV_PI/180, 75, 300, 150 );

  // ### check if lines start in the left or right side of picture to seperate
  vector<Vec4i> left_lines, right_lines;
  for(int i=0; i<linesP.size(); i++) 
  {
    Vec4i l = linesP[i];
    line(frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,0), 1, CV_AA);
    bool good = 1;
    //cout << "pre found line " << i << ": " << linesP[i] << endl;
    if((l[0]<frame.cols*0.3 && l[2] <frame.cols*0.3) || (l[0]>frame.cols*0.7 && l[2]>frame.cols*0.7) || l[2]-l[0] < frame.cols/7)
    {
      continue;
    }

    if(l[0] < frame.cols*0.29) { left_lines.push_back(l);}
    else if(l[2] > frame.cols*0.71) { right_lines.push_back(l); }

  }
  
  // ### if lines on both sides found interpolate them with gaussian avg
  if(left_lines.size() > 0 && right_lines.size() > 0)
  {
    Vec4i left_line;
    Vec4i right_line;
    gaussianSlopeAvg(left_line, left_lines,1);
    gaussianSlopeAvg(right_line, right_lines,0);
    
    // ### visualize determined lines
    line(frame, Point(left_line[0],left_line[1]), Point(left_line[2], left_line[3]), Scalar(255,0,0), 1, CV_AA);
    line(frame, Point(right_line[0],right_line[1]), Point(right_line[2], right_line[3]), Scalar(0,0,255), 1, CV_AA);

    // Project lines to world coordinates
    vector<Point3f> hom_lines, orig, hom_objs;
    vector<Point2f> img_points;
    img_points.push_back(Point(left_line[0], left_line[1]));
    img_points.push_back(Point(left_line[2], left_line[3]));
    img_points.push_back(Point(right_line[0], right_line[1]));
    img_points.push_back(Point(right_line[2], right_line[3]));
    convertPointsToHomogeneous(img_points, hom_lines);
    
    // ### project lines in 3d world coordinates
    Matx<float,3,3> cam_mat(camera_matrix_); 
    vector<Vec3f> world_3dlines;
    for(int i=0; i<img_points.size(); i++)
    {
      Matx<float,3,1> hom2(hom_lines[i]); 
      Matx<float,3,1> line_3dpoint; 
      line_3dpoint = cam_mat.inv()*hom2;
      Point3f line_3dpoint2;
      line_3dpoint2.x = line_3dpoint(0);
      line_3dpoint2.y = line_3dpoint(1);
      line_3dpoint2.z = line_3dpoint(2);

      world_3dlines.push_back(Point3f(to3D_*line_3dpoint2));

      world_3dlines[i] = identifyPlaneCoord(Point3f(world_3dlines[i][0], world_3dlines[i][1], world_3dlines[i][2]));

    }
    //vector<Point2f> image_points;
    //projectPoints(world_3dlines, rvec, tvec, camera_matrix, dist_coeffs, image_points, noArray(), 0);

    // Identify red image points in between the two lines and project them to world coordinates as well
    vector<Point2f> obj_img_points;
    // identify all laser object points
    for(int i=left_line[2]+1; i<right_line[0]; i++)
    {
      int count = 0, y_pos = 0;
      for(int j=max(0, left_line[3]-150); j<min(frame.rows, left_line[3]+150); j++)
      {
        // ### mean filter for laser points in y axis 
        if(frame.at<Vec3b>(j,i).val[2] < 200)
        {
          count ++; y_pos += j; 
        }
      }
      if(count > 0) 
      { 
        obj_img_points.push_back(Point2f(i,y_pos/count));  
        rectangle(frame, Point2f(i,y_pos/count), Point2f(i,y_pos/count), 0, 1, 8, 0); 
      }
    }
    cout << "obj_img_points size: " << obj_img_points.size() << endl;
    // Homogeneous all object points
    if(obj_img_points.size() > 0) convertPointsToHomogeneous(obj_img_points, hom_objs);
    vector<Point3f> world_point;
    vector<Vec3f> world_3dpoints;
    // convert all object points to 3d world coodrinates by their intersection with the laser plane
    for(int i=0; i<hom_objs.size(); i++)
    {
      Matx<float,3,1> hom2(hom_objs[i]); 
      Matx<float,3,1> world_point2; 
      world_point2 = cam_mat.inv()*hom2;
      Point3f world_point3 = Point3f(world_point2(0), world_point2(1), world_point2(2));
      
      world_point.push_back(identifyPlaneCoord(to3D_*world_point3));
      world_3dpoints.push_back(findObjectPos(world_3dlines, cam_world_pos_, world_point[i]));
      // Add found points to vector containing vertices that are eventually written to the .ply file
      if (world_3dpoints[i][0] > 0 && world_3dpoints[i][1] > 0 && world_3dpoints[i][2] > 0) verts_.push_back(world_3dpoints[i]);
    }
    vector<Point2f> img_ps;
    if(obj_img_points.size() > 0) projectPoints(world_3dpoints, rvec_, tvec_, camera_matrix_, dist_coeffs_, img_ps, noArray(), 0);
  } 

}

