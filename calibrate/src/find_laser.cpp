#include <Laser_Finder.h>

using namespace cv;

Laser_Finder::find_line(Mat* frame)
{
  Mat dst, cdst;
  Canny(frame, dst, 50, 200, 3);
  cvtColor(dst, cdst, CV_GRAY2BGR);

  vector<Vec2f> lines;
  HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0);

  

}
