#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace cv;

static int calibrate(vector<Mat>* images)
{
  vector <vector <Point2f> > corners;
  vector<vector<Point3f> > object_points(1);
  Size board_size = cvSize(7, 5);
  float square_size = 0.03;
  bool found = 0;
  float aspectratio = (float)images->at(0).cols/images->at(0).rows;
  corners.resize(images->size());
  Size image_size = images->at(0).size();
  // For all images find chessboard corners and throw out images in which no corners were found
  for(int i=0; i<images->size(); i++)
  {
    found = findChessboardCorners(images->at(i), board_size, corners[i], CV_CALIB_CB_ADAPTIVE_THRESH);
    std::cout << (found ? "Found" : "Failed") << std::endl;
    if(found)
    {
      drawChessboardCorners(images->at(i), board_size, corners[i], found);
      Mat view_gray;
      cvtColor(images->at(i), view_gray, CV_BGR2GRAY);
      cornerSubPix(view_gray, corners[i], Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
    }
    else corners.erase(corners.begin()+i);  
  // Show dem images with found corners drawn in
    while(1)
    {
      imshow("bam", images->at(i));
      if(waitKey(30) > 0) break;
    }

  }
  // Generate object points
  for(int i=0; i<board_size.height; i++)
  {
    for(int j=0; j<board_size.width; j++)
    {
      object_points[0].push_back(Point3f(float(j*square_size), float(i*square_size), 0));
    }
  }
  if(corners.size() < 1)
  {
    std::cerr << "No viable images. Get your shit together." << std::endl;
  }
  // Calibrate, cout intrinsic parameters and write 'em to config file
  else
  {
    std::cout << "Calibrating with " << corners.size() << " image(s)." << std::endl;
    object_points.resize(corners.size(), object_points[0]);
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
    vector<Mat> rvecs, tvecs;
    double rms = calibrateCamera(object_points, corners, image_size, cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_FIX_ASPECT_RATIO);
    std::cout << "rms: " << rms << std::endl;
    std::cout << "camera matrix: " << cameraMatrix << std::endl;
    std::cout << "distortion coefficients: " << distCoeffs << std::endl;

    FileStorage fs("../config/intrinsic_params", FileStorage::WRITE);
    fs << "Camera_Matrix" << cameraMatrix;
    fs << "Distortion_Coefficients" << distCoeffs;
    fs << "RMS" << rms;
    fs.release();
  }
}

static int calibrate_via_live_images()
{
  VideoCapture cap(1); 
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 768);

  if(!cap.isOpened())
  {
    std::cerr << "Camera could not be accessed" << std::endl;
    return -1;
  }

  vector<Mat> images;
  std::cout << "To save an image, press space.\nTo break, press escape.\nTo write the saved images to file, press s.\nTo calibrate with the previously saved images, press c." << std::endl;
  while(1)
  {
    Mat frame;
    cap >> frame;
    frame = frame.colRange((frame.size().width-frame.size().height)/2, (frame.size().width-frame.size().height)/2+frame.size().height);

    imshow("shit", frame);
    int key = waitKey(30);
    if(key == 1048603) break;
    else if(key == 1048608) images.push_back(frame);
    else if(key == 1048691 && images.size() > 0)
    {
      namespace pt = boost::posix_time;
      pt::ptime now = pt::second_clock::local_time();
      std::stringstream ss;
      ss << "../saved_images/" << now.date().year() << "_" << static_cast<int>(now.date().month()) << "_" << now.date().day()
          << "_" << now.time_of_day().hours() << "_" << now.time_of_day().minutes() << "_" << now.time_of_day().seconds() << "_img_"; 
      std::cout << ss << std::endl; 
      vector<int> compression_params;
      compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
      compression_params.push_back(3);
      std::cout << "Saving recorded images." << std::endl;
      for(int i=0; i<images.size(); i++)
      {
        std::stringstream st;
        st << ss.str() << i << ".png";
        std::cout << st.str() << std::endl;
        imwrite(st.str(), images[i], compression_params);
      }
    }
    else if(key == 1048675 && images.size() > 0)
    {
      calibrate(&images);
      break;
    }
  }


}

static void calibrate_via_prerecorded_images(int argc, char** argv)
{
  vector<Mat> images;
  imread(argv[0], CV_LOAD_IMAGE_COLOR);
  for (int i=1; i<argc; i++)
  {
    images.push_back(imread(argv[i], CV_LOAD_IMAGE_COLOR));
  }
  calibrate(&images);
}


int main(int argc, char** argv)
{
  if(argc == 1)
  {
    std::cout << "No images provided. Trying to access camera." << std::endl;
    
    calibrate_via_live_images();
  }

  else
  {
    std::cout << "Processing images." << std::endl;

    calibrate_via_prerecorded_images(argc, argv);
  }

  return 0;
}


