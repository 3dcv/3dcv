
#include "DavidScanner.hpp"


static void readCalibrateParams(string& intrinsicFile, string& extrinsicFile, Mat& rvec, Mat& tvec, Mat& camera_matrix, Mat& dist_coeffs)
{
  // ### Read intrinsic and extrinsic parameters from config files
  FileStorage fsip(intrinsicFile, FileStorage::READ);
  if(!fsip.isOpened())
  {
    cerr << "Parameter file could not be opened." << endl;
    return;
  }
  fsip["Camera_Matrix"] >> camera_matrix;
  fsip["Distortion_Coefficients"] >> dist_coeffs;

  FileStorage fsep(extrinsicFile, FileStorage::READ);
  if(!fsep.isOpened())
  {
    cerr << "Parameter file could not be opened." << endl;
    return;
  }
  fsep["RVec"] >> rvec;
  fsep["TVec"] >> tvec;
}

int main(int argc, char** argv)
{
  if(argc < 4)
  {
    cerr << "Not enough params. Use as: \n./david [CAPTURE DEVICE (cam or video file)] [INTRINSIC PARAMS FILE] [EXTRINSIC PARAMS FILE]"
         << endl;
    return -1;
  }
  // ### start video capture with device or file ### 
  string cap_device = string(argv[1]);
  VideoCapture cap;
  vector<Vec3f> verts;
  if(cap_device.length() == 1) 
  {
    cap.open(atoi(argv[1]));
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 768);
  }
  else cap.open(argv[1]);
  
  // ### Collect a few images to get a good reference. First ones are mostly useless for some reason. ###
  Mat rvec, tvec, camera_matrix, dist_coeffs, reference, reference_undist;
  for(int i=0; i < 50; i++) cap >> reference; 
  
  VideoWriter writie;
  
  // ### If cam save video ###
  if(cap_device.length() == 1)
  {
	 writie = VideoWriter("../saved_images/vid.avi", CV_FOURCC('M','J','P','G'), 25, cvSize(cap.get(CV_CAP_PROP_FRAME_WIDTH), 
	     cap.get(CV_CAP_PROP_FRAME_HEIGHT)));
  }
  
  // ### Read calibrate params
  string inFile = string(argv[2]);
  string exFile = string(argv[3]);
  readCalibrateParams(inFile, exFile, rvec,tvec, camera_matrix, dist_coeffs);
  
  undistort(reference, reference_undist, camera_matrix, dist_coeffs);

  // ### initialize David Scanner with camera params
  DavidScanner dave(reference_undist, camera_matrix, dist_coeffs, rvec, tvec);

  // ### start with main scanner loop
  while(1)
  {
    Mat frame;
    cap >> frame;
    if(cap_device.length() == 1) writie << frame;
    int key = waitKey(30);

    // ### Process David Scanning  
    dave.scan(frame);
    imshow("frame", frame);

    // ### ESC pressed write vertices to ply file
    if(key == 1048603) // ESC
    {
      cap.release();
      if(cap_device.length() == 1) 
	    writie.release(); 
	    dave.writePLY(string("../clouds/shit")); 
	    return 1;
    }
  }

}
