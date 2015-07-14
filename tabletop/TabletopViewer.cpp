#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>

using namespace std;
using namespace pcl;

boost::shared_ptr<visualization::CloudViewer> viewer;
PointCloud<PointXYZRGBA>::Ptr cloudptr(new PointCloud<PointXYZRGBA>);
PointCloud<PointXYZRGBA>::Ptr cloud_filtered(new PointCloud<PointXYZRGBA>);
PointCloud<PointXYZRGBA>::Ptr cloud_filtered2(new PointCloud<PointXYZRGBA>);
pcl::SACSegmentationFromNormals<PointXYZRGBA, pcl::Normal> seg;
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
pcl::search::KdTree<PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<PointXYZRGBA> ());
pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
pcl::NormalEstimation<PointXYZRGBA, pcl::Normal> ne;
pcl::ExtractIndices<PointXYZRGBA> extract;
pcl::ExtractIndices<pcl::Normal> extract_normals;
pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
bool clip = 0, seggy = 0, clusty = 0, ballmug = 0;
pcl::PassThrough<PointXYZRGBA> pass;

void 
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.addCylinder(*coefficients_cylinder, "sphere", 0);
    std::cout << "i only run once" << std::endl;
}


pcl::PointCloud<PointXYZRGBA>::Ptr segment()
{
  cout << "welcome to segmentation." << endl;
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (30);
  ne.compute (*cloud_normals);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (50);
  seg.setDistanceThreshold (0.02);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl::PointCloud<PointXYZRGBA>::Ptr cloud_plane (new pcl::PointCloud<PointXYZRGBA> ());
  extract.filter (*cloud_plane);
  return cloud_plane;
}
void grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud)
{
	if (! viewer->wasStopped())
  {
    //cout << clip << "?" << endl;
    if (!clip) viewer->showCloud(cloud);
    else
    {
      cout << "clippers" << endl;
      pcl::PassThrough<PointXYZRGBA> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits(0, 3);
      pass.filter(*cloud_filtered);
      cout << cloud->sensor_origin_ << endl; 
      if(seggy)
      {
          pcl::PointCloud<PointXYZRGBA>::Ptr cloud_plane = segment();
         cout << "seg seg " << cloud_plane->width << endl;
         for(int i = 0; i<cloud_plane->width;i++)
        {
            (*cloud_plane)[i].r = 255;
            (*cloud_plane)[i].g = 0;
            (*cloud_plane)[i].b = 0;
            (*cloud_plane)[i].a = 255;
        }
        //viewer->showCloud(cloud_filtered);
        //viewer->showCloud(cloud_plane);
        if(clusty)
        {
          cout << "get clusty " << endl;
          // Remove the planar inliers, extract the rest
          extract.setNegative (true);
          extract.filter (*cloud_filtered2);
          extract_normals.setNegative (true);
          extract_normals.setInputCloud (cloud_normals);
          extract_normals.setIndices (inliers_plane);
          extract_normals.filter (*cloud_normals2);
          
          seg.setOptimizeCoefficients (true);
          seg.setModelType (pcl::SACMODEL_CYLINDER);
          seg.setMethodType (pcl::SAC_RANSAC);
          seg.setNormalDistanceWeight (0.1);
          seg.setMaxIterations (2000);
          seg.setDistanceThreshold (0.07);
          seg.setRadiusLimits (0, 0.14);
          seg.setInputCloud (cloud_filtered2);
          seg.setInputNormals (cloud_normals2);
          seg.segment (*inliers_cylinder, *coefficients_cylinder);
          extract.setInputCloud (cloud_filtered2);
          extract.setIndices (inliers_cylinder);
          extract.setNegative (false);
          pcl::PointCloud<PointXYZRGBA>::Ptr cloud_cylinder (new pcl::PointCloud<PointXYZRGBA> ());
          extract.filter (*cloud_cylinder); 
          viewer->showCloud(cloud_cylinder);
          viewer->runOnVisualizationThreadOnce (viewerPsycho);
        }
      }
    }
  }
/*	if (saveCloud)
	{
		stringstream stream;
		stream << "inputCloud" << filesSaved << ".pcd";
		string filename = stream.str();
		if (io::savePCDFile(filename, *cloud, true) == 0)
		{
			filesSaved++;
			cout << "Saved " << filename << "." << endl;
		}
		else PCL_ERROR("Problem saving %s.\n", filename.c_str());
 
		saveCloud = false;
	}
*/
}


void keyboardEventOccurred(const visualization::KeyboardEvent& event, void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		 cout << "oi ah harr" << event.getKeySym() << endl;
	else if (event.getKeySym() == "KP_1" && event.keyDown())
  {
    cout << "hey hier huh" << event.getKeySym() << endl;
    clip = !clip;
  }
  if (event.keyDown()) cout << "baer: " << event.getKeySym() << " " << event.getKeyCode() << endl;
  if (event.getKeySym() == "KP_2" && event.keyDown())
  {
    seggy = !seggy;
  }
  if (event.getKeySym() == "KP_3" && event.keyDown())
  {
    clusty = !clusty;
  }
  
}
 
int main(int argc, char** argv)
{
  viewer = boost::shared_ptr<visualization::CloudViewer>(new visualization::CloudViewer("OpenNI viewer"));
	viewer->registerKeyboardCallback(keyboardEventOccurred);
  Grabber* openniGrabber = new OpenNIGrabber();
  bool justVisualize(1);
  string filename;
  if (console::find_argument(argc, argv, "-v") >= 0)
  {
    if (argc != 3)
    {
      //printUsage(argv[0]);
      cout << "bitch" << endl;
      return -1;
    }
 
    filename = argv[2];
  } 


  if(openniGrabber == 0)
  {
    return -1;
  }
  boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> f =
    boost::bind(&grabberCallback, _1);
  openniGrabber->registerCallback(f);

  openniGrabber->start();
 
  // Main loop.
  while (! viewer->wasStopped())
    boost::this_thread::sleep(boost::posix_time::seconds(1));
 
 // if (! justVisualize)
 //   openniGrabber->stop();
}
