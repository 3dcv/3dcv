#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/normal_3d.h>

using namespace std;
using namespace pcl;

boost::shared_ptr<visualization::CloudViewer> viewer;
PointCloud<PointXYZRGBA>::Ptr cloudptr(new PointCloud<PointXYZRGBA>);
PointCloud<PointXYZRGBA>::Ptr cloud_filtered(new PointCloud<PointXYZRGBA>);
PointCloud<PointXYZRGBA>::Ptr cloud_filtered2(new PointCloud<PointXYZRGBA>);
PointCloud<PointXYZRGBA>::Ptr cloud_filtered3(new PointCloud<PointXYZRGBA>);
pcl::SACSegmentationFromNormals<PointXYZRGBA, pcl::Normal> norm_seg;
pcl::SACSegmentation<PointXYZRGBA> seg;
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals4 (new pcl::PointCloud<pcl::Normal>);
pcl::search::KdTree<PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<PointXYZRGBA> ());
pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices), inliers_sphere (new pcl::PointIndices);
pcl::NormalEstimation<PointXYZRGBA, pcl::Normal> ne;
pcl::ExtractIndices<PointXYZRGBA> extract;
pcl::ExtractIndices<pcl::Normal> extract_normals;
pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients), coefficients_sphere(new pcl::ModelCoefficients);
bool clip = 0, seggy = 0, clusty = 0, ballmug = 0;
pcl::PassThrough<PointXYZRGBA> pass;
int countcount = 0;

void 
viewerPsychoCylinder(pcl::visualization::PCLVisualizer& viewer)
{
    viewer.addCylinder(*coefficients_cylinder, "sphere" + to_string(countcount), 0);
    countcount++;
}

void 
viewerPsychoSphere (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.addSphere(*coefficients_sphere, "cylinder" + to_string(countcount), 0);
    countcount++;
}

void
viewerPsychoRemoval (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.removeShape("sphere" + to_string(countcount));
    viewer.removeShape("cylinder" + to_string(countcount));
    countcount--;
}

pcl::PointCloud<PointXYZRGBA>::Ptr segment()
{
  cout << "welcome to segmentation." << endl;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (50);
  seg.setDistanceThreshold (0.012);
  seg.setInputCloud (cloud_filtered);
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
      pass.setFilterLimits(0, 1.5);
      pass.filter(*cloud_filtered);
	  if(!seggy) viewer->showCloud(cloud_filtered);
      //cout << cloud->sensor_origin_ << endl; 
      else
      {
          pcl::PointCloud<PointXYZRGBA>::Ptr cloud_plane = segment();
         //cout << "seg seg " << cloud_plane->width << endl;
         for(int i = 0; i<cloud_plane->width;i++)
        {
            (*cloud_plane)[i].r = 255;
            (*cloud_plane)[i].g = 0;
            (*cloud_plane)[i].b = 0;
            (*cloud_plane)[i].a = 255;
        }
        viewer->showCloud(cloud_plane);
        
        cout << "go cluster !! " << endl;
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> clustering;
        
        clustering.setClusterTolerance(0.018);
	  // Set the minimum and maximum number of points that a cluster can have.
	  clustering.setMinClusterSize(100);
	  clustering.setMaxClusterSize(500000);
	  clustering.setSearchMethod(tree);
	  clustering.setInputCloud(cloud_plane);
	  std::vector<pcl::PointIndices> plane_clusters;
	  clustering.extract(plane_clusters);
	  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGBA>);
	  size_t max_points = 0;
	  int index;
	  cout << "cluster planes " << plane_clusters.size() << endl;
	  for (int i = 0; i < plane_clusters.size(); ++i)
	  {
		  if(plane_clusters[i].indices.size() > max_points)
		  {
			  max_points = plane_clusters[i].indices.size();
			  index = i;
		  }      
      }
      
     for (auto point = plane_clusters[index].indices.begin(); point != plane_clusters[index].indices.end(); point++)
	 {
	   plane->points.push_back(cloud_plane->points[*point]);
	 }
	 plane->width = plane->points.size();
	 plane->height = 1;
	 plane->is_dense = true;
        viewer->showCloud(plane);
        PointXYZRGBA imin;
		PointXYZRGBA imax;
		PointXYZRGBA normal; 
		normal.x  = coefficients_plane->values[0];
		normal.y  = coefficients_plane->values[1];
		normal.z  = coefficients_plane->values[2];
		(*cloud_plane)[0].x += normal.x *3;
		(*cloud_plane)[0].y += normal.y *3;
		(*cloud_plane)[0].z += normal.z *3;
		pcl::getMinMax3D (*plane, imin, imax);
		// build the condition
		pcl::ConditionAnd<pcl::PointXYZRGBA>::Ptr range_cond (new
					  pcl::ConditionAnd<pcl::PointXYZRGBA> ());
					  
		range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr (new
		  pcl::FieldComparison<pcl::PointXYZRGBA> ("x", pcl::ComparisonOps::LT, imax.x)));
		range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr (new
		  pcl::FieldComparison<pcl::PointXYZRGBA> ("x", pcl::ComparisonOps::GT, imin.x)));
		range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr (new
		  pcl::FieldComparison<pcl::PointXYZRGBA> ("y", pcl::ComparisonOps::LT, imax.y)));
		range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr (new
		  pcl::FieldComparison<pcl::PointXYZRGBA> ("y", pcl::ComparisonOps::GT, imin.y)));
		range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr (new
		  pcl::FieldComparison<pcl::PointXYZRGBA> ("z", pcl::ComparisonOps::LT, imax.z)));
		range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGBA>::ConstPtr (new
		  pcl::FieldComparison<pcl::PointXYZRGBA> ("z", pcl::ComparisonOps::GT, imin.z)));

		// build the filter
		pcl::ConditionalRemoval<pcl::PointXYZRGBA> condrem (range_cond);		
		
        if(clusty)
        {
          cout << "get clusty " << endl;
          // Remove the planar inliers, extract the rest
          extract.setNegative (true);
          extract.filter (*cloud_filtered2);

		  condrem.setInputCloud (cloud_filtered2);
		  // apply filter
		  condrem.filter (*cloud_filtered3);
		  viewer->showCloud(cloud_filtered3);
          // Euclidean clustering object.
          
          // Set cluster tolerance to 2cm (small values may cause objects to be divided
          // in several clusters, whereas big values may join objects in a same cluster).
          clustering.setClusterTolerance(0.018);
          // Set the minimum and maximum number of points that a cluster can have.
          clustering.setMinClusterSize(2000);
          clustering.setMaxClusterSize(50000);
          clustering.setSearchMethod(tree);
          clustering.setInputCloud(cloud_filtered3);
          std::vector<pcl::PointIndices> clusters;
          clustering.extract(clusters);
          while(countcount >= 0)
          {
            viewer->runOnVisualizationThreadOnce (viewerPsychoRemoval);
          }
          countcount = 0;
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr all_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
          
          for (auto i = clusters.begin(); i != clusters.end(); ++i)
          {
             pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
             pcl::PointCloud<pcl::Normal>::Ptr cloud_normals3 (new pcl::PointCloud<pcl::Normal>);
             // ...add all its points to a new cloud...
             for (auto point = i->indices.begin(); point != i->indices.end(); point++)
             {
               cluster->points.push_back(cloud_filtered3->points[*point]);
               all_cluster->points.push_back(cloud_filtered2->points[*point]);
             }
             cluster->width = cluster->points.size();
             cluster->height = 1;
            cluster->is_dense = true;

            
            //viewer->showCloud(cluster);
            
            ne.setSearchMethod (tree);
			ne.setInputCloud (cluster);
			ne.setKSearch (50);
			ne.compute (*cloud_normals3);
            
            cout << "get cylindrical " << endl;
            norm_seg.setOptimizeCoefficients (true);
            norm_seg.setModelType (pcl::SACMODEL_CYLINDER);
            norm_seg.setMethodType (pcl::SAC_PROSAC );
            norm_seg.setNormalDistanceWeight (0.1);
            norm_seg.setMaxIterations (10000);
            norm_seg.setDistanceThreshold (0.02);
            norm_seg.setRadiusLimits (0, 0.10);
            norm_seg.setInputCloud (cluster);
            norm_seg.setInputNormals (cloud_normals3);
            //cout << "coeffis " << *coefficients_cylinder << endl;
            norm_seg.segment (*inliers_cylinder, *coefficients_cylinder);
            //coefficients_cylinder->[0]
            Eigen::VectorXf cyc_vec(3);
            coefficients_cylinder->values[3] = coefficients_cylinder->values[3]*0.2;
            coefficients_cylinder->values[4] = coefficients_cylinder->values[4]*0.2;
            coefficients_cylinder->values[5] = coefficients_cylinder->values[5]*0.2;
            
             //calc cylinder error
            auto cylinder_model = norm_seg.getModel();
            std::vector<double> cy_disti;
            Eigen::VectorXf vecci(7);
            vecci[0] = coefficients_cylinder->values[0];
            vecci[1] = coefficients_cylinder->values[1];
            vecci[2] = coefficients_cylinder->values[2];
            vecci[3] = coefficients_cylinder->values[3];
            vecci[4] = coefficients_cylinder->values[4];
            vecci[5] = coefficients_cylinder->values[5];
            vecci[6] = coefficients_cylinder->values[6];
            cylinder_model->getDistancesToModel(vecci, cy_disti);
            cout << "cylinder disti size " << cy_disti.size() << endl;
            double cylinder_error = 0;
            for(double err : cy_disti)
            {
				cylinder_error += pow(err,2);
			}
			cylinder_error = sqrt(cylinder_error);
            
            cylinder_error /= 2.0;
            
            cout << "get spherical " << endl;
            norm_seg.setOptimizeCoefficients (true);
            norm_seg.setModelType (pcl::SACMODEL_SPHERE);
            norm_seg.setMethodType (pcl::SAC_PROSAC);
            norm_seg.setNormalDistanceWeight (0.1);
            norm_seg.setMaxIterations (10000);
            norm_seg.setDistanceThreshold (0.02);
            norm_seg.setRadiusLimits (0, 0.10);
            norm_seg.setInputCloud (cluster);
            norm_seg.setInputNormals (cloud_normals3);
            //cout << "coeffis " << *coefficients_cylinder << endl;
            norm_seg.segment (*inliers_sphere, *coefficients_sphere);
 
            //cout << "coeffis " << *coefficients_cylinder << endl;
            //extract.setInputCloud (cluster);
            //extract.setIndices (inliers_cylinder);
            //extract.setNegative (false);
            //pcl::PointCloud<PointXYZRGBA>::Ptr cloud_cylinder (new pcl::PointCloud<PointXYZRGBA> ());
            //extract.filter (*cloud_cylinder);
            
            //calc sphere error
            auto sphere_model = norm_seg.getModel();
            std::vector<double> sp_disti;
            Eigen::VectorXf vecci2(4);
			vecci2[0] = coefficients_sphere->values[0];
            vecci2[1] = coefficients_sphere->values[1];
            vecci2[2] = coefficients_sphere->values[2];
            vecci2[3] = coefficients_sphere->values[3];
            sphere_model->getDistancesToModel(vecci2, sp_disti);
             cout << "sphere disti size " << sp_disti.size() << endl;
            double sphere_error = 0;
            for(double err : sp_disti)
            {
				sphere_error += pow(err,2);
			}
			sphere_error = sqrt(sphere_error);
			
			sphere_error *= 10.0;
			
			cout << "cylinder error " << cylinder_error << endl;
			cout << "sphere error " << sphere_error << endl;
			
			if(cylinder_error < sphere_error)
			{
				cout << "Its a rodcocker!!!! " << endl;
				viewer->runOnVisualizationThreadOnce (viewerPsychoCylinder);
			}
			else
			{
				cout << "Its a sphere!!!! " << endl;
				viewer->runOnVisualizationThreadOnce (viewerPsychoSphere);
			}	
			 boost::this_thread::sleep(boost::posix_time::seconds(5));
			
          }
          all_cluster->width = all_cluster->points.size();
          all_cluster->height = 1;
          all_cluster->is_dense = true;
          //viewer->showCloud(all_cluster);
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
    boost::bind(&grabberCallback,  _1);
  openniGrabber->registerCallback(f);

  openniGrabber->start();
 
  // Main loop.
  while (! viewer->wasStopped())
    boost::this_thread::sleep(boost::posix_time::seconds(1));
 
 // if (! justVisualize)
 //   openniGrabber->stop();
}
