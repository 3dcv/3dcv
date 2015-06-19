#include "DistanceFunction.hpp"
#include <pcl/features/normal_3d.h>
using namespace std;
using namespace pcl;

DistanceFunction::DistanceFunction(PointCloud<pcl::PointXYZ>::Ptr cloud, int kn, int ki, double px, double py, double pz)
{
  cloud_ = cloud;
  //Normal_Estimator normi(*cloud_, kn, ki, px, py, pz);
  cloud_normals_ = pcl::PointCloud<pcl::Normal>::Ptr(new PointCloud<pcl::Normal>());
  //normi.estimate_normals(cloud_normals_);
   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setViewPoint(px,py,pz); 
  ne.setInputCloud (cloud_);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  // Use all neighbors in a sphere of radius 3cm
  ne.setKSearch (kn);

  // Compute the features
  ne.compute (*cloud_normals_);
  //normi.average_normals(cloud_normals_);
  kdtree_.setInputCloud (cloud_);
}

DistanceFunction::~DistanceFunction() {}

float DistanceFunction::distance(float x, float y, float z, int k)
{
  //KdTreeFLANN<pcl::PointXYZ> kdtree;
  //kdtree.setInputCloud (cloud_);
  std::vector<int> tangentialPoint(k);
  std::vector<float> pointNKNSquaredDistance(k);
  kdtree_.nearestKSearch(PointXYZ(x, y, z), k, tangentialPoint, pointNKNSquaredDistance);   
  double dist_mean = 0;
  for(int i = 0; i < k; i++)
  {
    Vector3f v(cloud_normals_->points[tangentialPoint[i]].data_n); 
    Vector3f w(-(Vector3f(cloud_->points[tangentialPoint[i]].x, cloud_->points[tangentialPoint[i]].y,  cloud_->points[tangentialPoint[i]].z) - Vector3f(x, y, z)));
    dist_mean += fabs(v.dot(w)/v.norm());
  Vector3f o(w-v);  
    double o_dist = o.norm();
    if(o_dist > w.norm())
      dist_mean *= -1; 
  }
  return dist_mean /= k;
}


