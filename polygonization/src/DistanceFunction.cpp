#include "DistanceFunction.hpp"
#include <pcl/features/normal_3d.h>
using namespace std;
using namespace pcl;

DistanceFunction::DistanceFunction(PointCloud<pcl::PointXYZ>::Ptr cloud, int kn, int ki, double px, double py, double pz, int kd)
{
  cloud_ = cloud;
  cloud_normals_ = pcl::PointCloud<pcl::Normal>::Ptr(new PointCloud<pcl::Normal>());
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setViewPoint(px,py,pz); 
  ne.setInputCloud (cloud_);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  ne.setKSearch (kn);

  ne.compute (*cloud_normals_);
  kdtree_.setInputCloud (cloud_);
  kd_ = kd;
}

DistanceFunction::~DistanceFunction() {}

float DistanceFunction::distance(float x, float y, float z)
{
  std::vector<int> tangentialPoint(kd_);
  std::vector<float> pointNKNSquaredDistance(kd_);
  kdtree_.nearestKSearch(PointXYZ(x, y, z), kd_, tangentialPoint, pointNKNSquaredDistance);   
  double dist_mean = 0;
  for(int i = 0; i < kd_; i++)
  {
    Vector3f v(cloud_normals_->points[tangentialPoint[i]].data_n); 
    Vector3f w(-(Vector3f(cloud_->points[tangentialPoint[i]].x, cloud_->points[tangentialPoint[i]].y,  cloud_->points[tangentialPoint[i]].z) - Vector3f(x, y, z)));
    dist_mean += fabs(v.dot(w)/v.norm());
    Vector3f o(w-v);  
    double o_dist = o.norm();
  
    if(o_dist > w.norm())
      dist_mean *= -1; 
  }
  return dist_mean /= kd_;
}


