#ifndef DISTANCEFUNCTION_HPP_
#define DISTANCEFUNCTION_HPP_

#include "Normal_Estimator.hpp"
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>

using namespace Eigen;
using namespace std;
using namespace pcl;

class DistanceFunction {

public:
  DistanceFunction(PointCloud<pcl::PointXYZ>::Ptr cloud, int kn, int ki, double px, double py, double pz, int kd = 1);

  ~DistanceFunction();

  float distance(float x, float y, float z);

private:
  PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_;
  KdTreeFLANN<pcl::PointXYZ> kdtree_;

  int kd_;
};

#endif // DistanceFunction
