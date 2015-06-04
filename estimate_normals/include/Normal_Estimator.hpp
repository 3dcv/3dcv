#ifndef NORMAL_ESTIMATOR_HPP
#define NORMAL_ESTIMATOR_HPP

#include <fstream>
#include <stdio.h>

#include <boost/program_options.hpp>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <pcl/common/common.h>
#include "KDTree.hpp"

using namespace std;
using namespace Eigen;
using namespace pcl;

class Normal_Estimator
{
public:
  Normal_Estimator(PointCloud<PointXYZ>& cloud, int kn, int ki, double px, double py, double pz);

  ~Normal_Estimator();

  int estimate_normals(PointCloud<pcl::Normal>::Ptr cloud_normals);
  //void compute_pca(Vector3f);

private:
  PointCloud<PointXYZ> cloud_;
  KDTree ent_;
  Vector3d orient_pos_;
  int kn_, ki_;

  void compute_pca(int vertex, VectorXd true_normal);
};

#endif // NORMAL_ESTIMATOR_HPP
