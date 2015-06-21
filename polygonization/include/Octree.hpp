#ifndef OCTREE_HPP_
#define OCTREE_HPP_

#include <boost/program_options.hpp>
#include <Eigen/Dense>
#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <MCTable.hpp>
#include <DistanceFunction.hpp>

using namespace std;
using namespace pcl;
using namespace Eigen;
  struct onode{
    PointXYZ center;
    vector<int> children;
    double size;
    unsigned char valid;
    vector<int> points;
  } ;

class Octree
{
public:



  Octree(string &cloudfile, double octsize, DistanceFunction dfunk);

  ~Octree();

  void reconstruct();
  
  int writePLY(string ply_string);

private:

  void divide(int nc);

  double voxel_size_;
  vector<onode> nodes_;
  PointCloud<PointXYZ> cloud_;
  DistanceFunction dfunk_;
  vector<Vector3d> verts_; 
};

#endif // Octree
