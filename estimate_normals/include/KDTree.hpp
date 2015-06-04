#ifndef KDTREE_HPP_
#define KDTREE_HPP_

#include <Eigen/Dense>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
using namespace pcl;
using namespace Eigen;
using namespace std;

class KDTree
{
public:
  static int depth;
  int npts;
  union {
    struct {
      PointXYZ center;
      PointXYZ bounds;
      int splitaxis;
      KDTree *child1, *child2;
    } node;
    struct {
      PointXYZ p[8];
    } leaf;
  };

  KDTree(PointCloud<PointXYZ> &pts, int n);

  ~KDTree();

  void FindRClosest(PointXYZ *_p, double maxdist2, vector<PointXYZ*> &nn);

  void FindKClosest(PointXYZ *_p, int kneigh, map<double, PointXYZ*> &nn);

private:
  static vector<PointXYZ*> closest;
  static map<double, PointXYZ*> kclosest;
  static double closest_d2;
  static PointXYZ *p;
  static int kn;
  
  void _FindRClosest();
  void _FindKClosest();
  
};

#endif // KDTREE_HPP_
