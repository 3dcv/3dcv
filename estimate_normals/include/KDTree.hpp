#ifndef KDTREE_HPP_
#define KDTREE_HPP_

#include <Eigen/Dense>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
using namespace pcl;
using namespace Eigen;
using namespace std;

template<typename PointT>
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
      vector<int>* p;
    } leaf;
  };

  KDTree(){}
  
KDTree(vector<int> &pts)
{
  if(pts.size() <= 8)
  {
    npts = pts.size();
    if(npts > 0)
    {
	   leaf.p = new vector<int>(pts);
	}
    return;
  }
  npts = 0;
  PointXYZ minp, maxp;
  getMinMaxPoints(pts, minp, maxp);
  node.center = PointXYZ((maxp.x + minp.x)/2, (maxp.y + minp.y)/2, (maxp.z + minp.z)/2);
  node.bounds = PointXYZ(maxp.x - minp.x, maxp.y - minp.y, maxp.z - minp.z); 

  if (node.bounds.x > node.bounds.y)
    if (node.bounds.x > node.bounds.z) node.splitaxis = 0;
    else node.splitaxis = 2;
  else
    if (node.bounds.y > node.bounds.z) node.splitaxis = 1;
    else node.splitaxis = 2;
  double splitval = node.center.data[node.splitaxis];
  vector<int> left, right;
  for(int i=0; i<pts.size(); i++)
  {
    if(cloud_[pts[i]].data[node.splitaxis] < splitval) 
		left.push_back(i);
    else 
		right.push_back(i);
  }

  node.child1 = new KDTree<PointT>(left);
  node.child2 = new KDTree<PointT>(right);
}

~KDTree()
{
	/*if(npts)
		delete leaf.p;
	else
	{
		delete node.child1;
		delete node.child2;
	}*/
}

void getMinMaxPoints(vector<int>& pts, PointXYZ& minp, PointXYZ& maxp)
{
	maxp.x = 0; maxp.y = 0; maxp.z = 0;
	minp.x = 1000000; minp.y = 1000000; minp.z = 100000000;
	for(int i = 0; i < pts.size(); i++)
	{
		if(cloud_[pts[i]].x < minp.x)
			minp.x = cloud_[pts[i]].x;
		if(cloud_[pts[i]].y < minp.y)
			minp.y = cloud_[pts[i]].y;
		if(cloud_[pts[i]].z < minp.z)
			minp.z = cloud_[pts[i]].z;
		
		if(cloud_[pts[i]].x > maxp.x)
			maxp.x = cloud_[pts[i]].x;
		if(cloud_[pts[i]].y > maxp.y)
			maxp.y = cloud_[pts[i]].y;
		if(cloud_[pts[i]].z > maxp.z)
			maxp.z = cloud_[pts[i]].z;
	}
}

void FindRClosest(int _p, double maxdist2, vector<int> &nn)
{
  //closest = NULL;
  closest_d2 = pow(maxdist2,2);
  p = _p;
  _FindRClosest();
  nn = closest;
}

void _FindRClosest()
{
  if(npts)
  {
    for(int i=0; i<npts; i++)
    {
      double myd2 = squaredEuclideanDistance(cloud_[p], cloud_[(*leaf.p)[i]]);
      //double myd2 = squaredEuclideanDistance(cloud_[p], cloud_[leaf.p[i]]);
      if(myd2 < closest_d2 && myd2 > 0.00001) { closest.push_back((*leaf.p)[i]); }
      //if(myd2 < closest_d2 && myd2 > 0.00001) { closest.push_back(leaf.p[i]); }
      //closest.push_back(&leaf.p[i]);
    }
  return;
  }

  double approx_dist_bbox = max(max(fabs(cloud_[p].x-node.center.x)-node.bounds.x,
                                    fabs(cloud_[p].y-node.center.y)-node.bounds.y),
                                    fabs(cloud_[p].z-node.center.z)-node.bounds.z);
  if(approx_dist_bbox >= 0 && pow(approx_dist_bbox,2) >= closest_d2) return;

  double myd = node.center.data[node.splitaxis] - cloud_[p].data[node.splitaxis];
  if(myd >= 0.0f)
  {
    node.child1->_FindRClosest();
    //cout << "squary1: " << sqrt(fabs(myd)) << endl;
    if(pow(myd,2) < closest_d2) node.child2->_FindRClosest();
  }
  else
  {
    node.child2->_FindRClosest();
    //cout << "squary: " << sqrt(fabs(myd)) << endl;
    if(pow(myd,2) < closest_d2) node.child1->_FindRClosest();
  }

}

void FindKClosest(int _p, int kneigh, map<double, int> &nn)
{
  closest_d2 = 0.1*0.1;
  p = _p;
  kn = kneigh;
  _FindKClosest();
  nn = kclosest;
}

void _FindKClosest()
{
  if(npts)
  {
    for(int i=0; i<npts; i++)
    {
      if(kclosest.size() < kn) 
      {
        double dist = squaredEuclideanDistance(cloud_[p], cloud_[(*leaf.p)[i]]);
        if(dist > 0.00001)
        {
           kclosest.insert(pair<double, int>(dist, (*leaf.p)[i]));
        }
      }
      else
      {
        auto last = kclosest.cend()--;
        double dist = squaredEuclideanDistance(cloud_[p], cloud_[(*leaf.p)[i]]);
        if(dist < kclosest.cend()--->first && dist > 0.00001)
        {
          kclosest.erase(last);
          kclosest.insert(pair<double, int>(dist, (*leaf.p)[i]));
          closest_d2 = pow(kclosest.cend()--->first,2);
        }
      }
    }
    return;
  }

  double approx_dist_bbox = max(max(fabs(cloud_[p].x-node.center.x)-node.bounds.x,
                                    fabs(cloud_[p].y-node.center.y)-node.bounds.y),
                                    fabs(cloud_[p].z-node.center.z)-node.bounds.z);
  if(approx_dist_bbox >= 0 && pow(approx_dist_bbox,2) >= closest_d2) return;

  double myd = node.center.data[node.splitaxis] - cloud_[p].data[node.splitaxis];
  if(myd >= 0.0f)
  {
    node.child1->_FindKClosest();
    if(pow(myd,2) < closest_d2) node.child2->_FindKClosest();
  }
  else
  {
    node.child2->_FindKClosest();
    if(pow(myd,2) < closest_d2) node.child1->_FindKClosest();
  }

}

  static PointCloud<PointT> cloud_;
private:
  static vector<int> closest;
  static map<double, int> kclosest;
  static double closest_d2;
  static int p;
  static int kn;
  
};

template<typename PointT>
int KDTree<PointT>::depth = 0;
template<typename PointT>
vector<int> KDTree<PointT>::closest;
template<typename PointT>
double KDTree<PointT>::closest_d2 = 0;
template<typename PointT>
int KDTree<PointT>::p = 0;
template<typename PointT>
int KDTree<PointT>::kn = 0;
template<typename PointT>
map<double, int> KDTree<PointT>::kclosest;
template<typename PointT>
PointCloud<PointT> KDTree<PointT>::cloud_;

#endif // KDTREE_HPP_
