#include <KDTree.hpp>

int KDTree::depth = 0;
vector<PointXYZ*> KDTree::closest;
double KDTree::closest_d2 = 0;
PointXYZ* KDTree::p = NULL;
int KDTree::kn = 0;
map<double, PointXYZ*> KDTree::kclosest;
  
KDTree::KDTree(PointCloud<PointXYZ> &pts, int n)
{
  //cout << "gross! " << pts.width << endl;
  //cout << "tief! " << ++depth << endl;
  if(n <= 8)
  {
    npts = n;
    if(n > 0) 
      for(int i=0; i< pts.width; i++) leaf.p[i] = pts.points[i];
      depth++; 
    return;
  }
  npts = 0;
  
  PointXYZ min, max;
  getMinMax3D(pts, min, max);
  node.center = PointXYZ((max.x + min.x)/2, (max.y + min.y)/2, (max.z + min.z)/2);
  node.bounds = PointXYZ(max.x - min.x, max.y - min.y, max.z - min.z); 

  if (node.bounds.x > node.bounds.y)
    if (node.bounds.x > node.bounds.z) node.splitaxis = 0;
    else node.splitaxis = 2;
  else
    if (node.bounds.y > node.bounds.z) node.splitaxis = 1;
    else node.splitaxis = 2;
  double splitval = node.center.data[node.splitaxis];
  vector<int> left, right;
  for(int i=0; i<pts.width; i++)
  {
    if(pts.points[i].data[node.splitaxis] < splitval) left.push_back(i);
    else right.push_back(i);
  }
  PointCloud<PointXYZ> left_points, right_points;
  copyPointCloud(pts, left, left_points);
  copyPointCloud(pts, right, right_points);

  node.child1 = new KDTree(left_points, left.size());
  node.child2 = new KDTree(right_points, right.size());
}

KDTree::~KDTree()
{
}

void KDTree::FindRClosest(PointXYZ *_p, double maxdist2, vector<PointXYZ*> &nn)
{
  //closest = NULL;
  closest_d2 = pow(maxdist2,2);
  p = _p;
  _FindRClosest();
  nn = closest;
}

void KDTree::_FindRClosest()
{
  if(npts)
  {
    for(int i=0; i<npts; i++)
    {
      double myd2 = squaredEuclideanDistance(*p, leaf.p[i]);
      if(myd2 < closest_d2 && myd2 > 0.00001) { closest.push_back(&leaf.p[i]); }
      //closest.push_back(&leaf.p[i]);
    }
  return;
  }

  double approx_dist_bbox = max(max(fabs(p->x-node.center.x)-node.bounds.x,
                                    fabs(p->y-node.center.y)-node.bounds.y),
                                    fabs(p->z-node.center.z)-node.bounds.z);
  if(approx_dist_bbox >= 0 && pow(approx_dist_bbox,2) >= closest_d2) return;

  double myd = node.center.data[node.splitaxis] - p->data[node.splitaxis];
  if(myd >= 0.0f)
  {
    node.child1->_FindRClosest();
    cout << "squary1: " << sqrt(fabs(myd)) << endl;
    if(pow(myd,2) < closest_d2) node.child2->_FindRClosest();
  }
  else
  {
    node.child2->_FindRClosest();
    cout << "squary: " << sqrt(fabs(myd)) << endl;
    if(pow(myd,2) < closest_d2) node.child1->_FindRClosest();
  }

}

void KDTree::FindKClosest(PointXYZ *_p, int kneigh, map<double, PointXYZ*> &nn)
{
  closest_d2 = 10000;
  p = _p;
  kn = kneigh;
  _FindKClosest();
  nn = kclosest;
}

void KDTree::_FindKClosest()
{
  if(npts)
  {
    for(int i=0; i<npts; i++)
    {
      if(kclosest.size() < kn) 
      {
        double dist = squaredEuclideanDistance(*p, leaf.p[i]);
        if(dist > 0.00001)
        {
           kclosest.insert(pair<double, PointXYZ*>(dist, &leaf.p[i]));
           //closest_d2 = klosest.cend()-->first;
        }
      }
      else
      {
        auto last = kclosest.cend()--;
        double dist = squaredEuclideanDistance(*p, leaf.p[i]);
        if(dist < kclosest.cend()--->first && dist > 0.00001)
        {
          kclosest.erase(last);
          kclosest.insert(pair<double, PointXYZ*>(dist, &leaf.p[i]));
          closest_d2 = kclosest.cend()--->first;
        }
      }
    }
    return;
  }

  double approx_dist_bbox = max(max(fabs(p->x-node.center.x)-node.bounds.x,
                                    fabs(p->y-node.center.y)-node.bounds.y),
                                    fabs(p->z-node.center.z)-node.bounds.z);
  if(approx_dist_bbox >= 0 && pow(approx_dist_bbox,2) >= closest_d2) return;

  double myd = node.center.data[node.splitaxis] - p->data[node.splitaxis];
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

