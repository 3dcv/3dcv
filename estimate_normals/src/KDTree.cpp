#include <KDTree.hpp>

template<typename PointT>
int KDTree<PointT>::depth = 0;
template<typename PointT>
vector<PointT*> KDTree<PointT>::closest;
template<typename PointT>
double KDTree<PointT>::closest_d2 = 0;
template<typename PointT>
PointT* KDTree<PointT>::p = NULL;
template<typename PointT>
int KDTree<PointT>::kn = 0;
template<typename PointT>
map<double, PointT*> KDTree<PointT>::kclosest;

template<typename PointT>
void KDTree<PointT>::FindRClosest(PointT *_p, double maxdist2, vector<PointT*> &nn)
{
  //closest = NULL;
  closest_d2 = pow(maxdist2,2);
  p = _p;
  _FindRClosest();
  nn = closest;
}

template<typename PointT>
void KDTree<PointT>::_FindRClosest()
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

template<typename PointT>
void KDTree<PointT>::FindKClosest(PointT *_p, int kneigh, map<double, PointT*> &nn)
{
  closest_d2 = 10000;
  p = _p;
  kn = kneigh;
  _FindKClosest();
  nn = kclosest;
}

template<typename PointT>
void KDTree<PointT>::_FindKClosest()
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
           kclosest.insert(pair<double, PointT*>(dist, &leaf.p[i]));
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
          kclosest.insert(pair<double, PointT*>(dist, &leaf.p[i]));
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

