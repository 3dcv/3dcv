#include <Normal_Estimator.hpp>

Normal_Estimator::Normal_Estimator(PointCloud<PointXYZ> &cloud, int kn, int ki, double px, double py, double pz): ent_(cloud, cloud.width)
{
  orient_pos_ = Vector3d(px, py, pz);
  kn_ = kn;
  ki_ = ki;
  cloud_ = cloud;
  

//  PCLPointCloud2 cloud2;
//  PLYReader ply_reader2;
//  ply_reader2.read(ply, cloud2);
//  cout << cloud2.fields[0] << endl;
}

Normal_Estimator::~Normal_Estimator()
{

}

int Normal_Estimator::estimate_normals(PointCloud<pcl::Normal>::Ptr cloud_normals)
{
  //for(size_t i=0; i<cloud_.width; i++)
  for(size_t i=0; i<cloud_.width; i++)
  {
    VectorXd normal;
    compute_pca(i, normal);
    cloud_normals->push_back( Normal(normal[0], normal[1], normal[2])); 
  }

}

void Normal_Estimator::compute_pca(int vertex, VectorXd true_normal)
{
  map<double, PointXYZ*> knn;
  ent_.FindKClosest(&cloud_[vertex], kn_, knn);
  cout << kn_ << endl;
  Vector3d mean(cloud_[vertex].x, cloud_[vertex].y, cloud_[vertex].z);
  VectorXd the_point = (VectorXd)mean;
  for(auto it = knn.begin(); it != knn.end() ; it++) 
  {
    PointXYZ point = *it->second;
    mean += Vector3d(point.x, point.y, point.z);
  }
  mean /= kn_+1;
  cout << "mean: " << mean << endl;
  
  //const int ckn = const_cast<int>(kn_);
  //Matrix<double, , 3> barycentrics;
  MatrixXd barycentrics = MatrixXd::Ones(kn_+1,3);
  barycentrics(0,0) = (float)cloud_[vertex].x-mean[0];
  barycentrics(0,1) = (float)cloud_[vertex].y-mean[1];
  barycentrics(0,2) = (float)cloud_[vertex].z-mean[2];
  int i = 0;
  for(auto it = knn.begin(); it != knn.end() ; it++)
  {
    PointXYZ point = *it->second;
    barycentrics(i+1,0) = (float)point.x-mean[0];
    barycentrics(i+1,1) = (float)point.y-mean[1];
    barycentrics(i+1,2) = (float)point.z-mean[2];
    i++;
  }
  cout << "bary: " << barycentrics << endl << "-------" << endl;
  MatrixXd covar = MatrixXd::Ones(3,3);
  covar = ((double)1/kn_)*barycentrics.transpose()*barycentrics;
  cout << "covar: " << covar << endl << "-------" << endl;
  EigenSolver<MatrixXd> es(covar);
  MatrixXcd eivo = es.eigenvectors();
  MatrixXcd eiva = es.eigenvalues();
  cout << "eivo " << eivo << endl;
  cout << "eivo col 0 " << eivo.col(0) << endl;
  cout << "eiva " << eiva << endl;
  complex<double> compi = eivo(0,0);
  cout << "compi: " << compi << endl;
  VectorXcd normal = eivo.col(0);
  double baer = real(compi);
  cout << "baer " << baer << endl;

  if(real(eiva(0)) < real(eiva(1)))
    if(real(eiva(0)) < real(eiva(2))){} 
    else normal = eivo.col(2);
  else 
    if(real(eiva(1)) < real(eiva(2)) ) normal = eivo.col(1);
    else normal = eivo.col(2);
  cout << "vecci: " << normal << endl;

  VectorXcd inv_normal = -normal;
  cout << "inv: " << inv_normal << endl;

  VectorXd hui =inv_normal.real();
  cout << "hui: " << hui << endl;
  true_normal = normal.real();
  if(((normal.real()+the_point)-(VectorXd)orient_pos_).lpNorm<2>() > ((inv_normal.real()+the_point)-(VectorXd)orient_pos_).lpNorm<2>()) true_normal = inv_normal.real();
  cout << "ausdruck: " << true_normal << endl;
  
} 

