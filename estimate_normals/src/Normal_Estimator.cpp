#include <Normal_Estimator.hpp>

Normal_Estimator::Normal_Estimator(PointCloud<PointXYZ> &cloud, int kn, int ki, double px, double py, double pz)
{
  orient_pos_ = Vector3d(px, py, pz);
  kn_ = kn;
  ki_ = ki;
  cloud_ = cloud;
  ent_.cloud_ = cloud;
  vector<int> vec(cloud.width);
  for(int i = 0; i < cloud.width; i++)
	vec[i] = i;
  ent_ = KDTree<PointXYZ>(vec);
  

//  PCLPointCloud2 cloud2;
//  PLYReader ply_reader2;
//  ply_reader2.read(ply, cloud2);
//  cout << cloud2.fields[0] << endl;
}

Normal_Estimator::~Normal_Estimator()
{

}

void Normal_Estimator::estimate_normals(PointCloud<pcl::Normal>::Ptr cloud_normals)
{
  for(size_t i=0; i<cloud_.width; i++)
  {
    VectorXd normal;
    compute_pca(i, normal);
    cloud_normals->push_back( Normal(normal[0], normal[1], normal[2])); 
  }

}

void Normal_Estimator::knnPerformance()
{
  for(size_t i=0; i<cloud_.width; i++)
  {
    map<double, int> knn;
    ent_.FindKClosest(i, kn_, knn);
  }
}

void Normal_Estimator::average_normals(PointCloud<Normal>::Ptr cloud_normals)
{
  cout << "Averaging Normals ..." << endl;
  for(size_t i=0; i<cloud_.width; i++)
  {
	 map<double, int> knn;
     ent_.FindKClosest(i, ki_, knn);
     Normal mean_normal = (*cloud_normals)[i];
	 for(auto it = knn.begin(); it != knn.end(); it++)
	 {
		mean_normal.data_c[0] += (*cloud_normals)[it->second].data_c[0];
		mean_normal.data_c[1] += (*cloud_normals)[it->second].data_c[1];
		mean_normal.data_c[2] += (*cloud_normals)[it->second].data_c[2];
		mean_normal.data_c[3] += (*cloud_normals)[it->second].data_c[3];
	  }
	  mean_normal.data_c[0] = mean_normal.data_c[0]/(knn.size() + 1);
	  mean_normal.data_c[1] = mean_normal.data_c[1]/(knn.size() + 1);
	  mean_normal.data_c[2] = mean_normal.data_c[2]/(knn.size() + 1);
	  mean_normal.data_c[3] = mean_normal.data_c[3]/(knn.size() + 1);
	  (*cloud_normals)[i] = mean_normal;
  }

}

void Normal_Estimator::compute_pca(int vertex, VectorXd& true_normal)
{
  map<double, int> knn;
  ent_.FindKClosest(vertex, kn_, knn);
  Vector3d mean(cloud_[vertex].x, cloud_[vertex].y, cloud_[vertex].z);
  VectorXd the_point = (VectorXd)mean;
  for(auto it = knn.begin(); it != knn.end() ; it++) 
  {
    PointXYZ point = cloud_[it->second];
    mean += Vector3d(point.x, point.y, point.z);
  }
  mean /= kn_+1;
  //cout << "mean: " << mean << endl;
  
  //const int ckn = const_cast<int>(kn_);
  //Matrix<double, , 3> barycentrics;
  MatrixXd barycentrics = MatrixXd::Ones(kn_+1,3);
  barycentrics(0,0) = (float)cloud_[vertex].x-mean[0];
  barycentrics(0,1) = (float)cloud_[vertex].y-mean[1];
  barycentrics(0,2) = (float)cloud_[vertex].z-mean[2];
  int i = 0;
  for(auto it = knn.begin(); it != knn.end() ; it++)
  {
    PointXYZ point = cloud_[it->second];
    barycentrics(i+1,0) = (float)point.x-mean[0];
    barycentrics(i+1,1) = (float)point.y-mean[1];
    barycentrics(i+1,2) = (float)point.z-mean[2];
    i++;
  }
  //cout << "bary: " << barycentrics << endl << "-------" << endl;
  MatrixXd covar = MatrixXd::Ones(3,3);
  covar = ((double)1/kn_)*barycentrics.transpose()*barycentrics;
  //cout << "covar: " << covar << endl << "-------" << endl;
  EigenSolver<MatrixXd> es(covar);
  MatrixXcd eivo = es.eigenvectors();
  MatrixXcd eiva = es.eigenvalues();
  //cout << "eivo " << eivo << endl;
  //cout << "eivo col 0 " << eivo.col(0) << endl;
  //cout << "eiva " << eiva << endl;
  complex<double> compi = eivo(0,0);
  //cout << "compi: " << compi << endl;
  VectorXcd normal = eivo.col(0);
  double baer = real(compi);
  //cout << "baer " << baer << endl;

  if(real(eiva(0)) < real(eiva(1)))
    if(real(eiva(0)) < real(eiva(2))){} 
    else normal = eivo.col(2);
  else 
    if(real(eiva(1)) < real(eiva(2)) ) normal = eivo.col(1);
    else normal = eivo.col(2);
  //cout << "vecci: " << normal << endl;

  VectorXcd inv_normal = -normal;
  //cout << "inv: " << inv_normal << endl;

  VectorXd hui =inv_normal.real();
  //cout << "hui: " << hui << endl;
  true_normal = normal.real();
  if(((normal.real()+the_point)-(VectorXd)orient_pos_).lpNorm<2>() > ((inv_normal.real()+the_point)-(VectorXd)orient_pos_).lpNorm<2>()) true_normal = inv_normal.real();
  //cout << "ausdruck: " << true_normal << endl;
  
} 

