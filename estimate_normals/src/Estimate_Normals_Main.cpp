#include "Normal_Estimator.hpp"
#include "KDTree.hpp"
#include <pcl/visualization/cloud_viewer.h>

namespace po = boost::program_options;
using namespace std;

int main(int argc, char** argv)
{
  int ki, kn;
  double px, py, pz;
  string ply_file, config_file;
  try
  {
    po::options_description generic("Allowed options");
    generic.add_options()
      ("help,h", "produce help message")
      ("config,c", po::value<string>(&config_file),
                    "path to an optional configuration file in which the other parameters can be stored")
    ;

    po::options_description config("Configuration parameters");
    config.add_options()
      ("kn", po::value(&kn), "set number of neighbors to use for normal estimation")
      ("ki", po::value(&ki), "set number of neighbors over which to average normals for improving initial estimation")
      ("px", po::value(&px), "x coordinate of position to which normals are oriented")
      ("py", po::value(&py), "y coordinate of position to which normals are oriented")
      ("pz", po::value(&pz), "z coordinate of position to which normals are oriented")
      ("ply,input", po::value(&ply_file), "path to the .ply input file containing the point cloud")
    ;

    po::options_description cmdline_options;
    cmdline_options.add(generic).add(config);

    po::options_description config_file_options;
    config_file_options.add(config);

    po::positional_options_description p;
    p.add("ply", -1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(cmdline_options).positional(p).run(), vm);
    po::notify(vm);    

    if(vm.count("help")) {
      cout << cmdline_options << "\n";
      return 1;
    }

    if(vm.count("config"))
    {
      ifstream ifs(config_file.c_str());
      if(!ifs)
      {
        cout << "Can't open config file." << endl;
        return 0;
      }
      else
      {
        store(parse_config_file(ifs, config_file_options), vm);
        notify(vm);
      }
    }
    if(!vm.count("kn") || !vm.count("ki") || !vm.count("px") || !vm.count("py") || !vm.count("pz") || !(vm.count("ply") || vm.count("input")))
    {
      cout << "Some non-optional parameters missing. Check help for a list." << endl;
      return 0;
    }
  }
  catch(exception& e)
  {
    cout << e.what() << endl;
    return 0;
  }

  PLYReader ply_reader;
  PointCloud<PointXYZ> cloud;
  ply_reader.read(ply_file, cloud);
  PointXYZ min,max;
  getMinMax3D(cloud, min, max);
  cout << "min " << min << " max " << max << endl;
  //for(int i=0; i<7; i++) cout << cloud[i] << endl;
  KDTree ent(cloud, cloud.width);
  vector<PointXYZ*> rnn;
  map<double, PointXYZ*> knn;
  ent.FindRClosest(&cloud[250], 0.004, rnn);
  cout << "Neighbors incoming" << endl;
  //for(auto p : knn)
    //cout << *(p.second) << " dist: " << euclideanDistance(cloud[250], *(p.second)) << endl;
  //for(auto p : rnn)
  //  cout << *p << " dist: " << euclideanDistance(cloud[250], *p) << endl;
  //cout << "Total: " << rnn.size() << endl;
  //cout << "ent depth: " << KDTree::depth << endl;
  const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new PointCloud<pcl::Normal>());
  const pcl::PointCloud<PointXYZ>::Ptr cloudPtr (&cloud);
  Normal_Estimator normi(cloud, kn, ki, px, py, pz); 
  normi.estimate_normals(cloud_normals);
  visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor (0.0, 0.0, 0.5);
       
  viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloudPtr, cloud_normals);
        while (!viewer.wasStopped ())
        {
                viewer.spinOnce ();
        } 
  //normi.improve_normals();
  //normi.write_normals_to_ply();
}
