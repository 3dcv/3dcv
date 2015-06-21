#include <Octree.hpp>
#include <DistanceFunction.hpp>
#include <boost/program_options.hpp>
#include <stdio.h>
#include <pcl/io/ply_io.h>

namespace po = boost::program_options;
using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
  int kd, ki, kn;
  double v;
  string ply_file, config_file;
  try
  {
    po::options_description generic("Allowed Options");
    generic.add_options()
      ("help,h", "produce help message")
      ("config,c", po::value<string>(&config_file),
                    "path to an optional configuration file in which the other parameters can be stored")
    ;

    po::options_description config("Configuration parameters");
    config.add_options()
      ("kd", po::value(&kn), "set number of neighbors to use for distance calculation")
      ("ki", po::value(&kn), "set number of neighbors to use for normal interpolation")
      ("kn", po::value(&kn), "set number of neighbors to use for normal estimation")
      ("v", po::value(&v), "set octree voxel size")
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
    if(!vm.count("kn") || !vm.count("ki") || !vm.count("kd") || !(vm.count("ply") || !vm.count("v") || vm.count("input")))
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
  PointCloud<PointXYZ> cloud;
  PLYReader ply_reader;
  ply_reader.read(ply_file, cloud);
  PointCloud<PointXYZ>::Ptr cloudPtr (new PointCloud<PointXYZ>(cloud));
  DistanceFunction disti(cloudPtr, kn, ki, kd, 1, 1, 1);
  Octree octo(ply_file, v, disti);
  octo.reconstruct();
  return 0;
}
