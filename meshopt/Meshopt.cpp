#include <boost/program_options.hpp>
#include <stdio.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include "Vertex.hpp" 
#include "Edge.hpp" 
#include "Face.hpp" 
#include "HMesh.hpp" 

namespace po = boost::program_options;
using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
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
    if(vm.count("input"))
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
  PolygonMesh pome;
  PLYReader ply_reader;
  //ply_reader.read(ply_file, cloud);
  ply_reader.read(ply_file, pome);
  fromPCLPointCloud2(pome.cloud, cloud);
  PointCloud<PointXYZ>::Ptr cloudPtr (new PointCloud<PointXYZ>(cloud));
  HMesh hame;
  for (int i=0; i< cloud.width; i++)
  {
    hame.addVertex(Vertex(cloud[i].x, cloud[i].y, cloud[i].z));
  }
  for (int i=0; i< pome.polygons.size(); i++)
  {
    vector<size_t> indices(3);
    for(int j = 0; j < pome.polygons[i].vertices.size();j++)
    {
      indices[j] = pome.polygons[i].vertices[j];
    }
    hame.addTriangle(indices[0], indices[1], indices[2]);
  }
  hame.printFaces();
  hame.collapseEdge(hame.faces_[hame.faces_.size() - 1]->startEdge_);
  hame.printFaces();
  cout << "Ready " << endl;
  return 0;
}
