#include <boost/program_options.hpp>
#include <stdio.h>

#include "Vertex.hpp" 
#include "Edge.hpp" 
#include "Face.hpp" 
#include "HMesh.hpp" 

namespace po = boost::program_options;
using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{
  string ply_file, config_file, metric;
  double fred;
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
      ("metric", po::value(&metric), "metric to make it perfect; choose Heckbert, Melax, Shortest")
      ("face_reduction", po::value(&fred), "amount of faces to throw away")
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
    if(vm.count("input") || !(vm.count("metric")) || !(vm.count("face_reduction")))
    {
      cout << "Some non-optional parameters missing. Check help for a list." << endl;
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
	if(!metric.compare("Heckbert"))
	{
	  while(hame.faces_.size() > fred)
	  {
		  hame.removeHeckBertShit(1);
		  cout << "Size  " << hame.faces_.size() << endl;
	  }
	}
	else if(!metric.compare("Melax"))
	{
	  while(hame.faces_.size() > fred)
	  {
		  hame.removeMelaxShit(1);
		  cout << "Size  " << hame.faces_.size() << endl;
	  }
	}
	else if(!metric.compare("Shortest"))
	{
	  while(hame.faces_.size() > fred)
	  {
		  hame.removeShortestShit(1);
		  cout << "Size  " << hame.faces_.size() << endl;
	  }
	}
	else
	 cout << cmdline_options << "\n";
	   
	//hame.removeShortestShit(20);
	//hame.removeHeckBertShit(20);
	//hame.removeMelaxShit(20);
	//hame.printFaces();
	//for(int i = 1; i < 70; i++)
	//hame.collapseEdge(hame.faces_[hame.faces_.size() - i]->startEdge_);
	//hame.printFaces();
	hame.writeMesh("Mesh.ply");
	cout << "Ready " << endl;
    
  }
  catch(exception& e)
  {
    cout << e.what() << endl;
    return 0;
  }
  
  return 0;
}
