#include <Octree.hpp>

using namespace std;
using namespace pcl;

Octree::Octree(string &cloud_file, double octsize, DistanceFunction dfunk): dfunk_(dfunk)
{
  PLYReader ply_reader;
  ply_reader.read(cloud_file, cloud_);
  voxel_size_ = octsize;
  onode root;
  PointXYZ minp, maxp;
  getMinMax3D(cloud_, minp, maxp);
  root.points.resize(cloud_.width);
  root.center = PointXYZ((maxp.x + minp.x)/2, (maxp.y + minp.y)/2, (maxp.z + minp.z)/2);
  root.size = max(max(maxp.x - minp.x, maxp.y - minp.y), maxp.z - minp.z);
  for(int i=0; i<cloud_.width; i++)
  {
    root.points[i] = i;
  }
  nodes_.push_back(root);
  divide(0);

}

Octree::~Octree()
{}

void Octree::reconstruct()
{
  cout << "Nodes: " << nodes_.size() << endl;
  int validcounter = 0;
  size_t pointcount = 0;
  size_t validconfig = 0;
  for(int i=0; i<nodes_.size(); i++)
  {
    if(nodes_[i].valid == 0)
    {
      double size = nodes_[i].size;
      pointcount += nodes_[i].points.size();
      vector<Vector3d> corners(8);
      vector<double> corner_distances(8);
      PointXYZ corner;
      corner.x = nodes_[i].center.x + size/2;
      corner.y = nodes_[i].center.y + size/2;
      corner.z = nodes_[i].center.z + size/2;
      corner_distances[6] = dfunk_.distance(corner.x, corner.y, corner.z); // +++
      corners[6]= Vector3d(corner.x, corner.y, corner.z);
      corner.y = nodes_[i].center.y - size/2; 
      corner_distances[5] = dfunk_.distance(corner.x, corner.y, corner.z); // +-+
      corners[5]= Vector3d(corner.x, corner.y, corner.z);
      corner.x = nodes_[i].center.x - size/2; 
      corner_distances[4] = dfunk_.distance(corner.x, corner.y, corner.z); // --+
      corners[4]= Vector3d(corner.x, corner.y, corner.z);
      corner.z = nodes_[i].center.z - size/2; 
      corner_distances[0] = dfunk_.distance(corner.x, corner.y, corner.z); // ---
      corners[0]= Vector3d(corner.x, corner.y, corner.z);
      corner.x = nodes_[i].center.x + size/2; 
      corner_distances[1] = dfunk_.distance(corner.x, corner.y, corner.z); // +--
      corners[1]= Vector3d(corner.x, corner.y, corner.z);
      corner.y = nodes_[i].center.y + size/2; 
      corner_distances[2] = dfunk_.distance(corner.x, corner.y, corner.z); // ++-
      corners[2]= Vector3d(corner.x, corner.y, corner.z);
      corner.x = nodes_[i].center.x - size/2; 
      corner_distances[3] = dfunk_.distance(corner.x, corner.y, corner.z); // -+-
      corners[3]= Vector3d(corner.x, corner.y, corner.z);
      corner.z = nodes_[i].center.z + size/2; 
      corner_distances[7] = dfunk_.distance(corner.x, corner.y, corner.z); // -++
      corners[7]= Vector3d(corner.x, corner.y, corner.z);
      unsigned char corner_config = 0;
      for(int j = 0; j < 8; j++)
      {
        if(corner_distances[j] > 0)
        {
          corner_config |= (1 << j);
        }
      }
      if(corner_config != 0 && corner_config != 255)
        validconfig++;
      const int* edges = lssr::MCTable[corner_config];
      
      vector<Vector3d> edge_intersections(12);
      for(int k = 0; k < 7;k++)
      {
        edge_intersections[k] = (corners[k] + corners[k+1])/2;
      }
      edge_intersections[3] = (corners[0] + corners[3])/2;       
      edge_intersections[7] = (corners[7] + corners[4])/2;       
      edge_intersections[8] = (corners[0] + corners[4])/2;       
      edge_intersections[9] = (corners[1] + corners[5])/2;       
      edge_intersections[10] = (corners[3] + corners[7])/2;       
      edge_intersections[11] = (corners[2] + corners[6])/2;       
      for(int g = 0; g < 13; g++)
      {
        if(edges[g] == -1)
          break;
         verts_.push_back(edge_intersections[edges[g]]);
      }
      validcounter++;
    }    
  }
  cout << "valid nodes: " << validcounter << endl;
  cout << "valid points: " << pointcount << endl;
  cout << "valid config: " << validconfig << endl;
  cout << "vertices : " << verts_.size() << endl;
  writePLY("shit.ply");
}

void Octree::divide(int nc)
{
  if(nodes_[nc].size <= voxel_size_) 
  {
    nodes_[nc].valid = 0;
    return;
  }
  double new_size = nodes_[nc].size/2;
  vector<vector<int> > voxels(8);
  for(int i = 0; i < nodes_[nc].points.size(); i++)
  {
    int pi = nodes_[nc].points[i];
    if(cloud_[pi].x < nodes_[nc].center.x)
    {
      if(cloud_[pi].y < nodes_[nc].center.y)
      {
        if(cloud_[pi].z < nodes_[nc].center.z)
          voxels[0].push_back(pi);
        else
          voxels[1].push_back(pi);
      }
      else
      { 
        if(cloud_[pi].z < nodes_[nc].center.z)
          voxels[2].push_back(pi);
        else
          voxels[3].push_back(pi);
      }
    }
    else
    {
      if(cloud_[pi].y < nodes_[nc].center.y)
      {
        if(cloud_[pi].z < nodes_[nc].center.z)
          voxels[4].push_back(pi);
        else
          voxels[5].push_back(pi);
      }
      else
      {
        if(cloud_[pi].z < nodes_[nc].center.z)
          voxels[6].push_back(pi);
        else
          voxels[7].push_back(pi);
      }
    } 
  }
  for(int i = 0; i < 8; i++)
  { 
    onode new_node;
    if(voxels[i].size() == 0) 
      nodes_[nc].valid |= (0 << i);
    else
      nodes_[nc].valid |= (1 << i);
    if(i < 4)
      new_node.center.x = nodes_[nc].center.x - nodes_[nc].size/4;
    else
      new_node.center.x = nodes_[nc].center.x + nodes_[nc].size/4;
    if(i == 0 || i == 1 || i == 4 || i == 5)
      new_node.center.y = nodes_[nc].center.y - nodes_[nc].size/4;
    else
      new_node.center.y = nodes_[nc].center.y + nodes_[nc].size/4; 
    if(i == 0 || i == 2 || i == 4 || i == 6)
      new_node.center.z = nodes_[nc].center.z - nodes_[nc].size/4;
    else
      new_node.center.z = nodes_[nc].center.z + nodes_[nc].size/4;
    new_node.points = voxels[i];
    new_node.size = new_size;
    if(voxels[i].size() > 0)
    {
      nodes_[nc].children.push_back(nodes_.size());
      nodes_.push_back(new_node);
      divide(nodes_.size() - 1);
    }
  }

}


int Octree::writePLY(string ply_string)
{
  if(ply_string.substr(ply_string.size()-4, 4) != ".ply")
  {
    ply_string.append(".ply");
  }
  cout << "Creating file " << ply_string << "." << endl;
  ofstream ofst(ply_string.c_str(), ios::out | ios::trunc);
  if(!ofst.is_open())
  {
    cout << "Could not create file at specified location." << endl;
    return -1;
  }
  
  // Write header
  ofst << "ply\nformat ascii 1.0\nelement vertex " << verts_.size() << "\n"; 
  ofst << "property float x\nproperty float y\nproperty float z\n"; 
  ofst << "element face " << verts_.size()/3 << "\n";
  ofst << "property list uchar int vertex_index\n" << "end_header\n"; 

  for(int i=0; i<verts_.size(); i++)
  {
    ofst << verts_.at(i)[0] << " " << verts_.at(i)[1] << " " << verts_.at(i)[2] << "\n";
  }
  for(int i=0; i < verts_.size(); i+=3)
  {
    ofst << "3 " <<  i  << " " << i + 1  << " " << i + 2 << "\n";
  }
  ofst.close();
}
