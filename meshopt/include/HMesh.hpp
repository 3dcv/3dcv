#ifndef __HMESH_HPP__
#define __HMESH_HPP__

#include <boost/unordered_map.hpp>

#include <vector>
#include <map>
#include <sstream>
#include <float.h>
#include <math.h>
#include <algorithm>
using namespace std;

#include "Vertex.hpp"
#include "Edge.hpp"
#include "Face.hpp"
#include <queue>
class HMesh
{
public:

	HMesh(){};

	/**
	 * @brief   Dtor.
	 */
	~HMesh(){};

	void addVertex(Vertex v);
	
	void addTriangle(size_t one, size_t two, size_t three);
  
  void findEdgeNeighbors(size_t face_ind, vector<Face*>& neighbors_inds);
  void findVertexNeighbors(size_t vert_ind, vector<Face*>& neighbors_inds);
  void collapseEdge(Edge* edgy);
  void printFaces();
  void deleteFace(Face* f, bool del = true);
  void deleteEdge(Edge* edgy, bool del = true);

	vector<Face*>                                 faces_;

	vector<Vertex*>                               vertices_;

	size_t                                        vertexIndex_;

  priority_queue<Face*>                         p_q_;

};

#endif 
