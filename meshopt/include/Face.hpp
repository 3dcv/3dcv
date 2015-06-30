
#ifndef __FACE_HPP__
#define __FACE_HPP__

#include <vector>
#include <set>

using namespace std;

#include "Vertex.hpp"
#include "Edge.hpp"
class Edge;
class Face 
{
public:

	Face() :
	    startEdge_(0),
	    used_(false){};

	~Face();

	Edge* startEdge_;

	bool	used_;

};



#endif
