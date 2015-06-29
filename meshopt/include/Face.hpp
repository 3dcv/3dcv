
#ifndef __FACE_HPP__
#define __FACE_HPP__

#include <vector>
#include <set>

using namespace std;

#include "Vertex.hpp"
#include "Edge.hpp"
class Face
{
public:

	HalfEdgeFace() :
	    startEdge(0),
	    used_(false){};

	~HalfEdgeFace();

	Edge* startEdge_;

	bool	used_;

};

}


#endif
