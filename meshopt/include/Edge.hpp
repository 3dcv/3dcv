#ifndef __EDGE_H__
#define __EDGE_H__

#include <Vertex.hpp>
#include <Face.hpp>
class Vertex;
class Edge
{

public:

	Edge() : used(false), start(0), end(0), face(0), next(0), pair(0)
        {
        } 
	
    Edge* next;

	Edge* pair;

	Vertex* start;

	Vertex* end;

	Face* face;

	bool used;
};

#endif
