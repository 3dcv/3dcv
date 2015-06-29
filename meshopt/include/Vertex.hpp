#ifndef __VERTEX_HPP__
#define __VERTEX_HPP__


#include <iostream>
#include <vector>
#include "Edge.hpp"

using namespace std;

class Vertex{

    public:

        Vertex()
        {
            x = y = z = 0;
        }

        Vertex(const double  x_, const double y_, const double _z)
        {
            x = _x;
            y = _y;
            z = _z;
        }

        /**
         * @brief    Copy Ctor.
         */
        Vertex(const Vertex &o)
        {
            x = o.x;
            y = o.y;
            z = o.z;
        }

        bool operator<(const Vertex &other) const
        {
            return ( this->x < other.x ) 
                || ( ( this->x - other.x <= 0.00001 )
                    && ( ( this->y < other.y ) 
                        || ( ( this->z < other.z )
                            && ( this->y - other.y <= 0.00001 ) ) ) );
        }

        double operator[](const int &index) const
	{
          switch(index)
          {
            case 0: return x;
            case 1: return y;
            case 2: return z;
          }
       }
            

        double& operator[](const int &index)
        {
          switch(index)
          {
            case 0: return x;
            case 1: return y;
            case 2: return z;
          }
       }


        double x;

        double y;

        double  z;

        vector<Edge*> in_edges;
        vector<Edge*> out_edges;

};


/**
 * @brief    Output operator for vertex types
 */
inline ostream& operator<<(ostream& os, const Vertex v)
{
    os << "Vertex: " << v.x << " " << v.y << " " << v.z << endl;
    return os;
}

#endif
