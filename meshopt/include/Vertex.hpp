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

        Vertex(const double  x_, const double y_, const double z_)
        {
            x = x_;
            y = y_;
            z = z_;
        }

        /**
         * @brief    Copy Ctor.
         */
        Vertex(const Vertex &o)
        {
            x = o.x;
            y = o.y;
            z = o.z;
            in_edges = o.in_edges;
            out_edges = o.out_edges;
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
       
       Vertex operator+(const Vertex& o)
       {
          Vertex v(*this);
          v.x = this->x + o.x;
          v.y = this->y + o.y;
          v.z = this->z + o.z;
          return v;
       }
       Vertex operator*(double factor)
       {
          Vertex v(*this);
          v.x *= factor;
          v.y *= factor;
          v.z *= factor;
          return v;
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
