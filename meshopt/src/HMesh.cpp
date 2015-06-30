#include<HMesh.hpp>

void HMesh::addVertex(Vertex v)
{
  vertices_.push_back(new Vertex(v));
  vertexIndex_++;
}

void HMesh::addTriangle(size_t one, size_t two, size_t three)
{
  Face* face = new Face();
  faces_.push_back(face);
  Edge* edges[3];
  for(int i = 0; i < 3; i++)
  {
    Vertex* cur;
    Vertex* next;
    if(i == 0)
    {
       cur = vertices_[one];
       next = vertices_[two];
    }
    else if(i == 1)
    {
       cur = vertices_[two];
       next = vertices_[three];
    }
    else
    {
       cur = vertices_[three];
       next = vertices_[one];
    }
    Edge* edgeVertex = 0;
    for(int j = 0; j < cur->in_edges.size();j++)
    {
       Edge* cur_edge = cur->in_edges[j];
       if(cur_edge->end == cur && cur_edge->start == next)
       {
         edgeVertex = cur_edge;
       }
    }
    if(edgeVertex)
    {
      if(edgeVertex->pair != NULL)
        edges[i] = edgeVertex->pair;
      else
      {
        Edge* new_edge = new Edge();
        new_edge->start = edgeVertex->end;
        new_edge->end = edgeVertex->start;
        edges[i] = new_edge;
      }
      edges[i]->face = face;
    }
    else
    {
      Edge* new_edge = new Edge();
      new_edge->face= face;
      new_edge->start = cur;
      new_edge->end = next;

      Edge* pair = new Edge();
      pair->start = next;
      pair->end = cur;
      pair->face = 0;

      // Link Half edges
      new_edge->pair = pair;
      pair->pair = new_edge;

      // Save outgoing edge
      cur->out_edges.push_back(new_edge);
      next->in_edges.push_back(new_edge);

      // Save incoming edges
      cur->in_edges.push_back(pair);
      next->out_edges.push_back(pair);

      // Save pointer to new edge
      edges[i] = new_edge;
    }
  }
  for(int k = 0; k < 3; k++)
  {
        edges[k]->next = edges[(k + 1) % 3];
  }

  face->startEdge_ = edges[0];
}



void HMesh::findEdgeNeighbors(size_t face_ind, vector<Face*>& neighbors_inds)
{
  Edge* edgeptr = faces_[face_ind]->startEdge_;
  neighbors_inds.push_back(edgeptr->pair->face);
  for(int i = 0; i < 2; i++)
  {
    edgeptr = edgeptr->next;
    neighbors_inds.push_back(edgeptr->pair->face);
  }
}


void HMesh::findVertexNeighbors(size_t vert_ind, vector<Face*>& neighbors_inds)
{
  auto in_edges = vertices_[vert_ind]->in_edges;
  for(int i = 0; i < in_edges.size(); i++)
  {
    neighbors_inds.push_back(in_edges[i]->face);
  }
  auto out_edges = vertices_[vert_ind]->out_edges;
  for(int i = 0; i < out_edges.size(); i++)
  {
    neighbors_inds.push_back(out_edges[i]->face);
  }
}

void HMesh::printFaces()
{
	for(int i = 0; i < faces_.size();i++)
	{
		cout << "face: " << i << endl;
		cout << "vert 1: " << *faces_[i]->startEdge_->start << endl;
		cout << "vert 2: " << *faces_[i]->startEdge_->next->start << endl;
		cout << "vert 3: " << *faces_[i]->startEdge_->next->next->start << endl;
	}
}
