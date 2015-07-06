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
	//for(int i = 0; i < faces_.size();i++)
	//{
    int index = faces_.size() - 1; 
    
    cout << "edge 1: " << faces_[index]->startEdge_ << endl;
    cout << "edge 2: " << faces_[index]->startEdge_->next << endl;
    cout << "edge 3: " << faces_[index]->startEdge_->next->next << endl;
    
    cout << "edge 1 pair: " << faces_[index]->startEdge_->pair << endl;
	  
		cout << "face: " << faces_.size() - 1 << endl;
		cout << "vert 1: " << *faces_[index]->startEdge_->start << endl;
		cout << "vert 2: " << *faces_[index]->startEdge_->next->start << endl;
		cout << "vert 3: " << *faces_[index]->startEdge_->next->next->start << endl;
	
    //}
}

void HMesh::collapseEdge(Edge* edgy)
{
    Vertex* v1 = edgy->start;
    Vertex* v2 = edgy->end;

    if(v1 == v2) return;
    cout << "v1 " << *v1 << " v2 " << *v2 << endl;
    *v1 = (*v1 + *v2) * 0.5;
    edgy->start = v1;
    cout << "face test: " << edgy->face << endl;    
    if (edgy->face)
    {   
         cout << "in edgy->face" << endl;
         edgy->next->next->pair->pair = edgy->next->pair;
         edgy->next->pair->pair = edgy->next->next->pair;

        Edge* e1 = edgy->next->next;

        Edge* e2 = edgy->next;

        //delete old edges
        deleteEdge(e1, false);    
        deleteEdge(e2, false);
    }
    
    if (edgy->pair->face)
    {
         cout << "in edgy->pair->face" << endl;
        edgy->pair->next->next->pair->pair = edgy->pair->next->pair;
        edgy->pair->next->pair->pair = edgy->pair->next->next->pair;
        //delete old edges

        Edge* e1 = edgy->pair->next->next;
        Edge* e2 = edgy->pair->next;

        deleteEdge(e1, false);    
        deleteEdge(e2, false);
    }

    // Now really delete faces
    if(edgy->pair->face)
    {
        cout << "in deleting pair face" << endl;
        deleteFace(edgy->pair->face);
        edgy->pair = 0;
    }

    if(edgy->face)
    {
        cout << "in deleting face" << endl;
        deleteFace(edgy->face);
    }

    deleteEdge(edgy);

    auto it =v2->out_edges.begin();

    while(it !=v2->out_edges.end())
    {
        (*it)->start = v1;
        v1->out_edges.push_back(*it);
        it++;
    }

    auto it2 = v2->in_edges.begin();
    while(it2 != v2->in_edges.end())
    {
        (*it2)->end = v1;
        v1->in_edges.push_back(*it2);
        it2++;
    }
    cout << *v1 << endl;
    cout << *v2 << endl;
}

void HMesh::deleteEdge(Edge* edgy, bool del)
{
  
   if(edgy->start != NULL)
   {
      //delete references from start point to outgoing edge
        auto it = find(edgy->start->out_edges.begin(), edgy->start->out_edges.end(), edgy);
        if(it != edgy->start->out_edges.end())
        {
            edgy->start->out_edges.erase(it);
        }
        
        it = find(edgy->end->in_edges.begin(), edgy->end->in_edges.end(), edgy);
        if(it !=  edgy->end->in_edges.end())
        {
            edgy->end->in_edges.erase(it);
        }
    }
    if(del)
    {
        
       if(edgy->pair != NULL)
       {
           //delete references from start point to outgoing edge
            auto it = find(edgy->pair->start->out_edges.begin(), edgy->pair->start->out_edges.end(), edgy->pair);
            if(it != edgy->pair->start->out_edges.end())
            {
                edgy->pair->start->out_edges.erase(it);
            }
            it = find(edgy->pair->end->in_edges.begin(), edgy->pair->end->in_edges.end(), edgy->pair);
            if(it != edgy->pair->end->in_edges.end())
            {
                edgy->pair->end->in_edges.erase(it);
            }
            edgy->pair->pair = 0;
        edgy->pair = 0;
       }
    }
} 
void HMesh::deleteFace(Face* f, bool del)
{

    /*if(f->startEdge_->pair->face == NULL)
    {
        deleteEdge(f->startEdge_);
    }

    if(f->startEdge_->next->pair->face == NULL)
    {
        deleteEdge(f->startEdge_->next);
    }

    if(f->startEdge_->next->next->pair->face == NULL)
    {
        deleteEdge(f->startEdge_->next->next);
    }*/

    if(del)
    {
        auto it = find(faces_.begin(), faces_.end(), f);
        if(it != faces_.end())
        {
            faces_.erase(it);
        }
    }
}
