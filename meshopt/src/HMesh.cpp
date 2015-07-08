#include<HMesh.hpp>

void HMesh::addVertex(Vertex v)
{
  Vertex* v_new = new Vertex(v);
  vertices_.push_back(v_new);
  vert_map_.insert(pair<Vertex*, size_t>(v_new, vertices_.size() - 1));
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
    *v1 = (*v1 + *v2) * 0.5;
    edgy->start = v1;
    if (edgy->face)
    {   
		 if(edgy->next->next->pair != NULL)
		 {
			edgy->next->next->pair->pair = edgy->next->pair;
			if(edgy->next->pair != NULL)
			{
				edgy->next->pair->pair = edgy->next->next->pair;

				Edge* e1 = edgy->next->next;

				Edge* e2 = edgy->next;

				//delete old edges
				deleteEdge(e1, false);    
				deleteEdge(e2, false);
			}
		}
    }
    
    if(edgy->pair != NULL)
    {
		if (edgy->pair->face)
		{
			if(edgy->pair->next->next->pair != NULL)
			{	
				edgy->pair->next->next->pair->pair = edgy->pair->next->pair;
				if(edgy->pair->next->pair != NULL)
				{
					edgy->pair->next->pair->pair = edgy->pair->next->next->pair;
					//delete old edges
				
					Edge* e1 = edgy->pair->next->next;
					Edge* e2 = edgy->pair->next;

					deleteEdge(e1, false);    
					deleteEdge(e2, false);
				}
			}
		}

		// Now really delete faces
		if(edgy->pair->face)
		{
			deleteFace(edgy->pair->face);
			edgy->pair = 0;
		}
	}
	
	if(edgy->face)
	{
		deleteFace(edgy->face);
		edgy->face = 0;
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
    //v2->out_edges.clear();
    //v2->in_edges.clear();
    
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
        
       }
       edgy->pair = 0;
    }
    //delete edgy;
    //edgy = NULL;
} 
void HMesh::deleteFace(Face* f, bool del)
{

    if(f->startEdge_->pair == NULL || f->startEdge_->pair->face == NULL)
    {
        deleteEdge(f->startEdge_);
    }

    if(f->startEdge_->next->pair == NULL || f->startEdge_->next->pair->face == NULL)
    {
        deleteEdge(f->startEdge_->next);
    }

    if(f->startEdge_->next->next->pair == NULL || f->startEdge_->next->next->pair->face == NULL)
    {
        deleteEdge(f->startEdge_->next->next);
    }

    if(del)
    {
        auto it = find(faces_.begin(), faces_.end(), f);
        if(it != faces_.end())
        {
            faces_.erase(it);
        }
    }
}

void HMesh::removeShortestShit(size_t amount)
{
	map<double, Edge*> shortest_edges;
	
	for(int i = 0; i < vertices_.size(); i++)
	{
		for(Edge* edgy : vertices_[i]->out_edges)
		{
			if(edgy != NULL)
				shortest_edges.insert(pair<double, Edge*>((*edgy->end - *edgy->start).length(), edgy));
		}
	}
	collapseEdge(shortest_edges.begin()->second);
	for(int i = 0; i < vertices_.size(); i++)
	{
		auto elem = find(vertices_[i]->out_edges.begin(), vertices_[i]->out_edges.end(), shortest_edges.begin()->second) ;
		if(elem != vertices_[i]->out_edges.end())
		{  
			vertices_[i]->out_edges.erase(elem);
		}
	}
}

void HMesh::removeHeckBertShit(size_t amount)
{
	map<double, pair<int, Edge*> > shortest_edges;
	for(int i = 0; i < vertices_.size(); i++)
	{
		for(Edge* edgy : vertices_[i]->in_edges)
		{
			Vertex* v1 = edgy->start;
			Vertex* v2 = edgy->end;
			Vertex v_neu = (*v1 + *v2) * 0.5;
			Vertex v_b1 = v_neu - *v1;
			Vector3f ev_b1 = Vector3f(v_b1.x, v_b1.y, v_b1.z);
		    double heck_meck = 0;
		    for(int i = 0;i < v1->out_edges.size();i++)
		    {
				Face* plane = v1->out_edges[i]->face;
				if(plane != NULL)
				{
					Vertex s1 = (*plane->startEdge_->end - *plane->startEdge_->start);
					Vertex s2 = (*plane->startEdge_->next->end - *plane->startEdge_->start);
					Vector3f stv1 = Vector3f(s1.x, s1.y, s1.z);
					Vector3f stv2 = Vector3f(s2.x, s2.y, s2.z);
					Vector3f normal = stv1.cross(stv2);
					heck_meck += pow((normal.dot(ev_b1)), 2);
				} 
			}
			Vertex v_b2 = v_neu - *v2;
			Vector3f ev_b2 = Vector3f(v_b2.x, v_b2.y, v_b2.z);
		    for(int i = 0;i < v2->out_edges.size();i++)
		    {
				Face* plane = v2->out_edges[i]->face;
				if(plane != NULL)
				{
					Vertex s1 = (*plane->startEdge_->end - *plane->startEdge_->start);
					Vertex s2 = (*plane->startEdge_->next->end - *plane->startEdge_->start);
					Vector3f stv1 = Vector3f(s1.x, s1.y, s1.z);
					Vector3f stv2 = Vector3f(s2.x, s2.y, s2.z);
					Vector3f normal = stv1.cross(stv2);
					heck_meck += pow((normal.dot(ev_b2)), 2);
				} 
			}
			shortest_edges.insert(pair<double, pair<int, Edge*> >(heck_meck, pair<int, Edge*>(i,edgy)));
		}
	}
	collapseEdge(shortest_edges.begin()->second.second);
	int i = shortest_edges.begin()->second.first;
	auto elem = find(vertices_[i]->in_edges.begin(), vertices_[i]->in_edges.end(), shortest_edges.begin()->second.second) ;
	if(elem != vertices_[i]->in_edges.end())
	{
		vertices_[i]->in_edges.erase(elem);
	}
	elem = find(vertices_[i]->out_edges.begin(), vertices_[i]->out_edges.end(), shortest_edges.begin()->second.second) ;
	if(elem != vertices_[i]->out_edges.end())
	{  
		vertices_[i]->out_edges.erase(elem);
	}
}

void HMesh::removeMelaxShit(size_t amount)
{
	map<double, pair<int, Edge*> > shortest_edges;
	for(int i = 0; i < vertices_.size(); i++)
	{
		for(Edge* edgy : vertices_[i]->in_edges)
		{
			Vertex* u = edgy->start;
			Vertex* v = edgy->end;
			double uv_length = (*v - *u).length();
		    vector<Vector3f> uv_planes;
		    vector<Vector3f> u_planes;
		    for(int i = 0;i < u->out_edges.size();i++)
		    {
				Face* plane = u->out_edges[i]->face;
				if(plane != NULL)
				{
					Vertex s1 = (*plane->startEdge_->end - *plane->startEdge_->start);
					Vertex s2 = (*plane->startEdge_->next->end - *plane->startEdge_->start);
					Vector3f stv1 = Vector3f(s1.x, s1.y, s1.z);
					Vector3f stv2 = Vector3f(s2.x, s2.y, s2.z);
					Vector3f normal = stv1.cross(stv2);
					uv_planes.push_back(normal);
					u_planes.push_back(normal);
				}
			}
		    for(int i = 0;i < v->out_edges.size();i++)
		    {
			    Face* plane = v->out_edges[i]->face;
			    if(plane != NULL)
			    {
					Vertex s1 = (*plane->startEdge_->end - *plane->startEdge_->start);
					Vertex s2 = (*plane->startEdge_->next->end - *plane->startEdge_->start);
					Vector3f stv1 = Vector3f(s1.x, s1.y, s1.z);
					Vector3f stv2 = Vector3f(s2.x, s2.y, s2.z);
					Vector3f normal = stv1.cross(stv2);
					uv_planes.push_back(normal);
				}
			}
			double max_val = 0;
			for(int j = 0; j < u_planes.size(); j++)
			{
				Vector3f normal_a = u_planes[j];
				double min_val = 1000000;
				for(int k = 0; k < uv_planes.size(); k++)
				{
					double step = (1 - (normal_a.dot(uv_planes[k])));
					if(step < min_val)
						min_val = step;
				}
				if(min_val > max_val)
					max_val = min_val;
			}
			double mex_meck = uv_length * max_val;
			shortest_edges.insert(pair<double, pair<int, Edge*> >(mex_meck, pair<int, Edge*>(i,edgy)));
		}
	}
	collapseEdge(shortest_edges.begin()->second.second);
	int i = shortest_edges.begin()->second.first;
	auto elem = find(vertices_[i]->in_edges.begin(), vertices_[i]->in_edges.end(), shortest_edges.begin()->second.second) ;
	if(elem != vertices_[i]->in_edges.end())
	{
		vertices_[i]->in_edges.erase(elem);
	}
	elem = find(vertices_[i]->out_edges.begin(), vertices_[i]->out_edges.end(), shortest_edges.begin()->second.second) ;
	if(elem != vertices_[i]->out_edges.end())
	{  
		vertices_[i]->out_edges.erase(elem);
	}
}

void HMesh::writeMesh(string file_name)
{
	PolygonMesh triangles;
	PointCloud<PointXYZ> cloud;
	for(int i = 0; i < vertices_.size(); i++)
		cloud.push_back(PointXYZ(vertices_[i]->x, vertices_[i]->y, vertices_[i]->z));
	toPCLPointCloud2(cloud, triangles.cloud);
    for(int i = 0; i < faces_.size(); i++)
    {
		pcl::Vertices triangle;
		triangle.vertices.push_back(vert_map_[faces_[i]->startEdge_->start]);
		triangle.vertices.push_back(vert_map_[faces_[i]->startEdge_->end]);
		triangle.vertices.push_back(vert_map_[faces_[i]->startEdge_->next->end]);
		triangles.polygons.push_back(triangle);
	}
	pcl::io::savePLYFile(file_name,triangles); 
}
