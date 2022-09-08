	for (auto it = m_mesh.vf_begin(vecVH_PointPair[closeVertexIndexList[i]].first); it != m_mesh.vf_end(vecVH_PointPair[closeVertexIndexList[i]].first); it++)	// merge vertices
			{
				TriMesh::FaceHandle fh = it;
				vector<TriMesh::VertexHandle> vecVH_triangle;
				for (auto it_fv = m_mesh.fv_begin(fh); it_fv != m_mesh.fv_end(fh); it_fv++)
				{
					if (*it_fv == vecVH_PointPair[closeVertexIndexList[i]].first) ///???
						vecVH_triangle.push_back(vecVH_PointPair[newVertexIndex].first);
					else
						vecVH_triangle.push_back(*it_fv);
				}
				vecvecVH_Triangles.push_back(vecVH_triangle);
				vecVH_triangle.clear();
				//m_mesh.delete_face(it, false);
			}
			m_mesh.delete_vertex(vecVH_PointPair[closeVertexIndexList[i]].first);
			//deleteOneRingTriangles(vecVH_PointPair[closeVertexIndexList[i]].first);
			//vecVH_PointPair.erase(vecVH_PointPair.begin() + closeVertexIndexList[i]);
			addFaces(vecvecVH_Triangles);
			vecvecVH_Triangles.clear();