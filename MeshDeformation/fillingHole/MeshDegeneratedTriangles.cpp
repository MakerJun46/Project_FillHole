#include "stdafx.h"
#include "MeshDegeneratedTriangles.h"

MeshDegeneratedTriangles::MeshDegeneratedTriangles(TriMesh& mesh, double _minAngle, double _maxAngle, double _minArea) :
	m_mesh(mesh), m_minAngle(_minAngle), m_maxAngle(_maxAngle), m_minArea(_minArea) 
{
	findAllDT();
};

double MeshDegeneratedTriangles::getAngle(vec3 v1, vec3 v2, vec3 targetV)
{
	v1 -= targetV;
	v2 -= targetV;

	double innerP = v1 * v2;

	double u = v1.length();
	double v = v2.length();

	double deg = acos(innerP / (u * v));

	return deg * (180 / M_PI);
}

vec3 toVec3DT(TriMesh::Point from)
{
	return vec3(from[0], from[1], from[2]);
}

void MeshDegeneratedTriangles::findAllDT()
{
	m_vecDT.clear();

	vector<TriMesh::FaceHandle> AllFaces;

	for (auto it = m_mesh.faces_begin(); it != m_mesh.faces_end(); it++)
	{
		AllFaces.push_back(it);
	}

	for (auto fh_it = m_mesh.faces_begin(); fh_it != m_mesh.faces_end(); fh_it++)
	{
		DegeneratedTriangle dt_tmp;

		auto vh_it = m_mesh.fv_begin(fh_it);

		auto vh1 = vh_it++;
		auto vh2 = vh_it++;
		auto vh3 = vh_it;

		vec3 v1 = toVec3DT(m_mesh.point(vh1));
		vec3 v2 = toVec3DT(m_mesh.point(vh2));
		vec3 v3 = toVec3DT(m_mesh.point(vh3));

		vector<double> Angles{ getAngle(v2, v3, v1), getAngle(v1, v3, v2), getAngle(v1, v2, v3) };


		if (m_mesh.calc_face_area(fh_it) < m_minArea)
		{
			dt_tmp.m_idx = find(AllFaces.begin(), AllFaces.end(), fh_it) - AllFaces.begin(); // 면적이 기준값 이하인 경우 => SmallArea

			dt_tmp.m_type = SmallArea;

			dt_tmp.m_face = fh_it;

			m_vecDT.push_back(dt_tmp);
		}
		else if (180.0 > *max_element(Angles.begin(), Angles.end()) && *max_element(Angles.begin(), Angles.end()) > m_maxAngle) // 가장 큰 각이 기준값 이상인 경우 => Obtuse
		{
			dt_tmp.m_idx = find(AllFaces.begin(), AllFaces.end(), fh_it) - AllFaces.begin();

			dt_tmp.m_type = Obtuse;

			dt_tmp.m_face = fh_it;

			m_vecDT.push_back(dt_tmp);
		}
		else if (*min_element(Angles.begin(), Angles.end()) < m_minAngle) // 큰 값이 기준값 이하이면서 가장 작은 각이 기준값 이하인 경우 => needle
		{
			dt_tmp.m_idx = find(AllFaces.begin(), AllFaces.end(), fh_it) - AllFaces.begin();

			dt_tmp.m_type = Needle;

			dt_tmp.m_face = fh_it;

			m_vecDT.push_back(dt_tmp);
		}
	}

	sortDTbyIdx();
}

bool MeshDegeneratedTriangles::isMeshHasDT()
{
	return !m_vecDT.empty();
}

void MeshDegeneratedTriangles::printDTInfo()
{
	if(!isMeshHasDT())
	{
		cout << "=== This Mesh has No Degenerated Triangle ===\n";
		return;
	}

	for (DegeneratedTriangle& dt : m_vecDT)
	{
		cout << "===========\n";
		cout << "idx : " << dt.m_idx << endl;

		switch (dt.m_type)
		{
		case SmallArea:
			cout << "type : SmallArea\n";
			break;
		case Obtuse:
			cout << "type : Obtuse\n";
			break;
		case Needle:
			cout << "type : Needle\n";
			break;
		case None:
			cout << "type : error - can't find type\n";
			break;
		default:
			break;
		}

		cout << endl;
	}

	cout << "===========\n";
}

void MeshDegeneratedTriangles::mergeVertex(int targetidx, int mergeidx)
{
	TriMesh::VertexHandle targetVertex = m_mesh.vertex_handle(targetidx);
	TriMesh::VertexHandle mergeVertex = m_mesh.vertex_handle(mergeidx);

	TriMesh::Point p1 = m_mesh.point(targetVertex);
	TriMesh::Point p2 = m_mesh.point(mergeVertex);

	vec3 center_coord((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2, (p1[2] + p2[2]) / 2);

	TriMesh::Point p_new(center_coord[0], center_coord[1], center_coord[2]);

	m_mesh.set_point(mergeVertex, p_new);

	vector<vector<TriMesh::VertexHandle>> vecvecVH_Triangles;

	for (auto it = m_mesh.vf_begin(targetVertex); it != m_mesh.vf_end(targetVertex); it++)	// merge vertices
	{
		TriMesh::FaceHandle fh = it;
		vector<TriMesh::VertexHandle> tmp;
		for (auto it_f = m_mesh.fv_begin(fh); it_f != m_mesh.fv_end(fh); it_f++)
		{
			if (*it_f == targetVertex)
				tmp.push_back(mergeVertex);
			else
				tmp.push_back(*it_f);
		}
		vecvecVH_Triangles.push_back(tmp);
		tmp.clear();
	}

	m_mesh.request_face_status();
	m_mesh.request_edge_status();
	m_mesh.request_vertex_status();

	m_mesh.delete_vertex(targetVertex);

	for (int i = 0; i < vecvecVH_Triangles.size(); i++)
		m_mesh.add_face(vecvecVH_Triangles[i]);

	vecvecVH_Triangles.clear();

	//saveMesh(to_string(targetidx));

	m_mesh.garbage_collection();

	m_mesh.request_face_status();
	m_mesh.request_edge_status();
	m_mesh.request_vertex_status();
}


void MeshDegeneratedTriangles::mergeAllDT()
{
	for (int i = 0; i < m_vecDT.size(); i++)
	{
		TriMesh::FaceHandle fh = m_vecDT[i].m_face;

		vector<TriMesh::VertexHandle> vecvh;

		for (auto v_it = m_mesh.fv_begin(fh); v_it != m_mesh.fv_end(fh); v_it++)
		{
			vecvh.push_back(v_it);
		}

		auto vh_it = m_mesh.fv_begin(fh);

		auto vh1 = vh_it++;
		auto vh2 = vh_it++;
		auto vh3 = vh_it;

		vec3 v1 = toVec3DT(m_mesh.point(vh1));
		vec3 v2 = toVec3DT(m_mesh.point(vh2));
		vec3 v3 = toVec3DT(m_mesh.point(vh3));

		vector<double> Angles{ getAngle(v2, v3, v1), getAngle(v1, v3, v2), getAngle(v1, v2, v3) };

		switch (m_vecDT[i].m_type)
		{
		case SmallArea:
		{
			auto it = m_mesh.fv_begin(fh);
			TriMesh::VertexHandle vh0 = it;
			TriMesh::VertexHandle vh1 = ++it;
			TriMesh::VertexHandle vh2 = ++it;
			mergeVertex(vh0.idx(), vh1.idx());
			mergeVertex(vh1.idx(), vh2.idx());
			break;
		}
		case Obtuse:
		{
			for (int i = 0; i < 3; i++)
			{
				if (Angles[i] > m_maxAngle)
				{
					vecvh.erase(vecvh.begin() + i);
					break;
				}
			}

			mergeVertex(vecvh[0].idx(), vecvh[1].idx());
			break;
		}
		case Needle:
		{
			for (int i = 0; i < 3; i++)
			{
				if (Angles[i] > m_minAngle)
				{
					vecvh.erase(vecvh.begin() + i);
					break;
				}
			}

			mergeVertex(vecvh[0].idx(), vecvh[1].idx());
			break;
		}
		default:
			break;
		}
	}

	m_vecDT.clear();
}

void MeshDegeneratedTriangles::mergeDT(int idx)
{
	DegeneratedTriangle target;
	int vecDTIdx;

	for (int i = 0; i < m_vecDT.size(); i++)
	{
		if (m_vecDT[i].m_idx == idx)
		{
			vecDTIdx = i;
			target = m_vecDT[i];
			break;
		}
	}

	TriMesh::FaceHandle fh = target.m_face;		
	vector<TriMesh::VertexHandle> vecvh;

	for (auto v_it = m_mesh.fv_begin(fh); v_it != m_mesh.fv_end(fh); v_it++)
	{
		vecvh.push_back(v_it);
	}

	auto vh_it = m_mesh.fv_begin(fh);

	auto vh1 = vh_it++;
	auto vh2 = vh_it++;
	auto vh3 = vh_it;

	vec3 v1 = toVec3DT(m_mesh.point(vh1));
	vec3 v2 = toVec3DT(m_mesh.point(vh2));
	vec3 v3 = toVec3DT(m_mesh.point(vh3));

	vector<double> Angles{ getAngle(v2, v3, v1), getAngle(v1, v3, v2), getAngle(v1, v2, v3) };

	switch (target.m_type)
	{
	case SmallArea:
	{
		auto it = m_mesh.fv_begin(fh);
		TriMesh::VertexHandle vh0 = it;
		TriMesh::VertexHandle vh1 = ++it;
		TriMesh::VertexHandle vh2 = ++it;
		mergeVertex(vh0.idx(), vh1.idx());
		mergeVertex(vh1.idx(), vh2.idx());
		break;
	}
	case Obtuse:
	{
		for (int i = 0; i < 3; i++)
		{
			if (Angles[i] > m_maxAngle)
			{
				vecvh.erase(vecvh.begin() + i);
				break;
			}
		}

		mergeVertex(vecvh[0].idx(), vecvh[1].idx());
		break;
	}
	case Needle:
	{
		for (int i = 0; i < 3; i++)
		{
			if (Angles[i] > m_minAngle)
			{
				vecvh.erase(vecvh.begin() + i);
				break;
			}
		}

		mergeVertex(vecvh[0].idx(), vecvh[1].idx());
		break;
	}
	default:
		break;

	}

	m_vecDT.erase(m_vecDT.begin() + vecDTIdx);
}

bool cmp(DegeneratedTriangle& A, DegeneratedTriangle& B)
{
	return A.m_idx < B.m_idx;
}

void MeshDegeneratedTriangles::sortDTbyIdx()
{
	sort(m_vecDT.begin(), m_vecDT.end(), cmp);
}

void MeshDegeneratedTriangles::saveMesh(string name)
{
	m_mesh.garbage_collection();

	try
	{
		if (!OpenMesh::IO::write_mesh(m_mesh, name + ".obj"))
		{
			std::cerr << "Cannot write mesh to file 'output.off'" << std::endl;
		}

		std::cout << "Save Complete" << endl;
	}
	catch (std::exception& x)
	{
		std::cerr << x.what() << std::endl;
	}
}