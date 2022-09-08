
#include "stdafx.h"
#include "MeshDegeneratedTriangles.h"
#include <quadrics_src/algebra3.h>
#include "AdvancingFrontMesh.cpp"

MeshDegeneratedTriangles::MeshDegeneratedTriangles(TriMesh& mesh, double _minAngle, double _maxAngle, double _minArea)
{
	m_mesh = mesh;
	minAngle = _minAngle;
	maxAngle = _maxAngle;
	minArea = _minArea;
}

double MeshDegeneratedTriangles::Angle(TriMesh::VertexHandle vh)
{
	double deg;

	TriMesh::HalfedgeHandle prev_halfedge = m_mesh.prev_halfedge_handle(m_mesh.halfedge_handle(vh));

	deg = m_mesh.calc_sector_angle(prev_halfedge) * 180 / M_PI;

	if (deg < 0)
	{
		deg += 360;
	}

	return deg;
}

void MeshDegeneratedTriangles::FindAllDT()
{
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

		vector<double> Angles{ Angle(vh1), Angle(vh2), Angle(vh3) };


		if (m_mesh.calc_face_area(fh_it) < minArea)
		{
			dt_tmp.idx = find(AllFaces.begin(), AllFaces.end(), fh_it) - AllFaces.begin(); // 면적이 기준값 이하인 경우 => SmallArea

			dt_tmp.type = SmallArea;

			dt_tmp.face = fh_it;

			vecDT.push_back(dt_tmp);
		}
		else if (*max_element(Angles.begin(), Angles.end()) > maxAngle) // 가장 큰 각이 기준값 이상인 경우 => Obtuse
		{
			dt_tmp.idx = find(AllFaces.begin(), AllFaces.end(), fh_it) - AllFaces.begin();

			dt_tmp.type = Obtuse;

			dt_tmp.face = fh_it;

			vecDT.push_back(dt_tmp);
		}
		else if (*min_element(Angles.begin(), Angles.end()) < minAngle) // 큰 값이 기준값 이하이면서 가장 작은 각이 기준값 이하인 경우 => needle
		{
			dt_tmp.idx = find(AllFaces.begin(), AllFaces.end(), fh_it) - AllFaces.begin();

			dt_tmp.type = Needle;

			dt_tmp.face = fh_it;

			vecDT.push_back(dt_tmp);
		}
	}

	SortDTbyIdx();
}

void MeshDegeneratedTriangles::PrintDTInfo()
{
	for (DegeneratedTriangle& dt : vecDT)
	{
		cout << "idx : " << dt.idx << endl;

		switch (dt.type)
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
}

void MeshDegeneratedTriangles::MergeVertex(int targetidx, int mergeidx)
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

	//fillTrianglesAfterMerge(m_mesh, mergeVertex);

	//m_mesh.garbage_collection();

	m_mesh.request_face_status();
	m_mesh.request_edge_status();
	m_mesh.request_vertex_status();
}


void MeshDegeneratedTriangles::MergeAllDT()
{
	for (int i = 0; i < vecDT.size(); i++)
	{
		TriMesh::FaceHandle fh = vecDT[i].face;
		vector<TriMesh::VertexHandle> vecvh(m_mesh.fv_begin(fh), m_mesh.fv_end(fh));

		switch (vecDT[i].type)
		{
		case SmallArea:
		{
			auto it = m_mesh.fv_begin(fh);
			TriMesh::VertexHandle vh0 = it;
			TriMesh::VertexHandle vh1 = ++it;
			TriMesh::VertexHandle vh2 = ++it;
			MergeVertex(vh0.idx(), vh1.idx());
			MergeVertex(vh1.idx(), vh2.idx());
			break;
		}
		case Obtuse:
		{
			for (auto it = m_mesh.fv_begin(fh); it != m_mesh.fv_end(fh); it++)
			{
				if (Angle(it) > maxAngle)
				{
					vecvh.erase(find(vecvh.begin(), vecvh.end(), it));
					break;
				}
			}

			MergeVertex(vecvh[0].idx(), vecvh[1].idx());
			break;
		}
		case Needle:
		{
			for (auto it = m_mesh.fv_begin(fh); it != m_mesh.fv_end(fh); it++)
			{
				if (Angle(it) < minAngle)
				{
					vecvh.erase(find(vecvh.begin(), vecvh.end(), it));
					break;
				}
			}

			MergeVertex(vecvh[0].idx(), vecvh[1].idx());
			break;
		}
		default:
			break;
		}
	}

	vecDT.clear();
}

void MeshDegeneratedTriangles::MergeDT(int idx)
{
	DegeneratedTriangle target;
	int vecDTIdx;

	for (int i = 0; i < vecDT.size(); i++)
	{
		if (vecDT[i].idx == idx)
		{
			vecDTIdx = i;
			target = vecDT[i];
			break;
		}
	}

	TriMesh::FaceHandle fh = target.face;
	vector<TriMesh::VertexHandle> vecvh(m_mesh.fv_begin(fh), m_mesh.fv_end(fh));

	switch (target.type)
	{
	case SmallArea:
	{
		auto it = m_mesh.fv_begin(fh);
		TriMesh::VertexHandle vh0 = it;
		TriMesh::VertexHandle vh1 = ++it;
		TriMesh::VertexHandle vh2 = ++it;
		MergeVertex(vh0.idx(), vh1.idx());
		MergeVertex(vh1.idx(), vh2.idx());
		break;
	}
	case Obtuse:
	{
		for (auto it = m_mesh.fv_begin(fh); it != m_mesh.fv_end(fh); it++)
		{
			if (Angle(it) > maxAngle)
			{
				vecvh.erase(find(vecvh.begin(), vecvh.end(), it));
				break;
			}
		}

		MergeVertex(vecvh[0].idx(), vecvh[1].idx());
		break;
	}
	case Needle:
	{
		for (auto it = m_mesh.fv_begin(fh); it != m_mesh.fv_end(fh); it++)
		{
			if (Angle(it) < minAngle)
			{
				vecvh.erase(find(vecvh.begin(), vecvh.end(), it));
				break;
			}
		}

		MergeVertex(vecvh[0].idx(), vecvh[1].idx());
		break;
	}
	default:
		break;
	
	}

	vecDT.erase(vecDT.begin() + vecDTIdx);
}

bool cmp(DegeneratedTriangle& A, DegeneratedTriangle& B)
{
	return A.idx < B.idx;
}

void MeshDegeneratedTriangles::SortDTbyIdx()
{
	sort(vecDT.begin(), vecDT.end(), cmp);
}