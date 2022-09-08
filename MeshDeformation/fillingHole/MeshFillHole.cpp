#include "stdafx.h"
#pragma warning(disable:4351)
#include "FillHole.h"
#include "../MeshDisplay/MyTriMesh.h"
#include <iostream>

// -------------------- OpenMesh
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
 // -------------------- OpenMesh

using namespace std;

extern "C" { FILE __iob_func[3] = { *stdin,*stdout,*stderr }; }

void saveMesh(TriMesh mesh, string name, bool resultSave)
{
	//return;
	try
	{
		if (resultSave)
		{
			if (!OpenMesh::IO::write_mesh(mesh, "Result/" + name + ".obj"))
			{
				std::cerr << "Cannot write mesh to file 'output.off'" << std::endl;
			}
		}
		else
		{
			if (!OpenMesh::IO::write_mesh(mesh, "Log/" + name + ".obj"))
			{
				std::cerr << "Cannot write mesh to file 'output.off'" << std::endl;
			}
		}

		std::cout << "Save Complete" << endl;
	}
	catch (std::exception& x)
	{
		std::cerr << x.what() << std::endl;
	}
}
double calcBoundaryAngle(TriMesh& mesh, TriMesh::VertexHandle vh)
{
	int idx = vh.idx();

	TriMesh::HalfedgeHandle prev_halfedge = mesh.prev_halfedge_handle(mesh.halfedge_handle(vh));

	double deg = mesh.calc_sector_angle(prev_halfedge) * 180 / M_PI;

	if (deg < 0) {
		deg += 360;
		assert(0);
	}

	return deg;
}
bool isOtherHoleVertex(TriMesh::VertexHandle vh, int targetHoleIndex, vector<Hole> vecHoles)
{
	for (int i = 0; i < vecHoles.size(); i++)
	{
		if (i == targetHoleIndex)
			continue;

		for (auto v : vecHoles[i].m_newVertices)
		{
			if (vh == v)
				return true;
		}
	}

	return false;
}
bool isOtherHoleFace(TriMesh::FaceHandle fh, int targetHoleIndex, vector<Hole> vecHoles)
{
	for (int i = 0; i < vecHoles.size(); i++)
	{
		if (i == targetHoleIndex)
			continue;

		for (auto f : vecHoles[i].m_newFaces)
		{
			if (fh == f)
				return true;
		}
	}

	return false;
}
vector<TriMesh::VertexHandle> findNewVertices(TriMesh& mesh, Hole hole, vector<Hole> vecHoles, map<int, bool> m_isOriginalVertex)
{
	vector<TriMesh::VertexHandle> newVertexHandles;

	for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); it++)
	{
		if (m_isOriginalVertex[it->idx()] || isOtherHoleVertex(it, hole.m_holeNumber, vecHoles))
			continue;
		else
			newVertexHandles.push_back(*it);
	}

	return newVertexHandles;
}
vector<TriMesh::FaceHandle> findNewFaces(TriMesh& mesh, Hole hole, vector<Hole> vecHoles, map<int, bool> m_isOriginalFace)
{
	vector<TriMesh::FaceHandle> newFaceHandles;

	for (auto it = mesh.faces_begin(); it != mesh.faces_end(); it++)
	{
		if (m_isOriginalFace[it->idx()] || isOtherHoleFace(it, hole.m_holeNumber, vecHoles))
			continue;
		else
			newFaceHandles.push_back(*it);
	}

	return newFaceHandles;
}
void statusSettings(TriMesh& mesh)
{
	mesh.request_face_status();
	mesh.request_edge_status();
	mesh.request_vertex_status();
	mesh.request_halfedge_status();

	mesh.request_face_normals();
	mesh.request_vertex_normals();
	mesh.update_normals();
}
void eraseFaceHasTwoHalfEdgeBoundary(TriMesh& mesh)
{

	cout << "eraseTwoBoundaryTriangles start\n";
	statusSettings(mesh);

	vector<TriMesh::FaceHandle> vecFh;
	bool justDeleted = true;
	do {
		vecFh.clear();
		for (auto it = mesh.faces_begin(); it != mesh.faces_end(); it++)
		{
			int cnt = 0;

			for (auto it_he = mesh.fh_begin(it); it_he != mesh.fh_end(it); it_he++)
			{
				if (mesh.is_boundary(mesh.opposite_halfedge_handle(it_he)))
				{
					cnt++;
				}
			}

			if (cnt > 1)
			{
				vecFh.push_back(it);
			}
		}

		for (int i = 0; i < vecFh.size(); i++)
		{
			mesh.delete_face(vecFh[i]);
		}
		mesh.garbage_collection();
	} while (vecFh.size() > 0);
	cout << "eraseTwoBoundaryTriangles end\n";

	//saveMesh(mesh, std::string("afterDeleteTwoSided"), false);
}

vec3 toVec(TriMesh::Point from)
{
	vec3 ret;
	ret[0] = from[0];
	ret[1] = from[1];
	ret[2] = from[2];
	return ret;
}

extern double calcAngle(vec3 v1, vec3 targetV, vec3 v2, vec3 targetNormal);

void FillHole::setBoundaryAngles(vector<pair<TriMesh::VertexHandle, TriMesh::Point>>& vecVH_PointPair, int& minAngleVertexIndex, vector<double>& angles)
{
	float minAngle = 360;
	angles.clear();
	m_mesh.request_face_normals();

	for (int i = 0; i < vecVH_PointPair.size(); i++)
	{
		//double angle = calcBoundaryAngle(m_mesh, vecVH_PointPair[i].first);

		double angle;

		if (i == vecVH_PointPair.size() - 1)
			angle = calcAngle(toVec(vecVH_PointPair[i - 1].second), toVec(vecVH_PointPair[i].second), toVec(vecVH_PointPair[0].second), toVec(m_mesh.normal(vecVH_PointPair[i].first)));
		else if(i == 0)
			angle = calcAngle(toVec(vecVH_PointPair[vecVH_PointPair.size() - 1].second), toVec(vecVH_PointPair[i].second), toVec(vecVH_PointPair[i + 1].second), toVec(m_mesh.normal(vecVH_PointPair[i].first)));
		else
			angle = calcAngle(toVec(vecVH_PointPair[i - 1].second), toVec(vecVH_PointPair[i].second), toVec(vecVH_PointPair[i + 1].second), toVec(m_mesh.normal(vecVH_PointPair[i].first)));

		angles.push_back(angle);
		if (angle < minAngle) {
			minAngle = angle;
			minAngleVertexIndex = i;
		}
	}
}
vector<TriMesh::VertexHandle> FillHole::originalBoundaryVertexSort(vector<TriMesh::VertexHandle>& vecVhBoundary, vector<TriMesh::FaceHandle>& vecFhBoundary)
{
	vector<TriMesh::VertexHandle> holeVertices;

	holeVertices.push_back(vecVhBoundary[0]);
	auto startVertex = vecVhBoundary[0];
	auto nowVertex = vecVhBoundary[0];
	TriMesh::HalfedgeHandle connectedHalfedge = m_mesh.halfedge_handle(nowVertex);
	TriMesh::VertexHandle nextVertex = m_mesh.to_vertex_handle(connectedHalfedge);
	vecVhBoundary.erase(vecVhBoundary.begin());

	while (true)
	{
		auto target_it = find(vecVhBoundary.begin(), vecVhBoundary.end(), nextVertex);
		vecFhBoundary.push_back(m_mesh.face_handle(m_mesh.opposite_halfedge_handle(connectedHalfedge)));

		if (target_it == vecVhBoundary.end() && nextVertex == startVertex)
		{
			break;
		}
		else
		{
			holeVertices.push_back(*target_it);
			nowVertex = *target_it;

			connectedHalfedge = m_mesh.halfedge_handle(nowVertex);
			nextVertex = m_mesh.to_vertex_handle(connectedHalfedge);

			vecVhBoundary.erase(target_it);
		}
	}

	return holeVertices;
}
void FillHole::setIsOriginalVertexArray()
{
	for (auto it = m_mesh.vertices_begin(); it != m_mesh.vertices_end(); it++)
	{
		m_isOriginalVertex[it->idx()] = true;
	}
}
void FillHole::setIsOriginalFaceArray()
{
	for (auto it = m_mesh.faces_begin(); it != m_mesh.faces_end(); it++)
	{
		m_isOriginalFace[it->idx()] = true;
	}
}
void FillHole::findHole()
{
	vector<TriMesh::VertexHandle> vecVhBoundary;
	vector<TriMesh::FaceHandle> vecFhBoundary;
	vector<TriMesh::VertexHandle> vecVhTmp;
	int hole_number = 0;

	for (auto it = m_mesh.vertices_begin(); it != m_mesh.vertices_end(); it++)
	{
		if (m_mesh.is_boundary(it))
			vecVhBoundary.push_back(*it);
	}

	while (!vecVhBoundary.empty())
	{
		vecFhBoundary.clear();
		vecVhTmp = originalBoundaryVertexSort(vecVhBoundary, vecFhBoundary);
		Hole hole(&m_mesh, m_averageEdgeLength);

		for (auto vh : vecVhTmp)
		{
			TriMesh::Normal vertex_normal = m_mesh.normal(vh);

			hole.m_vecOriginalVH_NormalPair.push_back(make_pair(vh, vertex_normal));
			hole.m_vecOriginalVH_PositionPair.push_back(make_pair(vh, m_mesh.point(vh)));
		}

		hole.m_originalFaces = vecFhBoundary;
		hole.m_isFilled = false;

		setBoundaryAngles(hole.m_vecOriginalVH_PositionPair, hole.m_minAngleVertexIndex, hole.m_angles);

		m_Holes.push_back(hole);
	}

	for (int i = 0; i < m_Holes.size(); i++)
		m_Holes[i].eraseDuplicateOriginalFaces();

	sort(m_Holes.begin(), m_Holes.end());

	for (int i = 0; i < m_Holes.size(); i++)
	{
		m_Holes[i].m_holeNumber = i + 1;
	}
}


double edgeLengthAverage(TriMesh& mesh)
{
	int count = 0;
	double res = 0;

	for (auto it = mesh.edges_begin(); it != mesh.edges_end(); it++)
	{
		res += mesh.calc_edge_length(*it);
		count++;
	}

	return res / count;
}

// ==== public

FillHole::FillHole(TriMesh& _mesh) : m_mesh(_mesh)
{
	if(m_mesh.faces_empty())
		return;
	
	setIsOriginalVertexArray(); // set Original Vertices
	setIsOriginalFaceArray(); // set Original Faces

	cout << "average edge length calc start\n";
	m_averageEdgeLength = edgeLengthAverage(m_mesh);
	cout << "average edge length calc end\n";
}

void FillHole::showMeshHoleInfo()
{
	for (int i = 0; i < m_Holes.size(); i++)
	{
		cout << "\n" << "idx : " << i << " Hole Info\n";
		cout << "Hole Boundary Vertex Cnt : " << m_Holes[i].m_vecOriginalVH_PositionPair.size() << "\n";
		cout << "Hole Filling Status : " << m_Holes[i].m_isFilled << "\n";
	}
}

void FillHole::findAllHoles()
{
	statusSettings(m_mesh);

	eraseFaceHasTwoHalfEdgeBoundary(m_mesh);

	statusSettings(m_mesh);

	findHole();

	statusSettings(m_mesh);
}

void FillHole::set(TriMesh& _mesh)
{
	m_Holes.clear();
	m_mesh = _mesh;

	//saveMesh(m_mesh, "mesh_Original", true);

	setIsOriginalVertexArray(); // set Original Vertices
	setIsOriginalFaceArray(); // set Original Faces
	
	cout << "average edge length calc start\n";
	m_averageEdgeLength = edgeLengthAverage(m_mesh);
	cout << "average edge length calc end\n";
}

void FillHole::fillHole(int nHole)
{
	m_Holes[nHole].fill();

	//saveMesh(m_mesh, to_string(nHole) + "hole", true);
	m_Holes[nHole].m_isFilled = true;
}

void FillHole::fillAllHoles(bool includeBiggestHole)
{
	int filledHoleCount = 1;

	for (int i = 0; i < m_Holes.size(); i++)
	{
		if (i == 0 && !includeBiggestHole)
		{
			continue;
		}
		fillHole(i);
		
	}

	std::cout << "fillAllHoles Complete" << endl;
}

void FillHole::save(string filename)
{
	saveMesh(m_mesh, filename, true);
}

TriMesh FillHole::getMesh()
{
	return m_mesh;
}

