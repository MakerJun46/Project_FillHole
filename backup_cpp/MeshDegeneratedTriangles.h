#pragma once

#include "../MeshDisplay/MyTriMesh.h"

enum TriangleType
{
	None,
	Needle,
	Obtuse,
	SmallArea
};

struct DegeneratedTriangle
{
	TriangleType type = None;
	TriMesh::FaceHandle face;
	int idx;
};

class MeshDegeneratedTriangles
{
public:
	TriMesh tmp;
	TriMesh& m_mesh = tmp;
	double minAngle;
	double maxAngle;
	double minArea;

	vector<DegeneratedTriangle> vecDT;

	MeshDegeneratedTriangles(TriMesh& mesh, double _minAngle, double _maxAngle, double _minArea);
	double Angle(TriMesh::VertexHandle vh);
	void FindAllDT();
	void PrintDTInfo();
	void MergeVertex(int targetidx, int mergeidx);
	void MergeAllDT();
	void MergeDT(int idx);
	void SortDTbyIdx();
	void saveMesh(string name, string path);
};
