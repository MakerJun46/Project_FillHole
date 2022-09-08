#pragma once

#include "../MeshDisplay/MyTriMesh.h"
#include "algebra3.h"

enum TriangleType
{
	None,
	Needle,
	Obtuse,
	SmallArea
};

struct DegeneratedTriangle
{
	TriangleType m_type = None;
	TriMesh::FaceHandle m_face;
	int m_idx;
};

class MeshDegeneratedTriangles
{
private:
	TriMesh tmp;
	TriMesh m_mesh = tmp;
	double m_minAngle;
	double m_maxAngle;
	double m_minArea;

	void findAllDT();
	double getAngle(vec3 v1, vec3 v2, vec3 targetV);
	void mergeVertex(int targetidx, int mergeidx);
	void sortDTbyIdx();

public:
	vector<DegeneratedTriangle> m_vecDT;

	MeshDegeneratedTriangles(TriMesh& mesh, double _minAngle, 
							double _maxAngle, double _minArea);
	void printDTInfo();
	void mergeAllDT();
	void mergeDT(int idx);
	bool isMeshHasDT();
	void saveMesh(string name);

	TriMesh returnMesh()
	{
		return m_mesh;
	}
};
