#pragma once

#include "../MeshDisplay/MyTriMesh.h"
#include "../Hole.h"

class FillHole
{
private:
	map<int, bool> m_isOriginalFace;
	map<int, bool> m_isOriginalVertex;
	vector<int> m_deletedVertices;
	double m_averageEdgeLength;

	vector<TriMesh::VertexHandle> originalBoundaryVertexSort(vector<TriMesh::VertexHandle>& vecVhBoundary, vector<TriMesh::FaceHandle>& vecFhBoundary);
	void setIsOriginalVertexArray();
	void setIsOriginalFaceArray();
	void setBoundaryAngles(vector<pair<TriMesh::VertexHandle, TriMesh::Point>>& vecVH_PointPair, int& minAngleVertexIndex, vector<double>& angles);
	void findHole();

public:
	TriMesh m_mesh;
	vector<Hole> m_Holes;

	FillHole(TriMesh& _mesh);
	void set(TriMesh& _mesh);
	void showMeshHoleInfo();
	void findAllHoles();
	void fillAllHoles(bool include0thHole = true);
	void fillHole(int nHole);
	void save(string filename);
	TriMesh getMesh();
};