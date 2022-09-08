#pragma once

#include "../MeshDisplay/MyTriMesh.h"

enum connectStatus
{
	none,
	CompletlySeperation,
	VertexConnectionToMainMesh,
	VertexconnectionToOtherIsland
};

struct Island
{
	vector<TriMesh::FaceHandle> m_islandFaces;
	vector<TriMesh::VertexHandle> m_boundaryVertex;
	int m_idx;
	double m_Area = 0;
	int m_TriangleCount = 0;
	int m_connectOherIsland = -1;
	connectStatus m_status = none;
};

class MeshIsland
{
private:
	TriMesh tmp;
	TriMesh& m_mesh = tmp;
	void sortIsland();
	void setConnectStatus();
	void findAllIsland();

public:
	vector<Island> m_vecIsland;

	MeshIsland(TriMesh& mesh);
	void printAllInfo();
	bool isMeshHasIsland();
	void eraseAllIsland();
	void eraseIsland(int idx);
	void saveMesh(string name);

	TriMesh returnMesh()
	{
		return m_mesh;
	}
};

