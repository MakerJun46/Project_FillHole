#pragma once

#include "../MeshDisplay/MyTriMesh.h"
#include "algebra3.h"

struct Spike
{
	vector<TriMesh::VertexHandle> m_vertexList;
	vector<TriMesh::FaceHandle> m_faceList;
	vector<vector<TriMesh::VertexHandle>> m_vecOneRingVH;
	TriMesh::VertexHandle m_startVH;
	vec3 m_bottomGradient;
	int m_idx;
};

class MeshSpike
{
private:
	TriMesh tmp;
	TriMesh& m_mesh = tmp;
	double m_spikeAngle;
	double m_theta;
	void findAllSplikes();
	void fillSpikeBySimpleDelete(int idx);
	void fillSpikeBySmoothingNRing(int idx, int cnt);
	void fillSpikeBySmoothing();
	double GetAngle(vec3 v1, vec3 v2, vec3 targetV);
	double GetAngleSum(TriMesh::VertexHandle vh);
	vec3 GetOneRingCenter(vector<TriMesh::VertexHandle> vecOneRingVH);
	Spike FindOneRing(TriMesh::VertexHandle vh);
	void BFSSpike(Spike& tmp);

public:
	vector<Spike> m_vecSpikes;

	MeshSpike(TriMesh& _mesh, double _theta);
	void printInfo();
	bool meshHasSpike();
	void deleteAllSpikes(int option);
	void deleteSpike(int idx, int option);
	void saveMesh(string name);

	TriMesh returnMesh()
	{
		return m_mesh;
	}
};
