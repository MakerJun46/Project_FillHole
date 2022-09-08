#pragma once

#include "../MeshDisplay/MyTriMesh.h"
#include "../fillingHole/algebra3.h"
#include <CDT/CDT.h>
using namespace CDT;
class Hole
{
private:
	TriMesh* m_mesh;

	void addFaces(vector<vector<TriMesh::VertexHandle>> vecvecTriangles);
	void deleteBoundaryORInvalidInnerVertices();
	vector<vector< pair<int, V2d<double> >>> flattenBoundary();
	vector<vector<pair<int, V2d<double>>>> m_vecvecBoundaryVertexIndex;
	void fillHoleWithDelaunayTriangulation(vector< pair<int, V2d<double> >> vecVIndexV2dPair);
	void fillByAngle(vector < pair<TriMesh::VertexHandle, TriMesh::Point>> vecVH_PositionPair);

public:
	vector<pair<TriMesh::VertexHandle, TriMesh::Normal>> m_vecOriginalVH_NormalPair;
	vector<pair<TriMesh::VertexHandle, TriMesh::Point>> m_vecOriginalVH_PositionPair;
	vector<TriMesh::FaceHandle> m_originalFaces;
	vector<double> m_angles;
	vector<TriMesh::VertexHandle> m_newVertices;
	vector<TriMesh::FaceHandle> m_newFaces;
	vector<pair<int, int>> m_vecConnectEdge;
	map <pair<int, int>, vector<TriMesh::VertexHandle>> m_mapEdgeToVertexHandleVector;
	vector<TriMesh::VertexHandle> m_vecVhForCurrSubBoundary;
	int m_holeNumber;
	bool m_isFilled = false;
	int m_minAngleVertexIndex;
	double m_averageEdgeLength;

	Hole(TriMesh* _mesh, float averageEdgeLength) : m_mesh(_mesh), m_averageEdgeLength(averageEdgeLength) {};

	void eraseDuplicateOriginalFaces() {
		vector<TriMesh::FaceHandle>temp_originalFaces;
		map<TriMesh::FaceHandle, int> mapFaces;
		temp_originalFaces.resize(m_originalFaces.size());
		std::copy(m_originalFaces.begin(), m_originalFaces.end(), temp_originalFaces.begin());
		m_originalFaces.clear();
		for (int i = 0; i < temp_originalFaces.size(); i++) {
			if (mapFaces.find(temp_originalFaces[i]) == mapFaces.end()) {
				mapFaces[temp_originalFaces[i]] = 1;
				m_originalFaces.push_back(temp_originalFaces[i]);
			}
			else
			{
				int k = 0;
				k++;
			}
		}
	}

	void fill();




	bool operator< (const Hole& ref) const
	{
		return this->m_vecOriginalVH_PositionPair.size() > ref.m_vecOriginalVH_PositionPair.size();
	}
};