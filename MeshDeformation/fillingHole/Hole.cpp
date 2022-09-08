#include "stdafx.h"
#include "..\Hole.h"
#include "igl/exact_geodesic.h"
#include <numeric>
#include "../Utility/MeshLaplacianSolver.h"
#include <igl/list_to_matrix.h>
#include "../MeshDeform/PoissonDeform/PoissonDeformation.h"
extern mat3 rotateAlign(vec3 v1, vec3 v2);
V2d<double> toV2d(vec3 from)
{
	V2d<double> ret;
	ret.x = from[0];
	ret.y = from[1];
	return ret;
}

TriMesh::Point toPoint(vec3 from)
{
	TriMesh::Point ret;
	ret[0] = from[0];
	ret[1] = from[1];
	ret[2] = from[2];
	return ret;
}

vec3 crossProductFunc(vec3 v1, vec3 v2)
{
	return vec3
	(
		v1[1] * v2[2] - v1[2] * v2[1],
		v1[2] * v2[0] - v1[0] * v2[2],
		v1[0] * v2[1] - v1[1] * v2[0]
	);
}
vec3 toVec3(TriMesh::Point from)
{
	vec3 ret;
	ret[0] = from[0];
	ret[1] = from[1];
	ret[2] = from[2];
	return ret;
}

vec3 toVec3(V2d<double> from)
{
	vec3 ret;

	ret[0] = from.x;
	ret[1] = from.y;
	ret[2] = 0;
	return ret;
}

using std::setprecision;
double rand01()
{
	std::random_device rd;
	std::default_random_engine eng(rd());
	std::uniform_real_distribution<double> distr(0, 1);
	return distr(eng);
}

vec3 randomBarycentricCoord()
{
	std::random_device rd;
	std::default_random_engine eng(rd());
	std::uniform_real_distribution<double> distr(0, 1);

	vec3 ret;
	ret[0] = distr(eng);
	ret[1] = distr(eng);
	if (ret[0] + ret[1] > 1) {
		ret[0] = 1 - ret[0];
		ret[1] = 1 - ret[1];
	}
	ret[2] = 1 - ret[0] - ret[1];
	return ret;
}


vec3 barycentric(vec3 a, vec3 b, vec3 c, vec3 p)
{
	vec3 v0 = b - a, v1 = c - a, v2 = p - a;
	float d00 = v0 * v0;
	float d01 = v0 * v1;
	float d11 = v1 * v1;
	float d20 = v2 * v0;
	float d21 = v2 * v1;
	float invDenom = 1.0 / (d00 * d11 - d01 * d01);

	vec3 ret;
	ret[0] = (d11 * d20 - d01 * d21) * invDenom;
	ret[1] = (d00 * d21 - d01 * d20) * invDenom;
	ret[2] = 1 - ret[0] - ret[1];

	return ret;
}

vec3 fromBarycentric(vec3 a, vec3 b, vec3 c, vec3 barycentric)
{
	return a * barycentric[0] + b * barycentric[1] + c * barycentric[2];
}

vec3 fromBarycentric(V2d<double> a, V2d<double> b, V2d<double> c, vec3 barycentric)
{
	return fromBarycentric(toVec3(a), toVec3(b), toVec3(c), barycentric);
}
vec3 normalFromPoints(vector<vec3> vecVec3Points)
{
	Eigen::MatrixXd V;

	vec3 mean = vec3(0);
	for (int i = 0; i < vecVec3Points.size(); i++)
	{
		mean += vecVec3Points[i];
	}
	mean /= vecVec3Points.size();

	vector<vector<double>> v_tmp;
	for (int i = 0; i < vecVec3Points.size(); i++) {
		v_tmp.push_back(vector<double>());
		for (int j = 0; j < 3; j++)
			v_tmp[i].push_back(vecVec3Points[i][j] - mean[j]);

	}
	igl::list_to_matrix(v_tmp, V);

	Eigen::Matrix3d C = V.transpose() * V;
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(C, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d v = svd.matrixV();
	Eigen::Vector3d eigens = svd.singularValues();
	double a[5];
	for (int i = 0; i < 5; i++)
		a[i] = eigens[i];
	//std::cout << "U: " << svd.matrixU() << std::endl;;
	//std::cout << "V: " << svd.matrixV() << std::endl;;

	vec3 vectors[2];
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 3; j++)
			vectors[i][j] = v(j, i);

	return crossProductFunc(vectors[0], vectors[1]).normalize();
	//return svd.matrixV().
}

vec3 normalFrom5Points(vector<vec3> vecVec3Points)
{
	vector<vec3> vec5Points;
	vec5Points.push_back(vecVec3Points[0]);
	vec5Points.push_back(vecVec3Points[1]);
	vec5Points.push_back(vecVec3Points[2]);
	vec5Points.push_back(vecVec3Points[vecVec3Points.size() - 1]);
	vec5Points.push_back(vecVec3Points[vecVec3Points.size() - 2]);
	return normalFromPoints(vec5Points);

}


vector<vec3> toVecVec3(vector<pair<int, V2d<double>>> vecBoundaryVertexIndex, vector<pair<TriMesh::VertexHandle, TriMesh::Point>>& vecVH_PointPair)
{
	vector<vec3> res;
	for (int i = 0; i < vecBoundaryVertexIndex.size(); i++)
		res.push_back(toVec3(vecVH_PointPair[vecBoundaryVertexIndex[i].first].second));
	return res;
}

double calcAngle(vec3 v1, vec3 targetV, vec3 v2, vec3 targetNormal)
{
	v1 -= targetV;
	v2 -= targetV;

	double innerP = v1 * v2;


	double u = v1.length();
	double v = v2.length();

	double deg = acos(innerP / (u * v));
	//if (crossProductFunc(v1, v2) * targetNormal > 0)
		//deg = M_PI * 2 - deg;

	double targetInnerProduct = crossProductFunc(v1, v2).normalize()* targetNormal;
	//return deg * (180 / M_PI);
	return targetInnerProduct;// *(innerP / (u * v));
}


void print(int nIndex, vector < pair<int, V2d<double>>> vecIntV2dBoundary)
{
	string filePath = "boundary" + to_string(nIndex) + ".txt";

	// write File
	ofstream writeFile(filePath.data());
	if (writeFile.is_open()) {
		writeFile << "(";
		for (int i = 0; i < vecIntV2dBoundary.size() - 1; i++) {
			writeFile << "(" + to_string(vecIntV2dBoundary[i].second.x) + "," + to_string(vecIntV2dBoundary[i].second.y) + "),";
		}
		writeFile << "(" + to_string(vecIntV2dBoundary[vecIntV2dBoundary.size() - 1].second.x) + "," + to_string(vecIntV2dBoundary[vecIntV2dBoundary.size() - 1].second.y) + "))";
		writeFile.close();
	}
}

void print(int nIndex, vector < pair<int, V2d<double>>> vecIntV2dBoundary, int index)
{
	string filePath = "boundary" + to_string(index) + "_" + to_string(nIndex) + ".txt";

	// write File
	ofstream writeFile(filePath.data());
	if (writeFile.is_open()) {
		writeFile << "(";
		for (int i = 0; i < (int)vecIntV2dBoundary.size() - 1; i++) {
			writeFile << "(" + to_string(vecIntV2dBoundary[i].second.x) + "," + to_string(vecIntV2dBoundary[i].second.y) + "),";
		}
		if (vecIntV2dBoundary.size() > 0)
			writeFile << "(" + to_string(vecIntV2dBoundary[vecIntV2dBoundary.size() - 1].second.x) + "," + to_string(vecIntV2dBoundary[vecIntV2dBoundary.size() - 1].second.y) + "))";
		writeFile.close();
	}
}

void print(vector<vector< pair<int, V2d<double> >>> vecvecIntV2dBoundaries)
{
	for (int i = 0; i < vecvecIntV2dBoundaries.size(); i++)
		print(i, vecvecIntV2dBoundaries[i]);
}

void print(vector<vector< pair<int, V2d<double> >>> vecvecIntV2dBoundaries, int index)
{
	for (int i = 0; i < vecvecIntV2dBoundaries.size(); i++)
		print(i, vecvecIntV2dBoundaries[i], index);
}
void Hole::fill()
{
	vector<vector<TriMesh::VertexHandle>> vecvecVH_Triangles;
	vector<TriMesh::VertexHandle> second_hole;
	int k = 0;
	bool save = false;
	m_mapEdgeToVertexHandleVector.clear(); // 0810

	cout << "draft fill start!\n";
	fillByAngle(m_vecOriginalVH_PositionPair);
	cout << "draft fill finished\n";
	vector<vector< pair<int, V2d<double> >>> vecvecV2dBoundaries = flattenBoundary();
	//print(vecvecV2dBoundaries);
	cout << "flatten Boundary finished\n";
	//saveMesh(m_mesh, "original_hole");
	cout << "found " << vecvecV2dBoundaries.size() << "boundaries";

	//print(vecvecBoundaryVertexIndex, vecVH_PointPair, vecVec3Normal);
	m_newVertices.clear();
	m_newFaces.clear();

	for (int i = 0; i < vecvecV2dBoundaries.size(); i++)
	{

		cout << i << " th" << "\n";
		fillHoleWithDelaunayTriangulation(vecvecV2dBoundaries[i]);
		cout << to_string(i) << "th boundary triangulation finished\n";
		//saveMesh(m_mesh, to_string(i) + "hole");

	}

}

vector<V2d<double>> toVecV2d(vector< pair<int, V2d<double> >> vecVIndexV2dPair)
{
	vector<V2d<double>> ret;
	ret.resize(vecVIndexV2dPair.size());
	for (int i = 0; i < vecVIndexV2dPair.size(); i++)
		ret[i] = vecVIndexV2dPair[i].second;
	return ret;
}


vector<Edge> toEdges(int nNumVertices)
{
	vector<Edge> ret;
	for (int i = 0; i < nNumVertices - 1; i++)
		ret.push_back(Edge(i, (i + 1) % nNumVertices));
	ret.push_back(Edge(0, nNumVertices - 1));

	return ret;
}


int calcNumAddedVertex(double areaRatio)
{
	if (areaRatio < 1)
		return 0;
	int numAddedEdge = floor(areaRatio);
	double prob = areaRatio - numAddedEdge;
	return rand01() < prob ? numAddedEdge + 1 : numAddedEdge;
}

float distance(vec3 line_point1, vec3 line_point2,
	vec3 point)
{
	vec3 AB = line_point2 - line_point1;
	vec3 AC = point - line_point1;
	float area = crossProductFunc(AB, AC).length();
	float CD = area / AB.length();
	return CD;
}

float distance(TriMesh::Point line_point1, TriMesh::Point line_point2, TriMesh::Point point)
{
	return distance(toVec3(line_point1), toVec3(line_point2), toVec3(point));
}

double findShortEdge(TriMesh::Point p[3], int& left, int& right, int& opposite)
{
	left = 0, right = 2, opposite = 1;
	double minLength = (p[2] - p[0]).length();
	double currLength;
	for (int i = 0; i < 1; i++)
		if ((currLength = (p[i + 1] - p[i]).length()) < minLength) {
			minLength = currLength;
			left = i;
			right = i + 1;
			opposite = (i + 2) % 3;
		}
	return minLength;
}
int calcNumAddedVertex(TriMesh::Point p[3], float averageEdgeLength)
{
	int left, right, opposite;
	int shortestEdgeLength = findShortEdge(p, left, right, opposite)/averageEdgeLength;
	int distToOpposite = distance(p[left], p[right], p[opposite])/ averageEdgeLength;
	return (shortestEdgeLength - 2) * (distToOpposite - 2) / 2;
}

double triangleArea(TriMesh::Point p[3])
{
	TriMesh::Point a, b;
	a = p[1] - p[0];
	b = p[2] - p[0];
	return a.cross(b).length() * 0.5;
}




double findMinEdge(TriMesh::Point p[3], int& left, int& right, int& opposite)
{
	left = 0, right = 2, opposite = 1;
	double minLength = (p[2] - p[0]).length();
	double currLength;
	for (int i = 0; i < 1; i++)
		if ((currLength = (p[i + 1] - p[i]).length()) < minLength) {
			minLength = currLength;
			left = i;
			right = i + 1;
			opposite = (i + 2) % 3;
		}
	return minLength;
}

void Hole::fillHoleWithDelaunayTriangulation(vector< pair<int, V2d<double> >> vecVIndexV2dPair)
{
	m_vecVhForCurrSubBoundary.clear();
	//int nNumOfVertexBeforeCurrentTriangulation = vecVH_PointPair.size() + m_newVertices.size();
	//m_averageEdgeLength *= 10;
	CDT::Triangulation<double> cdt;
	vector<Edge> edgeVector;

	vector<V2d<double>> vertices = toVecV2d(vecVIndexV2dPair);
	//mapEdgeToVertexHandleVector.clear();
	vector<vec3> vecVertex3DForCurrSubBoundary;
	vector<bool> vecValid3DVertex;

	set <pair<int, int>> set2DEdge;

	// m_newVertices: 새로추가된 모든 점들을 저장
	// m_vecVhForCurrSubBoundary: 현재 subBoundary에서 추가된 점들을 저장
	// vertexHandleVector: 현재 edge에서 추가된 점들을 저장
	cdt.insertVertices(vertices);
	edgeVector = toEdges((int)vecVIndexV2dPair.size());
	cdt.insertEdges(edgeVector);
	cdt.eraseOuterTriangles();

	TriangleVec triangles = cdt.triangles;

	for (int i = 0; i < triangles.size(); i++)
	{
		int vIndex[3];
		vec3 vertex3D;
		V2d<double> vertex2D;

		for (int j = 0; j < 3; j++)
			vIndex[j] = vecVIndexV2dPair[triangles[i].vertices[j]].first;
		for (int j = 0; j < 3; j++) {
			int nMinHoleIndex, nMaxHoleIndex, nMinSubHoleIndex, nMaxSubHoleIndex;
			if (vIndex[j] > vIndex[(j + 1) % 3]) {
				nMinHoleIndex = vIndex[(j + 1) % 3];
				nMinSubHoleIndex = triangles[i].vertices[(j + 1) % 3];

				nMaxHoleIndex = vIndex[j];
				nMaxSubHoleIndex = triangles[i].vertices[j];
			}
			else {
				nMaxHoleIndex = vIndex[(j + 1) % 3];
				nMaxSubHoleIndex = triangles[i].vertices[(j + 1) % 3];

				nMinHoleIndex = vIndex[j];
				nMinSubHoleIndex = triangles[i].vertices[j];
			}

			double edgeLength = (m_vecOriginalVH_PositionPair[nMinHoleIndex].second - m_vecOriginalVH_PositionPair[nMaxHoleIndex].second).length();
			if (nMaxHoleIndex - nMinHoleIndex != 1 && nMaxHoleIndex - nMinHoleIndex != m_vecOriginalVH_PositionPair.size() - 1 && (m_vecOriginalVH_PositionPair[nMinHoleIndex].second - m_vecOriginalVH_PositionPair[nMaxHoleIndex].second).length() > 2 * m_averageEdgeLength) {
				double edgeLength = (m_vecOriginalVH_PositionPair[nMinHoleIndex].second - m_vecOriginalVH_PositionPair[nMaxHoleIndex].second).length();
				int nNumAddedEdges = edgeLength / m_averageEdgeLength - 1;
				//nNumAddedEdges = 0;
				pair<int, int > edge = make_pair(nMinHoleIndex, nMaxHoleIndex);
				if ((set2DEdge.find(edge) == set2DEdge.end()) && nNumAddedEdges > 0) {
					set2DEdge.insert(make_pair(nMinHoleIndex, nMaxHoleIndex));

					int nPrevVertexIndex = -1;

					auto it = find(edgeVector.begin(), edgeVector.end(), Edge(nMinSubHoleIndex, nMaxSubHoleIndex));
					if (!(it == edgeVector.end())) {
						edgeVector.erase(it);
						nPrevVertexIndex = nMinSubHoleIndex;
					}

					vector<TriMesh::VertexHandle> vertexHandleVector;

					for (int k = 0; k < nNumAddedEdges; k++) {
						vertex2D = toV2d(toVec3(vecVIndexV2dPair[nMinSubHoleIndex].second) + m_averageEdgeLength / edgeLength * (k + 1) * (toVec3(vecVIndexV2dPair[nMaxSubHoleIndex].second) - toVec3(vecVIndexV2dPair[nMinSubHoleIndex].second)));
						vertices.push_back(vertex2D);
						if (m_mapEdgeToVertexHandleVector.find(edge) == m_mapEdgeToVertexHandleVector.end()) {
							vertex3D = toVec3(m_vecOriginalVH_PositionPair[nMinHoleIndex].second + m_averageEdgeLength / edgeLength * (k + 1) * (m_vecOriginalVH_PositionPair[nMaxHoleIndex].second - m_vecOriginalVH_PositionPair[nMinHoleIndex].second));
							vecVertex3DForCurrSubBoundary.push_back(vertex3D);
							m_newVertices.push_back(m_mesh->add_vertex(toPoint(vertex3D)));
							m_vecVhForCurrSubBoundary.push_back(m_newVertices[m_newVertices.size() - 1]);
							vertexHandleVector.push_back(m_newVertices[m_newVertices.size() - 1]);
						}
						else
							//m_newVertices.push_back(mapEdgeToVertexHandleVector[edge][k]);
							m_vecVhForCurrSubBoundary.push_back(m_mapEdgeToVertexHandleVector[edge][k]);
						if (nPrevVertexIndex >= 0) {
							edgeVector.push_back(Edge(nPrevVertexIndex, (int)vertices.size() - 1));
							nPrevVertexIndex = (int)vertices.size() - 1;
						}
					}
					if (nPrevVertexIndex >= 0)
						edgeVector.push_back(Edge(nPrevVertexIndex, nMaxSubHoleIndex));
					if (m_mapEdgeToVertexHandleVector.find(edge) == m_mapEdgeToVertexHandleVector.end())
						m_mapEdgeToVertexHandleVector[edge] = vertexHandleVector;
				}
			}
		}
	}
	
	for (int i = 0; i < triangles.size(); i++)
	{
		TriMesh::Point triVertices[3];
		for (int j = 0; j < 3; j++)
			triVertices[j] = m_vecOriginalVH_PositionPair[vecVIndexV2dPair[triangles[i].vertices[j]].first].second;

		int numAddedVertex = calcNumAddedVertex(triVertices, m_averageEdgeLength);
		int left, right, opposite;
		if(findMinEdge(triVertices, left, right, opposite)< 2*m_averageEdgeLength)
			numAddedVertex = 0;

		for (int j = 0; j < numAddedVertex; j++)
		{
			vec3 barycentric = randomBarycentricCoord();
			V2d<double> vertex2D = toV2d(fromBarycentric(vertices[triangles[i].vertices[0]], vertices[triangles[i].vertices[1]], vertices[triangles[i].vertices[2]], barycentric));
			vec3 vertex3D = fromBarycentric(toVec3(m_vecOriginalVH_PositionPair[vecVIndexV2dPair[triangles[i].vertices[0]].first].second)
				, toVec3(m_vecOriginalVH_PositionPair[vecVIndexV2dPair[triangles[i].vertices[1]].first].second)
				, toVec3(m_vecOriginalVH_PositionPair[vecVIndexV2dPair[triangles[i].vertices[2]].first].second)
				, barycentric);
			vecVertex3DForCurrSubBoundary.push_back(vertex3D);
			//vertex3D = surface.project(vertex3D, normal);
			m_newVertices.push_back(m_mesh->add_vertex(toPoint(vertex3D)));
			m_vecVhForCurrSubBoundary.push_back(m_newVertices[m_newVertices.size() - 1]);
			vertices.push_back(vertex2D);
		}
	}

	CDT::Triangulation<double> cdtNew;
	cdtNew.insertVertices(vertices);
	cdtNew.insertEdges(edgeVector);
	cdtNew.eraseOuterTriangles();
	TriangleVec newTriangles = cdtNew.triangles;
	vector<V2d<double>> newVertices = cdtNew.vertices;

	/*
	vector<vector<int>> neighbors;
	neighbors.resize(vertices.size());
	
	for (int i = 0; i < newTriangles.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			neighbors[newTriangles[i].vertices[j]].push_back(newTriangles[i].vertices[(j + 1) % 3]);
			neighbors[newTriangles[i].vertices[j]].push_back(newTriangles[i].vertices[(j + 2) % 3]);
		}
	}
	for (int i = vecVIndexV2dPair.size(); i < vertices.size(); i++)
	{
		if (neighbors[i].size() > 0)
		{
			mergeVertices(i, neighbors, )
		}
	}
	*/

	for (int i = 0; i < newTriangles.size(); i++)
	{
		vector<TriMesh::VertexHandle> vhTriangle;

		for (int j = 0; j < 3; j++)
		{

			if (newTriangles[i].vertices[j] < vecVIndexV2dPair.size())
				vhTriangle.push_back(m_vecOriginalVH_PositionPair[vecVIndexV2dPair[newTriangles[i].vertices[j]].first].first);
			else
				vhTriangle.push_back(m_vecVhForCurrSubBoundary[newTriangles[i].vertices[j] - vecVIndexV2dPair.size()]);
			/*
			vhTriangle.push_back(
				newTriangles[i].vertices[j] < vecVIndexV2dPair.size()
				?
				m_vecOriginalVH_PositionPair[vecVIndexV2dPair[newTriangles[i].vertices[j]].first].first
				: m_newVertices[newTriangles[i].vertices[j] - vecVIndexV2dPair.size()]);
			*/
		}
		m_newFaces.push_back(m_mesh->add_face(vhTriangle));
	}
}


void Hole::fillByAngle(vector<pair<TriMesh::VertexHandle, TriMesh::Point>> vecVH_PositionPair)
{
	vector<pair<int, int>> vecConnectEdge;
	int k = 0;
	vector<int> vecOriginalVertexIndex;
	for (int i = 0; i < vecVH_PositionPair.size(); i++)
		vecOriginalVertexIndex.push_back(i);

	while (!vecVH_PositionPair.empty())
	{
		vector<TriMesh::VertexHandle> vecVHtmp;
		int index = m_minAngleVertexIndex;
		int nPrev;
		int nNext;

		if (vecVH_PositionPair.size() < 3)
		{
			break;
		}

		if (vecVH_PositionPair.size() == 3)
		{
			//for (int i = 0; i < 3; i++)
				//vecVHtmp.push_back(vecVH_PositionPair[i].first);

			//m_mesh->add_face(vecVHtmp);
			//break;
		}

		if (index == 0)
		{
			vecVHtmp.push_back(vecVH_PositionPair[vecVH_PositionPair.size() - 1].first);
			nPrev = (int)vecVH_PositionPair.size() - 1;
		}
		else
		{
			vecVHtmp.push_back(vecVH_PositionPair[index - 1].first);
			nPrev = index - 1;
		}

		vecVHtmp.push_back(vecVH_PositionPair[index].first);

		if (index == vecVH_PositionPair.size() - 1)
		{
			vecVHtmp.push_back(vecVH_PositionPair[0].first);
			nNext = 0;
		}
		else
		{
			vecVHtmp.push_back(vecVH_PositionPair[index + 1].first);
			nNext = index + 1;
		}

		vecConnectEdge.push_back(make_pair(min(vecOriginalVertexIndex[nPrev], vecOriginalVertexIndex[nNext]), max(vecOriginalVertexIndex[nPrev], vecOriginalVertexIndex[nNext])));
		//m_mesh->add_face(vecVHtmp);

		if (nPrev == 0)
		{
			m_angles[nPrev] = calcAngle(toVec3(vecVH_PositionPair[vecVH_PositionPair.size() - 1].second), toVec3(vecVH_PositionPair[nPrev].second), toVec3(vecVH_PositionPair[nNext].second), toVec3(m_mesh->normal(vecVH_PositionPair[nPrev].first)));
		}
		else
		{
			m_angles[nPrev] = calcAngle(toVec3(vecVH_PositionPair[nPrev - 1].second), toVec3(vecVH_PositionPair[nPrev].second), toVec3(vecVH_PositionPair[nNext].second), toVec3(m_mesh->normal(vecVH_PositionPair[nPrev].first)));
		}

		if (nNext == vecVH_PositionPair.size() - 1)
		{
			m_angles[nNext] = calcAngle(toVec3(vecVH_PositionPair[nPrev].second), toVec3(vecVH_PositionPair[nNext].second), toVec3(vecVH_PositionPair[0].second), toVec3(m_mesh->normal(vecVH_PositionPair[nNext].first)));
		}
		else
		{
			m_angles[nNext] = calcAngle(toVec3(vecVH_PositionPair[nPrev].second), toVec3(vecVH_PositionPair[nNext].second), toVec3(vecVH_PositionPair[nNext + 1].second), toVec3(m_mesh->normal(vecVH_PositionPair[nNext].first)));
		}

		vecVH_PositionPair.erase(vecVH_PositionPair.begin() + index);
		vecOriginalVertexIndex.erase(vecOriginalVertexIndex.begin() + index);
		m_angles.erase(m_angles.begin() + index);

		m_minAngleVertexIndex = min_element(m_angles.begin(), m_angles.end()) - m_angles.begin();
	}

	m_vecConnectEdge = vecConnectEdge;

}


#define FLAT_THRESHOLD 0.01




bool isFlat(vector<vec3>& vecVec3Points, vec3 principleAxis[2])
{
	Eigen::MatrixXd V;


	if (vecVec3Points.size() < 4)
		return true;

	//return false;
	vec3 mean = vec3(0);
	for (int i = 0; i < vecVec3Points.size(); i++)
	{
		mean += vecVec3Points[i];
	}
	mean /= vecVec3Points.size();

	vector<vector<double>> v_tmp;
	for (int i = 0; i < vecVec3Points.size(); i++) {
		v_tmp.push_back(vector<double>());
		for (int j = 0; j < 3; j++)
			v_tmp[i].push_back(vecVec3Points[i][j] - mean[j]);

	}
	igl::list_to_matrix(v_tmp, V);

	Eigen::Matrix3d C = V.transpose() * V;
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(C, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d v = svd.matrixV();
	Eigen::Vector3d eigens = svd.singularValues();
	double singularValues[3];
	for (int i = 0; i < 3; i++)
		singularValues[i] = eigens[i];
	//std::cout << "U: " << svd.matrixU() << std::endl;;
	//std::cout << "V: " << svd.matrixV() << std::endl;;

	vec3 vectors[2];
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 3; j++)
			vectors[i][j] = v(j, i);

	principleAxis[0] = vectors[0];
	principleAxis[1] = vectors[1];

	return  (eigens[2]) / (eigens[0] + eigens[1] + eigens[2]) < FLAT_THRESHOLD;
}

void divideVertexIndex(vector<pair<int, V2d<double>>> vecVertexIndex, int index1, int index2, vector<pair<int, V2d<double>>>& vecIntLeftIndex, vector<pair<int, V2d<double>>>& vecIntRightIndex, map<int, int>& mapPositionInLeftSubBoundary, map<int, int>& mapPositionInRightSubBoundary, vector<pair<int, int>>& vecEdge, vector<pair<int, int>>& vecLeftEdge, vector<pair<int, int>>& vecRightEdge)
{
	int nMin = min(index1, index2);
	int nMax = max(index1, index2);
	vector<int> edgeIndex;
	int i;
	for (i = 0; i < vecVertexIndex.size(); i++) {
		if (nMin <= vecVertexIndex[i].first && vecVertexIndex[i].first <= nMax) {
			mapPositionInLeftSubBoundary[vecVertexIndex[i].first] = (int)vecIntLeftIndex.size();
			vecIntLeftIndex.push_back(make_pair(vecVertexIndex[i].first, V2d<double>()));
		}
		else {
			mapPositionInRightSubBoundary[vecVertexIndex[i].first] = (int)vecIntRightIndex.size();
			vecIntRightIndex.push_back(make_pair(vecVertexIndex[i].first, V2d<double>()));
		}

		if (nMin == vecVertexIndex[i].first || vecVertexIndex[i].first == nMax) {
			mapPositionInRightSubBoundary[vecVertexIndex[i].first] = (int)vecIntRightIndex.size();
			vecIntRightIndex.push_back(make_pair(vecVertexIndex[i].first, V2d<double>()));
		}
	}

	for (i = 0; i < vecEdge.size(); i++)
	{
		if (nMin <= vecEdge[i].first && vecEdge[i].second <= nMax)
			vecLeftEdge.push_back(vecEdge[i]);
		else
			vecRightEdge.push_back(vecEdge[i]);
		if (nMin == vecEdge[i].first && vecEdge[i].second == nMax)
			vecRightEdge.push_back(vecEdge[i]);

	}
	return;
}
vec3 boundaryNormal(vector<pair<int, V2d<double>>> vecBoundaryVertexIndex, vector<pair<TriMesh::VertexHandle, TriMesh::Point>>& vecVH_PointPair)
{
	vec3 normal(0);
	int i, j;
	for (i = 0, j = 1; i < vecBoundaryVertexIndex.size(); i++, j++) {
		if (j == vecBoundaryVertexIndex.size()) j = 0;
		vec3 pi = toVec3(vecVH_PointPair[vecBoundaryVertexIndex[i].first].second);
		vec3 pj = toVec3(vecVH_PointPair[vecBoundaryVertexIndex[j].first].second);

		normal[0] += (((pi[2]) + (pj[2])) *
			((pj[1]) - (pi[1])));

		normal[1] += (((pi[0]) + (pj[0])) *
			((pj[2]) - (pi[2])));

		normal[2] += (((pi[1]) + (pj[1])) *
			((pj[0]) - (pi[0])));
	}
	return -normal.normalize();
}

vec3 boundaryAverageNormal(vector<pair<int, V2d<double>>> vecBoundaryVertexIndex, vector<pair<TriMesh::VertexHandle, TriMesh::Point>>& vecVH_PointPair, TriMesh& mesh)
{
	vec3 ret(0);
	for (int i = 0; i < vecBoundaryVertexIndex.size(); i++) {
		ret += toVec3(mesh.normal(vecVH_PointPair[vecBoundaryVertexIndex[i].first].first));
	}

	return ret.normalize();
}

int distance(int position1, int position2, int size)
{
	int ret = abs(position1 - position2);
	if (ret > size / 2)
		ret = size - ret;
	return ret;
}
pair<int, int> farthestEdge(vector < pair<int, int> >vecEdge, map<int, int> mapPositionInSubBoundary)
{
	pair<int, int> ret = vecEdge[0];
	int maxDistance = -1;

	for (int i = 0; i < vecEdge.size(); i++) {
		int currDistance = distance(mapPositionInSubBoundary[vecEdge[i].first], mapPositionInSubBoundary[vecEdge[i].second], (int)mapPositionInSubBoundary.size());
		if (currDistance > maxDistance) {
			ret = vecEdge[i];
			maxDistance = currDistance;
		}
	}
	assert(maxDistance > 0);
	return ret;
}



double orient2d(V2d<double> a, V2d<double> b, V2d<double> c)
{
	return predicates::adaptive::orient2d(a.x, a.y, b.x, b.y, c.x, c.y);
}

bool intersect(V2d<double> a1, V2d<double> a2, V2d<double> b1, V2d<double> b2)
{
	return (orient2d(a1, b1, b2) * orient2d(a2, b1, b2) <= 0 && orient2d(b1, a1, a2) * orient2d(b2, a1, a2) <= 0);
}

bool intersect(TriMesh::Point a1, TriMesh::Point a2, TriMesh::Point b1, TriMesh::Point b2, mat3 matRot, vec3 origin)
{
	V2d<double> v2dA1, v2dA2, v2dB1, v2dB2;
	v2dA1 = toV2d(matRot * (toVec3(a1) - origin));
	v2dA2 = toV2d(matRot * (toVec3(a2) - origin));
	v2dB1 = toV2d(matRot * (toVec3(b1) - origin));
	v2dB2 = toV2d(matRot * (toVec3(b2) - origin));
	return intersect(v2dA1, v2dA2, v2dB1, v2dB2);
}

bool intersect(V2d<double> v2dStart, V2d<double> v2dEnd, vector< pair<int, V2d<double> >> vecVIndexV2dPair, int nStart, int nEnd)
{
	if (nEnd + 1 < 0)
		return false;
	for (int i = nStart; i < nEnd + 1; i++)
	{
		if (intersect(v2dStart, v2dEnd, vecVIndexV2dPair[i % vecVIndexV2dPair.size()].second, vecVIndexV2dPair[(i + 1) % vecVIndexV2dPair.size()].second))
			return true;
	}
	return false;
}


int intersectIndex(V2d<double> v2dStart, V2d<double> v2dEnd, vector< pair<int, V2d<double> >> vecVIndexV2dPair, int nStart, int nEnd)
{
	for (int i = nStart; i < nEnd + 1; i++)
	{
		if (intersect(v2dStart, v2dEnd, vecVIndexV2dPair[i % vecVIndexV2dPair.size()].second, vecVIndexV2dPair[(i + 1) % vecVIndexV2dPair.size()].second))
			return i;
	}
	return -1;
}

vector<vector<pair<int, V2d<double>>>> partitionByPCA(vector<pair<int, V2d<double>>>& vecVertexIndex, vector<pair<TriMesh::VertexHandle, TriMesh::Point>>& vecVH_PointPair, TriMesh& mesh, vector<pair<int, int>>& vecEdge, map<int, int> mapPositionInSubBoundary)
{
	vector < vector<pair<int, V2d<double>>>> vecvecVertexIndex, vecvecLeftVertexIndex, vecvecRightVertexIndex;
	vector<pair<int, V2d<double>>> vecIntLeftIndex, vecIntRightIndex;
	vec3 principalAxis[2];
	vec3 PCANormal;
	vector <vec3> vecVec3LeftNormal, vecVec3RightNormal;
	bool bFlat;
	vec3 vec3OriginPosition;
	vec3 vec3RotatedPosition;
	V2d<double> v2dPosition, v2dPrevPosition;
	bFlat = isFlat(toVecVec3(vecVertexIndex, vecVH_PointPair), principalAxis);
	if (bFlat) {
		vec3 averageNormal = boundaryNormal(vecVertexIndex, vecVH_PointPair);
		vec3OriginPosition = toVec3(vecVH_PointPair[vecVertexIndex[0].first].second);
		mat3 matRot = rotateAlign(averageNormal, vec3(0, 0, 1));
		vec3RotatedPosition = matRot * (toVec3(vecVH_PointPair[vecVertexIndex[0].first].second) - vec3OriginPosition);
		v2dPosition = toV2d(vec3RotatedPosition);
		v2dPrevPosition = v2dPosition;
		for (int i = 1; i < vecVertexIndex.size(); i++)
		{
			vec3RotatedPosition = matRot * (toVec3(vecVH_PointPair[vecVertexIndex[i].first].second) - vec3OriginPosition);
			v2dPosition = toV2d(vec3RotatedPosition);

			if (intersect(v2dPosition, v2dPrevPosition, vecVertexIndex, 0, i - 3))
			{
				bFlat = false;
				break;
			}
			else
				vecVertexIndex[i].second = v2dPosition;
			v2dPrevPosition = v2dPosition;
		}
		v2dPosition = V2d<double>();
		if (intersect(v2dPosition, v2dPrevPosition, vecVertexIndex, 1, (int)vecVertexIndex.size() - 3))
			bFlat = false;
	}
	//if (vecVertexIndex.size() < 4 || bFlat && PCANormal * averageNormal > 0.8)
	if (bFlat)
	{
		vecvecVertexIndex.push_back(vecVertexIndex);
	}
	else {
		map<int, int> mapPositionInLeftSubBoundary, mapPositionInRightSubBoundary;
		vector<pair<int, int>> vecLeftEdge;
		vector<pair<int, int>> vecRightEdge;
		pair<int, int> longestEdge = farthestEdge(vecEdge, mapPositionInSubBoundary);
		divideVertexIndex(vecVertexIndex, longestEdge.first, longestEdge.second, vecIntLeftIndex, vecIntRightIndex, mapPositionInLeftSubBoundary, mapPositionInRightSubBoundary, vecEdge, vecLeftEdge, vecRightEdge);
		assert(vecIntLeftIndex.size() >= 3);
		assert(vecIntRightIndex.size() >= 3);
		vecvecLeftVertexIndex = partitionByPCA(vecIntLeftIndex, vecVH_PointPair, mesh, vecLeftEdge, mapPositionInLeftSubBoundary);
		vecvecRightVertexIndex = partitionByPCA(vecIntRightIndex, vecVH_PointPair, mesh, vecRightEdge, mapPositionInRightSubBoundary);
		vecvecVertexIndex.insert(vecvecVertexIndex.end(), vecvecLeftVertexIndex.begin(), vecvecLeftVertexIndex.end());
		vecvecVertexIndex.insert(vecvecVertexIndex.end(), vecvecRightVertexIndex.begin(), vecvecRightVertexIndex.end());
	}
	return vecvecVertexIndex;
}



vector<vector< pair<int, V2d<double> >>> Hole::flattenBoundary()
{
	vec3 vec3Normal;
	vec3 vec3PCANormal;
	vec3 vec3OriginPosition;
	vec3 vec3Position;
	vec3 vec3RotatedPosition;
	vector<vector< pair<int, V2d<double> >>> vecvecVIndexV2dPair;
	m_vecvecBoundaryVertexIndex.clear();

	m_vecvecBoundaryVertexIndex.push_back(vector<pair<int, V2d<double>>>());
	m_vecvecBoundaryVertexIndex[0].resize(m_vecOriginalVH_PositionPair.size());
	for (int i = 0; i < m_vecOriginalVH_PositionPair.size(); i++)
		m_vecvecBoundaryVertexIndex[0][i].first = i;

	vec3 vec3MixedNormal;
	int index = 0;

	map<int, int> mapPositionInBoundary;
	for (int i = 0; i < m_vecvecBoundaryVertexIndex[0].size(); i++)
		mapPositionInBoundary[i] = i;

	m_vecvecBoundaryVertexIndex = partitionByPCA(m_vecvecBoundaryVertexIndex[0], m_vecOriginalVH_PositionPair, *m_mesh, m_vecConnectEdge, mapPositionInBoundary);



	return m_vecvecBoundaryVertexIndex;
}
