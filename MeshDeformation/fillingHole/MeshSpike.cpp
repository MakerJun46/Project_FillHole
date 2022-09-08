#include "stdafx.h"
#include "MeshSpike.h"
#include <queue>
#include <set>


MeshSpike::MeshSpike(TriMesh& mesh, double _theta) : m_mesh(mesh)
{
	m_spikeAngle = sin(_theta) * 2.0 * M_PI * (180.0 / M_PI);

	if (m_spikeAngle < 0)
		m_spikeAngle += 360.0;

	m_theta = _theta;

	m_mesh.request_face_status();
	m_mesh.request_edge_status();
	m_mesh.request_vertex_status();

	cout << "Spike Angle : " << m_spikeAngle << endl;

	findAllSplikes();
}

void MeshSpike::printInfo()
{
	if (!meshHasSpike())
	{
		cout << "=== This Mesh has No Spike ===\n";
		return;
	}

	for (Spike& spike : m_vecSpikes)
	{
		cout << "==============\n";
		cout << "Spike idx : " << spike.m_idx << "\n";
		cout << "Spike FaceCnt : " << spike.m_faceList.size() << "\n";
		cout << "Spike VertexCnt : " << spike.m_vertexList.size() << "\n";
		cout << "Spike N-th Ring cnt : " << spike.m_vecOneRingVH.size() << "\n";
		cout << "==============\n";
	}
}

bool MeshSpike::meshHasSpike()
{
	return !m_vecSpikes.empty();
}

double MeshSpike::GetAngle(vec3 v1, vec3 v2, vec3 targetV)
{
	v1 -= targetV;
	v2 -= targetV;

	double innerP = v1 * v2;

	double u = v1.length();
	double v = v2.length();

	double deg = acos(innerP / (u * v));

	return deg * (180 / M_PI);
}


vec3 PointtoVec3(TriMesh::Point from)
{
	return vec3(from[0], from[1], from[2]);
}

double MeshSpike::GetAngleSum(TriMesh::VertexHandle vh)
{
	double deg = 0;

	//cout << "VH IDX : " << vh.idx() << endl;
	//cout << "연결 edge 개수 : " << (m_mesh.ve_end(vh) - m_mesh.ve_begin(vh)) << endl;
	
	auto targetVP = m_mesh.point(vh);

	// 첫번째 edge에 연결된 반대편 vetex
	auto currV = m_mesh.to_vertex_handle(m_mesh.ve_begin(vh).current_halfedge_handle());

	auto currP = m_mesh.point(currV);

	for (auto it = ++m_mesh.ve_begin(vh); it != m_mesh.ve_end(vh); it++)
	{
		auto v1 = m_mesh.to_vertex_handle(it.current_halfedge_handle());

		auto p1 = m_mesh.point(v1);

		double tmp = GetAngle(PointtoVec3(currP), PointtoVec3(p1), PointtoVec3(targetVP));

		deg += tmp;

		//cout << tmp << endl;

		currP = p1; // 이전 정점에 현재 점 대입
	}

	deg += GetAngle(PointtoVec3(currP), PointtoVec3(m_mesh.point(currV)), PointtoVec3(targetVP));

	//cout << deg << endl;

	//cout << "===============================";

	return deg;
}

vec3 MeshSpike::GetOneRingCenter(vector<TriMesh::VertexHandle> vecOneRingVH)
{
	vec3 g;

	g[0] = 0;
	g[1] = 0;
	g[2] = 0;

	for (TriMesh::VertexHandle vh : vecOneRingVH)
	{
		TriMesh::Point p = m_mesh.point(vh);

		vec3 v = PointtoVec3(p);

		g[0] += v[0];
		g[1] += v[1];
		g[2] += v[2];
	}

	g[0] /= (double)vecOneRingVH.size();
	g[1] /= (double)vecOneRingVH.size();
	g[2] /= (double)vecOneRingVH.size();

	return g;
}

Spike MeshSpike::FindOneRing(TriMesh::VertexHandle vh)
{
	Spike spike_tmp;
	vector<TriMesh::VertexHandle> vecVH_tmp;

	spike_tmp.m_idx = (int)m_vecSpikes.size();

	spike_tmp.m_startVH = vh;

	spike_tmp.m_vertexList.push_back(vh);

	auto targetVP = m_mesh.point(vh);

	for (auto it = m_mesh.ve_begin(vh); it != m_mesh.ve_end(vh); it++)
	{
		vecVH_tmp.push_back(m_mesh.to_vertex_handle(it.current_halfedge_handle()));
		spike_tmp.m_vertexList.push_back(m_mesh.to_vertex_handle(it.current_halfedge_handle()));
	}

	for (auto it = m_mesh.vf_begin(vh); it != m_mesh.vf_end(vh); it++)
	{
		spike_tmp.m_faceList.push_back(it);
	}

	spike_tmp.m_vecOneRingVH.push_back(vecVH_tmp);

	return spike_tmp;
}


void MeshSpike::BFSSpike(Spike& tmp)
{
	std::queue<TriMesh::VertexHandle> q;

	vector<TriMesh::VertexHandle> AllVertices;
	vector<TriMesh::FaceHandle> AllFaces;
	vector<bool> Visited_vertex(m_mesh.n_vertices(), false);
	vector<bool> Visited_face(m_mesh.n_faces(), false);

	for (auto it = m_mesh.vertices_begin(); it != m_mesh.vertices_end(); it++)
	{
		AllVertices.push_back(it);
	}

	for (auto it = m_mesh.faces_begin(); it != m_mesh.faces_end(); it++)
	{
		AllFaces.push_back(it);
	}

	for (int i = 0; i < tmp.m_vecOneRingVH[0].size(); i++)
	{
		q.push(tmp.m_vecOneRingVH[0][i]);

		Visited_vertex[find(AllVertices.begin(), AllVertices.end(), tmp.m_vecOneRingVH[0][i]) - AllVertices.begin()];
	}

	Visited_vertex[find(AllVertices.begin(), AllVertices.end(), tmp.m_startVH) - AllVertices.begin()];

	for (auto it = m_mesh.vf_begin(tmp.m_startVH); it != m_mesh.vf_end(tmp.m_startVH); it++)
	{
		Visited_face[find(AllFaces.begin(), AllFaces.end(), it) - AllFaces.begin()];
	}

	bool noMoreSpike = false;
	vector<TriMesh::VertexHandle> vecNextOneRingVH;
	vector<TriMesh::FaceHandle> vecNextOneRingFH;
	vec3 prevCenter = GetOneRingCenter(tmp.m_vecOneRingVH[0]);

	while (!q.empty())
	{
		queue<TriMesh::VertexHandle> q2 = q;

		while (!q.empty())
			q.pop();

		while (!q2.empty())
		{
			TriMesh::VertexHandle item = q2.front(); q2.pop();

			int index = find(AllVertices.begin(), AllVertices.end(), item) - AllVertices.begin();

			if (Visited_vertex[index])
				continue;

			for (auto it = m_mesh.vf_begin(item); it != m_mesh.vf_end(item); it++)
			{
				index = find(AllFaces.begin(), AllFaces.end(), it) - AllFaces.begin();

				if (!Visited_face[index]) // nextOneRing의 Face
				{
					for (auto v_it = m_mesh.fv_begin(it); v_it != m_mesh.fv_end(it); v_it++) // nextOneRing의 vetex들 저장
					{
						index = find(AllVertices.begin(), AllVertices.end(), v_it) - AllVertices.begin();

						if (item != v_it && !Visited_vertex[index])
						{
							q.push(v_it); // 다음 탐색을 위해 queue에 저장

							vecNextOneRingVH.push_back(v_it);

							Visited_vertex[index] = true;
						}
					}

					vecNextOneRingFH.push_back(it);

					Visited_face[index] = true;
				}
			}
		}

		vec3 g = GetOneRingCenter(vecNextOneRingVH);

		vec3 c = prevCenter - g;

		prevCenter = g;

		for (int i = 0; i < vecNextOneRingFH.size(); i++)
		{
			TriMesh::Normal face_n = m_mesh.normal(vecNextOneRingFH[i]);

			vec3 n(face_n[0], face_n[1], face_n[2]);

			double deg = 90.0 - (acos(n * c) * (180.0 / M_PI));

			if (deg > m_theta)
			{
				noMoreSpike = true;
				break;
			}
		}

		if (noMoreSpike)
			break;
		else
		{
			vecNextOneRingFH.erase(unique(vecNextOneRingFH.begin(), vecNextOneRingFH.end()), vecNextOneRingFH.end());
			vecNextOneRingVH.erase(unique(vecNextOneRingVH.begin(), vecNextOneRingVH.end()), vecNextOneRingVH.end());

			for (int i = 0; i < vecNextOneRingFH.size(); i++)
			{
				tmp.m_faceList.push_back(vecNextOneRingFH[i]);
			}

			for (int i = 0; i < vecNextOneRingVH.size(); i++)
			{
				tmp.m_vertexList.push_back(vecNextOneRingVH[i]);
			}

			tmp.m_vecOneRingVH.push_back(vecNextOneRingVH);

			vecNextOneRingFH.clear();
			vecNextOneRingVH.clear();
		}
	}
}


void MeshSpike::findAllSplikes()
{
	m_vecSpikes.clear();

	for (auto it = m_mesh.vertices_begin(); it != m_mesh.vertices_end(); it++)
	{
		// boundary vertex인 경우 spike로 분류하지 않음
		// vertex에 연결된 모든 삼각형의 내각의 각도가 기준 a(SpikeAngle)보다 작은 경우 spike
		if (!m_mesh.is_boundary(it) && GetAngleSum(it) < m_spikeAngle)
		{
			Spike tmp = FindOneRing(it);

			BFSSpike(tmp);

			m_vecSpikes.push_back(tmp);
		}
	}
}

void MeshSpike::fillSpikeBySimpleDelete(int idx)
{
	Spike& target = m_vecSpikes[idx];
	
	for (int i = 0; i < target.m_vertexList.size(); i++)
	{
		m_mesh.delete_vertex(target.m_vertexList[i]);
	}
}

void MeshSpike::fillSpikeBySmoothingNRing(int idx, int cnt)
{
	Spike& target = m_vecSpikes[idx];

	for (int i = 0; i < cnt; i++)
	{
		for (int j = 0; j < target.m_vecOneRingVH[0].size(); j++)
		{
			m_mesh.delete_vertex(target.m_vecOneRingVH[i][j]);
		}

		target.m_vecOneRingVH.erase(target.m_vecOneRingVH.begin());
	}
}

void MeshSpike::fillSpikeBySmoothing()
{
	// spike의 모든 정점들을 spike의 평균점으로 축소
}

void MeshSpike::deleteAllSpikes(int option)
{
	int input = 1;

	switch (option)
	{
	case 1:
		for (int i = 0; i < m_vecSpikes.size(); i++)
		{
			fillSpikeBySimpleDelete(i);
		}
		break;
	case 2:
		cout << "Select Option n(n > 0, n : Nth-Ring Smoothing) : ";
		cin >> input;
		for (int i = 0; i < m_vecSpikes.size(); i++)
		{
			fillSpikeBySmoothingNRing(i, input);
		}
		break;
	case 3:
		fillSpikeBySmoothing();
		break;

	default:
		cout << "Not Option\n";
		break;
	}

	findAllSplikes();
}

void MeshSpike::deleteSpike(int idx, int option)
{
	int input;

	switch (option)
	{
	case 1 :
		fillSpikeBySimpleDelete(idx);
		break;
	case 2 :
		cout << "Select Option n(n > 0, n : Nth-Ring Smoothing) : ";
		cin >> input;
		fillSpikeBySmoothing();
		break;
	case 3 :
		//FillSpikeByFillingHole(idx);
		break;

	default:
		cout << "Not Option\n";
		break;
	}

	findAllSplikes();
}

void MeshSpike::saveMesh(string name)
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