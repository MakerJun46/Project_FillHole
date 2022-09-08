#include "stdafx.h"
#include "MeshIsland.h"
#include <queue>

using namespace std;

MeshIsland::MeshIsland(TriMesh& mesh) : m_mesh(mesh)
{
	m_mesh.request_edge_status();
	m_mesh.request_face_status();
	m_mesh.request_vertex_status();

	findAllIsland();
}

bool cmp(Island& A, Island& B)
{
	return A.m_islandFaces.size() > B.m_islandFaces.size();
}

void MeshIsland::findAllIsland()
{
	m_vecIsland.clear();

	vector<TriMesh::FaceHandle> AllFaces;
	vector<bool> Visited(m_mesh.n_faces(), false);
	bool hasBoundary = false;

	for (auto it = m_mesh.faces_begin(); it != m_mesh.faces_end(); it++)
	{
		AllFaces.push_back(it);

		if (!hasBoundary && m_mesh.is_boundary(it))
			hasBoundary = true;
	}

	while (hasBoundary && find(Visited.begin(), Visited.end(), false) != Visited.end()) // 모든 face를 방문할 때까지 반복
	{
		Island tmpIsland;

		queue<TriMesh::FaceHandle> que;

		que.push(AllFaces[find(Visited.begin(), Visited.end(), false) - Visited.begin()]);

		while (!que.empty())
		{
			TriMesh::FaceHandle item = que.front();

			que.pop();

			auto it = find(AllFaces.begin(), AllFaces.end(), item);

			if (Visited[it - AllFaces.begin()]) // 이미 방문한 경우 pass
			{
				continue;
			}
			else if (it == AllFaces.end()) // 현재 que에 담긴 face가 allface 안에 없는 경우 : error
			{
				cout << "error : face가 mesh에 포함되지 않음\n";
			}

			tmpIsland.m_islandFaces.push_back(item);
			tmpIsland.m_Area += m_mesh.calc_face_area(item);
			tmpIsland.m_TriangleCount++;

			Visited[it - AllFaces.begin()] = true;

			for (auto he = m_mesh.fh_begin(item); he != m_mesh.fh_end(item); he++)
			{
				auto oppoHE = m_mesh.opposite_halfedge_handle(he);

				if (!m_mesh.is_boundary(oppoHE))
				{
					auto face = m_mesh.face_handle(oppoHE);

					que.push(face);
				}
			}
		}

		m_vecIsland.push_back(tmpIsland);
	}

	sortIsland();

	setConnectStatus();
}

void MeshIsland::sortIsland()
{
	sort(m_vecIsland.begin(), m_vecIsland.end(), cmp);

	for (int i = 0; i < m_vecIsland.size(); i++)
	{
		m_vecIsland[i].m_idx = i;
	}
}


void MeshIsland::setConnectStatus()
{
	for (Island& island : m_vecIsland)
	{
		for (auto fh : island.m_islandFaces)
		{
			for (auto it = m_mesh.fv_begin(fh); it != m_mesh.fv_end(fh); it++)
			{
				if (m_mesh.is_boundary(it))
					island.m_boundaryVertex.push_back(it); // boundary 인 vertex들만 담기
			}
		}

		for (auto vh : island.m_boundaryVertex)
		{
			for (auto it = m_mesh.vf_begin(vh); it != m_mesh.vf_end(vh); it++)
			{
				// island boundary vertex가 연결된 face가 현재 island faces에 없는 경우 => vertexConnection
				if (find(island.m_islandFaces.begin(), island.m_islandFaces.end(), it) == island.m_islandFaces.end())
				{
					// 어떤 island에 vertexconnection인지 판별
					for (Island& tmp : m_vecIsland)
					{
						if (tmp.m_idx == island.m_idx) // 현재 검사 중인 island는 pass
							continue;

						if (find(tmp.m_islandFaces.begin(), tmp.m_islandFaces.end(), it) != tmp.m_islandFaces.end())
						{
							if (tmp.m_idx == 0) // 0번째 idx(가장 큰 island에 연결된 경우 => ConnectionToMainMesh
							{
								island.m_connectOherIsland = 0;
								island.m_status = VertexConnectionToMainMesh;
							}
							else // 아닌 경우 => ConnectionToOtherIsland
							{
								island.m_connectOherIsland = tmp.m_idx;
								island.m_status = VertexconnectionToOtherIsland;
							}

							break;
						}
					}
				}
			}

			if (island.m_status != none)
				break;
		}

		if (island.m_status == none) // VertexConnection이 아닌 island는 모두 완전분리형 => CompletlySeperation
		{
			island.m_status = CompletlySeperation;
		}
	}
}

void MeshIsland::printAllInfo()
{
	if (!isMeshHasIsland())
	{
		cout << "=== This Mesh has No Island ===\n";
		return;
	}

	cout << "총 Island 개수 : " << m_vecIsland.size() << endl;

	for (Island& island : m_vecIsland)
	{
		cout << "======================================\n";
		cout << "Island " << island.m_idx << " Inforamtion\n";
		cout << "Triangle Count : " << island.m_TriangleCount << "\n";
		cout << "Vertex Count : " << island.m_islandFaces.size() * 3 << "\n";
		cout << "Island Area : " << island.m_Area << "\n";

		switch (island.m_status)
		{
		case CompletlySeperation:
			cout << "Connection Status : Completly Seperation\n";
			break;
		case VertexconnectionToOtherIsland:
			cout << "Connection Status : Vertex Connection to Other Island\n";
			cout << "Connect Island idx : " << island.m_connectOherIsland << "\n";
			break;
		case VertexConnectionToMainMesh:
			cout << "Connection Status : Vertex Connection to Main Mesh\n";
			break;
		default:
			break;
		}
	}

	cout << "======================================\n";
}

bool MeshIsland::isMeshHasIsland()
{
	return m_vecIsland.size() == 0;
}

void MeshIsland::eraseAllIsland()
{
	for (int i = 1; i < m_vecIsland.size(); i++)
	{
		m_mesh.request_face_status();
		m_mesh.request_vertex_status();

		for (int j = 0; j < m_vecIsland[i].m_islandFaces.size(); j++)
		{
			m_mesh.delete_face(m_vecIsland[i].m_islandFaces[j]);
		}

		m_vecIsland[i].m_boundaryVertex.clear();
		m_vecIsland[i].m_islandFaces.clear();
	}

	if (m_vecIsland.size() > 1)
		m_vecIsland.erase(m_vecIsland.begin() + 1);
}

void MeshIsland::eraseIsland(int idx)
{
	try
	{
		m_vecIsland.at(idx);

		m_mesh.request_face_status();
		m_mesh.request_vertex_status();

		for (int i = 0; i < m_vecIsland[idx].m_islandFaces.size(); i++)
		{
			m_mesh.delete_face(m_vecIsland[idx].m_islandFaces[i]);
		}

		m_vecIsland[idx].m_boundaryVertex.clear();
		m_vecIsland[idx].m_islandFaces.clear();
	}
	catch (exception e)
	{
		cout << "idx error" << endl;
	}

	findAllIsland();
}

void statusupdate(TriMesh& mesh)
{
	mesh.request_face_status();
	mesh.request_edge_status();
	mesh.request_vertex_status();
	mesh.request_halfedge_status();

	mesh.update_normals();
}

void MeshIsland::saveMesh(string name)
{
	m_mesh.garbage_collection();
	statusupdate(m_mesh);

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