#include "stdafx.h"

#include "../fillingHole/PoissonDeform.h"
#include "../MeshDeform/PoissonDeform/PoissonDeformation.h"
#include "FillHole.h"
#include <igl/exact_geodesic.h>
#include <igl/list_to_matrix.h>

vec3 crossProduct(vec3 v1, vec3 v2)
{
	return vec3
	(
		v1[1] * v2[2] - v1[2] * v2[1],
		v1[2] * v2[0] - v1[0] * v2[2],
		v1[0] * v2[1] - v1[1] * v2[0]
	);
}

double dotProduct(vec3 v1, vec3 v2)
{
	double sum = 0.0;

	sum += v1[0] * v2[0];
	sum += v1[1] * v2[1];
	sum += v1[2] * v2[2];

	return sum;
}

mat3 rotateAlign(vec3 v1, vec3 v2)
{
	vec3 axis = crossProduct(v1, v2);

	const float cosA = dotProduct(v1, v2);
	const float k = 1.0f / (1.0f + cosA);

	vec3 tmp((axis[0] * axis[0] * k) + cosA, (axis[1] * axis[0] * k) - axis[2], (axis[2] * axis[0] * k) + axis[1]);
	vec3 tmp1((axis[0] * axis[1] * k) + axis[2], (axis[1] * axis[1] * k) + cosA, (axis[2] * axis[1] * k) - axis[0]);
	vec3 tmp2((axis[0] * axis[2] * k) - axis[1], (axis[1] * axis[2] * k) + axis[0], (axis[2] * axis[2] * k) + cosA);

	mat3 result(tmp, tmp1, tmp2);

	return result;
}

vec3 getG(TriMesh::Point v1, TriMesh::Point v2, TriMesh::Point v3)
{
	return vec3((v1[0] + v2[0] + v3[0]) / 3,
		(v1[1] + v2[1] + v3[1] / 3),
		(v1[2] + v2[2] + v3[2] / 3));
}

void Deformation(TriMesh& mesh, Hole& hole, vector<bool>& isBoundaryVertex, VectorXd divMatri_x, VectorXd divMatri_y, VectorXd divMatri_z)
{
	MeshLaplaceSolver myLPLsolver;
	cout << "compute laplacianMatrix start\n";
	myLPLsolver.SetDesMesh(mesh);
	myLPLsolver.SetControlVertex(isBoundaryVertex);
	myLPLsolver.ComputeLalacianMatrixA(hole);
	cout << "compute laplacianMatrix end\n";
	cout << "solve start\n";
	myLPLsolver.SetRightHandB(divMatri_x);
	VectorXd x = myLPLsolver.LplacianSolve();

	//solve y
	myLPLsolver.SetRightHandB(divMatri_y);
	VectorXd y = myLPLsolver.LplacianSolve();

	//solve z
	myLPLsolver.SetRightHandB(divMatri_z);
	VectorXd z = myLPLsolver.LplacianSolve();
	cout << "solve end\n";
	//update mesh
	TriMesh::VertexHandle vh;

	for (int i = 0; i < hole.m_newVertices.size(); i++)
	{
		vh = hole.m_newVertices[i];

		mesh.point(vh)[0] = x[vh.idx()];
		mesh.point(vh)[1] = y[vh.idx()];
		mesh.point(vh)[2] = z[vh.idx()];
	}
}

map<int, int> mapOldVIndexToNewVIndex;
map<int, int> mapOldFIndexToNewFIndex;

TriMesh SaveFVtoMesh(vector<vector<double>> vertices, vector<vector<int>> faces)
{
	TriMesh mesh;
	vector<TriMesh::VertexHandle> vh;
	vector<TriMesh::VertexHandle> face_vhandles;
	vh.resize(vertices.size());

	for (int i = 0; i < vertices.size(); i++)
		vh[i] = mesh.add_vertex(TriMesh::Point(vertices[i][0], vertices[i][1], vertices[i][2]));
	for (int i = 0; i < faces.size(); i++)
	{
		face_vhandles.clear();
		for (int j = 0; j < 3; j++)
			face_vhandles.push_back(vh[faces[i][j]]);
		mesh.add_face(face_vhandles);
	}

	return mesh;
}
void ComputeEuclidianDistance(TriMesh& mesh, Hole& hole, vector<vector<double>>& euclidianDistance)
{
	vector<TriMesh::Point> newFacePos, originalFacePos;
	TriMesh::Point pos;
	for (int i = 0; i < hole.m_newFaces.size(); i++)
	{
		pos = TriMesh::Point(0);
		auto fv_itr = mesh.fv_iter(hole.m_newFaces[i]);
		for (; fv_itr.is_valid(); ++fv_itr) {
			pos += mesh.point(*fv_itr);
		}
		newFacePos.push_back(pos / 3);
	}
	for (int i = 0; i < hole.m_originalFaces.size(); i++)
	{
		pos = TriMesh::Point(0);
		auto fv_itr = mesh.fv_iter(hole.m_originalFaces[i]);
		for (; fv_itr.is_valid(); ++fv_itr) {
			pos += mesh.point(*fv_itr);
		}
		originalFacePos.push_back(pos / 3);
	}
	euclidianDistance.resize(hole.m_newFaces.size());
	for (int i = 0; i < hole.m_newFaces.size(); i++) {
		euclidianDistance[i].resize(hole.m_originalFaces.size());
		for (int j = 0; j < hole.m_originalFaces.size(); j++)
			euclidianDistance[i][j] = (newFacePos[i] - originalFacePos[j]).length();
	}

}


void ComputeGeodesicDistance(TriMesh& mesh, Hole& hole, vector<vector<double>>& geodesicDistance)
{
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;
	Eigen::VectorXi VT, FT;
	Eigen::VectorXd d;


	vector<vector<double>> v_tmp;
	vector<vector<int>> f_tmp;
	vector<int> vt_tmp;
	vector<int> ft_tmp;
	mapOldVIndexToNewVIndex.clear();
	mapOldFIndexToNewFIndex.clear();

	int newVIndex = 0;
	int newFIndex = 0;


	for (int i = 0; i < hole.m_originalFaces.size(); i++)
	{
		mapOldFIndexToNewFIndex[hole.m_originalFaces[i].idx()] = newFIndex;
		newFIndex++;
		TriMesh::FaceHandle f = hole.m_originalFaces[i];
		vector<int> tmp;

		for (auto fv_it = mesh.fv_begin(f); fv_it != mesh.fv_end(f); fv_it++)
		{
			if (mapOldVIndexToNewVIndex.find(fv_it->idx()) == mapOldVIndexToNewVIndex.end())
			{
				mapOldVIndexToNewVIndex[fv_it->idx()] = newVIndex;
				newVIndex++;
				TriMesh::Point p = mesh.point(*fv_it);
				vector<double> tmp;

				tmp.push_back(p[0]);
				tmp.push_back(p[1]);
				tmp.push_back(p[2]);

				v_tmp.push_back(tmp);
			}
			int oldFIndex1 = fv_it->idx();
			int newFIndex1 = mapOldVIndexToNewVIndex[fv_it->idx()];
			tmp.push_back(mapOldVIndexToNewVIndex[fv_it->idx()]);
		}

		f_tmp.push_back(tmp);

	}


	for (int i = 0; i < hole.m_newFaces.size(); i++)
	{
		mapOldFIndexToNewFIndex[hole.m_newFaces[i].idx()] = newFIndex;
		newFIndex++;
		TriMesh::FaceHandle f = hole.m_newFaces[i];
		vector<int> tmp;

		for (auto fv_it = mesh.fv_begin(f); fv_it != mesh.fv_end(f); fv_it++)
		{
			if (mapOldVIndexToNewVIndex.find(fv_it->idx()) == mapOldVIndexToNewVIndex.end())
			{
				mapOldVIndexToNewVIndex[fv_it->idx()] = newVIndex;
				newVIndex++;
				TriMesh::Point p = mesh.point(*fv_it);
				vector<double> tmp;

				tmp.push_back(p[0]);
				tmp.push_back(p[1]);
				tmp.push_back(p[2]);

				v_tmp.push_back(tmp);
			}
			tmp.push_back(mapOldVIndexToNewVIndex[fv_it->idx()]);
		}

		f_tmp.push_back(tmp);

	}

	for (int i = 0; i < hole.m_vecOriginalVH_PositionPair.size(); i++)
	{
		vt_tmp.push_back(mapOldVIndexToNewVIndex[hole.m_vecOriginalVH_PositionPair[i].first.idx()]);
	}

	for (int i = 0; i < hole.m_originalFaces.size(); i++)
	{
		ft_tmp.push_back(mapOldFIndexToNewFIndex[hole.m_originalFaces[i].idx()]);
	}


	igl::list_to_matrix(v_tmp, V);	// V matrix에 점의 좌표 입력 (행 : 점, 열 : 좌표값(double 3개))
	igl::list_to_matrix(f_tmp, F);  // F matrix에 face에 포함된 vertex의 idx 입력
	//igl::list_to_matrix(vt_tmp, VT);	// VT 에 boundary Verteices index 입력
	VT.resize(0);
	igl::list_to_matrix(ft_tmp, FT);	// FT 에 boundary Faces index 입력

	//TriMesh meshForDistanceCompute = SaveFVtoMesh(v_tmp, f_tmp);
	//saveMesh(meshForDistanceCompute, "meshForGeodesicDistance", true);

	for (int i = 0; i < hole.m_newFaces.size(); i++)
	{
		TriMesh::FaceHandle fh = hole.m_newFaces[i];

		Eigen::VectorXi VS, FS;
		VS.resize(0);
		FS.resize(1);

		auto it = mesh.fv_begin(fh);


		FS[0] = mapOldFIndexToNewFIndex[fh.idx()];


		//igl::exact_geodesic(V, F, VS, FS, VT, FT, d);
		igl::exact_geodesic(V, F, VS, FS, VT, FT, d);


		vector<double> tmp;


		it = mesh.fv_begin(fh);

		for (int k = 0; k < d.size(); k++)
		{
			tmp.push_back(d[k]);
		}

		geodesicDistance[i] = tmp; // geodesicDistance (face index) 마다 해당 Face의 모든 d 값 저장
	}
}


vec3 toVec3(TriMesh::Point from);

void MeshPoissonDeformation(TriMesh& mesh, Hole& hole)
{
#pragma region geodesic, controlVertex Settings
	mesh.update_normals();
	vector<vector<double>> geodesicDistance(hole.m_newFaces.size());
	//vector<vector<double>> geodesicDistance1(mesh.n_faces());

	//ComputeGeodesicDistance(mesh, hole, geodesicDistance); // V, F에 데이터 입력
	cout << "compute distance start\n";
	ComputeEuclidianDistance(mesh, hole, geodesicDistance); // V, F에 데이터 입력
	cout << "compute distance end\n";
	// calc geodesicdistance

	VectorXd divMatri_x = VectorXd::Zero(mesh.n_vertices());
	VectorXd divMatri_y = VectorXd::Zero(mesh.n_vertices());
	VectorXd divMatri_z = VectorXd::Zero(mesh.n_vertices());

	vector<bool> isBoundaryVertex;
	isBoundaryVertex.resize(mesh.n_vertices(), true);

	for (int i = 0; i < hole.m_newVertices.size(); i++)
	{
		isBoundaryVertex[hole.m_newVertices[i].idx()] = false;
	}

	PoissonDeformation PD(&mesh);	// poissonDeformation 객체 생성
#pragma endregion


	int vid = 0, l = 0, r = 0;
	TriMesh::VertexHandle vh0, vh1, vh2;

	vector<pair<TriMesh::FaceHandle, Vector3D>> result_Fh_Vec3D; // face handle & result vec3d
	cout << "compute divmatrix start\n";
	for (int i = 0; i < hole.m_newFaces.size(); i++) // new Verteices
	{
		TriMesh::FaceVertexIter fv_it = mesh.fv_begin(hole.m_newFaces[i]);
		vector<vec3> vertices_vec3;

		vid = mesh.fv_begin(hole.m_newFaces[i])->idx();

		int tri_vid0 = fv_it.handle().idx(); fv_it++;
		int tri_vid1 = fv_it.handle().idx(); fv_it++;
		int tri_vid2 = fv_it.handle().idx();

		if (tri_vid0 == vid) { l = tri_vid1; r = tri_vid2; }
		else if (tri_vid1 == vid) { l = tri_vid0; r = tri_vid2; }
		else { l = tri_vid0; r = tri_vid1; }
		vh0 = mesh.vertex_handle(vid);
		vh1 = mesh.vertex_handle(l);
		vh2 = mesh.vertex_handle(r);

		vertices_vec3.push_back(toVec3(mesh.point(vh0)));
		vertices_vec3.push_back(toVec3(mesh.point(vh1)));
		vertices_vec3.push_back(toVec3(mesh.point(vh2)));

		TriMesh::Normal currentNormal = mesh.normal(hole.m_newFaces[i]);	// this interior triangle의 normal

		TriMesh::Normal desiredNormal(0); // new normal

		for (int j = 0; j < hole.m_originalFaces.size(); j++)
		{
			TriMesh::Normal boundaryNormal = mesh.normal(hole.m_originalFaces[j]);
			//desiredNormal += boundaryNormal / geodesicDistance[hole.m_newFaces[i].idx()][j];
			desiredNormal += boundaryNormal / (geodesicDistance[i][j] * geodesicDistance[i][j]);
		}

		vec3 vec3CurrentNormal(currentNormal[0], currentNormal[1], currentNormal[2]);
		vec3 vec3DesiredNormal(desiredNormal[0], desiredNormal[1], desiredNormal[2]);
		vec3DesiredNormal.normalize();

		mat3 rotateAlign_mat3 = rotateAlign(vec3CurrentNormal, vec3DesiredNormal); // rotateAlign

		/*
		삼각형의 무게중심이 원점으로 가도록 이동하고 mat3행렬을 곱해서 회전하고 다시 원점이 원래 무게중심 위치로 가도록 이동
		*/

		vec3 G = (vertices_vec3[0] + vertices_vec3[1] + vertices_vec3[2]) / 3;	// 무게 중심 계산

		vec3 result_vh0 = (rotateAlign_mat3 * (vertices_vec3[0] - G)) + G;
		vec3 result_vh1 = (rotateAlign_mat3 * (vertices_vec3[1] - G)) + G;
		vec3 result_vh2 = (rotateAlign_mat3 * (vertices_vec3[2] - G)) + G;


		//triangle local transform 
		Point3D source_, right_, left_;
		source_.m_x = result_vh0[0];
		source_.m_y = result_vh0[1];
		source_.m_z = result_vh0[2];

		left_.m_x = result_vh1[0];
		left_.m_y = result_vh1[1];
		left_.m_z = result_vh1[2];

		right_.m_x = result_vh2[0];
		right_.m_y = result_vh2[1];
		right_.m_z = result_vh2[2];

		//compute divergence
		Vector3D W = PD.ComputeTriangleDiv(source_, left_, right_);

		result_Fh_Vec3D.push_back(make_pair(hole.m_newFaces[i], W));

		divMatri_x[vid] += W.m_x;
		divMatri_y[vid] += W.m_y;
		divMatri_z[vid] += W.m_z;

		W = PD.ComputeTriangleDiv(left_, right_, source_);

		divMatri_x[l] += W.m_x;
		divMatri_y[l] += W.m_y;
		divMatri_z[l] += W.m_z;

		W = PD.ComputeTriangleDiv(right_, source_, left_);

		divMatri_x[r] += W.m_x;
		divMatri_y[r] += W.m_y;
		divMatri_z[r] += W.m_z;
	}

	for (int i = 0; i < hole.m_vecOriginalVH_PositionPair.size(); i++)
	{
		TriMesh::Point p = mesh.point(hole.m_vecOriginalVH_PositionPair[i].first);

		divMatri_x[hole.m_vecOriginalVH_PositionPair[i].first.idx()] = p[0];
		divMatri_y[hole.m_vecOriginalVH_PositionPair[i].first.idx()] = p[1];
		divMatri_z[hole.m_vecOriginalVH_PositionPair[i].first.idx()] = p[2];
	}

	cout << "compute divmatrix end\n";
	Deformation(mesh, hole, isBoundaryVertex, divMatri_x, divMatri_y, divMatri_z);
}

PoissonDeform::PoissonDeform(TriMesh& _mesh, vector<Hole> _meshHoles) : mesh(_mesh), meshHoles(_meshHoles)
{

}

void PoissonDeform::deform()
{
	cout << "=====MeshDeformation Start=====\n";
	cout << "Unit Hole Count : " << meshHoles.size() << endl;

	for (int i = 0; i < meshHoles.size(); i++)
	{
		bool isDeformed = false;

		if (meshHoles[i].m_isFilled)
		{
			MeshPoissonDeformation(mesh, meshHoles[i]);
			isDeformed = true;
		}

		if (isDeformed)
			cout << i << "번째 Hole MeshDeformation 완료" << endl;
	}

	cout << "MeshDeformation 작업 완료" << endl;
}