#pragma once

#include "../Deformation.h"
#include "../../Utility/MeshLaplacianSolver.h"
//#include "ControlVertex.h"
#include "../../Utility/baseHeadFile.h"
#include "../../Utility/CP_PointVector.h"
#include "../../Utility/quaternion.h"
#include <vector>

class PoissonDeformation : public Deformation
{
private:
	TriMesh  m_static_mesh;
	VectorXd divMatri_x;
	VectorXd divMatri_y;
	VectorXd divMatri_z;
	MeshLaplaceSolver myLPLslover;
	CQuaternion m_quater_fixed;
	CQuaternion m_quater_hand;
	Matrix4d m_handTransMat;

public:

public:
	PoissonDeformation(TriMesh* mesh);
	~PoissonDeformation(void) {};

	//����ӿ�
public:
	void MyDeformation();
	void reset();
	void InteracTransform(const Matrix4d mat, bool localTransform = true);

	//�����㷨
public:
	void ComputeCoefficientMatrix();
	void ComputeDivergence();
	Vector3D ComputeTriangleDiv(const Point3D& source, const Point3D& vleft, const Point3D& vright);
	void TriangleLocalTransform(TriMesh::VertexHandle vh_s, TriMesh::VertexHandle vh_l, TriMesh::VertexHandle vh_r,
		Point3D& source, Point3D& left, Point3D& right);
	void DeformReset();
};