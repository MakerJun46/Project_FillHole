#include "stdafx.h"


#include "PoissonDeformation.h"
#include "igl/exact_geodesic.h"
#include "../../Utility/MyAfxFunction.h"
#include "../../Utility/baseHeadFile.h"
#include "../../Utility/CP_PointVector.h"

PoissonDeformation::PoissonDeformation(TriMesh* mesh)
{
	m_pMesh = mesh;
	//m_mesh = *mesh;
	//m_static_mesh = *mesh;
	m_handTransMat = Matrix4d::Identity();
	m_quater_fixed.R2Q(0, 1, 0, 0);
}

void PoissonDeformation::reset()
{
	m_mesh = m_static_mesh;
	m_handTransMat = Matrix4d::Identity();
}

void PoissonDeformation::InteracTransform(const Matrix4d mat, bool localTransform)
{
	m_handTransMat = mat * m_handTransMat;
	m_quater_hand.RotationMatrix2Qua(m_handTransMat);
}

void PoissonDeformation::DeformReset()
{
	if (!m_static_mesh.vertices_empty())
	{
		m_mesh = m_static_mesh;
		afxGetMainMesh()->UpdateDisList(&m_static_mesh);
		m_handTransMat = Matrix4d::Identity();
	}
}

//compute laplacian matrix
void PoissonDeformation::ComputeCoefficientMatrix()
{

}

//compute divergence
Vector3D PoissonDeformation::ComputeTriangleDiv(const Point3D& source, const Point3D& vleft, const Point3D& vright)
{
	Vector3D s_l = source - vleft;
	Vector3D s_r = source - vright;
	Vector3D l_s = s_l * (-1);
	Vector3D l_r = vleft - vright;
	Vector3D r_s = s_r * (-1);
	Vector3D r_l = vright - vleft;

	//¨Œ¦µiT
	Vector3D ha = SimpleCompute::GetTriangleVertexGradient(s_l, s_r);
	Vector3D hb = SimpleCompute::GetTriangleVertexGradient(l_r, l_s);
	Vector3D hc = SimpleCompute::GetTriangleVertexGradient(r_s, r_l);

	//gradient field
	Vector3D wx = hb * (l_s.m_x) + hc * (r_s.m_x);
	Vector3D wy = hb * (l_s.m_y) + hc * (r_s.m_y);
	Vector3D wz = hb * (l_s.m_z) + hc * (r_s.m_z);

	//S¡÷
	double area = SimpleCompute::GetTriangleArea(source, vleft, vright);

	//divergence
	Vector3D div = Vector3D(wx * ha * area, wy * ha * area, wz * ha * area);
	return   div;
}

void PoissonDeformation::TriangleLocalTransform(TriMesh::VertexHandle vh_s, TriMesh::VertexHandle vh_l, TriMesh::VertexHandle vh_r,
	Point3D& source, Point3D& left, Point3D& right)
{
	Matrix4d triangleTransMatrix = Matrix4d::Identity();
	Matrix4d interpMat = Matrix4d::Identity();

	int s = vh_s.idx(), l = vh_l.idx(), r = vh_r.idx();
	Point3D v = Point3D(m_pMesh->point(vh_s)[0], m_pMesh->point(vh_s)[1], m_pMesh->point(vh_s)[2]);
	Point3D	vleft = Point3D(m_pMesh->point(vh_l)[0], m_pMesh->point(vh_l)[1], m_pMesh->point(vh_l)[2]);
	Point3D vright = Point3D(m_pMesh->point(vh_r)[0], m_pMesh->point(vh_r)[1], m_pMesh->point(vh_r)[2]);
	Point3D center((v.m_x + vleft.m_x + vright.m_x) / 3.0, (v.m_y + vleft.m_y + vright.m_y) / 3.0, (v.m_z + vleft.m_z + vright.m_z) / 3.0);

	//ensure local transform
	Matrix4d  t1 = SimpleCompute::Translate2Matrix(center.m_x, center.m_y, center.m_z);
	Matrix4d  t2 = SimpleCompute::Translate2Matrix(-center.m_x, -center.m_y, -center.m_z);

	//interpolation with bc`s geodesic distance 
	double factor = 0;

	//quaternuon  interpolation
	CQuaternion quater;
	quater.Slerp(m_quater_hand, m_quater_fixed, factor);
	quater.Q2R(interpMat);

	//local transform
	triangleTransMatrix = t1 * interpMat * t2;

	source = SimpleCompute::ComputeMatrixMultiPoint(triangleTransMatrix, v);
	left = SimpleCompute::ComputeMatrixMultiPoint(triangleTransMatrix, vleft);
	right = SimpleCompute::ComputeMatrixMultiPoint(triangleTransMatrix, vright);
}

void PoissonDeformation::ComputeDivergence()
{

}

void PoissonDeformation::MyDeformation()
{



	ComputeDivergence();
	myLPLslover.SetRightHandB(divMatri_x);
	VectorXd x = myLPLslover.LplacianSolve();

	//solve y
	myLPLslover.SetRightHandB(divMatri_y);
	VectorXd y = myLPLslover.LplacianSolve();

	//solve z
	myLPLslover.SetRightHandB(divMatri_z);
	VectorXd z = myLPLslover.LplacianSolve();

	//update mesh
	TriMesh::VertexHandle vh;

	for (int i = 0; i < m_pMesh->n_vertices(); i++)
	{
		vh = m_pMesh->vertex_handle(i);

		m_pMesh->point(vh)[0] = x[i];
		m_pMesh->point(vh)[1] = y[i];
		m_pMesh->point(vh)[2] = z[i];
	}

	m_handTransMat = Matrix4d::Identity();
	afxGetMainMesh()->UpdateDisList(m_pMesh);


}