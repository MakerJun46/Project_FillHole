#pragma once
//#include "windows.h"
#include "../Utility/baseHeadFile.h"
#include "../Utility/predefine.h"
#include "glut.h"

class MyTriMesh
{
protected:

private:
	bool m_updateDistList;

public:
	int m_disType;
	bool m_show;
	GLuint m_drawMesh;
	TriMesh m_mesh;

	//包围盒中心
	Point3D m_centerOfAABB;
	double boundBoxEdgeLen;
	vector<Vector3D> m_vertexNormal;
	Eigen::MatrixXd V;
	Eigen::MatrixXi F;

public:
	MyTriMesh(void);
	~MyTriMesh(void);

public:
	void init();
	void Draw();
	void LoadMesh();
	void LoadMeshAll();
	void SaveMesh();
	int vertexSize(){return (int) m_mesh.n_vertices();};
	int faceSize(){return (int) m_mesh.n_faces();};
	void SetMeshDisMode(int mode){m_disType = mode;UpdateDisList();};
	bool empty(){return m_mesh.vertices_empty();;};
	void DrawList();
	void UpdateDisList(TriMesh* tmesh = 0){if(tmesh){m_mesh = *tmesh;} m_updateDistList = true; };
	void SetMesh(TriMesh* mesh_);
	TriMesh* GetMesh(){return &m_mesh;};

	//网格分片显示配置
	void GetAABBofObject();
	void ReComputeVertexNormal();
	double GetAverageEdgeLength();
};

 