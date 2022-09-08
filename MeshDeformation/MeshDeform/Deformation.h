#pragma once
#include "../Utility/baseHeadFile.h"
#include "../MeshDisplay/MyTriMesh.h"

class Deformation
{
public:
	TriMesh m_mesh;
	TriMesh* m_pMesh;

public:
	Deformation(void);
	virtual ~Deformation(void);

public:
	virtual void MyDeformation() = 0;
	virtual void reset() = 0;
	virtual void InteracTransform(const Matrix4d mat, bool localTransform = true) = 0;
};