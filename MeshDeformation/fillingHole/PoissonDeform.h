#pragma once

#include "../MeshDisplay/MyTriMesh.h"
#include "../fillingHole/FillHole.h"

class PoissonDeform
{
public:
	TriMesh tmp;
	TriMesh& mesh = tmp;
	vector<Hole> meshHoles;

	PoissonDeform(TriMesh& _mesh, vector<Hole> _meshHoles);
	void deform();
	TriMesh getMesh()
	{
		return mesh;
	}
};