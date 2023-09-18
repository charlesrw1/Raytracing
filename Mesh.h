#ifndef MESH_H
#define MESH_H

#include "Def.h"
#include "BVH.h"
#include <vector>

struct Vertex
{
	vec3 position;
	vec3 normal;
	vec2 uv;
};

struct MeshTriangle
{
	uint32_t indicies[3] = { 0,0,0 };
	uint32_t materialIndex = 0;
};

class Mesh
{
public:
	void build();

	std::vector<Vertex> verts;
	std::vector<MeshTriangle> triangles;
	BVH accelStructure;
};


#endif // !MESH_H
