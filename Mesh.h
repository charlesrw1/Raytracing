#ifndef MESH_H
#define MESH_H

#include "Def.h"
#include <vector>

struct Vertex
{
	vec3 position;
	vec3 normal;
	vec2 uv;
};

class Mesh
{
public:
	std::vector<Vertex> verticies;
	std::vector<int> indicies;
	Bounds mesh_aabb;
};

Mesh* import_mesh(const char* obj_file);


#endif // !MESH_H
