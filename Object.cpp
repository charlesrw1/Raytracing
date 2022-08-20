#include "Object.h"

Bounds tri_bounds(vec3 v1, vec3 v2, vec3 v3)
{
	Bounds b(v1);
	b = bounds_union(b, v2);
	b = bounds_union(b, v3);
	return b;
}

void bvh_from_triangles(BVH* bvh, const Mesh& mesh)
{
	std::vector<Bounds> b;
	for (int i = 0; i < mesh.indicies.size(); i += 3) {
		b.push_back(tri_bounds(
			mesh.verticies[mesh.indicies[i]].position,
			mesh.verticies[mesh.indicies[i + 1]].position,
			mesh.verticies[mesh.indicies[i + 2]].position
		));
	}
	*bvh = BVH::build(b, 1,BVH_SAH);
}

TriangleMesh::TriangleMesh(const Mesh* mesh) : mesh(mesh)
{
	bvh_from_triangles(&bvh, *mesh);

	mesh_bounds = bvh.nodes[0].aabb;
}
