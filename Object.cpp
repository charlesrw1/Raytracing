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
	*bvh = BVH::build(b, 3);
}

TriangleMesh::TriangleMesh(const Mesh* mesh) : mesh(mesh)
{
	bvh_from_triangles(&bvh, *mesh);

	mesh_bounds = bvh.nodes[0].aabb;
	/*for (int i = 0; i < bvh.nodes.size(); i++) {
		if (bvh.nodes[i].count == BVH_BRANCH)
			continue;
		int start = bvh.nodes[i].left_node;
		int count = bvh.nodes[i].count;

		int new_start = flattened_triangles.size();
		for (int j = 0; j < count; j++) {
			int mesh_element_idx = bvh.indicies[start + j] * 3;
			flattened_triangles.push_back(mesh->verticies[mesh->indicies[mesh_element_idx]].position);
			flattened_triangles.push_back(mesh->verticies[mesh->indicies[mesh_element_idx+1]].position);
			flattened_triangles.push_back(mesh->verticies[mesh->indicies[mesh_element_idx+2]].position);


		}
		int new_count = count * 3;
		bvh.nodes[i].left_node = new_start;
		bvh.nodes[i].count = new_count;
	}*/

}
