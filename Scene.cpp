#include "Scene.h"
#include "BVH.h"

void Scene::build_top_level_bvh()
{
	// Get object bounds
	std::vector<Bounds> bounds;
	for (int i = 0; i < instances.size(); i++)
		bounds.push_back(instances[i].get_world_bounds());

	tlas = BVH::build(bounds, 1);

}

bool Scene::closest_hit(Ray r, Intersection* res) const
{
	const BVHNode* stack[64];
	stack[0] = &tlas.nodes[0];

	int stack_count = 1;
	const BVHNode* node = nullptr;

	float t = INFINITY;

	while (stack_count > 0)
	{
		node = stack[--stack_count];

		if (node->count != BVH_BRANCH)
		{
			int index_count = node->count;
			int index_start = node->left_node;
			for (int i = 0; i < index_count; i++) {
				const Instance& obj = instances[tlas.indicies[index_start + i]];
				bool hit = obj.intersect(r, 0, t, res);

				if (!hit)
					continue;
				t = res->t;
				res->index = tlas.indicies[index_start + i];
			}
			continue;
		}

		float left_dist, right_dist;
		bool left_aabb = tlas.nodes[node->left_node].aabb.intersect(r, left_dist);
		bool right_aabb = tlas.nodes[node->left_node + 1].aabb.intersect(r, right_dist);
		left_aabb = left_aabb && left_dist < t;
		right_aabb = right_aabb && right_dist < t;

		if (left_aabb && right_aabb) {
			if (left_dist < right_dist) {
				stack[stack_count++] = &tlas.nodes[node->left_node + 1];
				stack[stack_count++] = &tlas.nodes[node->left_node];
			}
			else {
				stack[stack_count++] = &tlas.nodes[node->left_node];
				stack[stack_count++] = &tlas.nodes[node->left_node + 1];
			}
		}
		else if (left_aabb) {
			stack[stack_count++] = &tlas.nodes[node->left_node];
		}
		else if (right_aabb) {
			stack[stack_count++] = &tlas.nodes[node->left_node + 1];
		}
	}

	if (!std::isfinite(t)) {
		return false;
	}

	res->w0 = -r.dir;
	return true;
}