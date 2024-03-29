#include "BVH.h"
#include "Mesh.h"
#include <algorithm>

class BVHBuilder
{
public:
	BVHBuilder(const std::vector<Bounds>& bounds, int max_per_leaf, PartitionStrategy strat) 
		: bounds(bounds), strat(strat), max_per_leaf(max_per_leaf)
	{
		indicies.resize(bounds.size());
		centroids.resize(bounds.size());
		for (int i = 0; i < bounds.size(); i++)
			indicies[i] = i;
		for (int i = 0; i < bounds.size(); i++)
			centroids[i] = bounds[i].get_center();
	}
	void build_R(int start, int end, int node_num);

	int add_new_node() {
		nodes.resize(nodes.size() + 1);
		return nodes.size() - 1;
	}
	Bounds calc_bounds(int start, int end) {
		Bounds b;
		for (int i = start; i < end; i++)
			b = bounds_union(b, bounds[indicies[i]]);
		return b;
	}
	float EvalSAH(int start, int end, int axis, float pos) const {
		Bounds left, right;
		int left_count = 0, right_count = 0;
		for (int i = start; i < end; i++)
		{
			if (centroids[indicies[i]][axis] < pos) {
				left_count++;
				left = bounds_union(left, bounds[indicies[i]]);
			}
			else {
				right_count++;
				right = bounds_union(right, bounds[indicies[i]]);
			}
		}
		float cost = left_count * left.surface_area() + right_count * right.surface_area();
		return cost > 0 ? cost : 1e30f;
	}


	std::vector<vec3> centroids;
	std::vector<BVHNode> nodes;
	std::vector<int> indicies;
	const std::vector<Bounds>& bounds;
	
	PartitionStrategy strat;
	int max_per_leaf;
};

void BVHBuilder::build_R(int start, int end, int node_number)
{
	assert(start >= 0 && end <= bounds.size());
	int num_elements = end - start;
	BVHNode* node = &nodes[node_number];
	node->aabb = calc_bounds(start, end);

	//node->aabb.bmin -= vec3(BVH_EPSILON);
	//node->aabb.bmax += vec3(BVH_EPSILON);

	if (num_elements <= max_per_leaf) {
		node->left_node = start;
		node->count = num_elements;
		return;
	}
	int split{};
	
	if (strat == BVH_MIDDLE) {
		vec3 bounds_center = node->aabb.get_center();
		int longest_axis = node->aabb.longest_axis();
		float mid = bounds_center[longest_axis];

		auto split_iter = std::partition(indicies.begin() + start, indicies.begin() + end,
			[longest_axis, mid, this](int index) {
				return this->centroids[index][longest_axis] < mid;
			}
		);
		split = split_iter - indicies.begin();
		if (split == start || split == end) {
			split = (start + end) / 2;
		}
	}
	else if(strat==BVH_SAH){
		float mid;
		int axis;
		float best_cost = 1e30f;
		for (int a = 0; a < 3; a++) {
			float bounds_min = node->aabb.bmin[a];
			float bounds_max = node->aabb.bmax[a];
			if (bounds_min == bounds_max)continue;
			float scale = (bounds_max - bounds_min) / 100.f;
			for (int i = 0; i < 100; i++) {
				//float centroid = centroids[indicies[start + i]][a];
				float pos = bounds_min + i * scale;
				float cost = EvalSAH(start, end, a, pos);
				if (cost < best_cost) {
					mid = pos;
					axis = a;
					best_cost = cost;
				}
			}
		}
		auto split_iter = std::partition(indicies.begin() + start, indicies.begin() + end,
			[axis, mid, this](int index) {
				return this->centroids[index][axis] < mid;
			}
		);
		split = split_iter - indicies.begin();
		if (split == start || split == end)
			split = (start + end) / 2;
	}
	
	node->count = BVH_BRANCH;
	// These may invalidate the pointer
	int left_child = add_new_node();
	int right_child = add_new_node();
	// So just set it again lol
	nodes[node_number].left_node = left_child;
	build_R(start, split, left_child);
	build_R(split, end, right_child);
}

// Static builder function
BVH BVH::build(const std::vector<Bounds>& bounds, int max_per_node, PartitionStrategy strat)
{
	BVHBuilder builder(bounds, max_per_node, strat);

	int start_node = builder.add_new_node();	// =0
	builder.build_R(0, bounds.size(), start_node);

	if (builder.nodes.size() == 0) {
		printf("Error on BVH construction\n");
		return BVH();
	}
	BVH bvh;
	bvh.nodes = std::move(builder.nodes);
	bvh.indicies = std::move(builder.indicies);

	std::cout << "Nodes: " << bvh.nodes.size() << '\n';

	return bvh;
}
