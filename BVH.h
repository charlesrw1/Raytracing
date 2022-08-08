#ifndef BVH_H
#define BVH_H

#include "Def.h"
#include <vector>

struct BVHNode
{
	Bounds aabb;
	u32 left_node;
	u32 element_index;
	u32 element_count;
};

class BVH
{
	static BVH build_bvh(Bounds* bounds, int num_bounds);
	static BVH build_bvh(Bounds* bounds, float* surface_area, int num_elements);



	std::vector<BVHNode> nodes;
};

#endif // !BVH
