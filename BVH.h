#ifndef BVH_H
#define BVH_H

#include "Def.h"
#include <vector>

enum PartitionStrategy
{
	BVH_MIDDLE,
	BVH_MEDIAN,
	BVH_SAH
};

#define BVH_BRANCH -1
struct BVHNode
{
	Bounds aabb;
	int left_node;	// points to start of indicies if count != BVH_BRANCH
	int count=BVH_BRANCH;	// if not a branch, count of indicies
};



class BVHBuilder;

class BVH
{
public:
	static BVH build(const std::vector<Bounds>& bounds, PartitionStrategy strat);

	std::vector<BVHNode> nodes;
	std::vector<int> indicies;
};

struct Mesh;

//void bvh_from_triangles(BVH* bvh, const Mesh& mesh);


#endif // !BVH
