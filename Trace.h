#ifndef TRACE_H
#define TRACE_H

#include "Math.h"

struct BVHNode
{
	vec3 box_min, box_max;
	unsigned int left_node, first_prim_idx, prim_count;
	bool is_leaf() { return prim_count > 0; }
};




class BVH
{

};



#endif // !TRACE_H
