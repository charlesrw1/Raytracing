#ifndef DEF_H
#define DEF_H

#include "Math.h"
#include <cmath>

using u32 = unsigned int;

class Material;

struct Intersection
{
	Intersection() {}
	Intersection(vec3 point, Ray r, vec3 outward_normal, const Material* material)
		: point(point), w0(-r.dir), material(material)
	{
		front_face = dot(r.dir, outward_normal) < 0;
		normal = (front_face) ? outward_normal : -outward_normal;
	}
	vec3 point;
	vec3 normal;
	vec3 w0;		// -(incoming ray dir)
	float t;
	bool front_face;

	float u = 0, v = 0;	// texture coordinates

	int index = 0;
	const Material* material = nullptr;

	void set_face_normal(Ray r, vec3 outward) {
		front_face = dot(r.dir, outward) < 0;
		normal = (front_face) ? outward : -outward;
	}
	void set_forward_face(Ray r, vec3 outward) {
		bool front = dot(r.dir, outward) < 0;
		front_face = true;
		normal = (front) ? outward : -outward;
	}
};

#endif // !DEF_H
