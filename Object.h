#ifndef OBJECT_H
#define OBJECT_H

#include "Def.h"

class Geometry
{
public:
	virtual ~Geometry() {}
	//virtual Bounds object_bounds() const = 0;
	//virtual Bounds world_bounds() const;
	virtual bool intersect(Ray r, float tmin, float tmax, SurfaceInteraction* si) const = 0;
	//	virtual float area() const = 0;

};

class Sphere : public Geometry
{
public:
	Sphere(float radius)
		: radius(radius) {}

	virtual bool intersect(Ray r, float tmin, float tmax, SurfaceInteraction* si) const override {
		vec3 center = vec3(0);

		vec3 oc = r.pos - center;
		float a = dot(r.dir, r.dir);
		float halfb = dot(oc, r.dir);
		float c = dot(oc, oc) - radius * radius;
		float discriminant = halfb * halfb - a * c;
		if (discriminant < 0)
			return false;
		float sqrt_dist = sqrt(discriminant);
		float root = (-halfb - sqrt_dist) / a;
		if (root<tmin || root>tmax) {
			root = (-halfb + sqrt_dist) / a;
			if (root<tmin || root>tmax)
				return false;
		}
		si->t = root;
		si->point = r.at(root);
		si->set_face_normal(r, (si->point - center) / radius);
		return true;
	}/*
	virtual Bounds object_bounds() const override {
		return Bounds(vec3(-radius), vec3(radius));
	}*/

private:
	float radius;
};
class Rectangle : public Geometry
{
public:
	Rectangle(vec3 normal, vec3 tangent, float u_length, float v_length) {
		N = normal;
		T = tangent;
		B = normalize(cross(N, T));
		u = u_length / 2;
		v = v_length / 2;
	}
	Rectangle(vec3 normal, float u_length, float v_length) {
		N = normal;
		vec3 up = vec3(0, 1, 0);
		if (N.y < -0.999) up = vec3(1, 0, 0);
		else if (N.y > 0.999) up = vec3(0, 0, -1);
		T = normalize(cross(up, N));
		B = normalize(cross(N, T));
		u = u_length / 2;
		v = v_length / 2;
	}
	virtual bool intersect(Ray r, float tmin, float tmax, SurfaceInteraction* si) const override {
		// Intersect plane
		float denom = dot(N, r.dir);
		if (fabs(denom) < 0.01)	// parallel
			return false;
		float dist = dot(r.pos, N);
		float t = -dist / denom;

		if (t<tmin || t>tmax)
			return false;

		vec3 plane_intersect = r.pos + r.dir * t;
		// Check if its within tangents
		float u_dist = dot(plane_intersect, T);
		if (u_dist < -u || u_dist > u)
			return false;
		float v_dist = dot(plane_intersect, B);
		if (v_dist < -v || v_dist > v)
			return false;

		// Ray intersects plane
		si->point = plane_intersect;
		si->set_face_normal(r, N);
		si->u = u_dist / u;
		si->v = v_dist / v;
		si->t = t;
		return true;
	}/*
	virtual Bounds object_bounds() const override {
		Bounds b(T * -u);
		b = bounds_union(b, T * u);
		b = bounds_union(b, B * -v);
		b = bounds_union(b, B * v);

		return b;
	}*/
private:
	vec3 N, T, B;// normal, tangent, bitangent
	float u, v;	// half side lengths
};

class Disk : public Geometry
{
public:
	Disk(vec3 normal, float outer_radius, float inner_radius = -1.f) : normal(normal), outer_radius(outer_radius), inner_radius(inner_radius) {}
	virtual bool intersect(Ray r, float tmin, float tmax, SurfaceInteraction* si) const override {
		// Intersect plane
		float denom = dot(normal, r.dir);
		if (fabs(denom) < 0.01)	// parallel
			return false;
		float dist = dot(r.pos, normal);
		float t = -dist / denom;

		if (t<tmin || t>tmax)
			return false;

		vec3 plane_intersect = r.pos + r.dir * t;

		float dist_radius = plane_intersect.length();
		if (dist_radius > outer_radius || dist_radius < inner_radius)
			return false;

		si->point = plane_intersect;
		si->set_face_normal(r, normal);
		si->t = t;
		return true;
	}

private:
	vec3 normal;
	float outer_radius;
	float inner_radius;
};

class Box : public Geometry
{
public:
	Box(vec3 box_size) : bmin(box_size / -2.f), bmax(box_size / 2) {}

	virtual bool intersect(Ray r, float tmin, float tmax, SurfaceInteraction* si) const override {
		float greatest_minimum = -INFINITY;
		int gm_idx;

		for (int a = 0; a < 3; a++) {
			float t0 = fmin((bmin[a] - r.pos[a]) / r.dir[a],
				(bmax[a] - r.pos[a]) / r.dir[a]);
			float t1 = fmax((bmin[a] - r.pos[a]) / r.dir[a],
				(bmax[a] - r.pos[a]) / r.dir[a]);
			tmin = fmax(t0, tmin);
			tmax = fmin(t1, tmax);
			if (tmax <= tmin)
				return false;

			if (tmin > greatest_minimum) {
				greatest_minimum = tmin;
				gm_idx = a;
			}
		}

		int idx = gm_idx;
		float t = greatest_minimum;
		vec3 normal = vec3(0);
		normal[idx] = (r.dir[idx] > 0) ? -1.f : 1.f;
		si->point = r.at(t);
		si->set_face_normal(r, normal);
		si->t = t;

		return true;
	}


private:
	vec3 bmin, bmax;
};

class Cylinder : public Geometry
{
public:
	Cylinder(float radius, float height) : radius(radius), ymin(height / -2.f), ymax(height / 2.f) {}

	virtual bool intersect(Ray r, float tmin, float tmax, SurfaceInteraction* si) const override {
		float a = r.dir.x * r.dir.x + r.dir.z * r.dir.z;
		float b = 2 * (r.pos.x * r.dir.x + r.pos.z * r.dir.z);
		float c = r.pos.x * r.pos.x + r.pos.z * r.pos.z - radius * radius;
	
		float discriminant = b * b - 4 * a * c;
		if (discriminant < 0)
			return false;
		float sqrt_dist = sqrt(discriminant);
		float root = (-b - sqrt_dist) / (2 * a);
		vec3 point = r.pos + r.dir * root;
		bool inward = false;
		if (root<tmin || root>tmax || point.y < ymin || point.y>ymax) {
			root = (-b + sqrt_dist) / (2 * a);
			point = r.pos + r.dir * root;
			if (root<tmin || root>tmax || point.y < ymin || point.y > ymax)
				return false;
		}
		vec3 normal = vec3(point.x, 0, point.z) / radius;
		si->point = point;
		si->set_forward_face(r, normal);
		si->t = root;
		return true;
	}
private:
	float radius;
	float ymin, ymax;
};


class TriangleMesh : public Geometry
{
public:

private:
	// BVH for triangles
	// triangle list
};

class Instance
{
public:
	Instance(Geometry* geo, Material* mat, vec3 offset) : geometry(geo), material(mat)
	{
		mat4 temp_mat = translate(mat4(1), offset);
		transform = Transform(temp_mat);
	}
	Instance(Geometry* geo, Material* mat, mat4 transform) 
		: geometry(geo), material(mat), transform(transform) {}

	void free_data() {
		delete geometry;
		delete material;
	}

	bool intersect(Ray r, float tmin, float tmax, SurfaceInteraction* si) const {
		// Transform ray to model space
		Ray model_space_ray = transform.to_model_ray(r);

		si->material = material;
		if (!geometry->intersect(model_space_ray, tmin, tmax, si))
			return false;

		// transform hit to world space
		si->point = transform.to_world_point(si->point);
		si->normal = transform.to_world_normal(si->normal);


		return true;
	}
private:
	Geometry* geometry = nullptr;
	Material* material = nullptr;
	Transform transform;
};


#endif // !OBJECT_H
