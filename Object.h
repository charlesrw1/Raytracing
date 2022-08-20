#ifndef OBJECT_H
#define OBJECT_H

#include "Def.h"
#include "Utils.h"
#include "Mesh.h"
#include "BVH.h"

class Geometry
{
public:
	virtual ~Geometry() {}
	virtual Bounds object_bounds() const {
		return Bounds();
	}
	//virtual Bounds world_bounds() const;
	virtual bool intersect(Ray r, float tmin, float tmax, Intersection* si) const = 0;
	virtual float area() const {
		return 0.f;
	}
	virtual void sample_point(const vec3& from, vec3& point, vec3& normal) const {
		point = vec3(0);
	}
};

class Sphere : public Geometry
{
public:
	Sphere(float radius)
		: radius(radius) {}
	virtual Bounds object_bounds() const {
		return Bounds(vec3(-radius), vec3(radius));
	}
	virtual bool intersect(Ray r, float tmin, float tmax, Intersection* si) const override {
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
	}
	

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
	virtual Bounds object_bounds() const {
		
		Bounds b(vec3(-u, 0, -v), vec3(u, 0, v));
		mat4 rotation = mat4(
			T.x, T.y, T.z, 0,
			N.x, N.y, N.z, 0,
			B.x, B.y, B.z, 0,
			0, 0, 0, 1);
		Transform t(rotation);
		return b.transform_bounds(t);

	}
	virtual bool intersect(Ray r, float tmin, float tmax, Intersection* si) const override {
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
	}
	virtual float area() const {
		return (2 * u) * (2 * v);
	}
	virtual void sample_point(const vec3& from, vec3& point, vec3& normal) const override{

		float r1 = random_float(-1,1);
		float r2 = random_float(-1,1);
		point = vec3(0);
		point = T * u * r1 + B * v * r2;
		normal = N;
	}
private:
	vec3 N, T, B;// normal, tangent, bitangent
	float u, v;	// half side lengths
};

class Disk : public Geometry
{
public:
	Disk(vec3 normal, float outer_radius, float inner_radius = -1.f) : normal(normal), outer_radius(outer_radius), inner_radius(inner_radius) {}
	virtual bool intersect(Ray r, float tmin, float tmax, Intersection* si) const override {
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

	virtual bool intersect(Ray r, float tmin, float tmax, Intersection* si) const override {
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

	virtual bool intersect(Ray r, float tmin, float tmax, Intersection* si) const override {
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

inline void IntersectTriRay2(const Ray& r, const vec3& v0, const vec3& v1, const vec3& v2, float& t_out, vec3& bary_out)
{
	const vec3 edge1 = v1 - v0;
	const vec3 edge2 = v2 - v0;
	const vec3 h = cross(r.dir, edge2);
	const float a = dot(edge1, h);
	if (a > -0.0001f && a < 0.0001f) return; // ray parallel to triangle
	const float f = 1 / a;
	const vec3 s = r.pos - v0;
	const float u = f * dot(s, h);
	if (u < 0 || u > 1) return;
	const vec3 q = cross(s, edge1);
	const float v = f * dot(r.dir, q);
	if (v < 0 || u + v > 1) return;
	const float t = f * dot(edge2, q);
	if (t > 0.0001f) {
		t_out = t;
		bary_out = vec3(u, v, 1 - u - v);
	}
}

class TriangleMesh : public Geometry
{
public:
	TriangleMesh(const Mesh* mesh);
	virtual Bounds object_bounds() const {
		return mesh_bounds;
	}
	virtual bool intersect(Ray r, float tmin, float tmax, Intersection* si) const override 
	{
		const BVHNode* stack[64];
		stack[0] = &bvh.nodes[0];
		
		int stack_count = 1;
		const BVHNode* node = nullptr;

		float t = INFINITY;
		vec3 bary=vec3(0);
		int tri_vert_start = 0;


		int node_check = 0;
		bool found_hit = false;
		while (stack_count > 0)
		{
			node = stack[--stack_count];

			if (node->count != BVH_BRANCH)
			{
				node_check++;
				int index_count = node->count;
				int index_start = node->left_node;
				for (int i = 0; i < index_count; i++) {
					vec3 bary_temp = vec3(0);
					vec3 N_temp;
					float t_temp=-1;
					
					int mesh_element_index = bvh.indicies[index_start + i] * 3;
					vec3 v0 = mesh->verticies[mesh->indicies[mesh_element_index]].position;
					vec3 v1 = mesh->verticies[mesh->indicies[mesh_element_index + 1]].position;
					vec3 v2 = mesh->verticies[mesh->indicies[mesh_element_index + 2]].position;
					IntersectTriRay2(r,v0,
						v1,
						v2, t_temp, bary_temp);
					bool res = t_temp > 0;


					if (!res || t_temp > tmax || t_temp < tmin)
						continue;
					tmax = t_temp;
					t = t_temp;
					tri_vert_start = mesh_element_index;
					bary = vec3(bary_temp.z,bary_temp.x,bary_temp.y);
				}
				continue;
			}

			float left_dist, right_dist;
			bool left_aabb = bvh.nodes[node->left_node].aabb.intersect(r,left_dist);
			bool right_aabb = bvh.nodes[node->left_node + 1].aabb.intersect(r,right_dist);
			left_aabb = left_aabb && left_dist < t;
			right_aabb = right_aabb && right_dist < t;

			if (left_aabb && right_aabb) {
				if (left_dist < right_dist) {
					stack[stack_count++] = &bvh.nodes[node->left_node + 1];
					stack[stack_count++]= &bvh.nodes[node->left_node];
				}
				else {
					stack[stack_count++] = &bvh.nodes[node->left_node];
					stack[stack_count++] = &bvh.nodes[node->left_node + 1];
				}
			}
			else if (left_aabb) {
				stack[stack_count++] = &bvh.nodes[node->left_node];
			}
			else if (right_aabb) {
				stack[stack_count++] = &bvh.nodes[node->left_node + 1];
			}
		}

		if (!std::isfinite(t)) {
			return false;
		}

		si->point = r.at(t);
		vec3 N = bary.u * mesh->verticies[mesh->indicies[tri_vert_start]].normal +
			bary.v * mesh->verticies[mesh->indicies[tri_vert_start + 1]].normal +
			bary.w * mesh->verticies[mesh->indicies[tri_vert_start + 2]].normal;
		si->set_face_normal(r, normalize(N));
		si->t = t;
		return true;
	}
private:
	const Mesh* mesh;
	BVH bvh;
	Bounds mesh_bounds;
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

	bool intersect(Ray r, float tmin, float tmax, Intersection* si) const {
		// Transform ray to model space
		Ray model_space_ray = transform.to_model_ray(r);

		if (!geometry->intersect(model_space_ray, tmin, tmax, si))
			return false;

		si->material = material;
		// transform hit to world space
		si->point = transform.to_world_point(si->point);
		si->normal = normalize(transform.to_world_normal(si->normal));


		return true;
	}

	void sample_geometry(const vec3& from, vec3& point, vec3& normal) const{
		vec3 transformed_from = transform.to_model_point(from);

		geometry->sample_point(transformed_from,point,normal);

		point = transform.to_world_point(point);
		normal = transform.to_world_normal(normal);
	}
	float get_area() const {
		// This doesnt take into account transform scaling
		return geometry->area();
	}
	const Material* get_material() const {
		return material;
	}

	Bounds get_world_bounds() const {
		Bounds bounds = geometry->object_bounds();
		bounds = bounds.transform_bounds(transform);
		return bounds;
	}


private:
	Geometry* geometry = nullptr;
	Material* material = nullptr;
	Transform transform;
};


#endif // !OBJECT_H
