#ifndef VEC3_H
#define VEC3_H
#include <cmath>
#include <iostream>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>

const float PI = 3.14159;

inline float radians(float degrees)
{
	return degrees * (PI / 180.f);
}


struct vec3
{
	vec3() {};
	vec3(float e0, float e1, float e2) : x(e0),y(e1),z(e2) {};
	vec3(float e) : x(e), y(e), z(e) {}
	
	vec3 operator-() const { return vec3(-x, -y, -z); }
	vec3& operator+=(const vec3& v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}
	vec3& operator*=(float t)
	{
		x *= t;
		y *= t;
		z *= t;
		return *this;
	}
	vec3& operator/=(float t)
	{
		return *this *= (1 / t);
	}
	float length_squared() const
	{
		return x * x + y * y + z * z;
	}
	float length() const
	{
		return sqrt(length_squared());
	}
	bool near_zero() const 
	{
		const static float EPSILON = 0.00001f;
		return fabs(x) < EPSILON && fabs(y) < EPSILON && fabs(z) < EPSILON;
	}
	union { float x, r; };
	union { float y, g; };
	union { float z, b; };

	float& operator[](int idx) {
		switch (idx) {
		case 0: return x;
		case 1: return y;
		case 2: return z;
		default: return x;
		}
	}
	float const& operator[](int idx) const {
		switch (idx) {
		case 0: return x;
		case 1: return y;
		case 2: return z;
		}
		return 0;
	}
};

using point3f = vec3;
using color3f = vec3;

inline std::ostream& operator<<(std::ostream& out, const vec3& v)
{
	out << v[0] << " " << v[1] << " " << v[2] << "\n";
	return out;
}
inline vec3 operator-(const vec3& u, const vec3& v)
{
	return vec3(u[0] - v[0], u[1] - v[1], u[2] - v[2]);
}
inline vec3 operator+(const vec3& u, const vec3& v)
{
	return vec3(u[0] + v[0], u[1] + v[1], u[2] + v[2]);
}
inline vec3 operator*(const vec3& u, const vec3& v)
{
	return vec3(u[0] * v[0], u[1] * v[1], u[2] * v[2]);
}
inline vec3 operator*(float t, const vec3& v)
{
	return vec3(t * v[0], t * v[1], t * v[2]);
}
inline vec3 operator/(const vec3& u, float t)
{
	return (1 / t) * u;
}
inline float dot(const vec3& u, const vec3& v)
{
	return u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
}
inline vec3 cross(const vec3& u, const vec3& v)
{
	return vec3(u[1] * v[2] - u[2] * v[1],
		u[2] * v[0] - u[0] * v[2],
		u[0] * v[1] - u[1] * v[0]);
}
inline vec3 normalize(vec3 v)
{
	return v / v.length();
}

inline vec3 vec_min(const vec3& a, const vec3& b)
{
	return vec3(fmin(a.x, b.x), fmin(a.y, b.y), fmin(a.z, b.z));
}
inline vec3 vec_max(const vec3& a, const vec3& b)
{
	return vec3(fmax(a.x, b.x), fmax(a.y, b.y), fmax(a.z, b.z));
}

// Vec4

struct vec4
{
	vec4() {};
	vec4(float x, float y, float z,float w) : x(x), y(y), z(z),w(w) {};
	vec4(float xyzw) : x(xyzw), y(xyzw), z(xyzw),w(xyzw){}
	vec4(vec3 v, float w) : x(v.x), y(v.y), z(v.z), w(w) {}
	vec4 operator-() const { return vec4(-x, -y, -z,-w); }
	vec4& operator+=(const vec4& v)
	{
		x += v.x;
		y += v.y;
		z += v.z;
		w += v.w;
		return *this;
	}
	vec4& operator*=(float t)
	{
		x *= t;
		y *= t;
		z *= t;
		w *= t;
		return *this;
	}
	vec4& operator/=(float t)
	{
		return *this *= (1 / t);
	}
	float length_squared() const
	{
		return x * x + y * y + z * z + w*w;
	}
	float length() const
	{
		return sqrt(length_squared());
	}
	bool near_zero() const
	{
		const static float EPSILON = 0.00001f;
		return fabs(x) < EPSILON && fabs(y) < EPSILON && fabs(z) < EPSILON && fabs(w) < EPSILON;
	}
	union { float x, r; };
	union { float y, g; };
	union { float z, b; };
	union { float w, a; };


	float& operator[](int idx) {
		switch (idx) {
		case 0: return x;
		case 1: return y;
		case 2: return z;
		case 3: return w;
		default: return x;
		}
	}
	float const& operator[](int idx) const {
		switch (idx) {
		case 0: return x;
		case 1: return y;
		case 2: return z;
		case 3: return w;
		}
		return 0;
	}

	vec3 xyz() const {
		return vec3(x, y, z);
	}
};

inline std::ostream& operator<<(std::ostream& out, const vec4& v)
{
	out << v[0] << " " << v[1] << " " << v[2] << "  " << v[3] << "\n";
	return out;
}
inline vec4 operator-(const vec4& u, const vec4& v)
{
	return vec4(u[0] - v[0], u[1] - v[1], u[2] - v[2], u[3]-v[3]);
}
inline vec4 operator+(const vec4& u, const vec4& v)
{
	return vec4(u[0] + v[0], u[1] + v[1], u[2] + v[2], u[3]+v[3]);
}
inline vec4 operator*(const vec4& u, const vec4& v)
{
	return vec4(u[0] * v[0], u[1] * v[1], u[2] * v[2],u[3]*v[3]);
}
inline vec4 operator*(float t, const vec4& v)
{
	return vec4(t * v[0], t * v[1], t * v[2], t*v[3]);
}
inline vec4 operator/(const vec4& u, float t)
{
	return (1 / t) * u;
}
inline float dot(const vec4& u, const vec4& v)
{
	return u[0] * v[0] + u[1] * v[1] + u[2] * v[2] + u[3]*v[3];
}
inline vec4 normalize(vec4 v)
{
	return v / v.length();
}

// End vec4

// Mat4

struct mat4
{
	float m[16];
	/*
		0 4 8 12
		1 5 9 13
		2 6 10 14
		3 7 11 15
	*/

	mat4() {}
	mat4(float a) {
		m[0] = m[5] = m[10] = m[15] = a;
		m[4] = m[8] = m[12] = m[1] = m[9] = m[13] = m[2] = m[6] = m[14] = m[3] = m[7] = m[11] = 0;
	}
	mat4(float a, float b, float c, float d,
		float e, float f, float g, float h,
		float i, float j, float k, float l,
		float mm, float n, float o, float p)
	{
		m[0] = a; m[1] = b; m[2] = c; m[3] = d;
		m[4] = e; m[5] = f; m[6] = g; m[7] = h;
		m[8] = i; m[9] = j; m[10] = k; m[11] = l;
		m[12] = mm; m[13] = n; m[14] = o; m[15] = p;
	}
	vec4 operator[](int column) const {
		column <<= 2;
		return vec4(m[column], m[column + 1], m[column + 2], m[column + 3]);
	}

	vec4 operator* (const vec4& rhs) const {
		float x = 
			m[0] * rhs.x +
			m[4] * rhs.y + 
			m[8] * rhs.z + 
			m[12] * rhs.w;
		float y =
			m[1] * rhs.x +
			m[5] * rhs.y +
			m[9] * rhs.z +
			m[13] * rhs.w;
		float z =
			m[2] * rhs.x + 
			m[6] * rhs.y +
			m[10] * rhs.z + 
			m[14] * rhs.w;
		float w =
			m[3] * rhs.x +
			m[7] * rhs.y +
			m[11] * rhs.z +
			m[15] * rhs.w;
		return vec4(x, y, z, w);
	}

	mat4 operator*(const mat4& rhs) const {
		mat4 r = mat4(0.f);

		vec4 mrows[4];
		mrows[0] = vec4(m[0], m[4], m[8], m[12]);
		mrows[1] = vec4(m[1], m[5], m[9], m[13]);
		mrows[2] = vec4(m[2], m[6], m[10], m[14]);
		mrows[3] = vec4(m[3], m[7], m[11], m[15]);
		
		r.m[0] = dot(mrows[0], rhs[0]);
		r.m[1] = dot(mrows[1], rhs[0]);
		r.m[2] = dot(mrows[2], rhs[0]);
		r.m[3] = dot(mrows[3], rhs[0]);

		r.m[4] = dot(mrows[0], rhs[1]);
		r.m[5] = dot(mrows[1], rhs[1]);
		r.m[6] = dot(mrows[2], rhs[1]);
		r.m[7] = dot(mrows[3], rhs[1]);

		r.m[8] = dot(mrows[0], rhs[2]);
		r.m[9] = dot(mrows[1], rhs[2]);
		r.m[10] = dot(mrows[2], rhs[2]);
		r.m[11] = dot(mrows[3], rhs[2]);

		r.m[12] = dot(mrows[0], rhs[3]);
		r.m[13] = dot(mrows[1], rhs[3]);
		r.m[14] = dot(mrows[2], rhs[3]);
		r.m[15] = dot(mrows[3], rhs[3]);
		
		return r;
	}
};

inline float determinant(const mat4& mm)
{
	return 
		mm.m[12] * mm.m[9] * mm.m[6] * mm.m[3] -
		mm.m[8] * mm.m[13] * mm.m[6] * mm.m[3] -
		mm.m[12] * mm.m[5] * mm.m[10] * mm.m[3] +
		mm.m[4] * mm.m[13] * mm.m[10] * mm.m[3] +
		mm.m[8] * mm.m[5] * mm.m[14] * mm.m[3] -
		mm.m[4] * mm.m[9] * mm.m[14] * mm.m[3] -
		mm.m[12] * mm.m[9] * mm.m[2] * mm.m[7] +
		mm.m[8] * mm.m[13] * mm.m[2] * mm.m[7] +
		mm.m[12] * mm.m[1] * mm.m[10] * mm.m[7] -
		mm.m[0] * mm.m[13] * mm.m[10] * mm.m[7] -
		mm.m[8] * mm.m[1] * mm.m[14] * mm.m[7] +
		mm.m[0] * mm.m[9] * mm.m[14] * mm.m[7] +
		mm.m[12] * mm.m[5] * mm.m[2] * mm.m[11] -
		mm.m[4] * mm.m[13] * mm.m[2] * mm.m[11] -
		mm.m[12] * mm.m[1] * mm.m[6] * mm.m[11] +
		mm.m[0] * mm.m[13] * mm.m[6] * mm.m[11] +
		mm.m[4] * mm.m[1] * mm.m[14] * mm.m[11] -
		mm.m[0] * mm.m[5] * mm.m[14] * mm.m[11] -
		mm.m[8] * mm.m[5] * mm.m[2] * mm.m[15] +
		mm.m[4] * mm.m[9] * mm.m[2] * mm.m[15] +
		mm.m[8] * mm.m[1] * mm.m[6] * mm.m[15] -
		mm.m[0] * mm.m[9] * mm.m[6] * mm.m[15] -
		mm.m[4] * mm.m[1] * mm.m[10] * mm.m[15] +
		mm.m[0] * mm.m[5] * mm.m[10] * mm.m[15];
}

inline mat4 inverse(const mat4& mm)
{
	float det = determinant(mm);
	if (det == 0.f)
		return mm;
	float inv_det = 1.f / det;
	return mat4(
		inv_det * (
			mm.m[9] * mm.m[14] * mm.m[7] - mm.m[13] * mm.m[10] * mm.m[7] +
			mm.m[13] * mm.m[6] * mm.m[11] - mm.m[5] * mm.m[14] * mm.m[11] -
			mm.m[9] * mm.m[6] * mm.m[15] + mm.m[5] * mm.m[10] * mm.m[15]
			),
		inv_det * (
			mm.m[13] * mm.m[10] * mm.m[3] - mm.m[9] * mm.m[14] * mm.m[3] -
			mm.m[13] * mm.m[2] * mm.m[11] + mm.m[1] * mm.m[14] * mm.m[11] +
			mm.m[9] * mm.m[2] * mm.m[15] - mm.m[1] * mm.m[10] * mm.m[15]
			),
		inv_det * (
			mm.m[5] * mm.m[14] * mm.m[3] - mm.m[13] * mm.m[6] * mm.m[3] +
			mm.m[13] * mm.m[2] * mm.m[7] - mm.m[1] * mm.m[14] * mm.m[7] -
			mm.m[5] * mm.m[2] * mm.m[15] + mm.m[1] * mm.m[6] * mm.m[15]
			),
		inv_det * (
			mm.m[9] * mm.m[6] * mm.m[3] - mm.m[5] * mm.m[10] * mm.m[3] -
			mm.m[9] * mm.m[2] * mm.m[7] + mm.m[1] * mm.m[10] * mm.m[7] +
			mm.m[5] * mm.m[2] * mm.m[11] - mm.m[1] * mm.m[6] * mm.m[11]
			),
		inv_det * (
			mm.m[12] * mm.m[10] * mm.m[7] - mm.m[8] * mm.m[14] * mm.m[7] -
			mm.m[12] * mm.m[6] * mm.m[11] + mm.m[4] * mm.m[14] * mm.m[11] +
			mm.m[8] * mm.m[6] * mm.m[15] - mm.m[4] * mm.m[10] * mm.m[15]
			),
		inv_det * (
			mm.m[8] * mm.m[14] * mm.m[3] - mm.m[12] * mm.m[10] * mm.m[3] +
			mm.m[12] * mm.m[2] * mm.m[11] - mm.m[0] * mm.m[14] * mm.m[11] -
			mm.m[8] * mm.m[2] * mm.m[15] + mm.m[0] * mm.m[10] * mm.m[15]
			),
		inv_det * (
			mm.m[12] * mm.m[6] * mm.m[3] - mm.m[4] * mm.m[14] * mm.m[3] -
			mm.m[12] * mm.m[2] * mm.m[7] + mm.m[0] * mm.m[14] * mm.m[7] +
			mm.m[4] * mm.m[2] * mm.m[15] - mm.m[0] * mm.m[6] * mm.m[15]
			),
		inv_det * (
			mm.m[4] * mm.m[10] * mm.m[3] - mm.m[8] * mm.m[6] * mm.m[3] +
			mm.m[8] * mm.m[2] * mm.m[7] - mm.m[0] * mm.m[10] * mm.m[7] -
			mm.m[4] * mm.m[2] * mm.m[11] + mm.m[0] * mm.m[6] * mm.m[11]
			),
		inv_det * (
			mm.m[8] * mm.m[13] * mm.m[7] - mm.m[12] * mm.m[9] * mm.m[7] +
			mm.m[12] * mm.m[5] * mm.m[11] - mm.m[4] * mm.m[13] * mm.m[11] -
			mm.m[8] * mm.m[5] * mm.m[15] + mm.m[4] * mm.m[9] * mm.m[15]
			),
		inv_det * (
			mm.m[12] * mm.m[9] * mm.m[3] - mm.m[8] * mm.m[13] * mm.m[3] -
			mm.m[12] * mm.m[1] * mm.m[11] + mm.m[0] * mm.m[13] * mm.m[11] +
			mm.m[8] * mm.m[1] * mm.m[15] - mm.m[0] * mm.m[9] * mm.m[15]
			),
		inv_det * (
			mm.m[4] * mm.m[13] * mm.m[3] - mm.m[12] * mm.m[5] * mm.m[3] +
			mm.m[12] * mm.m[1] * mm.m[7] - mm.m[0] * mm.m[13] * mm.m[7] -
			mm.m[4] * mm.m[1] * mm.m[15] + mm.m[0] * mm.m[5] * mm.m[15]
			),
		inv_det * (
			mm.m[8] * mm.m[5] * mm.m[3] - mm.m[4] * mm.m[9] * mm.m[3] -
			mm.m[8] * mm.m[1] * mm.m[7] + mm.m[0] * mm.m[9] * mm.m[7] +
			mm.m[4] * mm.m[1] * mm.m[11] - mm.m[0] * mm.m[5] * mm.m[11]
			),
		inv_det * (
			mm.m[12] * mm.m[9] * mm.m[6] - mm.m[8] * mm.m[13] * mm.m[6] -
			mm.m[12] * mm.m[5] * mm.m[10] + mm.m[4] * mm.m[13] * mm.m[10] +
			mm.m[8] * mm.m[5] * mm.m[14] - mm.m[4] * mm.m[9] * mm.m[14]
			),
		inv_det * (
			mm.m[8] * mm.m[13] * mm.m[2] - mm.m[12] * mm.m[9] * mm.m[2] +
			mm.m[12] * mm.m[1] * mm.m[10] - mm.m[0] * mm.m[13] * mm.m[10] -
			mm.m[8] * mm.m[1] * mm.m[14] + mm.m[0] * mm.m[9] * mm.m[14]
			),
		inv_det * (
			mm.m[12] * mm.m[5] * mm.m[2] - mm.m[4] * mm.m[13] * mm.m[2] -
			mm.m[12] * mm.m[1] * mm.m[6] + mm.m[0] * mm.m[13] * mm.m[6] +
			mm.m[4] * mm.m[1] * mm.m[14] - mm.m[0] * mm.m[5] * mm.m[14]
			),
		inv_det * (
			mm.m[4] * mm.m[9] * mm.m[2] - mm.m[8] * mm.m[5] * mm.m[2] +
			mm.m[8] * mm.m[1] * mm.m[6] - mm.m[0] * mm.m[9] * mm.m[6] -
			mm.m[4] * mm.m[1] * mm.m[10] + mm.m[0] * mm.m[5] * mm.m[10]
			)
	);
}

inline mat4 transpose(const mat4& m)
{
	return mat4(
		m.m[0], m.m[4], m.m[8], m.m[12],
		m.m[1], m.m[5], m.m[9], m.m[13],
		m.m[2], m.m[6], m.m[10], m.m[14],
		m.m[3], m.m[7], m.m[11], m.m[15]
		);
}

inline mat4 translate(const mat4& m,const vec3& v)
{
	mat4 r = mat4(1.f);
	r.m[12] = v.x;
	r.m[13] = v.y;
	r.m[14] = v.z;
	return r * m;
}
inline mat4 rotate_y(const mat4& m, float degrees)
{
	float rads = radians(degrees);
	float sinrad = sin(rads);
	float cosrad = cos(rads);
	mat4 r = mat4(1.f);
	r.m[0] = cosrad;
	r.m[8] = sinrad;
	r.m[2] = -sinrad;
	r.m[10] = cosrad;
	return r * m;
}
inline mat4 rotate_x(const mat4& m, float degrees)
{
	float rads = radians(degrees);
	float sinrad = sin(rads);
	float cosrad = cos(rads);
	mat4 r = mat4(1.f);
	r.m[5] = cosrad;
	r.m[9] = -sinrad;
	r.m[6] = sinrad;
	r.m[10] = cosrad;
	return r * m; 
}

inline std::ostream& operator<<(std::ostream& out, const mat4& m)
{
	out << "[ " << m.m[0] << " " << m.m[4] << " " << m.m[8] << " " << m.m[12] << " ]\n";
	out << "[ " << m.m[1] << " " << m.m[5] << " " << m.m[9] << " " << m.m[13] << " ]\n";
	out << "[ " << m.m[2] << " " << m.m[6] << " " << m.m[10] << " " << m.m[14] << " ]\n";
	out << "[ " << m.m[3] << " " << m.m[7] << " " << m.m[11] << " " << m.m[15] << " ]\n";

	return out;
}

struct Ray
{
	Ray() : pos(vec3(0)), dir(vec3(1, 0, 0)) {}
	Ray(vec3 pos, vec3 dir) :pos(pos), dir(dir) {}

	vec3 at(float t) const {
		return pos + dir * t;
	}

	vec3 pos;
	vec3 dir;
};

/* For reference
	0 4 8 12
	1 5 9 13
	2 6 10 14
	3 7 11 15
*/
class Transform
{
public:
	Transform(mat4 model_to_world) {
		m = model_to_world;
		inv_m = inverse(m);
	}
	Transform() {
		m = mat4(1);
		inv_m = mat4(1);
	}
	vec3 to_model_point(const vec3& p) const {
		return (inv_m * vec4(p, 1.0)).xyz();
	}
	vec3 to_world_point(const vec3& p) const {
		return (m * vec4(p, 1.0)).xyz();
	}
	vec3 to_model_normal(const vec3& n) const {
		float x = n.x, y = n.y, z = n.z;
		// mat3 of transpose of the inverse of the world-to-model (so mat3 tranpose of model-to-world)
		return vec3(
			m.m[0] * x + m.m[1] * y + m.m[2] * z,
			m.m[4] * x + m.m[5] * y + m.m[6] * z,
			m.m[8] * x + m.m[9] * y + m.m[10] * z
		);
	}
	vec3 to_world_normal(const vec3& n) const {
		float x = n.x, y = n.y, z = n.z;
		return vec3(
			inv_m.m[0] * x + inv_m.m[1] * y + inv_m.m[2] * z,
			inv_m.m[4] * x + inv_m.m[5] * y + inv_m.m[6] * z,
			inv_m.m[8] * x + inv_m.m[9] * y + inv_m.m[10] * z
		);
	}
	vec3 to_model_vector(const vec3& v) const {
		// mat3 of the world-to-model, no transpose or inverse
		float x = v.x, y = v.y, z = v.z;
		return vec3(
			inv_m.m[0] * x + inv_m.m[4] * y + inv_m.m[8] * z,
			inv_m.m[1] * x + inv_m.m[5] * y + inv_m.m[9] * z,
			inv_m.m[2] * x + inv_m.m[6] * y + inv_m.m[10] * z
		);
	}
	vec3 to_world_vector(const vec3& v) const {
		float x = v.x, y = v.y, z = v.z;
		return vec3(
			m.m[0] * x + m.m[4] * y + m.m[8] * z,
			m.m[1] * x + m.m[5] * y + m.m[9] * z,
			m.m[2] * x + m.m[6] * y + m.m[10] * z
		);
	}
	Ray to_model_ray(const Ray& r) const {
		return Ray(to_model_point(r.pos), to_model_vector(r.dir));
	}
private:
	mat4 m;	// model-to-world matrix
	mat4 inv_m;	// world-to-model matrix
};

// End Mat4

struct Bounds
{
	Bounds() {}
	Bounds(vec3 pos) : min(pos),max(pos) {}
	Bounds(vec3 min, vec3 max) : min(min), max(max) {}

	float surface_area() const {
		vec3 size = max - min;
		return 2.0 * (size.x * size.y + size.x * size.z + size.y * size.z);
	}
	Bounds transform_bounds(const Transform& transform) const; 

	bool intersect(const Ray& r, float tmin, float tmax) const {
		for (int a = 0; a < 3; a++) {
			float t0 = fmin((min[a] - r.pos[a]) / r.dir[a],
				(max[a] - r.pos[a]) / r.dir[a]);
			float t1 = fmax((min[a] - r.pos[a]) / r.dir[a],
				(max[a] - r.pos[a]) / r.dir[a]);
			tmin = fmax(t0, tmin);
			tmax = fmin(t1, tmax);
			if (tmax <= tmin)
				return false;
		}
		return true;
	}

	vec3 min, max;
};
Bounds bounds_union(const Bounds& b1, const Bounds& b2) {
	Bounds b;
	b.min = vec_min(b1.min, b2.min);
	b.max = vec_max(b1.max, b2.max);
	return b;
}
Bounds bounds_union(const Bounds& b1, const vec3& v) {
	Bounds b;
	b.min = vec_min(b1.min, v);
	b.max = vec_max(b1.max, v);
	return b;
}
Bounds Bounds::transform_bounds(const Transform& transform) const {
	Bounds b(transform.to_world_point(min));
	b = bounds_union(b, transform.to_world_point(vec3(max.x, min.y, min.z)));
	b = bounds_union(b, transform.to_world_point(vec3(max.x, min.y, max.z)));
	b = bounds_union(b, transform.to_world_point(vec3(min.x, min.y, max.z)));

	b = bounds_union(b, transform.to_world_point(vec3(min.x, max.y, min.z)));
	b = bounds_union(b, transform.to_world_point(vec3(min.x, max.y, max.z)));
	b = bounds_union(b, transform.to_world_point(vec3(max.x, max.y, min.z)));
	b = bounds_union(b, transform.to_world_point(vec3(max.x, max.y, max.z)));

	return b;
}


bool AABB_hit(const Ray& r, const vec3& bmin, const vec3& bmax, float tmin, float tmax)
{
	for (int a = 0; a < 3; a++) {
		float t0 = fmin((bmin[a] - r.pos[a]) / r.dir[a],
			(bmax[a] - r.pos[a]) / r.dir[a]);
		float t1 = fmax((bmin[a] - r.pos[a]) / r.dir[a],
			(bmax[a] - r.pos[a]) / r.dir[a]);
		tmin = fmax(t0, tmin);
		tmax = fmin(t1, tmax);
		if (tmax <= tmin)
			return false;
	}
	return true;
}




#endif // !VEC3_H
