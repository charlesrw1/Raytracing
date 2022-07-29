#ifndef VEC3_H
#define VEC3_H
#include <cmath>
#include <iostream>
#include <glm/vec3.hpp>

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

	float operator[](int idx) const {
		switch (idx) {
		case 0: return x;
		case 1: return y;
		case 2: return z;
		}
		return 0;
	}

};

using point3 = vec3;
using color = vec3;

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

bool AABB_hit(const Ray& r, const vec3& bmin, const vec3& bmax, float tmin, float tmax)
{
	for (int a = 0; a < 3; a++) {
		float t0 = fmin((bmin[a] - r.pos[a]) / r.dir[a],
						(bmax[a]-r.pos[a])/r.dir[a]);
		float t1 = fmax((bmin[a] - r.pos[a]) / r.dir[a],
			(bmax[a] - r.pos[a]) / r.dir[a]);
		tmin = fmax(t0, tmin);
		tmax = fmin(t1, tmax);
		if (tmax <= tmin)
			return false;
	}
	return true;
}




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

};

float determinant()
{
	return 0;
}


#endif // !VEC3_H
