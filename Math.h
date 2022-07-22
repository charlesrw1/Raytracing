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
#endif // !VEC3_H
