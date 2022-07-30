#ifndef UTILS_H
#define UTILS_H
#include <random>
#include "Math.h"

const float PI = 3.14159;

inline float radians(float degrees)
{
	return degrees * (PI / 180.f);
}

inline float random_float()
{
	return rand() / (RAND_MAX + 1.0);
}

inline float random_float(float min, float max)
{
	return min + (max - min) * random_float();
}
inline float clamp(float x, float min, float max)
{
	return (x < min) ? min : ((x > max) ? max : x);
}
inline vec3 random_vec3(float min, float max)
{
	return vec3(random_float(min, max), random_float(min, max), random_float(min, max));
}
inline vec3 random_in_unit_sphere()
{
	while (1) {
		vec3 p = random_vec3(-1, 1);
		if (p.length_squared() >= 1) continue;
		return p;
	}
}
inline vec3 random_unit_vector()
{
	return normalize(random_in_unit_sphere());
}

inline vec3 random_in_unit_disk()
{
	while (1) {
		vec3 p = vec3(random_float(-1, 1), random_float(-1, 1), 0);
		if (p.length_squared() >= 1) continue;
		return p;
	}
}
inline vec3 reflect(vec3 v, vec3 n)
{
	return v - 2 * dot(v, n) * n;
}
inline vec3 refract(vec3 uv, vec3 n, float etai_over_etat)
{
	float theta = fmin(dot(-uv, n), 1.0);
	vec3 r_out_perp = etai_over_etat * (uv + theta * n);
	vec3 r_out_parallel = -sqrt(fabs(1.0 - r_out_perp.length_squared())) * n;
	return r_out_perp + r_out_parallel;
}
inline float reflectance(float cosine, float ref_idx)
{
	float r0 = (1 - ref_idx) / (1 + ref_idx);
	r0 = r0 * r0;
	return r0 + (1 - r0) * pow((1 - cosine), 5);
}
inline vec3 pow(vec3 V, float e)
{
	return vec3(pow(V.x, e), pow(V.y, e), pow(V.z, e));
}
inline vec3 abs(vec3 V)
{
	return vec3(fabs(V.x), fabs(V.y), fabs(V.z));
}

#endif // !UTILS_H