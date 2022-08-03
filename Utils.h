#ifndef UTILS_H
#define UTILS_H
#include <random>
#include "Math.h"

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
inline vec3 random_in_hemisphere(const vec3& normal)
{
	vec3 random = random_in_unit_sphere();
	if (dot(random, normal) < 0)
		random = -random;
	return normalize(random);
}

inline vec3 random_in_unit_disk()
{
	while (1) {
		vec3 p = vec3(random_float(-1, 1), random_float(-1, 1), 0);
		if (p.length_squared() >= 1) continue;
		return p;
	}
}

void ONB(const vec3& N, vec3& T, vec3& B)
{
	vec3 up = vec3(0, 0, 1);
	if (fabs(N.z) > 0.999) up = vec3(1, 0, 0);
	T = normalize(cross(up, N));
	B = cross(N, T);
}

inline vec3 random_cosine()
{
	float r1 = random_float();
	float r2 = random_float();
	float phi = 2 * PI * r1;
	float cos_theta = sqrt(1 - r2);

	float sin_theta = sqrt(r2);
	
	float x = cos(phi) * sin_theta;
	float y = sin(phi) * sin_theta;

	return vec3(x, y, cos_theta);
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

inline bool quadratic(float a, float b, float c, float& t0, float& t1)
{
	float discrim = b * b - 4 * a * c;
	if (discrim < 0)
		return false;
	float sqrtdist = sqrt(discrim);
	t0 = (-b - sqrtdist) / (2*a);
	t1 = (-b + sqrtdist) / (2*a);

	if (t1 < t0)
		std::swap(t0, t1);
	return true;
}

inline vec3 rgb_to_float(int r, int g, int b)
{
	return vec3(r / 255.f, g / 255.f, b / 255.f);
}

inline float power_heuristic(float pdf0, float pdf1)
{
	float denom = pdf0 * pdf0 + pdf1 * pdf1;
	if (fabs(denom) < 0.0001)
		return 0.f;
	return (pdf0 * pdf0) / denom;
}

#endif // !UTILS_H
