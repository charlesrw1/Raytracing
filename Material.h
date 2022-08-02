#ifndef MATERIALS_H
#define MATERIALS_H

#include "Def.h"
#include "Utils.h"

class Texture
{
public:
	virtual ~Texture() {}
	virtual vec3 sample(float u, float v, vec3 point) const = 0;
};
class ConstantTexture : public Texture
{
public:
	ConstantTexture(vec3 color) : color(color) {}
	ConstantTexture(float r, float g, float b) : color(vec3(r, g, b)) {}
	ConstantTexture(float rgb) : color(vec3(rgb)) {}
	virtual vec3 sample(float u, float v, vec3 point) const override {
		return color;
	}
private:
	vec3 color;
};
class CheckeredTexture : public Texture
{
public:
	CheckeredTexture(vec3 even, vec3 odd, float grid = 0.25f) : even(even), odd(odd), grid(PI / grid) {}
	virtual vec3 sample(float u, float v, vec3 p) const override {
		float sines = sin(grid * p.x) * sin(grid * p.y) * sin(grid * p.z);
		//uint8_t res = (fmod(fmod(p.x, mod)+mod,mod) < repeat) ^ (fmod(fmod(p.y, mod)+mod,mod) < repeat) ^ (fmod(fmod(p.z, mod)+mod,mod) < repeat);
		if (sines < 0)
			return odd;
		return even;
	}

private:
	vec3 even, odd;
	float grid;
};

class Material
{
public:
	virtual ~Material() {}
	virtual bool scatter(const SurfaceInteraction* SI, vec3& attenuation, Ray& scattered, float& pdf) const {
		return false;
	}
	virtual float scattering_pdf(const Ray& in, const SurfaceInteraction& si, const Ray& scattered) const {
		return 0.f;
	}
	virtual vec3 emitted() const {
		return vec3(0);
	}
};
class MatteMaterial : public Material
{
public:
	MatteMaterial(Texture* albedo) : albedo(albedo) {}
	~MatteMaterial() {
		delete albedo;
	}
	virtual bool scatter(const SurfaceInteraction* SI, vec3& attenuation, Ray& scattered, float& pdf) const override
	{
		vec3 scatter_dir = SI->normal + random_unit_vector();

		if (scatter_dir.near_zero())
			scatter_dir = SI->normal;

		scattered = Ray(SI->point + SI->normal * 0.001f, normalize(scatter_dir));
		attenuation = albedo->sample(SI->u, SI->v, SI->point);
		return true;
	}
private:
	Texture* albedo;
};

class MetalMaterial : public Material
{
public:
	MetalMaterial(vec3 albedo, float fuzz = 0.f) : albedo(albedo), fuzz(fuzz) {}
	virtual bool scatter(const SurfaceInteraction* SI, vec3& attenuation, Ray& scattered) const override
	{
		vec3 reflected = reflect(-SI->w0, SI->normal);
		scattered = Ray(SI->point + SI->normal * 0.001f, normalize(reflected + fuzz * random_in_unit_sphere()));
		attenuation = albedo;
		return (dot(scattered.dir, SI->normal) > 0);
	}

private:
	vec3 albedo;
	float fuzz;
};

class GlassMaterial : public Material
{
public:
	GlassMaterial(float index_refraction) : index_r(index_refraction) {}
	virtual bool scatter(const SurfaceInteraction* SI, vec3& attenuation, Ray& scattered) const override
	{
		attenuation = vec3(1);
		float refrac_ratio = (SI->front_face) ? (1.0 / index_r) : index_r;

		float cos_theta = fmin(dot(SI->w0, SI->normal), 1.0);
		float sin_theta = sqrt(1.f - cos_theta * cos_theta);

		bool cant_refract = refrac_ratio * sin_theta > 1.f;
		vec3 direction;
		if (cant_refract)
			direction = reflect(-SI->w0, SI->normal);
		else
			direction = refract(-SI->w0, SI->normal, refrac_ratio);

		//vec3 refracted = refract(ray_in.dir, res.normal, refrac_ratio);
		scattered = Ray(SI->point + SI->normal * ((cant_refract) ? 0.001f : -0.001f), normalize(direction));
		return true;
	}
private:
	float index_r;
};
class EmissiveMaterial : public Material
{
public:
	EmissiveMaterial(vec3 emissive_color) : emit(emissive_color) {}
	virtual vec3 emitted() const override {
		return emit;
	}

private:
	vec3 emit;
};

#endif // !MATERIALS_H

