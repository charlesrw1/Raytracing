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
		bool odd_ = (modulo(u, grid) < grid * 0.5) ^ (modulo(v, grid) < grid * 0.5);
		//float sines = sin(grid * p.x) * sin(grid * p.y) * sin(grid * p.z);
		//uint8_t res = (fmod(fmod(p.x, mod)+mod,mod) < repeat) ^ (fmod(fmod(p.y, mod)+mod,mod) < repeat) ^ (fmod(fmod(p.z, mod)+mod,mod) < repeat);
		if (odd_)
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
	virtual vec3 emitted() const {
		return vec3(0);
	}


	virtual vec3 Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const {
		*pdf = 0;
		return vec3(0);
	}
	virtual vec3 Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const {
		*pdf = 0;
		return vec3(0);
	}
	virtual float PDF(const vec3& in_dir, const vec3& out_dir, const vec3& normal) const {
		return 0.0;
	}
};
class MatteMaterial : public Material
{
public:
	MatteMaterial(Texture* albedo) : albedo(albedo) {}
	~MatteMaterial() {
		delete albedo;
	}
	virtual vec3 Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const override 
	{
		vec3 scatter_dir = random_cosine();
		vec3 T, B;
		ONB(normal, T, B);
		scatter_dir = scatter_dir.x * T + scatter_dir.y * B + scatter_dir.z * normal;
		*out_ray = Ray(si.point + normal * 0.001f, scatter_dir);
		return Eval(si, in_dir, scatter_dir, normal, pdf);
	}

	virtual vec3 Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const override
	{
		*pdf = PDF(in_dir, out_dir, normal);
		return albedo->sample(si.u, si.v, si.point) * max(dot(normal,out_dir),0.f) / PI;
	}
	virtual float PDF(const vec3& in_dir, const vec3& out_dir, const vec3& normal) const override final {
		return max(dot(normal, out_dir), 0.f) / PI;
	}

private:
	Texture* albedo;
};


class Microfacet : public Material
{
public:
	Microfacet(Texture* albedo, float roughness, float metalness)
		: albedo(albedo), roughness(roughness), metalness(metalness) {}

	virtual vec3 Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const override;
	virtual vec3 Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const override;
	virtual float PDF(const vec3& in_dir, const vec3& out_dir, const vec3& normal) const override;
private:

	Texture* albedo;
	float roughness;
	float metalness;
};

class DisneyDiffuse : public Material
{
public:
	DisneyDiffuse(vec3 albedo, float roughness, float subsurface)
		:albedo(albedo),roughness(roughness),subsurface(subsurface) {}

	virtual vec3 Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const override;
	virtual vec3 Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const override;
	virtual float PDF(const vec3& in_dir, const vec3& out_dir, const vec3& normal) const override;

private:
	vec3 albedo;
	float roughness;
	float subsurface;
};
class DisneyMetal : public Material
{
public:
	DisneyMetal(vec3 albedo, float roughness, float anisotropic)
		:albedo(albedo), roughness(roughness), anisotropic(anisotropic) {}

	virtual vec3 Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const override;
	virtual vec3 Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const override;
	virtual float PDF(const vec3& in_dir, const vec3& out_dir, const vec3& normal) const override;

private:
	vec3 albedo;
	float roughness;
	float anisotropic;
};

class DisneyClearcoat : public Material
{
public:
	DisneyClearcoat(float clearcoat_gloss)
		:clearcoat_gloss(clearcoat_gloss){}

	virtual vec3 Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const override;
	virtual vec3 Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const override;
	virtual float PDF(const vec3& in_dir, const vec3& out_dir, const vec3& normal) const override;

private:
	float clearcoat_gloss;
};

class DisneySheen : public Material
{
public:
	DisneySheen(vec3 base, float sheen_tint)
		:base_color(base),sheen_tint(sheen_tint) {}

	virtual vec3 Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const override;
	virtual vec3 Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const override;

private:
	vec3 base_color;
	float sheen_tint;
};

class DisneyUber : public Material
{
public:

	virtual vec3 Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const override;
	virtual vec3 Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const override;


	vec3 EvalDiffuse(const Intersection& si,const vec3& V, const vec3& L, const vec3& H, float* pdf) const;
	vec3 EvalMetal(const Intersection& si, const vec3& V, const vec3& L, const vec3& H, float eta, float ax, float ay, float* pdf) const;
	vec3 EvalGlass(const Intersection& si, const vec3& V, const vec3& L, const vec3& H, float eta, float* pdf) const;
	vec3 EvalClearcoat(const Intersection& si, const vec3& V, const vec3& L, const vec3& H, float* pdf) const;

	vec3 base_color = vec3(0);
	float specular_transmission = 0;
	float metallic=0;
	float subsurface=0;
	float specular = 0;
	float roughness=0;
	float specular_tint=0;
	float anisotropic=0;
	float sheen=0;
	float sheen_tint=0;
	float clearcoat=0;
	float clearcoat_gloss=0;

	float ior = 1.0;
};

class RoughDielectric : public Material
{
public:
	RoughDielectric(float eta,
		float roughness,
		vec3 specular_reflectance,
		vec3 specular_transmittance)
		: eta_i_e(eta), roughness(roughness),
		specular_reflectance(specular_reflectance),
		specular_transmittance(specular_transmittance)
	{}
	virtual vec3 Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const override;
	virtual vec3 Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const override;
	virtual float PDF(const vec3& in_dir, const vec3& out_dir, const vec3& normal) const override;

private:
	float eta_i_e;	// internal IOR / external IOR
	float roughness;
	vec3 specular_reflectance;
	vec3 specular_transmittance;
};

class DisneyGlass : public Material
{
public:
	DisneyGlass(vec3 albedo, float roughness, float anisotropic, float eta)
		:albedo(albedo), roughness(roughness), anisotropic(anisotropic), eta_ie(eta){}

	virtual vec3 Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const override;
	virtual vec3 Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const override;
	virtual float PDF(const vec3& in_dir, const vec3& out_dir, const vec3& normal) const override;

private:
	vec3 albedo;
	float roughness;
	float anisotropic;

	float eta_ie;	// internal/external IOR
};


class GlassMaterial : public Material
{
public:
	GlassMaterial(float index_refraction) : index_r(index_refraction) {}
	virtual vec3 Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const {
		*pdf = 0;
		return vec3(0);
	}
	virtual vec3 Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const override {
		float refrac_ratio = (si.front_face) ? (1.0 / index_r) : index_r;

		float cos_theta = fmin(dot(-in_dir, si.normal), 1.0);
		float sin_theta = sqrt(1.f - cos_theta * cos_theta);

		bool reflect_ = refrac_ratio * sin_theta > 1.f || reflectance(cos_theta, refrac_ratio) > random_float();
		vec3 direction;
		if (reflect_)
			direction = reflect(in_dir, si.normal);
		else
			direction = refract(in_dir, si.normal, refrac_ratio);

		//vec3 refracted = refract(ray_in.dir, res.normal, refrac_ratio);
		*out_ray = Ray(si.point + si.normal * ((reflect_) ? 0.001f : -0.001f), normalize(direction));
		*pdf = 1;

		return vec3(1);
	}
	virtual float PDF(const vec3& in_dir, const vec3& out_dir, const vec3& normal) const override {
		return 0.0;
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

