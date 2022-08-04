#include "Material.h"

float BeckmannDistribution(const vec3& H, const vec3& N, float roughness)
{
	float roughness_2 = roughness * roughness;
	float hdotn = max(dot(H, N), 0);
	if (hdotn <= 0 || roughness <= 0) return 0;
	float theta_h = acos(hdotn);

	float denom = roughness_2 * PI * pow(hdotn, 4);
	return exp(-pow(tan(theta_h), 2) / roughness_2) / denom;
}

// From 'Microfacet Models for Refraction through Rough Surfaces'
float GGXDistribution(const vec3& N, const vec3& H, float alpha)
{
	float NdotH = max(dot(H, N),0);
	if (NdotH <= 0) return 0;
	float half_angle = acos(NdotH);
	float alpha_2 = alpha * alpha;
	float denom = PI * pow(NdotH, 4.f) * pow(alpha_2 + pow(tan(half_angle), 2.f), 2.f)+0.0001;
	if (denom <= 0) return 0;
	return alpha_2 / denom;
}
vec3 GGXSample(const vec3& N, float alpha)
{
	float r1 = random_float();
	float r2 = random_float();
	float theta_m = atan((alpha * sqrt(r1)) / (sqrt(1 - r1)));
	float phi = 2 * PI * r2;

	float sin_theta = sin(theta_m);
	vec3 micro_n = vec3(cos(phi) * sin_theta, sin(phi) * sin_theta, cos(theta_m));

	vec3 T, B;
	ONB(N, T, B);
	// rotate micro normal to macro normal space
	micro_n = T * micro_n.x + B * micro_n.y + N * micro_n.z;

	return micro_n;
}
float GGXPdf(const vec3& V, const vec3& H, const vec3& N, float alpha)
{
	return GGXDistribution(N, H, alpha) * max(dot(H, N), 0) / max(0.0001,4.f*dot(V,H));
}


vec3 FresnelSchlick(float VdotH, vec3 F0)
{
	return F0 + (1.f - F0) * pow(clamp(1.f - VdotH, 0.0, 1.0), 5);
}

float GeometrySchlickGGX(float ndotv, float alpha)
{
	float k = 0.0001;// alpha* alpha * 0.5;
	float denom = ndotv * (1 - k) + k;
	if (denom == 0) return 0;
	return ndotv / denom;
}
float GeometrySmith(const vec3& N, const vec3& V, const vec3& L, float alpha)
{
	float ndotv = max(-dot(N, V), 0);
	float ndotl = max(dot(N, L), 0);
	return GeometrySchlickGGX(ndotv, alpha) * GeometrySchlickGGX(ndotl, alpha);
}


vec3 GGXEval(const Intersection& si, const vec3& V, const vec3& H, const vec3& L, float roughness, vec3 F0)
{
	float alpha = 0.2;// roughness* roughness;
	float D = GGXDistribution(si.normal, H, alpha);
	vec3 F = FresnelSchlick(max(-dot(V, H), 0), F0);
	float G = GeometrySmith(si.normal, V, L, alpha);

	float denom = 4.0 * max(dot(si.normal, V), 0.0) * max(dot(si.normal, L), 0.0) + 0.0001f;
	return max(dot(si.normal,L),0)*(D*F*G) / denom;
}

// -------------UNUSED------------------------
float GGXSmithG1(const vec3& v, const vec3& normal, float roughness)
{
	float vdotn = dot(v, normal);
	if (vdotn < 0)
		return 0;
	float theta_v = acos(vdotn);
	float denom = 1 + sqrt(1.f + (roughness * roughness) * pow(tan(theta_v), 2));
	return 2 / denom;
}
float GGXSmith(const vec3& normal, const vec3& ray_in, const vec3& ray_out, float roughness)
{
	return GGXSmithG1(ray_in, normal, roughness) * GGXSmithG1(ray_out, normal, roughness);
}

float DistributionGGX(const vec3& N, const vec3& H, float alpha)
{
	float alpha_2 = alpha * alpha;
	float NdotH = max(dot(N, H), 0);
	float denom = (NdotH * NdotH) * (alpha_2 - 1.f) + 1.f;
	return alpha_2 / (PI * denom * denom);
}

float PDF_ggx(const vec3& V, const vec3& H, const vec3& N,float alpha)
{
	float ndoth = max(dot(H, N), 0);
	float vdoth = max(dot(V, H), 0);

	return DistributionGGX(V, H, alpha) * ndoth;// /(4*vdoth)
}
// ---------------------------------------------


vec3 Microfacet::Sample_Eval(const Intersection& si, const vec3& in_dir, const vec3& normal, Ray* out_ray, float* pdf) const
{
	float alpha = 0.2; roughness* roughness;

	vec3 Wh = GGXSample(normal, alpha);
	vec3 Wi = -reflect(-in_dir, Wh);

	/*
	Wi.x = 0;
	Wi.y = 0;

	Wi.z = 1/GGXPdf(-in_dir, Wh, normal, alpha);
	if (Wi.z != Wi.z) {
		Wi.z = 0;
		//Wi.x = 1;
		Wi.y = 1;
	}
	*/
	*out_ray = Ray(si.point + si.normal * 0.0001, Wi);
	*pdf = GGXPdf(-in_dir, Wh, normal, alpha);

	bool print = random_float() < 0.0001;
	if (print) {
	//	std::cout << "PDF: " << *pdf << '\n';
	}

	//*pdf = 1;
	
	return GGXEval(si, -in_dir, Wh, Wi,roughness,vec3(0.8));
}
vec3 Microfacet::Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const
{
	vec3 Wh = normalize(-in_dir + out_dir);
	float alpha = 0.5; roughness* roughness;
	*pdf = GGXPdf(-in_dir, Wh, normal, alpha);
	return GGXEval(si, -in_dir, Wh, out_dir, roughness, vec3(0.03));
}

float Microfacet::PDF(const vec3& in_dir, const vec3& out_dir, const vec3& normal) const
{
	vec3 Wh = normalize(-in_dir + out_dir);
	return GGXPdf(-in_dir, Wh, normal, 0.5);// roughness* roughness);
}
/*

{
		vec3 micro_n = GGXSample(normal, roughness);
		vec3 ray_dir = normalize(reflect(-in_dir, micro_n));

		*out_ray = Ray(si.point + normal * 0.001f, ray_dir);
		return Eval(si, in_dir, ray_dir, normal, pdf);
	}

	{
		*pdf = CalcPDF(in_dir, out_dir, si.normal, normal);
		vec3 F0(0.04);
		float G = GGXSmith(normal, in_dir, out_dir,roughness);
		float D = GGXDistribution(normal,)


		return albedo->sample(si.u, si.v, si.point) * max(dot(normal, out_dir), 0.f) / PI;
	}
*/