#include "Material.h"

//const float TEMP_ALPHA = 0.1;// 0.01;

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
	float denom = PI * pow(NdotH, 4.f) * pow(alpha_2 + pow(tan(half_angle), 2.f), 2.f);// +0.0001;
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
	return GGXDistribution(N, H, alpha) * max(dot(H, N), 0) / (4.f * max(dot(V, H),0)+0.0000001f);
}


vec3 FresnelSchlick(float VdotH, vec3 F0)
{
	return F0 + (1.f - F0) * pow(clamp(1.f - VdotH, 0.0, 1.0), 5);
}

float GeometrySchlickGGX(float ndotv, float roughness)
{
	float k = roughness * roughness * 0.5;//;// 0.001;// alpha* alpha * 0.5;
	float denom = ndotv * (1 - k) + k;
	if (denom == 0) return 0;
	return ndotv / denom;
}
float GeometrySmith(const vec3& N, const vec3& V, const vec3& L, float roughness)
{
	float ndotv = max(dot(N, V), 0);
	float ndotl = max(dot(N, L), 0);
	return GeometrySchlickGGX(ndotv, roughness) * GeometrySchlickGGX(ndotl, roughness);
}


vec3 GGXEval(const Intersection& si, const vec3& V, const vec3& H, const vec3& L, float roughness, vec3 F0)
{
	float alpha = roughness* roughness;
	float D = GGXDistribution(si.normal, H, alpha);
	vec3 F = FresnelSchlick(max(dot(V, H), 0), F0);
	float G = GeometrySmith(si.normal, V, L, roughness);

	float denom = 4.0 * max(dot(si.normal, V), 0.0) * max(dot(si.normal, L), 0.0) + 0.0000001f;
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


vec3 Microfacet::Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const
{
	float alpha = roughness* roughness;

	vec3 Wo = -in_dir;
	vec3 Wh = GGXSample(normal, alpha);
	vec3 Wi = -reflect(-in_dir, Wh);

	out_ray->pos = si.point + si.normal * 0.0001;
	out_ray->dir = Wi;
	*pdf = GGXPdf(Wo, Wh, normal, alpha);
	if (*pdf != *pdf)*pdf = 0;

	//*pdf = min(*pdf, 100'000.0);
	//if (*pdf != *pdf)
	//	*pdf = 100'000.0;

	return GGXEval(si, Wo, Wh, Wi,roughness,vec3(0.8));
}
vec3 Microfacet::Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const
{
	vec3 Wo = -in_dir;
	vec3 Wh = normalize(Wo + out_dir);
	float alpha =  roughness* roughness;
	*pdf = GGXPdf(Wo, Wh, normal, alpha);

	//*pdf = min(*pdf, 100'000.0);
	//if (*pdf != *pdf)
	//	*pdf = 100'000.0;
	return GGXEval(si, Wo, Wh, out_dir, roughness, vec3(0.8));
}

float Microfacet::PDF(const vec3& in_dir, const vec3& out_dir, const vec3& normal) const
{
	vec3 Wo = -in_dir;
	vec3 Wh = normalize(Wo + out_dir);
	float pdf = GGXPdf(Wo, Wh, normal, roughness*roughness);// roughness* roughness);
	//pdf = min(pdf, 100'000.0);
	//if (pdf != pdf)
	//	pdf = 100'000.0;
	return pdf;
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

////////// Diffuse //////////

float DiffuseFresnel(float NdotW, float Fd90)
{
	return (1 + (Fd90 - 1) * pow(1 - NdotW,5));
}

vec3  DisneyDiffuse::Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const
{
	vec3 scatter_dir = random_cosine();
	vec3 T, B;
	ONB(normal, T, B);
	scatter_dir = scatter_dir.x * T + scatter_dir.y * B + scatter_dir.z * normal;
	*out_ray = Ray(si.point + normal * 0.001f, scatter_dir);
	return Eval(si, in_dir, scatter_dir, normal, pdf);
}
vec3  DisneyDiffuse::Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const
{
	*pdf = PDF(in_dir, out_dir, normal);

	vec3 Wh = normalize(-in_dir + out_dir);

	float HdotOut = max(dot(Wh, out_dir),0);
	float NdotOut = max(dot(normal, out_dir), 0);
	float NdotIn = max(dot(si.normal, -in_dir),0);


	float Fd90 = 0.5f + 2.f * roughness * HdotOut * HdotOut;
	vec3 f_base_diffuse = albedo / PI * DiffuseFresnel(NdotIn,Fd90)*DiffuseFresnel(NdotOut,Fd90)*NdotOut;

	float Fss90 = roughness * HdotOut * HdotOut;
	vec3 f_subsurface = (1.25 * albedo) / PI * (DiffuseFresnel(NdotIn, Fss90) * DiffuseFresnel(NdotOut, Fss90) * (1.f / (NdotIn + NdotOut) - 0.5f)+0.5f) * NdotOut;

	vec3 diffuse = (1 - subsurface) * f_base_diffuse + subsurface * f_subsurface;


	return diffuse;
}
float DisneyDiffuse::PDF(const vec3& in_dir, const vec3& out_dir, const vec3& normal) const
{
	return max(dot(normal, out_dir), 0.f) / PI;
}

////////// Metal //////////


float TrowbridgeReitzDistribution(vec3 H, float ax, float ay)
{
	float denom = PI * ax * ay * pow((pow(H.x / ax, 2) + pow(H.y / ay, 2) + H.z * H.z), 2);
	return 1 / denom;
}

float SmithOcclusionAni_(vec3 w, float ax, float ay)
{
	float inner_root = 1 + (w.x * ax * w.x * ax + w.y * ay * w.y * ay) / (w.z * w.z);
	float denom = 1.f + (sqrt(1 + inner_root) - 1)/2.f;
	return 1 / denom;
}
float SmithOcclusionAni(vec3 Wi, vec3 Wo, float ax, float ay)
{
	return SmithOcclusionAni_(Wi, ax, ay) * SmithOcclusionAni_(Wo, ax, ay);
}

// From sampling the GGX distribution of visible normals
vec3 sampleGGXVNDF(vec3 Wi, float ax, float ay)
{
	float r1 = random_float();
	float r2 = random_float();

	vec3 Vh = normalize(vec3(ax * Wi.x, ay * Wi.y, Wi.z));
	float length_2 = Wi.x * Wi.x + Wi.y * Wi.y;
	vec3 T1 = length_2 > 0 ? vec3(-Vh.y, Vh.x, 0) / sqrt(length_2) : vec3(1, 0, 0);
	vec3 T2 = cross(T1, Vh);

	float r = sqrt(r1);
	float phi = 2.0 * PI * r2;
	float t1 = r * cos(phi);
	float t2 = r * sin(phi);
	float s = 0.5 * (1 + Vh.z);

	t2 = (1 - s) * sqrt(1 - t1 * t1) + s * t2;
	vec3 Nh = t1 * T1 + t2 * T2 + sqrt(max(1 - t1 * t1 - t2 * t2, 0)) * Vh;
	vec3 normal = normalize(vec3(ax * Nh.x, ay * Nh.y, max(0, Nh.z)));

	return normal;
}

vec3 EvalDisneyMetal(vec3 V, vec3 L, vec3 H, float roughness, float ax, float ay, vec3 base_color, float* pdf)
{
	vec3 F = FresnelSchlick(max(dot(V, H),0), base_color);
	float D = TrowbridgeReitzDistribution(H, ax, ay);
	float g1 = SmithOcclusionAni_(V, ax, ay);
	float g2 = SmithOcclusionAni_(L, ax, ay);
	float G = g1 * g2;

	*pdf = D * g1 / (4 * V.z);

	return F * D * G / (4 * V.z);
}

vec3  DisneyMetal::Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const
{
	float aspect = sqrt(1.0 - anisotropic * 0.9);
	float ax = max(0.001, roughness*roughness / aspect);
	float ay = max(0.001, roughness*roughness* aspect);
	
	vec3 T, B;
	ONB(normal, T, B);
	// To tangent space
	vec3 V = vec3(dot(-in_dir, T), dot(-in_dir, B), dot(-in_dir, normal));
	vec3 H = sampleGGXVNDF(V, ax, ay);
	vec3 L = reflect(-V, H);

	vec3 out = EvalDisneyMetal(V, L, H, roughness, ax, ay, albedo, pdf);

	L = L.x * T + L.y * B + L.z * si.normal;
	*out_ray = Ray(si.point + si.normal * 0.00001, L);

	return out;
}
vec3  DisneyMetal::Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const
{
	float aspect = sqrt(1.0 - anisotropic * 0.9);
	float ax = max(0.001, roughness * roughness / aspect);
	float ay = max(0.001, roughness * roughness * aspect);

	vec3 T, B;
	ONB(normal, T, B);
	// To tangent space
	vec3 V = vec3(dot(-in_dir, T), dot(-in_dir, B), dot(-in_dir, normal));
	vec3 L = vec3(dot(out_dir, T), dot(out_dir, B), dot(out_dir, normal));
	vec3 H = normalize(V + L);

	return EvalDisneyMetal(V, L, H, roughness, ax, ay, albedo, pdf);
}
float DisneyMetal::PDF(const vec3& in_dir, const vec3& out_dir, const vec3& normal) const
{
	return 0;
}

////////// Clearcoat //////////

float R0(float index_r)
{
	return pow((index_r - 1), 2) / pow((index_r + 1), 2);
}
float ClearcoatDistribution(float ag, float HdotN)
{
	float ag_2 = ag * ag;
	float denom = PI * log(ag_2) * (1 + (ag_2 - 1) * HdotN * HdotN);
	return (ag_2 - 1) / denom;
}

vec3 ClearcoatSample(float ag)
{
	float ag_2 = ag * ag;
	float r1 = random_float();
	float r2 = random_float();

	float phi = 2 * PI * r2;
	float cos_theta = sqrt((1 - pow(ag_2, 1 - r1)) / (1 - ag_2));
	float sin_theta = clamp(sqrt(1 - (cos_theta * cos_theta)), 0, 1);
	float sin_phi = sin(phi);
	float cos_phi = cos(phi);

	return vec3(sin_theta * cos_phi, sin_theta * sin_phi, cos_theta);
}

vec3 DisneyClearcoatEval(vec3 V, vec3 L, vec3 H,float clearcoat_gloss, float* pdf)
{
	float r0 = R0(1.5);
	float ag = (1 - clearcoat_gloss)*0.1+clearcoat_gloss*0.001;
	vec3 F = r0 + (1 - r0) * pow(1 - max(dot(H, L), 0),5);
	float D = ClearcoatDistribution(ag, H.z);
	float G = SmithOcclusionAni(V, L, 0.25,0.25);

	// pdf here
	*pdf = ClearcoatDistribution(ag, H.z)/(4*max(dot(H,L),0)+0.00000001);

	return F * D * G / (4 * V.z);
}


vec3  DisneyClearcoat::Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const
{
	vec3 T, B;
	ONB(normal, T, B);
	// To tangent space
	vec3 V = vec3(dot(-in_dir, T), dot(-in_dir, B), dot(-in_dir, normal));
	float ag = (1 - clearcoat_gloss) * 0.1 + clearcoat_gloss * 0.001;
	vec3 H = ClearcoatSample(ag);
	vec3 L = reflect(-V, H);

	vec3 out = DisneyClearcoatEval(V, L, H, clearcoat_gloss, pdf);

	L = L.x * T + L.y * B + L.z * si.normal;
	*out_ray = Ray(si.point + si.normal * 0.00001, L);

	return out;
}
vec3  DisneyClearcoat::Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const
{
	vec3 T, B;
	ONB(normal, T, B);
	// To tangent space
	vec3 V = vec3(dot(-in_dir, T), dot(-in_dir, B), dot(-in_dir, normal));
	vec3 L = vec3(dot(out_dir, T), dot(out_dir, B), dot(out_dir, normal));
	vec3 H = normalize(V + L);

	return DisneyClearcoatEval(V, L, H, clearcoat_gloss, pdf);

}
float DisneyClearcoat::PDF(const vec3& in_dir, const vec3& out_dir, const vec3& normal) const
{
	return 0;
}

////////// Glass //////////

float DielectricFresnel(vec3 V, vec3 L, vec3 H, float eta)
{
	float hdotv = dot(V, H);
	float hdotl = dot(H, L);

	float rs = (hdotv - eta * hdotl) / (hdotv + eta * hdotl);
	float rp = (hdotv*eta - hdotl) / (hdotv*eta + hdotl);

	return (rs * rs + rp * rp) / 2;
}

vec3 EvalDisneyRefraction(vec3 V, vec3 L, vec3 H, float eta, float ax, float ay, vec3 base, float* pdf)
{
	float D = TrowbridgeReitzDistribution(H, ax, ay);
	float F = DielectricFresnel(V,L,H, eta);
	float g1 = SmithOcclusionAni_(V, ax, ay);
	float g2 = SmithOcclusionAni_(L, ax, ay);
	float G = g1 * g2;

	float denom = pow(dot(L, H) + dot(V, H) * eta,2);
	float jacobian = abs(dot(L, H)) / denom;

	*pdf = g1 * D * max(dot(V, H), 0) * jacobian / V.z;

	return sqrt(base) * (1 - F) *D*G* abs(dot(V, H)) * jacobian / V.z;
}

vec3 EvalDisneyReflection(vec3 V, vec3 L, vec3 H, float eta, float ax, float ay, vec3 base, float* pdf)
{
	float F = DielectricFresnel(V,L,H, eta);
	float D = TrowbridgeReitzDistribution(H, ax, ay);
	float g1 = SmithOcclusionAni_(V, ax, ay);
	float g2 = SmithOcclusionAni_(L, ax, ay);
	float G = g1 * g2;

	*pdf = D * g1 / (4 * V.z);

	return base*F * D * G / (4 * V.z);
}


vec3  DisneyGlass::Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const
{
	float refrac_ratio = (si.front_face) ? (1.0 / eta) : eta;


	float aspect = sqrt(1.0 - anisotropic * 0.9);
	float ax = max(0.001, roughness * roughness / aspect);
	float ay = max(0.001, roughness * roughness * aspect);

	vec3 T, B;
	ONB(normal, T, B);
	// To tangent space
	vec3 V = vec3(dot(-in_dir, T), dot(-in_dir, B), dot(-in_dir, normal));
	vec3 H = sampleGGXVNDF(V, ax, ay);
	vec3 L;
	vec3 out;
	float r0 = R0(refrac_ratio);
	float F = r0 + (1 - r0) * pow(1 - max(dot(H, V), 0), 5);
	if (0)
	{
		// reflect
		L = normalize(reflect(-V, H));
		out = EvalDisneyReflection(V, L, H, refrac_ratio, ax, ay, albedo, pdf);
	}
	else
	{
		// refract
		L = refract(-V, H, refrac_ratio);
		out = EvalDisneyRefraction(V, L, H, refrac_ratio, ax, ay, albedo, pdf);
	}


	L = L.x * T + L.y * B + L.z * si.normal;
	*out_ray = Ray(si.point + si.normal * ((si.front_face)?0.00001:-0.00001), L);

	return out;

}
vec3  DisneyGlass::Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const
{
	float aspect = sqrt(1.0 - anisotropic * 0.9);
	float ax = max(0.001, roughness * roughness / aspect);
	float ay = max(0.001, roughness * roughness * aspect);

	vec3 T, B;
	ONB(normal, T, B);
	// To tangent space
	vec3 V = vec3(dot(-in_dir, T), dot(-in_dir, B), dot(-in_dir, normal));
	vec3 L = vec3(dot(out_dir, T), dot(out_dir, B), dot(out_dir, normal));
	vec3 H = normalize(V + L);

	float refrac_ratio = (si.front_face) ? (1.0 / eta) : eta;



	return EvalDisneyReflection(V, L, H, eta, ax, ay, albedo, pdf);
}
float DisneyGlass::PDF(const vec3& in_dir, const vec3& out_dir, const vec3& normal) const
{
	return 0;
}