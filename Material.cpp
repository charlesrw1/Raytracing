#include "Material.h"

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
float DielectricFresnel(float n_dot_i,float n_dot_t, float eta)
{
	float rs = (n_dot_i - eta * n_dot_t) / (n_dot_i + eta * n_dot_t);
	float rp = (n_dot_i * eta - n_dot_t) / (n_dot_i * eta + n_dot_t);

	return (rs * rs + rp * rp) / 2;
}
float DielectricFresnel(float n_dot_i, float eta)
{
	float n_dot_t_sq = 1 - (1 - n_dot_i * n_dot_i) / (eta * eta);
	if (n_dot_t_sq < 0)
		return 1;
	float n_dot_t = sqrt(n_dot_t_sq);
	return DielectricFresnel(abs(n_dot_i), n_dot_t, eta);
}


vec3 EvalDisneyRefraction(vec3 V, vec3 L, vec3 H, float eta, float ax, float ay, vec3 base, float* pdf)
{
	float D = TrowbridgeReitzDistribution(H, ax, ay);
	float h_dot_in = dot(H, V);
	float F = DielectricFresnel(h_dot_in, 1/eta);
	float g1 = SmithOcclusionAni_(V, ax, ay);
	float g2 = SmithOcclusionAni_(L, ax, ay);
	float G = g1 * g2;

	float h_dot_out = abs(dot(H, L));
	float sqrt_denom = h_dot_in + eta * h_dot_out;
	vec3 out = base * (1 - F) * D * G * eta * eta * abs(h_dot_out * h_dot_in) / (abs(V.z) * sqrt_denom * sqrt_denom);

	float dh_dout = eta * eta * h_dot_out / (sqrt_denom * sqrt_denom);
	*pdf = (1 - F) * D * g1 * abs(dh_dout * h_dot_in / V.z);

	return out;
}

vec3 EvalDisneyReflection(vec3 V, vec3 L, vec3 H, float eta, float ax, float ay, vec3 base, float* pdf)
{
	float h_dot_in = dot(H, V);
	float F = DielectricFresnel(h_dot_in, eta);
	float D = TrowbridgeReitzDistribution(H, ax, ay);
	float g1 = SmithOcclusionAni_(V, ax, ay);
	float g2 = SmithOcclusionAni_(L, ax, ay);
	float G = g1 * g2;

	*pdf = F*D * g1 / (4 * V.z);

	return base * F * D * G / (4 * V.z);

}



vec3  DisneyGlass::Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const
{
	float eta = (si.front_face) ? (1 / eta_ie) : eta_ie;
	
	float aspect = sqrt(1.0 - anisotropic * 0.9);
	float ax = max(0.001, roughness * roughness / aspect);
	float ay = max(0.001, roughness * roughness * aspect);

	vec3 T, B;
	ONB(normal, T, B);
	// To tangent space
	vec3 V = vec3(dot(-in_dir, T), dot(-in_dir, B), dot(-in_dir, normal));
	vec3 H = sampleGGXVNDF(V, ax, ay);
	vec3 L;
	float h_dot_in = dot(H, V);
	float F = DielectricFresnel(h_dot_in, 1/eta);
	//F = reflectance(h_dot_in, eta);
	bool do_reflect = F > random_float();
	vec3 out;
	if (do_reflect)
	{
		// reflect
		L = normalize(reflect(-V, H));
		vec3 world_out = to_world(L, normal, T, B);
		*out_ray = Ray(si.point + si.normal * 0.00001, world_out);
		out = EvalDisneyReflection(V, L, H, eta, ax, ay, albedo, pdf);
	}
	else {
		// refract
		L = normalize(refract(-V, H, eta));
		vec3 world_out = to_world(L, normal, T, B);
		*out_ray = Ray(si.point - si.normal * 0.00001, world_out);
		out = EvalDisneyRefraction(V, L, H, eta, ax, ay, albedo, pdf);
	}
	return out;
}
vec3  DisneyGlass::Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const
{
	bool reflect = dot(normal, out_dir) >= 0;
	float eta = (si.front_face) ? 1 / eta_ie : eta_ie;
	
	
	float aspect = sqrt(1.0 - anisotropic * 0.9);
	float ax = max(0.001, roughness * roughness / aspect);
	float ay = max(0.001, roughness * roughness * aspect);

	vec3 T, B;
	ONB(normal, T, B);
	// To tangent space
	vec3 V = vec3(dot(-in_dir, T), dot(-in_dir, B), dot(-in_dir, normal));
	vec3 L = vec3(dot(out_dir, T), dot(out_dir, B), dot(out_dir, normal));
	vec3 H;
	if (reflect) {
		H = normalize(V + L);
		return EvalDisneyReflection(V, L, H, eta, ax, ay, albedo, pdf);
	}
	else {
		H = normalize(V + L * eta);
		return EvalDisneyRefraction(V, L, H, eta, ax, ay, albedo, pdf);
	}
}
float DisneyGlass::PDF(const vec3& in_dir, const vec3& out_dir, const vec3& normal) const
{
	return 0;
}


vec3 RoughDielectric::Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const
{
	float eta = (si.front_face) ? 1 / eta_i_e : eta_i_e;

	float alpha = roughness * roughness;
	vec3 T, B;
	ONB(normal, T, B);
	// To tangent space
	vec3 local_in = vec3(dot(-in_dir, T), dot(-in_dir, B), dot(-in_dir, normal));
	vec3 local_micro_normal = sampleGGXVNDF(local_in, alpha, alpha);

	vec3 world_half_vec = to_world(local_micro_normal, normal, T, B);
	float h_dot_in = dot(world_half_vec, -in_dir);
	float F = DielectricFresnel(h_dot_in, eta);

	float r1 = random_float();
	bool reflect_ = r1 <= F;
	if (reflect_) {
		// Reflect
		vec3 reflected = normalize(reflect(in_dir, world_half_vec));
		*out_ray = Ray(si.point + si.normal * 0.00001f, reflected);
	}
	else {
		// Refract

		vec3 refracted = refract(in_dir, world_half_vec, eta);
		*out_ray = Ray(si.point + si.normal * -0.00001f, normalize(refracted));
	}

	return Eval(si, in_dir, out_ray->dir, normal, pdf);
}
vec3 RoughDielectric::Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const
{
	bool reflect = dot(normal, out_dir) <=  0;
	float eta = (si.front_face) ? 1 / eta_i_e : eta_i_e;
	vec3 half_vec;
	if (reflect)
		half_vec = normalize(-in_dir + out_dir);
	else
		half_vec = normalize(-in_dir + out_dir * eta);


	vec3 T, B;
	ONB(normal, T, B);
	// To tangent space
	vec3 local_in = vec3(dot(-in_dir, T), dot(-in_dir, B), dot(-in_dir, normal));
	vec3 local_half = to_local(half_vec, normal, T, B);
	vec3 local_out = to_local(out_dir, normal, T, B);

	if (dot(half_vec, vec3(0, 0, 1)) < 0)
		half_vec = -half_vec;

	float h_dot_in = dot(half_vec, -in_dir);
	float F = DielectricFresnel(h_dot_in, eta);
	float alpha = roughness * roughness;
	float D = TrowbridgeReitzDistribution(local_half, alpha, alpha);
	float g1 = SmithOcclusionAni_(local_in, alpha, alpha);
	float g2 = SmithOcclusionAni_(local_out, alpha, alpha);
	float G = g1 * g2;

	if (reflect)
	{
		vec3 out = specular_reflectance* (F * D * G) / (4 * abs(local_in.z));
		*pdf = (F * D * g1) / (4 * abs(local_in.z));
		return out;
	}

	else {
		float h_dot_out = abs(dot(half_vec, out_dir));
		float sqrt_denom = h_dot_in + eta * h_dot_out;
		vec3 out = specular_transmittance*(1 - F) * D * G * eta * eta * abs(h_dot_out * h_dot_in) / (abs(local_in.z) * sqrt_denom * sqrt_denom);

		float dh_dout = eta * eta * h_dot_out / (sqrt_denom * sqrt_denom);
		*pdf= (1 - F) * D * g1 * abs(dh_dout * h_dot_in / local_in.z);

		return out;
	}

}
float RoughDielectric::PDF(const vec3& in_dir, const vec3& out_dir, const vec3& normal) const { return 0; }


vec3 DisneySheen::Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const
{
	vec3 scatter_dir = random_cosine();
	vec3 T, B;
	ONB(normal, T, B);
	scatter_dir = scatter_dir.x * T + scatter_dir.y * B + scatter_dir.z * normal;
	*out_ray = Ray(si.point + normal * 0.001f, scatter_dir);
	return Eval(si, in_dir, scatter_dir, normal, pdf);
}
vec3 DisneySheen::Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const
{
	*pdf = max(dot(normal, out_dir), 0.f) / PI;;

	vec3 half = normalize(-in_dir + out_dir);

	float denom = luminance(base_color);
	if (denom < 0)denom = 1;
	vec3 tint = base_color / denom;
	vec3 c_sheen = (1 - sheen_tint) + sheen_tint * tint;
	vec3 f_sheen = c_sheen * pow(1 - abs(dot(half, out_dir)), 5) * dot(normal, out_dir);

	return f_sheen;
}

//////////// UBER ///////////////

vec3 DisneyUber::EvalDiffuse(const Intersection& si, const vec3& V, const vec3& L, const vec3& H, float* pdf) const
{
	*pdf = max(L.z, 0.f) / PI;

	float HdotOut = max(dot(H, L), 0);
	float NdotOut = max(L.z, 0);
	float NdotIn = max(V.z, 0);


	float Fd90 = 0.5f + 2.f * roughness * HdotOut * HdotOut;
	vec3 f_base_diffuse = base_color / PI * DiffuseFresnel(NdotIn, Fd90) * DiffuseFresnel(NdotOut, Fd90) * NdotOut;

	float Fss90 = roughness * HdotOut * HdotOut;
	vec3 f_subsurface = (1.25 * base_color) / PI * (DiffuseFresnel(NdotIn, Fss90) * DiffuseFresnel(NdotOut, Fss90) * (1.f / (NdotIn + NdotOut) - 0.5f) + 0.5f) * NdotOut;

	vec3 diffuse = (1 - subsurface) * f_base_diffuse + subsurface * f_subsurface;

	return diffuse *(1 - specular_transmission)* (1 - metallic);
}

vec3 DisneyUber::EvalMetal(const Intersection& si, const vec3& V, const vec3& L, const vec3& H, float eta, float ax, float ay, float* pdf) const
{
	vec3 c_tint = base_color / luminance(base_color);
	vec3 ks = (1 - specular_tint) + specular_tint * c_tint;
	vec3 C0 = specular * R0(eta) * (1 - metallic) * ks + metallic * base_color;
	vec3 F = C0 + (1 - C0) * pow(1 - (max(dot(L, H), 0)), 5);

	//vec3 F = FresnelSchlick(max(dot(V, H), 0), base_color);
	float D = TrowbridgeReitzDistribution(H, ax, ay);
	float g1 = SmithOcclusionAni_(V, ax, ay);
	float g2 = SmithOcclusionAni_(L, ax, ay);
	float G = g1 * g2;

	*pdf = D * g1 / (4 * V.z);

	vec3 f= F * D * G / (4 * V.z);

	return f * (1 - specular_transmission * (1 - metallic));
}

vec3 DisneyUber::Sample_Eval(const Intersection& si, const vec3 in_dir, const vec3& normal, Ray* out_ray, float* pdf) const
{
	vec3 T, B;
	ONB(normal, T, B);
	// To tangent space
	vec3 V = vec3(dot(-in_dir, T), dot(-in_dir, B), dot(-in_dir, normal));

	float diffuse_weight = (1 - metallic) * (1 - specular_transmission);
	float metal_weight = (1 - specular_transmission * (1 - metallic));
	float glass_weight = (1 - metallic) * specular_transmission;
	float clearcoat_weight = 0.25 * clearcoat;
	if (!si.front_face)
		diffuse_weight = metal_weight = clearcoat_weight = 0;

	float cdf[4];
	cdf[0] = diffuse_weight;
	cdf[1] = cdf[0] + metal_weight;
	cdf[2] = cdf[1] + glass_weight;
	cdf[3] = cdf[2] + clearcoat_weight;
	
	float sum = cdf[3];
	float lobe = random_float()*sum;

	float aspect = sqrt(1.0 - anisotropic * 0.9);
	float ax = max(0.001, roughness * roughness / aspect);
	float ay = max(0.001, roughness * roughness * aspect);

	float eta = (si.front_face) ? 1 / ior : ior;

	*pdf = 0;
	vec3 f = vec3(0);
	if (lobe < cdf[0]) /* Diffuse */
	{
		vec3 L = random_cosine();
		vec3 scatter_dir = L.x * T + L.y * B + L.z * normal;
		*out_ray = Ray(si.point + normal * 0.001f, scatter_dir);

		f = EvalDiffuse(si,V, L, normalize(V + L), pdf);
		*pdf = (*pdf) * (diffuse_weight / sum);
	}
	else if (lobe < cdf[1]) /* Specular reflection/Metallic */
	{
		vec3 T, B;
		ONB(normal, T, B);
		// To tangent space
		vec3 H = sampleGGXVNDF(V, ax, ay);
		vec3 L = reflect(-V, H);
		vec3 scatter_dir = L.x * T + L.y * B + L.z * normal;
		*out_ray = Ray(si.point + normal * 0.001f, scatter_dir);

		f = EvalMetal(si,V, L, H, eta, ax, ay, pdf);

		*pdf *= (metal_weight / sum);
	}
	//else if (lobe < cdf[2])
	//{
	//
	//}
	//// clearcoat
	//else
	//{
	//
	//}


	return f;
}
vec3 DisneyUber::Eval(const Intersection& si, const vec3& in_dir, const vec3& out_dir, const vec3& normal, float* pdf) const
{
	vec3 T, B;
	ONB(normal, T, B);
	// To tangent space
	vec3 V = vec3(dot(-in_dir, T), dot(-in_dir, B), dot(-in_dir, normal));
	vec3 L = vec3(dot(out_dir, T), dot(out_dir, B), dot(out_dir, normal));
	vec3 H = normalize(V + L);

	float diffuse_weight = (1 - metallic) * (1 - specular_transmission);
	float metal_weight = (1 - specular_transmission * (1 - metallic));
	float glass_weight = (1 - metallic) * specular_transmission;
	float clearcoat_weight = 0.25 * clearcoat;
	if (!si.front_face)
		diffuse_weight = metal_weight = clearcoat_weight = 0;

	float sum = diffuse_weight + metal_weight + glass_weight + clearcoat_weight;
	diffuse_weight /= sum;
	metal_weight /= sum;
	glass_weight /= sum;
	clearcoat_weight /= sum;

	float aspect = sqrt(1.0 - anisotropic * 0.9);
	float ax = max(0.001, roughness * roughness / aspect);
	float ay = max(0.001, roughness * roughness * aspect);

	float eta = (si.front_face) ? 1 / ior : ior;

	float pdf_;
	float total_pdf = 0;
	vec3 f = vec3(0);
	if (diffuse_weight > 0)
	{
		f += EvalDiffuse(si, V, L, H, &pdf_);
		total_pdf += pdf_ * diffuse_weight ;
	}
	if (metal_weight > 0)
	{
		f += EvalMetal(si, V, L, H, eta, ax, ay, &pdf_);
		total_pdf += pdf_ * metal_weight;
	}
	*pdf = total_pdf;

	return f;
}
