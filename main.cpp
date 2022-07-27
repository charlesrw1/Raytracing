#include <iostream>
#include <vector>
#include <random>
#include "Math.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

typedef unsigned char u8;
typedef unsigned int u32;
const int WIDTH = 400;
const int HEIGHT = 250;
const float ARATIO = WIDTH / (float) HEIGHT;

const float VIEW_HEIGHT = 2.0;
const float VIEW_WIDTH = ARATIO * VIEW_HEIGHT;
const float NEAR = 1.0;

const vec3 CAM_POS = vec3(0.0,-0.1,0.3);

const float PI = 3.14159;
const int SAMPLES_PER_PIXEL = 5;
const int MAX_DEPTH = 50;
const float GAMMA = 2.2;


const char* OUTPUT_NAME = "Output.bmp";

float radians(float degrees)
{
	return degrees * (PI / 180.f);
}

float random_float()
{
	return rand() / (RAND_MAX + 1.0);
}

float random_float(float min, float max)
{
	return min + (max - min) * random_float();
}
float clamp(float x, float min, float max) 
{
	return (x < min) ? min : ((x > max) ? max : x);
}
vec3 random_vec3(float min, float max)
{
	return vec3(random_float(min,max), random_float(min,max), random_float(min,max));
}
vec3 random_in_unit_sphere()
{
	while (1) {
		vec3 p = random_vec3(-1, 1);
		if (p.length_squared() >= 1) continue;
		return p;
	}
}
vec3 random_unit_vector()
{
	return normalize(random_in_unit_sphere());
}
vec3 reflect(vec3 v, vec3 n)
{
	return v - 2 * dot(v, n) * n;
}
vec3 refract(vec3 uv, vec3 n, float etai_over_etat)
{
	float theta = fmin(dot(-uv, n),1.0);
	vec3 r_out_perp = etai_over_etat * (uv + theta * n);
	vec3 r_out_parallel = -sqrt(fabs(1.0 - r_out_perp.length_squared())) * n;
	return r_out_perp + r_out_parallel;
}
float reflectance(float cosine, float ref_idx)
{
	float r0 = (1 - ref_idx) / (1 + ref_idx);
	r0 = r0 * r0;
	return r0 + (1 - r0) * pow((1 - cosine), 5);
}

u8* buffer = nullptr;
void write_out(vec3 color, int x, int y) {
	assert(x < WIDTH&& y < HEIGHT);
	float scale = 1.0 / SAMPLES_PER_PIXEL;

	color *= scale;
	color.x = pow(color.x, 1 / GAMMA);
	color.y = pow(color.y, 1 / GAMMA);
	color.z = pow(color.z, 1 / GAMMA);



	int idx = y * WIDTH * 3 + x * 3;
	buffer[idx] = clamp(color.x,0,1) * 255;
	buffer[idx+1] = clamp(color.y,0,1)* 255;
	buffer[idx+2] = clamp(color.z,0,1)* 255;
}


struct Ray
{
	Ray() : pos(vec3(0)), dir(vec3(1,0,0)) {}
	Ray(vec3 pos, vec3 dir) :pos(pos), dir(dir) {}

	vec3 at(float t) const {
		return pos + dir * t;
	}

	vec3 pos;
	vec3 dir;
};
struct Material;
struct Trace
{
	const Material* mat_ptr = nullptr;

	vec3 point;
	vec3 normal;
	float t;
	bool front_face;

	void set_face_normal(Ray r, vec3 outward) {
		front_face = dot(r.dir, outward) < 0;
		normal = (front_face) ? outward : -outward;
	}
};

enum MaterialType
{
	LAMBERTIAN,
	METAL,
	DIELECTRIC,
};
struct Material
{
	Material() {
		type = LAMBERTIAN;
		lambertian.albedo = vec3(0.5);
	}

	static Material make_lambertian(vec3 albedo) {
		Material m;
		m.type = LAMBERTIAN;
		m.lambertian.albedo = albedo;
		return m;
	}
	static Material make_metal(vec3 albedo, float fuzz = 0.f) {
		Material m;
		m.type = METAL;
		m.metal.albedo = albedo;
		m.metal.fuzz = fuzz;
		return m;
	}
	static Material make_dielectric(float index) {
		Material m;
		m.type = DIELECTRIC;
		m.dielectric.index_r = index;
		return m;
	}

	MaterialType type;
	union {
		struct {
			vec3 albedo;
		}lambertian;
		struct {
			vec3 albedo;
			float fuzz;
		}metal;
		struct {
			float index_r;
		}dielectric;
	};

	bool scatter(Ray& ray_in, Trace& res, vec3& atteunuation, Ray& scattered_ray) const {
		switch (type)
		{
		case LAMBERTIAN:
		{
			vec3 scatter_dir = res.normal + random_unit_vector();

			if (scatter_dir.near_zero())
				scatter_dir = res.normal;

			scattered_ray = Ray(res.point + res.normal * 0.001f, normalize(scatter_dir));
			atteunuation = lambertian.albedo;
			return true;
		}break;
		case METAL:
		{
			vec3 reflected = reflect(ray_in.dir, res.normal);
			scattered_ray = Ray(res.point + res.normal * 0.001f, normalize(reflected + metal.fuzz*random_in_unit_sphere()));
			atteunuation = metal.albedo;
			return (dot(scattered_ray.dir, res.normal) > 0);
		}break;

		case DIELECTRIC:
		{
			atteunuation = vec3(1);
			float refrac_ratio = (res.front_face) ? (1.0 / dielectric.index_r) : dielectric.index_r;

			float cos_theta = fmin(dot(-ray_in.dir, res.normal), 1.0);
			float sin_theta = sqrt(1.f - cos_theta * cos_theta);

			bool cant_refract = refrac_ratio * sin_theta > 1.f;
			vec3 direction;
			if (cant_refract)
				direction = reflect(ray_in.dir, res.normal);
			else
				direction = refract(ray_in.dir, res.normal, refrac_ratio);

			//vec3 refracted = refract(ray_in.dir, res.normal, refrac_ratio);
			scattered_ray = Ray(res.point + res.normal * ((cant_refract) ? 0.001f:-0.001f), normalize(direction));
			return true;

		}break;

		default:
			return false;
		}
	}

};


struct Sphere
{
	Sphere() {}
	Sphere(vec3 center, float radius) : center(center), radius(radius) {}
	Sphere(vec3 center, float radius, Material mat) : center(center), radius(radius), mat(mat) {}

	bool hit(Ray r, float tmin, float tmax, Trace* res)const {
		vec3 oc = r.pos - center;
		float a = dot(r.dir, r.dir);
		float halfb = dot(oc, r.dir);
		float c = dot(oc, oc) - radius * radius;
		float discriminant = halfb * halfb - a * c;
		if (discriminant < 0)
			return false;
		float sqrt_dist = sqrt(discriminant);
		float root = (-halfb - sqrt_dist) / a;
		if (root<tmin || root>tmax) {
			root = (-halfb + sqrt_dist) / a;
			if (root<tmin || root>tmax)
				return false;
		}
		res->t = root;
		res->point = r.at(root);
		res->set_face_normal(r, (res->point - center) / radius);
		res->mat_ptr = &mat;
		return true;
	}


	vec3 center;
	float radius;

	Material mat;
};

struct Scene
{
	bool trace_scene(Ray r,float tmin,float tmax, Trace* res) {
		Trace temp;
		bool hit = false;
		float closest_so_far = tmax;
		for (const auto& obj : spheres) {
			if (obj.hit(r, tmin, closest_so_far, &temp)) {
				hit = true;
				closest_so_far = temp.t;
				*res = temp;
			}
		}
		return hit;
	}
	std::vector<Sphere> spheres;
};

struct Camera
{
	Camera() {
		origin = CAM_POS;
		horizontal = vec3(VIEW_WIDTH, 0, 0);
		vertical = vec3(0, VIEW_HEIGHT, 0);
		lower_left = CAM_POS - horizontal / 2 - vertical / 2 - vec3(0, 0, NEAR);
	}
	Ray get_ray(float u, float v) {
		return Ray(origin, normalize(lower_left + u * horizontal + v * vertical - origin));
	}
	Camera(vec3 cam_origin, vec3 look_pos, vec3 up, float yfov, float aspect_ratio)
	{

		float theta = radians(yfov);
		float h = tan(theta / 2);
		float view_height = h * 2;
		float view_width = view_height * aspect_ratio;

		vec3 w = normalize(cam_origin-look_pos);
		vec3 u = normalize(cross(up, w));
		vec3 v = normalize(cross(w, u));

		origin = cam_origin;
		horizontal = view_width * u;
		vertical = view_height * v;
		front = w;

		lower_left = origin - horizontal / 2 - vertical / 2 - front;
	}

	void look_dir(vec3 cam_origin, vec3 front_dir, vec3 up, float yfov, float aspect_ratio)
	{
		float theta = radians(yfov);
		float h = tan(theta / 2);
		float view_height = h * 2;
		float view_width = view_height * aspect_ratio;

		vec3 w = -front_dir;
		vec3 u = normalize(cross(up, w));
		vec3 v = normalize(cross(w, u));

		origin = cam_origin;
		horizontal = view_width * u;
		vertical = view_height * v;
		front = w;

		lower_left = origin - horizontal / 2 - vertical / 2 - front;
	}

	vec3 origin;
	vec3 front;
	vec3 up;


	vec3 horizontal;
	vec3 vertical;
	vec3 lower_left;
};

vec3 ray_color(Ray r, Scene& world, int depth)
{
	Trace trace;
	
	if (depth <= 0)
		return vec3(0);
	
	if (world.trace_scene(r, 0, 1000, &trace)) {
		Ray scattered_ray;
		vec3 attenuation;
		/*
		vec3 target = trace.point + trace.normal + random_unit_vector();
		return 0.5 * ray_color(Ray(trace.point+trace.normal*0.001f, normalize(target - trace.point)), world, depth - 1);
		*/
		if (trace.mat_ptr->scatter(r, trace, attenuation, scattered_ray))
			return attenuation * ray_color(scattered_ray, world, depth - 1);
		return vec3(0);

	}
	float t = 0.5 * (r.dir.y + 1.0);
	return (1.0 - t) * vec3(1) + t * vec3(0.5, 0.7, 1.0);
}

int main()
{
	srand(time(NULL));

	buffer = new u8[WIDTH * HEIGHT * 3];
	
	Scene world;
	//Camera cam;
	Camera cam (CAM_POS, CAM_POS+vec3(0,0,-1), vec3(0, 1, 0), 45, ARATIO);
	cam.look_dir(vec3(0,0,0.3),vec3(0,0,-1), vec3(0, 1, 0), 45, ARATIO);

	world.spheres.push_back(Sphere(vec3(0.15, -0.3, -0.55), 0.18, Material::make_metal(vec3(1))));


	world.spheres.push_back(Sphere( vec3(0, 0, -1), 0.5, Material::make_metal(vec3(0.2,0.2,1.0)) ));
	world.spheres.push_back(Sphere( vec3(1, -0.3, -1), 0.25, Material::make_lambertian(vec3(0.8,0.2,0.2)) ));
	world.spheres.push_back(Sphere(vec3(-0.4, -0.25, -0.5), 0.25, Material::make_dielectric(1.5)));
	world.spheres.push_back(Sphere(vec3(0, -100.5, -1), 100));

	printf("Starting: ");
	float complete = 0.0;
	for (int y = 0; y < HEIGHT; y++) {
		float percent = float(y) / (HEIGHT - 1);
		if (percent >= complete) {
			printf("%d...", int(round(percent * 10)));
			complete += 0.1;
		}
		for (int x = 0; x < WIDTH; x++) {
			vec3 total = vec3(0.0);
			for (int s = 0; s < SAMPLES_PER_PIXEL; s++) {
				float u = float(x+random_float()) / (WIDTH - 1);
				float v = float(y+random_float()) / (HEIGHT - 1);
				total += ray_color(cam.get_ray(u, v), world,MAX_DEPTH);
			}
			write_out(total, x, (HEIGHT-1)-y);
		}
	}
	printf("DONE\n");

	stbi_write_bmp(OUTPUT_NAME, WIDTH, HEIGHT,3, buffer);
	printf("Output written: %s\n", OUTPUT_NAME);

	delete[] buffer;
	return 0;
}
