#include <iostream>
#include <vector>
#include <random>
#include "Math.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

typedef unsigned char u8;
typedef unsigned int u32;
const int WIDTH = 400;
const int HEIGHT = 300;
const float ARATIO = WIDTH / (float) HEIGHT;

const float VIEW_HEIGHT = 2.0;
const float VIEW_WIDTH = ARATIO * VIEW_HEIGHT;
const float NEAR = 1.0;

const vec3 CAM_POS = vec3(0.0,-0.1,0.3);

const float PI = 3.14159;
const int SAMPLES_PER_PIXEL = 50;
const int MAX_DEPTH = 50;
const float GAMMA = 2.2;


const char* OUTPUT_NAME[2] = { "Output.bmp","Output2.bmp" };

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

vec3 random_in_unit_disk()
{
	while (1) {
		vec3 p = vec3(random_float(-1, 1), random_float(-1, 1), 0);
		if (p.length_squared() >= 1) continue;
		return p;
	}
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
vec3 pow(vec3 V, float e)
{
	return vec3(pow(V.x, e), pow(V.y, e), pow(V.z, e));
}
vec3 abs(vec3 V)
{
	return vec3(fabs(V.x), fabs(V.y), fabs(V.z));
}

u8* buffer[2] = { nullptr,nullptr };
void write_out(vec3 color, int x, int y, int samples, int file = 0) {
	assert(x < WIDTH&& y < HEIGHT);
	float scale = 1.0 / samples;

	color *= scale;
	color.x = pow(color.x, 1 / GAMMA);
	color.y = pow(color.y, 1 / GAMMA);
	color.z = pow(color.z, 1 / GAMMA);



	int idx = y * WIDTH * 3 + x * 3;
	buffer[file][idx] = clamp(color.x,0,1) * 255;
	buffer[file][idx+1] = clamp(color.y,0,1)* 255;
	buffer[file][idx+2] = clamp(color.z,0,1)* 255;
}
void write_out_no_scale(vec3 color, int x, int y, int file = 1)
{
	assert(x < WIDTH&& y < HEIGHT);
	int idx = y * WIDTH * 3 + x * 3;
	buffer[file][idx] = clamp(color.x, 0, 1) * 255;
	buffer[file][idx + 1] = clamp(color.y, 0, 1) * 255;
	buffer[file][idx + 2] = clamp(color.z, 0, 1) * 255;
}

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
	EMIT,
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
	static Material make_light(vec3 emit_color) {
		Material m;
		m.type = EMIT;
		m.emitted = emit_color;
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
	vec3 emitted = vec3(0.0);

	vec3 get_emitted() const {
		return emitted;
	}


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

struct BVHNode
{

};


class BVH
{

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

		vec3 rd = lens_radius * random_in_unit_disk();
		vec3 offset = u_unit * rd.x + v_unit * rd.y;

		return Ray(origin + offset, normalize(lower_left + u * horizontal + v * vertical - origin - offset));
	}
	Camera(vec3 cam_origin, vec3 look_pos, vec3 up, float yfov, float aspect_ratio, float aperature, float focus_dist)
	{

		float theta = radians(yfov);
		float h = tan(theta / 2);
		float view_height = h * 2;
		float view_width = view_height * aspect_ratio;

		vec3 w = normalize(cam_origin-look_pos);
		vec3 u = normalize(cross(up, w));
		vec3 v = normalize(cross(w, u));

		origin = cam_origin;
		horizontal = view_width * u * focus_dist;
		vertical = view_height * v * focus_dist;
		front = w;

		lower_left = origin - horizontal / 2 - vertical / 2 - front*focus_dist;

		lens_radius = aperature / 2.f;
		u_unit = u;
		v_unit = v;
	}

	void look_dir(vec3 cam_origin, vec3 front_dir, vec3 up, float yfov, float aspect_ratio, float aperature, float focus_dist)
	{
		float theta = radians(yfov);
		float h = tan(theta / 2);
		float view_height = h * 2;
		float view_width = view_height * aspect_ratio;

		vec3 w = -front_dir;
		vec3 u = normalize(cross(up, w));
		vec3 v = normalize(cross(w, u));

		origin = cam_origin;
		horizontal = view_width * u * focus_dist;
		vertical = view_height * v * focus_dist;
		front = w;

		lower_left = origin - horizontal / 2 - vertical / 2 - front*(focus_dist);

		lens_radius = aperature / 2.f;
		u_unit = u;
		v_unit = v;
	}

	vec3 origin;
	vec3 front;
	vec3 up;


	vec3 horizontal;
	vec3 vertical;
	vec3 lower_left;

	vec3 u_unit;	// unit vectors for horizontal and vertical
	vec3 v_unit;

	float lens_radius;
};

vec3 ray_color(Ray r, Scene& world, int depth)
{
	Trace trace;
	
	if (depth <= 0)
		return vec3(0);
	
	if (world.trace_scene(r, 0, 1000, &trace)) {
		Ray scattered_ray;
		vec3 attenuation;
		vec3 emitted = trace.mat_ptr->get_emitted();
		/*
		vec3 target = trace.point + trace.normal + random_unit_vector();
		return 0.5 * ray_color(Ray(trace.point+trace.normal*0.001f, normalize(target - trace.point)), world, depth - 1);
		*/
		if (trace.mat_ptr->scatter(r, trace, attenuation, scattered_ray))
			return emitted + attenuation * ray_color(scattered_ray, world, depth - 1);
		return emitted;

	}
	float t = 0.5 * (r.dir.y + 1.0);
	return (1.0 - t) * vec3(1) + t * vec3(0.5, 0.7, 1.0);
}

void random_scene(Scene* world)
{
	world->spheres.push_back(Sphere(vec3(0, -1000, 0), 1000));
	for (int i = -11; i < 11; i++) {
		for (int j = -11; j < 11; j++) {
			float rand_mat = random_float();
			vec3 center = vec3(i + 0.9 * random_float(), 0.2, j + 0.9 * random_float());
			if ((center - vec3(4, 0.2, 0)).length() <= 0.9)
				continue;

			if (rand_mat < 0.8) {
				world->spheres.push_back(Sphere(center, 0.2, Material::make_lambertian(random_vec3(0, 1) * random_vec3(0, 1))));
			}
			else if (rand_mat < 0.95) {
				world->spheres.push_back(Sphere(center, 0.2, Material::make_metal(random_vec3(0.5, 1.0))));
			}
			else {
				world->spheres.push_back(Sphere(center, 0.2, Material::make_dielectric(1.5)));
			}
		}
	}

	world->spheres.push_back(Sphere(vec3(0, 1, 0), 1.0, Material::make_dielectric(1.5)));
	world->spheres.push_back(Sphere(vec3(-4, 1, 0), 1.0, Material::make_lambertian(vec3(0.4,0.2,0.1))));
	world->spheres.push_back(Sphere(vec3(4, 1, 0), 1.0, Material::make_metal(vec3(0.7,0.6,0.5))));
}

int main()
{
	srand(time(NULL));

	buffer[0] = new u8[WIDTH * HEIGHT * 3];
	buffer[1] = new u8[WIDTH * HEIGHT * 3];

	
	Scene world;
//	Camera cam;
	Camera cam (vec3(-1, 2.0, 3.0), vec3(0), vec3(0, 1, 0), 40, ARATIO,0.05,5.0);
	//cam.look_dir(vec3(0,0,1.0),vec3(0,0,-1), vec3(0, 1, 0), 45, ARATIO,2.0,3.0);
	
	world.spheres.push_back(Sphere(vec3(0.15, -0.3, -0.2), 0.18, Material::make_metal(vec3(1))));
	
	
	world.spheres.push_back(Sphere( vec3(0, 0, -1), 0.5, Material::make_metal(vec3(0.2,0.2,1.0)) ));
	world.spheres.push_back(Sphere( vec3(1, -0.3, -1), 0.25, Material::make_lambertian(vec3(0.8,0.2,0.2)) ));
	world.spheres.push_back(Sphere(vec3(-0.4, -0.25, -0.5), 0.25, Material::make_dielectric(1.5)));
	world.spheres.push_back(Sphere(vec3(0, -100.5, -1), 100));

	//Camera cam(vec3(12, 2, 3), vec3(0), vec3(0, 1, 0), 20, ARATIO,0.1,10.0);
	//random_scene(&world);

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
			
			// Variance calculation
			int count;	// count
			vec3 mean = vec3(0);
			float mean_dist2 = 0;
			float variance;

			vec3 delta;

			for (int s = 0; s < SAMPLES_PER_PIXEL; s++) {
				count = s + 1;

				float u = float(x+random_float()) / (WIDTH - 1);
				float v = float(y+random_float()) / (HEIGHT - 1);
				Ray r = cam.get_ray(u, v);
				vec3 sample = ray_color(r, world,MAX_DEPTH);
				total += sample;


				delta = sample - mean;
				mean += delta / (float)count;
				mean_dist2 += dot(delta, delta) * (s>0);
				variance = mean_dist2 / (count - 1);

			}
			write_out(total, x, (HEIGHT-1)-y, count);
			write_out_no_scale(vec3(variance), x, (HEIGHT - 1) - y);
		}
	}
	printf("DONE\n");

	stbi_write_bmp(OUTPUT_NAME[0], WIDTH, HEIGHT,3, buffer[0]);

	stbi_write_bmp(OUTPUT_NAME[1], WIDTH, HEIGHT, 3, buffer[1]);


	printf("Output written: %s\n", OUTPUT_NAME[0]);

	delete[] buffer[0];
	delete[] buffer[1];
	return 0;
}
