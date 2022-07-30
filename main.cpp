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
const int SAMPLES_PER_PIXEL = 100;
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
class Material;
struct SurfaceInteraction
{
	SurfaceInteraction() {}
	SurfaceInteraction(vec3 point, Ray r, vec3 outward_normal, const Material* material) 
		: point(point), w0(-r.dir), material(material)
	{
		front_face = dot(r.dir, outward_normal) < 0;
		normal = (front_face) ? outward_normal : -outward_normal;
	}
	vec3 point;
	vec3 normal;
	vec3 w0;		// -(incoming ray dir)
	float t;
	bool front_face;

	float u=0, v=0;	// texture coordinates

	 const Material* material = nullptr;

	 void set_face_normal(Ray r, vec3 outward) {
		 front_face = dot(r.dir, outward) < 0;
		 normal = (front_face) ? outward : -outward;
	 }
};

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
	CheckeredTexture(vec3 even, vec3 odd, float grid = 0.25f) : even(even), odd(odd),repeat(grid),mod(grid*2.f) {}
	virtual vec3 sample(float u, float v, vec3 p) const override {
		float sines = sin(10 * p.x)* sin(10 * p.y)* sin(10 * p.z);
		//uint8_t res = (fmod(fmod(p.x, mod)+mod,mod) < repeat) ^ (fmod(fmod(p.y, mod)+mod,mod) < repeat) ^ (fmod(fmod(p.z, mod)+mod,mod) < repeat);
		if (sines < 0)
			return odd;
		return even;
	}

private:
	vec3 even, odd;
	float repeat = 1.f;
	float mod = 2.f;
};

class Material
{
public:
	virtual ~Material() {}
	virtual bool scatter(const SurfaceInteraction* SI, vec3& attenuation, Ray& scattered) const {
		return false;
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
	virtual bool scatter(const SurfaceInteraction* SI, vec3& attenuation, Ray& scattered) const override
	{
		vec3 scatter_dir = SI->normal + random_unit_vector();

		if (scatter_dir.near_zero())
			scatter_dir = SI->normal;

		scattered = Ray(SI->point + SI->normal * 0.001f, normalize(scatter_dir));
		attenuation = albedo->sample(SI->u,SI->v,SI->point);
		return true;
	}
private:
	Texture* albedo;
};

class MetalMaterial : public Material
{
public:
	MetalMaterial(vec3 albedo, float fuzz=0.f) : albedo(albedo), fuzz(fuzz) {}
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


class Geometry
{
public:
	virtual ~Geometry() {}
	//virtual Bounds object_bounds() const = 0;
	//virtual Bounds world_bounds() const;
	virtual bool intersect(Ray r, float tmin, float tmax, SurfaceInteraction* si) const = 0;
//	virtual float area() const = 0;

};

class Sphere : public Geometry
{
public:
	Sphere(float radius)
		: radius(radius) {}

	virtual bool intersect(Ray r, float tmin, float tmax, SurfaceInteraction* si) const override {
		vec3 center = vec3(0);
		
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
		si->t = root;
		si->point = r.at(root);
		si->set_face_normal(r, (si->point - center) / radius);
		return true;
	}

private:
	float radius;
};
class Rectangle : public Geometry
{
public:
	Rectangle(vec3 normal, vec3 tangent, float u_length, float v_length) {
		N = normal;
		T = tangent;
		B = normalize(cross(N, T));
		u = u_length;
		v = v_length;
	}
	Rectangle(vec3 normal, float u_length, float v_length) {
		N = normal;
		vec3 up = vec3(0, 1, 0);
		if (N.y < -0.999) up = vec3(1, 0, 0);
		else if (N.y > 0.999) up = vec3(0, 0, -1);
		T = normalize(cross(up, N));
		B = normalize(cross(N, T));
		u = u_length;
		v = v_length;
	}
	virtual bool intersect(Ray r, float tmin, float tmax, SurfaceInteraction* si) const override {
		// Intersect plane
		float denom = dot(N, r.dir);
		if (fabs(denom) < 0.01)	// parallel
			return false;
		float dist = dot(r.pos, N);
		float t = -dist / denom;
		
		if (t<tmin || t>tmax)
			return false;

		vec3 plane_intersect = r.pos + r.dir * t;
		// Check if its within normals
		float u_dist = dot(plane_intersect, T);
		if (u_dist < 0 || u_dist > u)
			return false;
		float v_dist = dot(plane_intersect, B);
		if (v_dist < 0 || v_dist > v)
			return false;

		// Ray intersects plane
		si->point = plane_intersect;
		si->set_face_normal(r, N);
		si->u = u_dist / u;
		si->v = v_dist / v;
		si->t = t;
		return true;
	}

private:
	vec3 N,T,B;// normal, tangent, bitangent
	float u, v;	// side lengths

};
class TriangleMesh : public Geometry
{
public:

private:
	// BVH for triangles
	// triangle list
};

class Instance
{
public:
	Instance(Geometry* geo, Material* mat, vec3 offset) : geometry(geo), material(mat), offset(offset) {}

	void free_data() {
		delete geometry;
		delete material;
	}

	bool intersect(Ray r, float tmin, float tmax, SurfaceInteraction* si) const {
		// Transform ray to model space
		r.pos = r.pos - offset;
		si->material = material;
		bool result = geometry->intersect(r, tmin, tmax, si);
		si->point += offset;
		return result;
	}
private:
	Geometry* geometry = nullptr;
	Material* material = nullptr;
	//mat4 object_to_world;
	//mat4 world_to_object;

	vec3 offset;	// temporary transform
};


struct Scene
{
	~Scene() {
		for (int i = 0; i < instances.size(); i++)
			instances[i].free_data();
	}

	bool trace_scene(Ray r,float tmin,float tmax, SurfaceInteraction* res) {
		//Trace temp;
		SurfaceInteraction temp;
		bool hit = false;
		float closest_so_far = tmax;
		for (const auto& obj : instances) {
			if (obj.intersect(r, tmin, closest_so_far, &temp)) {
				hit = true;
				closest_so_far = temp.t;
				*res = temp;
			}
		}
		res->w0 = -r.dir;
		return hit;
	}

	std::vector<Instance> instances;
	vec3 background_color = vec3(0);

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
	SurfaceInteraction si;
	
	if (depth <= 0)
		return vec3(0);
	
	if (world.trace_scene(r, 0, 1000, &si)) {
		Ray scattered_ray;
		vec3 attenuation;
		vec3 emitted = si.material->emitted();
		/*
		vec3 target = trace.point + trace.normal + random_unit_vector();
		return 0.5 * ray_color(Ray(trace.point+trace.normal*0.001f, normalize(target - trace.point)), world, depth - 1);
		*/
		if (si.material->scatter(&si, attenuation, scattered_ray))
			return emitted + attenuation * ray_color(scattered_ray, world, depth - 1);
		return emitted;

	}
	return world.background_color;

	//float t = 0.5 * (r.dir.y + 1.0);
	//return (1.0 - t) * vec3(1) + t * vec3(0.5, 0.7, 1.0);
}
/*
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
*/
void cornell_box_scene(Scene& world)
{
	world.instances.push_back(Instance(
		new Sphere(0.15),
		new EmissiveMaterial(
			vec3(2)
		),
		vec3(0.5, 0.5,-0.5)));
	world.instances.push_back(Instance(
		new Rectangle(vec3(0, -1, 0), 1, 1),
		new MatteMaterial(
			new ConstantTexture(vec3(0.8))
		),
		vec3(0, 1, 0)));

	world.instances.push_back(Instance(
		new Rectangle(vec3(0, 1, 0), 1, 1),
		new MatteMaterial(
			new ConstantTexture(vec3(0.8))
		),
		vec3(0, 0, 0)));
	world.instances.push_back(Instance(
		new Rectangle(vec3(0, 0, 1), 1, 1),
		new MatteMaterial(
			new ConstantTexture(vec3(0.8))
		),
		vec3(0, 0, -1)));

	world.instances.push_back(Instance(
		new Rectangle(vec3(1, 0, 0), 1, 1),
		new MatteMaterial(
			new ConstantTexture(vec3(0.9, 0, 0))
		),
		vec3(0, 0, 0)));
	world.instances.push_back(Instance(
		new Rectangle(vec3(-1, 0, 0), 1, 1),
		new MatteMaterial(
			new ConstantTexture(vec3(0, 0.9, 0))
		),
		vec3(1, 0, -1)));
}

int main()
{
	srand(time(NULL));

	buffer[0] = new u8[WIDTH * HEIGHT * 3];
	buffer[1] = new u8[WIDTH * HEIGHT * 3];

	
	Scene world;
//	Camera cam;
	Camera cam (vec3(0.5,0.5, 1.5), vec3(0.5,0.5,0), vec3(0, 1, 0), 45, ARATIO,0.01,4.0);
	//cam.look_dir(vec3(0,0,1.0),vec3(0,0,-1), vec3(0, 1, 0), 45, ARATIO,2.0,3.0);
	/*
	world.instances.push_back(Instance(
		new Rectangle(
			normalize(vec3(-0.8, 0.5, 0.4)), 1, 1),
		new MatteMaterial(
			new ConstantTexture(vec3(1, 1, 0))
		),
		vec3(0, 0, -2)
	));

	world.instances.push_back(Instance(new Sphere(0.18), new MetalMaterial(vec3(1)), vec3(0.15, -0.3, -0.2)));
	//world.spheres.push_back(Sphere(vec3(0.15, -0.3, -0.2), 0.18, Material::make_metal(vec3(1))));
	//
	//
	//world.spheres.push_back(Sphere( vec3(0, 0, -1), 0.5, Material::make_metal(vec3(0.2,0.2,1.0)) ));
	world.instances.push_back(Instance(new Sphere(0.5), new MetalMaterial(vec3(0.2, 0.2, 1.0)), vec3(0, 0, -1)));

	//world.spheres.push_back(Sphere( vec3(1, -0.3, -1), 0.25, Material::make_lambertian(vec3(0.8,0.2,0.2)) ));
	world.instances.push_back(Instance(
		new Sphere(0.25), 
		new MatteMaterial(
			new CheckeredTexture(vec3(0.8, 0.2, 0.2),vec3(1))), 
		vec3(1, -0.3, -1)));

	//world.spheres.push_back(Sphere(vec3(-0.4, -0.25, -0.5), 0.25, Material::make_dielectric(1.5)));
	world.instances.push_back(Instance(new Sphere(0.25), new GlassMaterial(1.5f), vec3(-0.4, -0.25, -0.5)));

	//world.spheres.push_back(Sphere(vec3(0, -100.5, -1), 100));
	world.instances.push_back(Instance(
		new Sphere(100), 
		new MatteMaterial(
			new CheckeredTexture(vec3(1),vec3(0))),
		vec3(0, -100.5, -1)));
		*/
	cornell_box_scene(world);

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
