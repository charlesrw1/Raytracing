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

const vec3 CAM_POS = vec3(0.0);

const float PI = 3.14159;
const int SAMPLES_PER_PIXEL = 50;
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
struct Trace
{
	vec3 point;
	vec3 normal;
	float t;
	bool front_face;

	void set_face_normal(Ray r, vec3 outward) {
		front_face = dot(r.dir, outward) < 0;
		normal = (front_face) ? outward : -outward;
	}
};


struct Sphere
{
	Sphere() {}
	Sphere(vec3 center, float radius) : center(center), radius(radius) {}

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
		return true;
	}


	vec3 center;
	float radius;
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


	vec3 origin;
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
		vec3 target = trace.point + trace.normal + random_in_unit_sphere();
		return 0.5 * ray_color(Ray(trace.point+trace.normal*0.001f, normalize(target - trace.point)), world, depth - 1);
	}
	float t = 0.5 * (r.dir.y + 1.0);
	return (1.0 - t) * vec3(1) + t * vec3(0.5, 0.7, 1.0);
}

int main()
{
	buffer = new u8[WIDTH * HEIGHT * 3];
	
	Scene world;
	Camera cam;
	world.spheres.push_back(Sphere(vec3(0, 0, -1), 0.5));
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
