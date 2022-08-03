#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include <thread>
#include <mutex>

#include "Object.h"
#include "Material.h"
#include "Math.h"
#include "Utils.h"
#include "Def.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

typedef unsigned char u8;
typedef unsigned int u32;
const int WIDTH = 256;
const int HEIGHT = 256;
const float ARATIO = WIDTH / (float) HEIGHT;

const float VIEW_HEIGHT = 2.0;
const float VIEW_WIDTH = ARATIO * VIEW_HEIGHT;
const float NEAR = 1.0;

const vec3 CAM_POS = vec3(0.0,-0.1,0.3);

const int SAMPLES_PER_PIXEL = 50;
const int DIRECT_SAMPLES = 10;
const int MAX_DEPTH = 25;
const float GAMMA = 2.2;


const char* OUTPUT_NAME[2] = { "Output.bmp","Output2.bmp" };


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


static unsigned int NUM_RAYCASTS = 0;
struct Scene
{
	~Scene() {
		for (int i = 0; i < instances.size(); i++)
			instances[i].free_data();
	}

	bool trace_scene(Ray r,float tmin,float tmax, Intersection* res) const{
		//Trace temp;
		Intersection temp;
		bool hit = false;
		float closest_so_far = tmax;
		for (const auto& obj : instances) {
			//NUM_RAYCASTS++;
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

	std::vector<Instance> lights;

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
	Ray get_ray(float u, float v) const {

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

vec3 shade_direct(const Intersection& si, const Scene& world, const Ray& ray_in)
{
	vec3 direct = vec3(0);
	for (int i = 0; i < world.lights.size(); i++)
	{
		float sample = 0;
		const Instance* light = &world.lights[i];

		float area = light->get_area();

		for (int s = 0; s < DIRECT_SAMPLES; s++) {
			// Choose sample point across instance geometry
			vec3 point = vec3(0);
			vec3 normal = vec3(0);
			light->sample_geometry(si.point, point, normal);

			vec3 light_dir = point - si.point;
			float length = light_dir.length();
			light_dir = light_dir / length;

			Ray shadow_ray(si.point + si.normal * 0.0001, light_dir);
			Intersection light_inter;
			// Continue if sample is occluded
			if (world.trace_scene(shadow_ray, 0, length - 0.001, &light_inter))
				continue;

			// Compute PDF
			//float cosine = fabs(dot(light_dir, normal));
			float cos_incident = max(dot(si.normal, light_dir), 0.f);
			float cos_light = max(dot(-normal, light_dir),0.f);
			float distance_squared = length * length;
			float geometry_term = (cos_incident * cos_light * area) / distance_squared;

			sample += geometry_term * si.material->scattering_pdf(-light_dir,normal);
		}

		direct += sample * (1.f / DIRECT_SAMPLES) * light->get_material()->emitted();
	}

	return direct;
}

vec3 ray_color(const Ray r, const Scene& world, int depth)
{
	Intersection si;
	
	if (depth <= 0)
		return vec3(0);
	
	if (!world.trace_scene(r, 0, 1000, &si)) {
		return world.background_color;
	}

	Ray scattered_ray;
	vec3 attenuation;
	vec3 emitted = si.material->emitted();
	float pdf;
	/*
	vec3 target = trace.point + trace.normal + random_unit_vector();
	return 0.5 * ray_color(Ray(trace.point+trace.normal*0.001f, normalize(target - trace.point)), world, depth - 1);
	*/
	if (!si.material->scatter(&si, attenuation, scattered_ray,pdf))
		return emitted;
	
	vec3 direct_light = shade_direct(si, world,r);

	return emitted + attenuation  * direct_light;//ray_color(scattered_ray, world, depth - 1) / pdf;
	//return emitted + attenuation * si.material->scattering_pdf(r, si, scattered_ray) * direct_light / pdf;//ray_color(scattered_ray, world, depth - 1) / pdf;
	

	//float t = 0.5 * (r.dir.y + 1.0);
	//return (1.0 - t) * vec3(1) + t * vec3(0.5, 0.7, 1.0);
}
vec3 ray_color_no_R(Ray r, const Scene& world, int depth)
{
	Intersection si;

	Ray ray = r;

	vec3 sample_color = vec3(0.0);
	vec3 throughput(1.f);
	for (int bounce = 0; bounce < MAX_DEPTH; bounce++)
	{
		if (!world.trace_scene(ray, 0, 1000, &si)) {
			sample_color += throughput * world.background_color;
			break;
		}
		const Material* material = si.material;
		vec3 emitted = material->emitted();
		sample_color += throughput * emitted;
		vec3 attenuation;
		float pdf;
		if (!material->scatter(&si, attenuation, ray, pdf)) {
			break;
		}
		throughput = throughput * attenuation;

	}
	return sample_color;
}

void cornell_box_scene(Scene& world, Camera& cam)
{
	/*
	world.instances.push_back(Instance(
		new Sphere(0.15),
		new EmissiveMaterial(
			vec3(3)
		),
		vec3(0.5, 0.5,-0.5)));
		*/

	world.instances.push_back(Instance(

		//new Disk(vec3(0, -1, 0), 0.25),
		new Rectangle(vec3(0, -1, 0), 0.25,0.25),
		
		new EmissiveMaterial(
			vec3(17,12,4)
		),
		vec3(0.5, 0.99, -0.5)));

	world.lights.push_back(Instance(

		//new Disk(vec3(0, -1, 0), 0.25),
		new Rectangle(vec3(0, -1, 0), 0.25, 0.25),

		new EmissiveMaterial(
			vec3(17, 12, 4)
			//vec3(0.5)
		),
		vec3(0.5, 0.99, -0.5)));

	world.instances.push_back(Instance(
		new Rectangle(vec3(0, -1, 0), 1, 1),
		new MatteMaterial(
			new ConstantTexture(vec3(0.725, 0.71, 0.68))
		),
		vec3(0.5, 1, -0.5)));

	world.instances.push_back(Instance(
		new Rectangle(vec3(0, 1, 0), 1, 1),
		new MatteMaterial(
			new ConstantTexture(vec3(0.725, 0.71, 0.68))
		),
		vec3(0.5, 0, -0.5)));
	world.instances.push_back(Instance(
		new Rectangle(vec3(0, 0, 1), 1, 1),
		new MatteMaterial(
			new ConstantTexture(vec3(0.725, 0.71, 0.68))
			//new CheckeredTexture(vec3(0.1,0.1,0.8),vec3(1),0.1)
		),
		vec3(0.5, 0.5, -1)));

	world.instances.push_back(Instance(
		new Rectangle(vec3(1, 0, 0), 1, 1),
		new MatteMaterial(
			new ConstantTexture(vec3(0.63, 0.065, 0.05))
		),
		vec3(0, 0.5, -0.5)));
	world.instances.push_back(Instance(
		new Rectangle(vec3(-1, 0, 0), 1, 1),
		new MatteMaterial(
			new ConstantTexture(vec3(0.14, 0.45, 0.091))
		),
		vec3(1, 0.5, -0.5)));
	
	// Tall box
	mat4 transform = mat4(1.f);
	transform = rotate_y(transform, 25);
	//transform = rotate_y(transform, 0);
	transform = translate(transform, vec3(0.32, 0.3, -0.6));
	world.instances.push_back(Instance(
		new Box(vec3(0.3,0.6,0.3)),
		//new Cylinder(0.2,0.4),
	
		new MatteMaterial(
			new ConstantTexture(0.725, 0.71, 0.68)),
		//new MetalMaterial(vec3(1.0),0.5),
		transform
	
	));
	transform = mat4(1);
	transform = rotate_y(transform, -25);
	//transform = rotate_y(transform, 0);
	transform = translate(transform, vec3(0.68, 0.15, -0.35));
	world.instances.push_back(Instance(
		new Box(vec3(0.3, 0.3, 0.3)),
		//new Cylinder(0.2,0.4),
		//new GlassMaterial(2.0),
		new MatteMaterial(
			new ConstantTexture(0.725, 0.71, 0.68)),
		//new MetalMaterial(vec3(0.8),0.6),
		transform
	
	));
	
	/*
	mat4 transform = mat4(1);
	//transform = rotate_x(transform, 125);
	////transform = rotate_y(transform, 0);
	transform = translate(transform, vec3(0.5, 0.35, -0.5));
	world.instances.push_back(Instance(
		new Sphere(0.25),
		//new Cylinder(0.25,0.6),
		//new Box(vec3(0.5)),
		//new MetalMaterial(vec3(1.0,1.0,0.0),0),
		//new MatteMaterial(
		//	new ConstantTexture(0.4)),
		new GlassMaterial(2.0),
		transform
	
	));
	*/
	
	//world.instances.back().print_matricies();

	cam = Camera(vec3(0.5, 0.5, 1.5), vec3(0.5, 0.5, -0.5), vec3(0, 1, 0), 40, ARATIO, 0.0, 5.0);
}

void checker_scene(Scene& world, Camera& cam)
{
	world.instances.push_back(Instance(new Sphere(0.18), new MetalMaterial(vec3(1)), vec3(0.15, -0.3, -0.2)));
	
	world.instances.push_back(Instance(
		new Sphere(0.5), 
		//new Cylinder(0.25,0.5),
		new MetalMaterial(vec3(0.2, 0.2, 1.0)), 
		vec3(0, 0, -1)));

	world.instances.push_back(Instance(
		new Sphere(0.25),
		new MatteMaterial(
			new CheckeredTexture(vec3(0.8, 0.2, 0.2), vec3(1))),
		vec3(1, -0.3, -1)));

	world.instances.push_back(Instance(new Sphere(0.25), new GlassMaterial(1.5f), vec3(-0.4, -0.25, -0.5)));

	world.instances.push_back(Instance(
		new Sphere(100),
		new MatteMaterial(
			new CheckeredTexture(vec3(1), vec3(0))),
		vec3(0, -100.5, -1)));

	world.instances.push_back(Instance(
		new Box(vec3(2.0,0.3,0.5)),
		new EmissiveMaterial(vec3(2.0,0.6,0)),
		vec3(0, 0.5, 0.5)

	));
	mat4 transform = translate(mat4(1), vec3(-1.4, -0.25, -1.0));
	transform = rotate_x(transform, 90);
	world.instances.push_back(Instance(
		new Cylinder(0.25, 0.5),
		new MetalMaterial(vec3(1), 0),
		//new GlassMaterial(1.5f), 
		transform));

	world.background_color = vec3(0.5, 0.7, 1.0);
	cam = Camera(vec3(2.0, 1.0, -6.0), vec3(-1.4, -0.25, -1.7), vec3(0, 1, 0), 40, ARATIO, 0.01, 4.0);
}

struct ThreadState
{
	std::mutex mutex;
	int current_line;
	float complete = 0.0;

	const Camera* cam;
	const Scene* world;
};
// per-thread function
void calc_pixels(int thread_id, ThreadState* ts)
{
	srand(time(NULL) + thread_id);
	while (1)
	{
		int line;
		ts->mutex.lock();
		line = ts->current_line++;
		float percent = float(line) / (HEIGHT - 1);
		if (percent >= ts->complete) {
			printf("%d...", int(round(percent * 10)));
			ts->complete += 0.1;
		}
		ts->mutex.unlock();

		if (line >= HEIGHT)
			return;

		for (int x = 0; x < WIDTH; x++) {
			vec3 total = vec3(0.0);

			// Variance calculation
			int count;	// count
			//vec3 mean = vec3(0);
			//float mean_dist2 = 0;
			//float variance;

			//vec3 delta;

			for (int s = 0; s < SAMPLES_PER_PIXEL; s++) {
				count = s + 1;

				float u = float(x + random_float()) / (WIDTH - 1);
				float v = float(line + random_float()) / (HEIGHT - 1);
				Ray r = ts->cam->get_ray(u, v);
				vec3 sample = ray_color(r, *ts->world, MAX_DEPTH);
				total += sample;


				//delta = sample - mean;
				//mean += delta / (float)count;
				//mean_dist2 += dot(delta, delta) * (s > 0);
				//variance = mean_dist2 / (count - 1);

			}
			write_out(total, x, (HEIGHT - 1) - line, count);
			//write_out_no_scale(vec3(variance), x, (HEIGHT - 1) - line);
		}

	}
}
int main()
{
	srand(time(NULL));

	buffer[0] = new u8[WIDTH * HEIGHT * 3];
	buffer[1] = new u8[WIDTH * HEIGHT * 3];

	
	Scene world;
//	Camera cam;
	Camera cam (vec3(0.15,0.8, 1.5), vec3(0.5,0.5,0), vec3(0, 1, 0), 45, ARATIO,0.01,4.0);
	cornell_box_scene(world,cam);
	//checker_scene(world, cam);

	//Camera cam(vec3(12, 2, 3), vec3(0), vec3(0, 1, 0), 20, ARATIO,0.1,10.0);
	//random_scene(&world);

	// Initalize threads
	std::vector<std::thread> threads;
	threads.resize(std::thread::hardware_concurrency());


	ThreadState state;
	state.complete = 0.0;
	state.current_line = 0;
	state.cam = &cam;
	state.world = &world;

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	printf("Starting: ");
#ifdef SINGLETHREAD
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
#else
	for (int i = 0; i < threads.size(); i++) {
		threads[i] = std::thread(&calc_pixels, i, &state);
	}
	for (int i = 0; i < threads.size(); i++) {
		threads[i].join();
	}


#endif // SINGLETHREAD



	printf("DONE\n");
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::cout << "Time elapsed = " << std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() << " seconds" << std::endl;
	std::cout << "Num raycasts: " << NUM_RAYCASTS << '\n';
	stbi_write_bmp(OUTPUT_NAME[0], WIDTH, HEIGHT,3, buffer[0]);

	//stbi_write_bmp(OUTPUT_NAME[1], WIDTH, HEIGHT, 3, buffer[1]);


	printf("Output written: %s\n", OUTPUT_NAME[0]);

	delete[] buffer[0];
	delete[] buffer[1];
	return 0;
}
