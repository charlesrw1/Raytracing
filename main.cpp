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
#include "Scene.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

typedef unsigned char u8;
typedef unsigned int u32;
const int WIDTH = 512;
const int HEIGHT = 512;
const float ARATIO = WIDTH / (float) HEIGHT;

const float VIEW_HEIGHT = 2.0;
const float VIEW_WIDTH = ARATIO * VIEW_HEIGHT;
const float NEAR = 1.0;

const vec3 CAM_POS = vec3(0.0,-0.1,0.3);

const int SAMPLES_PER_PIXEL = 16;
const int DIRECT_SAMPLES = 1;
const int MAX_DEPTH = 20;
const float GAMMA = 2.2;

const char* OUTPUT_NAME[2] = { "Output.bmp","Output2.bmp" };


enum AbstractLightType
{
	POINT,
	SPOTLIGHT
};
struct AbstractLight
{
	AbstractLight(AbstractLightType type, vec3 position, vec3 color)
		: type(type),position(position),color(color) {}
	AbstractLightType type;
	vec3 position;
	vec3 color;
};


/*
vec3 shade_direct_abstract_NEE(const Intersection& si, const Scene& world, const Ray& ray_in)
{
	vec3 direct = vec3(0);
	for (int i = 0; i < world.abs_lights.size(); i++)
	{
		const AbstractLight* al = &world.abs_lights[i];
		vec3 light_dir = al->position - si.point;
		float length = light_dir.length();
		light_dir /= length;

		Ray shadow_ray(si.point + si.normal * 0.0001, light_dir);
		Intersection light_inter;
		if (world.trace_scene(shadow_ray, 0, length - 0.001, &light_inter))
			continue;

		float cos_light = 1.f;
		float distance_2 = length * length;
		float light_pdf = distance_2;	// pdf is really infinite because area = 0
		float pdf_unused;
		vec3 F = si.material->Eval(si, ray_in.dir, light_dir, si.normal, &pdf_unused);

		direct += al->color * F / light_pdf;
	}
	return direct;
}
*/
vec3 shade_direct_NEE(const Intersection& si, const Scene& world, const Ray& ray_in)
{
	vec3 direct = vec3(0);
	for (int i = 0; i < world.lights.size(); i++)
	{
		vec3 sample = 0;
		const Instance* light = &world.lights[i];

		float area = light->get_area();

		for (int s = 0; s < DIRECT_SAMPLES; s++) {
			// Choose random sample point across instance geometry
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

			//if (dot(light_dir, normal) >= 0)
			//	normal = -normal;

			// Compute PDF
			//float cosine = fabs(dot(light_dir, normal));
			float cos_incident = max(dot(si.normal, light_dir), 0.f);
			float cos_light = max(dot(-normal, light_dir),0.f);	// 'cosine'
			float distance_squared = length * length;
			//float geometry_term = (cos_light * area) / distance_squared;	// 1/light_pdf
			float denom = cos_light * area;
			if (fabs(denom) < 0.00001)
				continue;
			float light_pdf = distance_squared/denom;

			// Option A
			//float brdf_pdf = si.material->scattering_pdf(light_dir, si.normal);

			// Option B
			float brdf_pdf;

			vec3 f = si.material->Eval(si, ray_in.dir, light_dir, si.normal, &brdf_pdf);

			float mis= power_heuristic(light_pdf, brdf_pdf);
		//	mis = light_pdf / (light_pdf + brdf_pdf);	// balance heuristic
			sample += mis*f / light_pdf;// (1.f / light_pdf)* brdf_pdf;;	// (cos(point_normal,L)/PI) / (distance^2)/(area*cos(light_normal,L)
		}

		direct += sample * (1.f / DIRECT_SAMPLES) * light->get_material()->emitted();
	}

	//direct += shade_direct_abstract_NEE(si, world, ray_in);

	return direct;
}

//#define PDF_DEBUG
//#define DIRECT_ONLY_DEBUG

//#define NORMAL_DEBUG
vec3 get_ray_color(const Ray& cam_ray, const Scene& world, int max_depth)
{
	Intersection si,prev;
	float scatter_pdf=1.f;
	Ray r = cam_ray;

	vec3 radiance = vec3(0.0);
	vec3 throughput(1.f);
	int bounces;

	for (bounces=0; bounces < MAX_DEPTH; bounces++)
	{
		//bool intersection = world.trace_scene(r, 0, 1000, &si);
		bool intersection=world.closest_hit(r, &si);

		if (!intersection)
		{
			radiance += throughput * world.background_color;
			break;
		}
#ifdef NORMAL_DEBUG
		return pow((si.normal + 1) * 0.5, 2.2);
#endif
#ifdef DEPTH_DEBUG
		return pow(vec3(1 / (si.t + 1.0)), 2.2);
#endif


		
		const Material* material = si.material;
		vec3 emitted = material->emitted();
		// Gather radiance from direct
		float weight = 1.f;
		if (bounces > 0 && dot(emitted, vec3(1.0)) > 0.1) {
			float brdf_pdf = scatter_pdf;// prev.material->PDF(prev.w0, r.dir, prev.normal);
			float cos_light = max(dot(r.dir, -si.normal),0);
			float area = world.instances[si.index].get_area();
			float denom = area * cos_light;
			if (denom <= 0) {
				weight = 0;
			}
			else
			{
				// MIS
				float light_pdf = (prev.point-si.point).length_squared() / (denom);
				weight = power_heuristic(brdf_pdf, light_pdf);
				//weight = brdf_pdf / (brdf_pdf + light_pdf);

				//if (prev.material->is_microfacet())
				//{
				//	bool chance= random_float() < 0.0001;
				//	if (chance) {
				//		std::cout << "BRDF: " << brdf_pdf << "; LIGHT: " << light_pdf << '\n';
				//	}
				//}
			}
		}
		//weight = 1.f;


		// brdf_pdf already factored in throughput
		radiance += weight * emitted * throughput;
#ifdef DIRECT_HIT_DEBUG
		if (bounces > 0) {
			return weight * emitted * throughput;
		}
#endif // DIRECT_HIT_DEBUG
		float pdf;
		Ray next_ray;
		vec3 f = material->Sample_Eval(si, r.dir, si.normal, &next_ray, &pdf);
		if (abs(pdf)<0.000001) {
			break;
		}
#ifdef BRDF_DEBUG
		return pow(f, 2.2);
#endif // BRDF_DEBUG

#ifdef RAY_OUT_DEBUG
		return pow((r.dir + 1) * 0.5, 2.2);
#endif
#ifdef PDF_DEBUG
		return pow(1/(2*pdf), 2.2);
#endif
		vec3 direct = shade_direct_NEE(si, world, r);

#ifdef DIRECT_ONLY_DEBUG
		return direct;
#endif // DIRECT_ONLY_DEBUG


		radiance += throughput * direct;
		//throughput = throughput * (attenuation * si.material->scattering_pdf(r.dir,si.normal) / pdf);

		//radiance += throughput * attenuation * direct;
		throughput = throughput * f / pdf;// (attenuation * si.material->scattering_pdf(r.dir, si.normal) / pdf);

		r = next_ray;
		prev = si;
		scatter_pdf = pdf;
	}
	return radiance;
}
void outside_scene(Scene& world, Camera& cs)
{
	world.instances.push_back(Instance(
		new Rectangle(vec3(0, 1, 0), 2, 2),
		new MatteMaterial(
			new ConstantTexture(vec3(0.725, 0.71, 0.68))
		),
		vec3(0.5, 0, -0.5)));


	mat4 transform = scale(mat4(1), vec3(0.2));

	transform = translate(transform, vec3(0.5, 0.0, -0.5));
	world.instances.push_back(Instance(
		new TriangleMesh(import_mesh("bunny.obj")),
		//new Microfacet(nullptr, 0.3, 0),
		new MatteMaterial(
			new ConstantTexture(0.725, 0.71, 0.68)),
		transform
	));

	world.background_color = vec3(0.5, 0.7, 1.0);// pow(rgb_to_float(244, 215, 193), 2.2);
	cs = Camera(
		look_at(vec3(0.5, 0.5, 1.5), vec3(0.5, 0.5, -0.5), vec3(0, 1, 0)),
		40, WIDTH, HEIGHT);

};
void cornell_box_scene(Scene& world, Camera& cs)
{
	/*
	world.instances.push_back(Instance(
		new Cylinder(0.2,0.3),
		new MatteMaterial(
			new ConstantTexture(1)
		),
		vec3(0.5, 0.85,-0.5)));
		
	world.abs_lights.push_back(AbstractLight(POINT, vec3(0.5, 0.8, -0.5), vec3(2, 0.5, 0.5)));
	*/

	world.instances.push_back(Instance(

		//new Disk(vec3(0, -1, 0), 0.25),
		new Rectangle(vec3(0, -1, 0), 0.25, 0.25),

		
		new EmissiveMaterial(
			vec3(17,12,4)
		),
		//vec3(0.5, 0.75, 0)));

		vec3(0.5, 0.999, -0.5)));

	world.lights.push_back(Instance(

		//new Disk(vec3(0, -1, 0), 0.25),
		new Rectangle(vec3(0, -1, 0), 0.25, 0.25),
		//new Rectangle(vec3(0, 0, -1), 0.25, 0.25),

		new EmissiveMaterial(
			vec3(17, 12, 4) * 2
			//vec3(0.5)
		),
		vec3(0.5, 0.999, -0.5)));
		//vec3(0.5, 0.75, 0)));
		
#if 0
	world.instances.push_back(Instance(

		//new Disk(vec3(0, -1, 0), 0.25),
		new Rectangle(vec3(-1, 0, 0), 0.25, 0.25),


		new EmissiveMaterial(
			vec3(4, 12, 17)
		),
		//vec3(0.5, 0.75, 0)));

		vec3(0.999,0.4, -0.5)));

	world.lights.push_back(Instance(

		//new Disk(vec3(0, -1, 0), 0.25),
		new Rectangle(vec3(-1, 0, 0), 0.25, 0.25),


		new EmissiveMaterial(
			vec3(4, 12, 17)
		),
		//vec3(0.5, 0.75, 0)));

		vec3(0.999, 0.4, -0.5)));
#endif
		
		
	/*
	world.instances.push_back(Instance(

		//new Disk(vec3(0, -1, 0), 0.25),
		new Rectangle(vec3(-1, 0, 0), 0.25, 0.25),

		new EmissiveMaterial(
			vec3(4, 12, 17)
		),
		vec3(0.95, 0.5, -0.5)));

	world.lights.push_back(Instance(

		//new Disk(vec3(0, -1, 0), 0.25),
		new Rectangle(vec3(-1, 0, 0), 0.25, 0.25),

		new EmissiveMaterial(
			vec3(4, 12, 17)
		),
		vec3(0.95, 0.5, -0.5)));

		*/
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
		new Rectangle(normalize(vec3(0, 0, 1)), 1, 1),
		new MatteMaterial(
			//new ConstantTexture(vec3(0.725, 0.71, 0.68))
			new CheckeredTexture(vec3(0.1,0.1,0.8),vec3(1),0.1)
		),
		//new Microfacet(nullptr, 0.5, 0),
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
			//new ConstantTexture(pow(rgb_to_float(52, 134, 224), 2.2))
		),
		vec3(1, 0.5, -0.5)));
	// Tall box
	mat4 transform = mat4(1.f);
	
	/*
	transform = rotate_y(transform, 25);
	//transform = rotate_x(transform, 35);
	transform = translate(transform, vec3(0.32, 0.3, -0.6));
	//transform = look_at(vec3(0.32, 0.3, -0.6), vec3(0.32, 0.3, -0.6) + vec3(0.2, 0.2, 0.8), vec3(0, 1, 0));
	world.instances.push_back(Instance(
		new Box(vec3(0.3,0.6,0.3)),
		//new Cylinder(0.2,0.4),
		new Microfacet(nullptr, 0.05, 0),
		//new MatteMaterial(
		//	new ConstantTexture(0.725, 0.71, 0.68)),
		//new MetalMaterial(vec3(1.0),0.5),
		transform
	
	));
	
	transform = mat4(1);
	transform = rotate_y(transform, -25);
	//transform = rotate_y(transform, 0);
	transform = translate(transform, vec3(0.68, 0.15, -0.35));
	//transform = translate(transform, vec3(0.25, 0.40, -0.6));
	world.instances.push_back(Instance(
		//new Sphere(0.13),
		new Box(vec3(0.3, 0.3, 0.3)),
		//new Cylinder(0.2,0.4),
		//new GlassMaterial(2.0),
		new Microfacet(nullptr,0.7,0),
		//new MatteMaterial(
		//	new ConstantTexture(0.725, 0.71, 0.68)),
		//new MetalMaterial(vec3(0.8),0.6),
		transform
	
	));
	*/
	/*
	//transform =  scale(mat4(1), vec3(1,1.0,1.3));
	transform = translate(mat4(1), vec3(0.3, 0.25, -0.1));
	world.instances.push_back(Instance(
		new Sphere(0.11),
		//new Cylinder(0.2,0.4),
		new Microfacet(nullptr, 0.3, 0),
		//new MatteMaterial(
		//	new ConstantTexture(0.725, 0.71, 0.68)),
		//new MetalMaterial(vec3(1.0),0.5),
		transform

	));*/
	
	transform = scale(mat4(1),vec3(0.26));

	//transform = translate(transform, vec3(0.5, 0.1, -0.5));
	//world.instances.push_back(Instance(
	//	new TriangleMesh(import_mesh("bunny.obj")),
	//	new GlassMaterial(1.5),
	//	//new Microfacet(nullptr, 0.3, 0),
	//	//new MatteMaterial(
	//	//	new ConstantTexture(0.725, 0.71, 0.68)),
	//	transform
	//));

	transform = mat4(1);
	//transform = scale(mat4(1), vec3(0.17));
	//transform = rotate_y(transform, 15);
//	transform = rotate_x(transform, -20);
	transform = translate(transform, vec3(0.5, 0.4, -0.5));
	world.instances.push_back(Instance(
		new Sphere(0.2),
		//new TriangleMesh(import_mesh("bunny.obj")),
		//new Microfacet(nullptr, 0.1, 0),
		//new DisneyDiffuse(vec3(0.5),0.9,0.99),
		//new DisneyMetal(pow(rgb_to_float(255, 215, 0), 2.2),0.7,0.9),
		new DisneyClearcoat(0.9),
		//new DisneyGlass(vec3(1),0.3,0.5,1.5),
		//new GlassMaterial(2.0),
		//new MatteMaterial(
		//	new ConstantTexture(0.725, 0.71, 0.68)),
		transform
	));

	/*
	
	transform = translate(mat4(1), vec3(0.5, 0.40, -0.6));
	world.instances.push_back(Instance(
		new Sphere(0.1),
		//new Box(vec3(0.3, 0.3, 0.3)),
		//new Cylinder(0.2,0.4),
		//new GlassMaterial(2.0),
		new Microfacet(nullptr, 0.4, 0),
		//new MatteMaterial(
		//	new ConstantTexture(0.725, 0.71, 0.68)),
		//new MetalMaterial(vec3(0.8),0.6),
		transform

	));
	transform = translate(mat4(1), vec3(0.75, 0.40, -0.6));
	world.instances.push_back(Instance(
		new Sphere(0.1),
		//new Box(vec3(0.3, 0.3, 0.3)),
		//new Cylinder(0.2,0.4),
		//new GlassMaterial(2.0),
		new Microfacet(nullptr, 0.5, 0),
		//new MatteMaterial(
		//	new ConstantTexture(0.725, 0.71, 0.68)),
		//new MetalMaterial(vec3(0.8),0.6),
		transform

	));
	transform = translate(mat4(1), vec3(0.25, 0.15, -0.4));
	world.instances.push_back(Instance(
		new Sphere(0.1),
		//new Box(vec3(0.3, 0.3, 0.3)),
		//new Cylinder(0.2,0.4),
		//new GlassMaterial(2.0),
		new Microfacet(nullptr, 0.6, 0),
		//new MatteMaterial(
		//	new ConstantTexture(0.725, 0.71, 0.68)),
		//new MetalMaterial(vec3(0.8),0.6),
		transform

	));
	transform = translate(mat4(1), vec3(0.5, 0.15, -0.4));
	world.instances.push_back(Instance(
		new Sphere(0.1),
		//new Box(vec3(0.3, 0.3, 0.3)),
		//new Cylinder(0.2,0.4),
		//new GlassMaterial(2.0),
		new Microfacet(nullptr, 0.7, 0),
		//new MatteMaterial(
		//	new ConstantTexture(0.725, 0.71, 0.68)),
		//new MetalMaterial(vec3(0.8),0.6),
		transform

	));
	transform = translate(mat4(1), vec3(0.75, 0.15, -0.4));
	world.instances.push_back(Instance(
		new Sphere(0.1),
		//new Box(vec3(0.3, 0.3, 0.3)),
		//new Cylinder(0.2,0.4),
		//new GlassMaterial(2.0),
		new Microfacet(nullptr, 0.8, 0),
		//new MatteMaterial(
		//	new ConstantTexture(0.725, 0.71, 0.68)),
		//new MetalMaterial(vec3(0.8),0.6),
		transform

	));
	
	
	
	mat4 transform = mat4(1);
	transform = rotate_x(transform, 100);
	////transform = rotate_y(transform, 0);
	transform = translate(transform, vec3(0.5, 0.35, -0.4));
	world.instances.push_back(Instance(
		//new Sphere(0.1),
		new Cylinder(0.25,0.6),
		//new Box(vec3(0.5)),
		//new MetalMaterial(vec3(1.0,1.0,0.0),0),
		new MatteMaterial(
			new ConstantTexture(0.725, 0.71, 0.68)),
		//new GlassMaterial(2.0),
		transform
	
	));
	
	
	*/
	
	//world.instances.back().print_matricies();
	world.background_color = vec3(0);// pow(rgb_to_float(48, 45, 57), 2.2);
	cs = Camera(
		look_at(vec3(0.5, 0.5, 1.5), vec3(0.5, 0.5, -0.5), vec3(0, 1, 0)),
		40, WIDTH, HEIGHT);
}

void checker_scene(Scene& world)
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
}

struct ThreadState
{
	std::mutex mutex;
	int current_line;
	float complete = 0.0;

	const Camera* cs;
	const Scene* world;
};
// per-thread function
void calc_pixels(int thread_id, ThreadState* ts)
{
	srand(time(NULL) + thread_id);
	Random rng(thread_id);
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

				float u = float(x + random_float());// / (WIDTH - 1);
				float v = float(line + random_float());// / (HEIGHT - 1);
				vec2 coords = hammersley_2d(s, SAMPLES_PER_PIXEL);
				coords += vec2(x, line);

				Ray r = ts->cs->get_ray(coords.x, coords.y);
				vec3 sample = get_ray_color(r, *ts->world, MAX_DEPTH);

				if (sample.x != sample.x)
					sample = vec3(0);

				total += sample;


				//delta = sample - mean;
				//mean += delta / (float)count;
				//mean_dist2 += dot(delta, delta) * (s > 0);
				//variance = mean_dist2 / (count - 1);

			}
			//write_out(total, x, (HEIGHT - 1) - line, count);
			//write_out_no_scale(vec3(variance), x, (HEIGHT - 1) - line);
		}

	}
}

struct Options
{
	std::string output_name = "Output.bmp";

	int num_threads = 0;

	int width=WIDTH, height=HEIGHT;
	int samples_per_pixel=SAMPLES_PER_PIXEL;
	int max_depth=MAX_DEPTH;
	int direct_samples=DIRECT_SAMPLES;

	int tile_size=16;
	
	float gamma=GAMMA;
};

const int OUTPUT_DELTA = 5000;
class Renderer
{
public:
	void initalize(const Options& options);
	void render_image(const Camera& cam, const Scene& scene, const Options& options);




private:
	void output_image(const Options& options);

	void add_to_buffer(int x, int y, vec3 color) {
		float scale = 1.0 / samples;

		color *= scale;
		color.x = pow(color.x, 1 / GAMMA);
		color.y = pow(color.y, 1 / GAMMA);
		color.z = pow(color.z, 1 / GAMMA);

		int idx = y * width * 3 + x * 3;
		final_buffer[idx] = clamp(color.x, 0, 1) * 255;
		final_buffer[idx + 1] = clamp(color.y, 0, 1) * 255;
		final_buffer[idx + 2] = clamp(color.z, 0, 1) * 255;
	}

	void update_cli_completion_output(int increment) {
		std::lock_guard<std::mutex> lock(output_lock);
		completed_pixels += increment;

		if ((completed_pixels - last_printed_pixel) > OUTPUT_DELTA) {
			printf("Pixel: %*d / %d\r", 9, completed_pixels, width*height);

			last_printed_pixel = completed_pixels;
		}
	}

	int width,height;
	int max_depth;
	int samples;
	int direct_samples;


	std::mutex output_lock;
	int completed_pixels;
	int last_printed_pixel;

	float gamma;

	std::vector<u8> final_buffer;
};
class TileQueue
{
public:
	TileQueue(int width, int height, int extent) : width(width), height(height), extent(extent) {
		current_index = 0;
		tile_width = ((width + extent - 1) / extent);
		tile_height = ((height + extent - 1) / extent);
		max_index = tile_width * tile_height;

	}
	bool next_tile(int& x0, int& x1, int& y0, int& y1) {
		std::lock_guard<std::mutex> lock(mutex);
		if (current_index >= max_index)
			return false;

		vec2i tile(current_index % tile_width, current_index / tile_width);

		x0 = tile.x * extent;
		x1 = imin(x0 + extent, width);
		y0 = tile.y * extent;
		y1 = imin(y0 + extent, height);

		current_index ++;
		return true;
	}

	const int width, height, extent;
	int tile_width, tile_height;
	int current_index;
	int max_index;
	std::mutex mutex;
};
void Renderer::initalize(const Options& options)
{
	width = options.width;
	height = options.height;
	max_depth = options.max_depth;
	samples = options.samples_per_pixel;
	direct_samples = options.direct_samples;

	completed_pixels = 0;
	last_printed_pixel = 0;

	final_buffer.resize(width * height * 3);
}

void Renderer::render_image(const Camera& cam, const Scene& scene, const Options& options)
{
	initalize(options);
	
	std::vector<std::thread> threads;
	TileQueue tq(options.width, options.height, options.tile_size);

	double time_start = get_seconds();

	int hardware_threads = std::thread::hardware_concurrency();
	int used_threads = options.num_threads;
	if (used_threads == 0)
		used_threads = hardware_threads;
	else if (used_threads < 0)
		used_threads = imin(1, hardware_threads + used_threads);

	for (int i = 0; i < used_threads; i++) {
		threads.push_back(std::thread( [&](int thread_id) {
			Random random(thread_id);
			int x0, x1, y0, y1;
			int pixel_counter = 0;
			while (tq.next_tile(x0, x1, y0, y1)) {
				for (int x = x0; x < x1; x++) {
					for (int y = y0; y < y1; y++) {
						vec3 pixel_total = vec3(0);

						for (int sample = 0; sample < options.samples_per_pixel; sample++) {

							vec2 coords = hammersley_2d(sample, options.samples_per_pixel);
							coords += vec2(x, y);
							Ray r = cam.get_ray(coords.x, coords.y);

							vec3 s = get_ray_color(r, scene, options.max_depth);

							pixel_total += s;
						}

						add_to_buffer(x, height-y-1, pixel_total);
					}
				}
				
				pixel_counter += (x1 - x0) * (y1 - y0);

				if (pixel_counter > 1000) {
					update_cli_completion_output(pixel_counter);
					pixel_counter = 0;
				}
			}
		}, i)
		);
	}
	for (auto& t : threads)
		t.join();

	// Just get it displayed
	completed_pixels = width * height;
	last_printed_pixel = 0;
	update_cli_completion_output(0);

	double time_end = get_seconds();

	std::cout << "\nDONE, seconds elapsed: " << time_end - time_start << '\n';


	stbi_write_bmp(options.output_name.c_str(), width, height, 3, final_buffer.data());
	printf("Output written: %s\n", options.output_name.c_str());

}

int main()
{

	srand(time(NULL));

	Scene world;
	Camera cam;
	cornell_box_scene(world,cam);
	world.build_top_level_bvh();
	Options options;
	
	Renderer render;
	render.render_image(cam, world, options);
	
	return 0;
}
