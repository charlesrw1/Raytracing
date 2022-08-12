#ifndef SCENE_H
#define SCENE_H

#include "Def.h"
#include "Object.h"
#include <vector>


class Camera
{
public:
	Camera() {}
	Camera(
		const mat4& view_matrix,
		float fov,
		int width,
		int height
	)
	{
		mat4 raster_to_screen = mat4(
			2.f / width, 0, 0, 0,
			0, 2.f / height, 0, 0,
			0, 0, 1, 0,
			-1, -1, 1, 1
		);
		float h = tan(radians(fov / 2.f));
		float aspect = (float)width / height;
		mat4 screen_to_camera = mat4(
			h * aspect, 0, 0, 0,
			0, h, 0, 0,
			0, 0, -1, 0,
			0, 0, 0, 1
		);

		raster_to_world = view_matrix * screen_to_camera * raster_to_screen;
		camera_pos = view_matrix[3].xyz();
	}
	Ray get_ray(float raster_x, float raster_y) const {
		vec3 pos = (raster_to_world * vec4(raster_x, raster_y, 0, 1)).xyz();

		return Ray(camera_pos, normalize(pos - camera_pos));
	}


	vec3 camera_pos;
	mat4 raster_to_world;
};
struct SunAndSky
{
	vec3 horizon_sky;
	vec3 zenith_sky;

	vec3 eval_sky(const vec3& out_dir) {
		return lerp(horizon_sky, zenith_sky, sqrt(out_dir.y));
	}

	vec3 sun_color;
	float sun_zenith;
	float sun_azimuth;
};

class Scene
{
public:
	~Scene() {
		for (int i = 0; i < instances.size(); i++)
			instances[i].free_data();
	}

	bool trace_scene(Ray r, float tmin, float tmax, Intersection* res) const {
		//Trace temp;
		Intersection temp;
		bool hit = false;
		float closest_so_far = tmax;
		for (int i = 0; i < instances.size(); i++) {
			const Instance* obj = &instances[i];
			if (obj->intersect(r, tmin, closest_so_far, &temp)) {
				hit = true;
				closest_so_far = temp.t;
				*res = temp;
				res->index = i;
			}
		}
		res->w0 = -r.dir;
		return hit;
	}

	void build_top_level_bvh();

	bool any_hit(Ray r) const;
	bool closest_hit(Ray r, Intersection* res) const;


	BVH tlas;
	std::vector<Instance> instances;

	std::vector<Instance> lights;	// analytic
	//std::vector<AbstractLight> abs_lights;

	vec3 background_color = vec3(0);

};

#endif // !SCENE_H
