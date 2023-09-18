#include "Scene.h"
#include "BVH.h"
#include "tiny_gltf.h"


void Scene::build_top_level_bvh()
{
	// Get object bounds
	std::vector<Bounds> bounds;
	for (int i = 0; i < instances.size(); i++)
		bounds.push_back(instances[i].get_world_bounds());

	tlas = BVH::build(bounds, 1, BVH_SAH);

}

bool Scene::closest_hit(Ray r, Intersection* res) const
{
	const BVHNode* stack[64];
	stack[0] = &tlas.nodes[0];

	int stack_count = 1;
	const BVHNode* node = nullptr;

	float t = INFINITY;

	while (stack_count > 0)
	{
		node = stack[--stack_count];

		if (node->count != BVH_BRANCH)
		{
			int index_count = node->count;
			int index_start = node->left_node;
			for (int i = 0; i < index_count; i++) {
				const Instance& obj = instances[tlas.indicies[index_start + i]];
				bool hit = obj.intersect(r, 0, t, res);

				if (!hit)
					continue;
				t = res->t;
				res->index = tlas.indicies[index_start + i];
			}
			continue;
		}

		float left_dist, right_dist;
		bool left_aabb = tlas.nodes[node->left_node].aabb.intersect(r, left_dist);
		bool right_aabb = tlas.nodes[node->left_node + 1].aabb.intersect(r, right_dist);
		left_aabb = left_aabb && left_dist < t;
		right_aabb = right_aabb && right_dist < t;

		if (left_aabb && right_aabb) {
			if (left_dist < right_dist) {
				stack[stack_count++] = &tlas.nodes[node->left_node + 1];
				stack[stack_count++] = &tlas.nodes[node->left_node];
			}
			else {
				stack[stack_count++] = &tlas.nodes[node->left_node];
				stack[stack_count++] = &tlas.nodes[node->left_node + 1];
			}
		}
		else if (left_aabb) {
			stack[stack_count++] = &tlas.nodes[node->left_node];
		}
		else if (right_aabb) {
			stack[stack_count++] = &tlas.nodes[node->left_node + 1];
		}
	}

	if (!std::isfinite(t)) {
		return false;
	}

	res->w0 = -r.dir;
	return true;
}




void Scene::load_gltf_models(Mesh& model, const tinygltf::Model& inputMod, const tinygltf::Mesh& mesh, const mat4& toWorld)
{
	mat4 toWorldRotation = transpose(inverse(toWorld));
	for (int i = 0; i < mesh.primitives.size(); i++)
	{
		const int vertexStart = model.verts.size();

		const float* bufferPos = nullptr;
		const float* bufferNormal = nullptr;
		const float* bufferUv = nullptr;
		const float* bufferColor = nullptr;
		int posByteStride = 0;
		int normByteStride = 0;
		int uvByteStride = 0;
		int colorByteStride = 0;


		const tinygltf::Primitive& prim = mesh.primitives[i];
		assert(prim.indices != -1);

		auto findBufferInfo = [&](const char* name, const float*& attribptr, int& attribStride) -> int {
			if (prim.attributes.find(name) != prim.attributes.end()) {
				const tinygltf::Accessor& accessor = inputMod.accessors[prim.attributes.find(name)->second];
				const tinygltf::BufferView& bufferView = inputMod.bufferViews[accessor.bufferView];
				attribptr = reinterpret_cast<const float*>(&(inputMod.buffers[bufferView.buffer].data[accessor.byteOffset + bufferView.byteOffset]));
				attribStride = accessor.ByteStride(bufferView) ? (accessor.ByteStride(bufferView) / sizeof(float)) : tinygltf::GetNumComponentsInType(TINYGLTF_TYPE_VEC3);
				return accessor.count;
			}
			else {
				return -1;
			}
		};

		const int numVerticies = findBufferInfo("POSITION", bufferPos, posByteStride);
		findBufferInfo("NORMAL", bufferNormal, normByteStride);
		findBufferInfo("TEXCOORD_0", bufferUv, uvByteStride);

		const int primitiveVertexStart = model.verts.size();
		model.verts.resize(model.verts.size() + numVerticies);
		for (int v = 0; v < numVerticies; v++)
		{
			Vertex& vertex = model.verts[primitiveVertexStart + v];
			vertex.position = vec3(bufferPos[v * posByteStride], bufferPos[v * posByteStride + 1], bufferPos[v * posByteStride + 2]);
			vertex.position = (toWorld * vec4(vertex.position,1.0)).xyz();

			vertex.normal = vec3(bufferNormal[v * normByteStride], bufferNormal[v * normByteStride + 1], bufferNormal[v * normByteStride + 2]);
			vertex.normal = (toWorld * vec4(vertex.normal, 0.0)).xyz();
			if (bufferUv)
				vertex.uv = vec2(bufferUv[v * uvByteStride], bufferUv[v * uvByteStride + 1]);
		}

		const tinygltf::Accessor& accessor = inputMod.accessors[prim.indices];
		const tinygltf::BufferView& bufferView = inputMod.bufferViews[accessor.bufferView];
		const tinygltf::Buffer& buffer = inputMod.buffers[bufferView.buffer];

		const int primitiveIndexCount = static_cast<uint32_t>(accessor.count);
		const int triangleCount = primitiveIndexCount / 3;
		const void* dataPtr = &(buffer.data[accessor.byteOffset + bufferView.byteOffset]);

		const int primTriangleOffset = model.triangles.size();

		model.triangles.resize(model.triangles.size() + triangleCount);
		switch (accessor.componentType) {
		case TINYGLTF_PARAMETER_TYPE_UNSIGNED_INT: {
			const uint32_t* buf = static_cast<const uint32_t*>(dataPtr);
			for (size_t index = 0; index < triangleCount; index++) {
				auto& tri = model.triangles[primTriangleOffset + index];
				tri.indicies[0] = buf[index * 3] + primitiveVertexStart;
				tri.indicies[1] = buf[index * 3 + 1] + primitiveVertexStart;
				tri.indicies[2] = buf[index * 3 + 2] + primitiveVertexStart;
				tri.materialIndex = prim.material;
			}
			break;
		}
		case TINYGLTF_PARAMETER_TYPE_UNSIGNED_SHORT: {
			const uint16_t* buf = static_cast<const uint16_t*>(dataPtr);
			for (size_t index = 0; index < triangleCount; index++) {
				auto& tri = model.triangles[primTriangleOffset + index];
				tri.indicies[0] = buf[index * 3] + primitiveVertexStart;
				tri.indicies[1] = buf[index * 3 + 1] + primitiveVertexStart;
				tri.indicies[2] = buf[index * 3 + 2] + primitiveVertexStart;
				tri.materialIndex = prim.material;
			}
			break;
		}
		default:
			assert(0);
			break;
		}
	}
}

Material* Scene::load_gltf_materials()

void Scene::traverse_gltf_node_heirarchy(const tinygltf::Model& model, const tinygltf::Node& node, mat4 transform)
{
	mat4 nodeTransform=mat4(1);
	for (int i = 0; i < node.matrix.size()&&i<16; i++) {
		nodeTransform.m[i] = node.matrix[i];
	}
	transform = transform * nodeTransform;

	if (node.mesh != -1)
		load_gltf_models(sceneMergedMesh, model, model.meshes.at(node.mesh), transform);

	if (node.camera != -1 && !hasCamera) {
		sceneCamera = Camera(transform, 75.f, 250, 250);
		// TODO:
	}

	for (int i = 0; i < node.children.size(); i++)
		traverse_gltf_node_heirarchy(model, model.nodes.at(node.children.at(i)));
}

bool Scene::load_from_gltf(const std::string& gltfPath)
{
	tinygltf::Model scene;
	tinygltf::TinyGLTF loader;
	std::string errStr;
	std::string warnStr;
	bool err = loader.LoadBinaryFromFile(&scene, &errStr, &warnStr, gltfPath);
	if (err) {
		printf("Couldn't load gltf model: %s\n", errStr.c_str());
		return false;
	}

	for (int i = 0; i < scene.materials.size(); i++) {
		materials.push_back(load_gltf_materials(scene.materials[i]));
	}

	traverse_gltf_node_heirarchy(scene, scene.nodes.at(0),mat4(1));

}