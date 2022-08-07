#include "Mesh.h"
#include <fstream>
#include <map>
#include <sstream>

struct VertexKey
{
	VertexKey(int vert, int normal, int uv) :vert(vert), normal(normal), uv(uv) {}
	int vert, normal, uv;

	bool operator<(const VertexKey& rhs) const{
		if (vert != rhs.vert)
			return vert < rhs.vert;
		else if (normal != rhs.normal)
			return normal < rhs.normal;
		else
			return uv < rhs.uv;
	}
	bool operator==(const VertexKey& rhs) const{
		return vert == rhs.vert && normal == rhs.normal && uv == rhs.uv;
	}
};

Mesh* import_mesh(const char* obj_file)
{
	printf("Loading mesh: %s\n", obj_file);
	std::ifstream infile(obj_file);
	if (!infile) {
		printf("Couldn't open file: %s\n", obj_file);
		return nullptr;
	}

	std::string buffer;
	buffer.reserve(1024);

	std::vector<Vertex> verticies;
	std::map<VertexKey, int> key_to_vertex;

	std::vector<vec3> positions;
	std::vector<vec3> normals;
	std::vector<vec2> uvs;

	std::vector<int> indicies;

	while (infile)
	{
		infile >> buffer;
		if (infile.fail())
			break;
		if (buffer == "v") {
			vec3 position;
			infile >> position.x >> position.y >> position.z;
			positions.push_back(position);
		}
		else if (buffer == "vt") {
			vec2 uv;
			infile >> uv.x >> uv.y;
			uvs.push_back(uv);
		}
		else if (buffer == "vn") {
			vec3 n;
			infile >> n.x >> n.y >> n.z;
			normals.push_back(n);
		}
		else if (buffer[0] == '#') {
			std::getline(infile, buffer);
		}
		else if (buffer == "f") {
			int face_indicies[4];
			int face_count=0;
			bool gen_normals = false;
			for (int i = 0; i < 4; i++) {
				int pos=0, normal=0, uv=0;
				infile >> pos;
				if (infile.fail()) {
					infile.clear();
					break;
				}
				if (infile.peek() == '/') {
					infile.ignore();
					if (infile.peek() != '/') {
						infile >> uv;
					}
					if (infile.peek() == '/') {
						infile.ignore();
						infile >> normal;
					}
				}
				VertexKey vk(pos, normal, uv);
				auto find = key_to_vertex.find(vk);
				if (find != key_to_vertex.end()) {
					face_indicies[face_count++] = find->second;
				}
				else {
					Vertex v;
					v.position = positions[pos - 1];	// obj format starts at 1
					if (normal != 0) {
						v.normal = normals[normal - 1];
					}
					else {
						gen_normals = true;
					}
					if (uv != 0) {
						v.uv = uvs[uv - 1];
					}
					else {
						v.uv = vec2(0);
					}
					verticies.push_back(v);
					key_to_vertex.insert({ vk,verticies.size() - 1 });
					face_indicies[face_count++] = verticies.size() - 1;
				}
			}
			if (gen_normals) {
				vec3 v0, v1, v2;
				v0 = verticies[face_indicies[0]].position;
				v1 = verticies[face_indicies[1]].position;
				v2 = verticies[face_indicies[2]].position;
				vec3 normal = normalize(cross(v2 - v1, v0 - v1));
				for (int i = 0; i < face_count; i++) {
					verticies[face_indicies[i]].normal = normal;
				}
			}
			if (face_count == 3) {
				indicies.push_back(face_indicies[0]);
				indicies.push_back(face_indicies[1]);
				indicies.push_back(face_indicies[2]);
			}
			else if (face_count == 4) {
				indicies.push_back(face_indicies[0]);
				indicies.push_back(face_indicies[1]);
				indicies.push_back(face_indicies[2]);

				indicies.push_back(face_indicies[2]);
				indicies.push_back(face_indicies[3]);
				indicies.push_back(face_indicies[0]);
			}
			else {
				printf("Face with more than 4 verticies\n");
				return nullptr;
			}
		}
		else if (buffer == "usemtl") {
			infile >> buffer;
		}
		else if (buffer == "mtllib") {
			infile >> buffer;
		}
		else if (buffer == "s") {
			infile >> buffer;
		}
		else if (buffer == "o") {
			infile >> buffer;
		}
		else if (buffer == "g") {
			infile >> buffer;
		}
		else {
			printf("Unknown symbol when loading mesh %s: %s\n", obj_file, buffer.c_str());
			return nullptr;
		}
	}

	Mesh* m = new Mesh;
	m->indicies = std::move(indicies);
	m->verticies = std::move(verticies);

	return m;
}