#include "Image.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

HDRImage* load_hdr(std::string filename)
{
	int width, height, n;
	float* data = stbi_loadf(filename.c_str(), &width, &height, &n, 0);
	if (!data) {
		std::cout << "Couldn't load image: " << filename << '\n';
		return nullptr;
	}

	HDRImage* img = new HDRImage(width, height);
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			int idx = (y * width + x) * n;
			vec3 col = vec3(data[idx], data[idx + 1], data[idx + 2]);

			img->set(x, y, col);
		}
	}

	return img;
}