#include "Image.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

HDRImage* load_hdr(const std::string& filename)
{
	int width, height, n;
	float* data = stbi_loadf(filename.c_str(), &width, &height, &n, 0);
	if (!data) {
		std::cout << "Couldn't load image: " << filename << '\n';
		return nullptr;
	}

	HDRImage* img = new HDRImage;
	img->width = width;
	img->height = height;
	if (n == 4)
		img->internalFormat = Image::Format::RGBA_32f;
	else
		img->internalFormat = Image::Format::RGB_32f;

	img->imageData.resize(width * height * sizeof(float) * n);

	memcpy(img->imageData.data(), data, width * height * sizeof(float) * n);

	stbi_image_free(data);

	return img;
}
Image* load_standard_img(const std::string& filename)
{
	int width, height, n;
	uint8_t* data = stbi_load(filename.c_str(), &width, &height, &n, 0);
	if (!data) {
		std::cout << "Couldn't load image: " << filename << '\n';
		return nullptr;
	}

	Image* img = new Image;
	img->width = width;
	img->height = height;
	if (n == 4)
		img->internalFormat = Image::Format::RGBA_u8;
	else
		img->internalFormat = Image::Format::RGB_u8;

	img->imageData.resize(width*height*n);
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			int offset = (width * y + x) * n;
			int alignment = ((width*n) % 4)* y;
			int offsetWithAlignment = offset + alignment;

			for (int channel = 0; channel < n; channel++) {
				img->imageData[offset + channel] = data[offsetWithAlignment + channel];
			}
		}
	}
	stbi_image_free(data);

	return img;
}