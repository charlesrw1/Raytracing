#ifndef IMAGE_H
#define IMAGE_H
#include <vector>
#include "Math.h"


class Image
{
public:
	
	enum class Format {
		None,
		RGBA_32f,
		RGB_32f,
		RGBA_u8,
		RGB_u8,
	};

	vec3 get(int x, int y) const {
		switch (internalFormat)
		{
		case Format::RGBA_32f: {
			float* data = (float*)imageData.data();
			int offset = (width * y + x) * 4;
			return vec3(data[offset], data[offset + 1], data[offset + 2]);
		} break;
		case Format::RGB_32f: {
			float* data = (float*)imageData.data();
			int offset = (width * y + x) * 3;
			return vec3(data[offset], data[offset + 1], data[offset + 2]);
		} break;
		case Format::RGBA_u8: {
			uint8_t* data = (uint8_t*)imageData.data();
			int offset = (width * y + x) * 4;
			return vec3(data[offset], data[offset + 1], data[offset + 2]) / 255.f;
		}
		case Format::RGB_u8: {
			uint8_t* data = (uint8_t*)imageData.data();
			int offset = (width * y + x) * 3;
			return vec3(data[offset], data[offset + 1], data[offset + 2]) / 255.f;
		}
		}
	}

	// Bilinear filtered
	vec3 lookup(float u, float v) const {
		u = u * width - 0.5;
		v = v * height - 0.5;
		int ufi = modulo((int)u,width);
		int vfi = modulo((int)v, height);
		int uci = modulo(ufi + 1, width);
		int vci = modulo(vfi + 1, height);
		float integral;
		float uoff = modf(u - ufi,&integral);
		float voff = modf(v - vfi,&integral);
		vec3 val_ff = get(ufi, vfi);
		vec3 val_fc = get(ufi, vci);
		vec3 val_cf = get(uci, vfi);
		vec3 val_cc = get(uci, vci);
		return val_ff * (1 - uoff) * (1 - voff) +
			val_fc * (1 - uoff) * (voff)+
			val_cf * (uoff) * (1 - voff) +
			val_cc * (uoff) * (voff);
	}

	int width=0, height=0;
	std::vector<char> imageData;
	Format internalFormat = Format::None;
};
using HDRImage = Image;

HDRImage* load_hdr(const std::string& filename);
Image* load_standard_image(const std::string& filename)

#endif // !IMAGE_H\
