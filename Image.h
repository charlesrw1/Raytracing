#ifndef IMAGE_H
#define IMAGE_H
#include <vector>
#include "Math.h"

template< typename T >
class Image
{
public:
	Image(int w, int h) 
		:width(w), height(h) {
		data.resize(w * h);
	}

	void downsample(int n)
	{
		int old_w = width;
		int old_h = height;
		int new_w = width >> 1;
		int new_h = height >> 1;
		std::vector<T> temp_data(new_w*new_h);
		bool writing_to_temp = true;
		T* write_buf = temp_data.data();
		T* read_buf = data.data();
		for (int i = 0; i < n; i++)
		{
			for (int x = 0; x < new_w; x++) {
				for (int y = 0; y < new_h; y++) {
					write_buf[y * new_w + x] = (read_buf[y * 2 * old_w + x * 2] +
						read_buf[(y * 2 + 1) * old_w + x * 2] +
						read_buf[(y * 2 + 1) * old_w + x * 2 + 1] +
						read_buf[y * 2 * old_w + x * 2 + 1]) * 0.25;
				}
 			}
			if (writing_to_temp) {
				write_buf = data.data();
				read_buf = temp_data.data();
			}
			else {
				write_buf = temp_data.data();
				read_buf = data.data();
			}
			writing_to_temp = !writing_to_temp;
			old_w = new_w;
			old_h = new_h;
			new_w >>= 1;
			new_h >>= 1;
		}
		if (!writing_to_temp)
			data = std::move(temp_data);

		width = old_w;
		height = old_h;
	}

	vec3 get(int x, int y) const {
		return data[width * y + x];
	}
	void set(int x, int y, vec3 color) {
		data[width * y + x] = color;
	}

	// Bilinear filtered
	vec3 lookup(float u, float v) const {
		u = u * width - 0.5;
		v = v * height - 0.5;
		int ufi = modulo((int)u,width);
		int vfi = modulo((int)v, height);
		int uci = modulo(ufi + 1, width);
		int vci = modulo(vfi + 1, height);
		float uoff = u - ufi;
		float voff = v - vfi;
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
	std::vector<T> data;
};
using HDRImage = Image<vec3>;

HDRImage* load_hdr(std::string filename);

#endif // !IMAGE_H\
