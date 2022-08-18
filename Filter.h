#ifndef FILTER_H
#define FILTER_H
#include "Math.h"

class Filter
{
public:
	~Filter() {}
	virtual vec2 sample(float dx, float dy) const = 0;
};

class BoxFilter : public Filter
{
public:
	BoxFilter(float width) 
		:width(width) {}

	virtual vec2 sample(float dx, float dy) const override {
		vec2 sample = vec2(2 * dx - 1, 2 * dy - 1);
		sample.x *= width * 0.5;
		sample.y *= width * 0.5;
		return sample;
	}


	float width;
};
class GaussianFilter : public Filter
{
public:
	GaussianFilter(float stddev)
		:stddev(stddev) {}

	virtual vec2 sample(float dx, float dy) const override {
		float r = stddev * sqrt(-2 * log(max(dx, 1e-8)));
		return vec2(r * cos(2 * PI * dy),
			r * sin(2 * PI * dy));
	}


	float stddev;
};



#endif // !FILTER_H
