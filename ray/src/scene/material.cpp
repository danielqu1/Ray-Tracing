#include "material.h"
#include "../ui/TraceUI.h"
#include "light.h"
#include "ray.h"
extern TraceUI* traceUI;

#include <glm/gtx/io.hpp>
#include <iostream>
#include "../fileio/images.h"

using namespace std;
extern bool debugMode;

Material::~Material()
{
}

// Apply the phong model to this point on the surface of the object, returning
// the color of that point.
glm::dvec3 Material::shade(Scene* scene, const ray& r, const isect& i) const
{
	// YOUR CODE HERE

	// For now, this method just returns the diffuse color of the object.
	// This gives a single matte color for every distinct surface in the
	// scene, and that's it.  Simple, but enough to get you started.
	// (It's also inconsistent with the phong model...)

	// Your mission is to fill in this method with the rest of the phong
	// shading model, including the contributions of all the light sources.
	// You will need to call both distanceAttenuation() and
	// shadowAttenuation()
	// somewhere in your code in order to compute shadows and light falloff.
	//	if( debugMode )
	//		std::cout << "Debugging Phong code..." << std::endl;

	// When you're iterating through the lights,
	// you'll want to use code that looks something
	// like this:
	//
	// for ( const auto& pLight : scene->getAllLights() )
	// {
	//              // pLight has type unique_ptr<Light>
	// 		.
	// 		.
	// 		.
	// }
	// return kd(i);

	// initialize colorC
	glm::dvec3 colorC = ke(i) + ka(i) * scene->ambient();

	glm::dvec3 pos = r.at(i);
	glm::dvec3 n = i.getN();

	// loop through all lights
	for (const auto& pLight : scene->getAllLights()) {
		glm::dvec3 l = pLight->getDirection(pos);

		// atten = dist atten * shadow atten
		glm::dvec3 atten = pLight->distanceAttenuation(pos) * pLight->shadowAttenuation(r, pos);

		// calculate diffuse term
		glm::dvec3 diffuse = kd(i) * glm::max(glm::dot(l, n), 0.0);

		// calculate specular term
		glm::dvec3 v = glm::normalize(scene->getCamera().getEye() - pos);
		// glm::dvec3 v = - r.getDirection();
		glm::dvec3 r = (2 * glm::dot(l, n) * n) - l;
		glm::dvec3 specular = ks(i) * glm::pow(glm::max(glm::dot(r, v), 0.0), shininess(i));

		colorC += atten * (diffuse + specular);
	}

	return colorC;
}

TextureMap::TextureMap(string filename)
{
	data = readImage(filename.c_str(), width, height);
	if (data.empty()) {
		width = 0;
		height = 0;
		string error("Unable to load texture map '");
		error.append(filename);
		error.append("'.");
		throw TextureMapException(error);
	}
}

glm::dvec3 TextureMap::getMappedValue(const glm::dvec2& coord) const
{
	// YOUR CODE HERE
	//
	// In order to add texture mapping support to the
	// raytracer, you need to implement this function.
	// What this function should do is convert from
	// parametric space which is the unit square
	// [0, 1] x [0, 1] in 2-space to bitmap coordinates,
	// and use these to perform bilinear interpolation
	// of the values.

	double w = (double) getWidth();
	double h = (double) getHeight();

	double left = floor(coord.x * w);
	double right = left + 1.0;
	double bot = floor(coord.y * h);
	double top = bot + 1.0;

	if (right >= w) {
		right = w - 1;
	}
	if (top >= h) {
		top = h - 1;
	}

	glm::dvec3 topleft = getPixelAt(left, top);
	glm::dvec3 topright = getPixelAt(right, top);
	glm::dvec3 botleft = getPixelAt(left, bot);
	glm::dvec3 botright = getPixelAt(right, bot);

	return (topleft + topright + botleft + botright) / glm::dvec3(4.0, 4.0, 4.0);
}

glm::dvec3 TextureMap::getPixelAt(int x, int y) const
{
	// YOUR CODE HERE
	//
	// In order to add texture mapping support to the
	// raytracer, you need to implement this function.

	// if (0 == data){
	// 	return glm::dvec3(1, 1, 1);
	// }

	int index = 3 * (y * getWidth() + x);
	return glm::dvec3(
		data[index] / 255.0,
		data[index + 1] / 255.0,
		data[index + 2] / 255.0);
}

glm::dvec3 MaterialParameter::value(const isect& is) const
{
	if (0 != _textureMap)
		return _textureMap->getMappedValue(is.getUVCoordinates());
	else
		return _value;
}

double MaterialParameter::intensityValue(const isect& is) const
{
	if (0 != _textureMap) {
		glm::dvec3 value(
		        _textureMap->getMappedValue(is.getUVCoordinates()));
		return (0.299 * value[0]) + (0.587 * value[1]) +
		       (0.114 * value[2]);
	} else
		return (0.299 * _value[0]) + (0.587 * _value[1]) +
		       (0.114 * _value[2]);
}
