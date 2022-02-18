#include <cmath>
#include <iostream>

#include "light.h"
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>


using namespace std;

double DirectionalLight::distanceAttenuation(const glm::dvec3& P) const
{
	// distance to light is infinite, so f(di) goes to 0.  Return 1.
	return 1.0;
}


glm::dvec3 DirectionalLight::shadowAttenuation(const ray& r, const glm::dvec3& p) const
{
	// YOUR CODE HERE:
	// You should implement shadow-handling code here.

	glm::dvec3 direction = glm::normalize(getDirection(p));

	isect i;
	ray shadow(p + direction * RAY_EPSILON, direction, glm::dvec3(1, 1, 1), ray::SHADOW);
	if (this->getScene()->intersect(shadow, i)) {
		return i.getMaterial().kt(i);
	}

	return glm::dvec3(1, 1, 1);
}

glm::dvec3 DirectionalLight::getColor() const
{
	return color;
}

glm::dvec3 DirectionalLight::getDirection(const glm::dvec3& P) const
{
	return -orientation;
}

double PointLight::distanceAttenuation(const glm::dvec3& P) const
{

	// YOUR CODE HERE

	// You'll need to modify this method to attenuate the intensity 
	// of the light based on the distance between the source and the 
	// point P.  For now, we assume no attenuation and just return 1.0
	// return 1.0;
	double d = glm::distance(position, P);
	return glm::min(1.0, 1 / (constantTerm + linearTerm * d + quadraticTerm * d * d));
}

glm::dvec3 PointLight::getColor() const
{
	return color;
}

glm::dvec3 PointLight::getDirection(const glm::dvec3& P) const
{
	return glm::normalize(position - P);
}


glm::dvec3 PointLight::shadowAttenuation(const ray& r, const glm::dvec3& p) const
{
	// YOUR CODE HERE:
	// You should implement shadow-handling code here.
	// return glm::dvec3(1,1,1);


	glm::dvec3 direction = glm::normalize(getDirection(p));

	isect i;
	ray shadow(p + direction * RAY_EPSILON, direction, glm::dvec3(1, 1, 1), ray::SHADOW);
	if (this->getScene()->intersect(shadow, i)) {
		double distToLight = glm::distance(position, p);

		if (i.getT() < distToLight) {
			return i.getMaterial().kt(i);
		}
	}

	return glm::dvec3(1, 1, 1);


	// ray shadowRay = ray(p, getDirection(p), glm::dvec3(1, 1 ,1), ray::SHADOW);
	// double length = glm::distance(p, position);
	// isect ii;
	// glm::dvec3 pos = glm::dvec3(0.0, 0.0, 0.0);
	// glm::dvec3 intensity = color, kt;
	// while(scene->intersect(shadowRay, ii)){
	// 	if((length -= ii.getT()) < RAY_EPSILON) return intensity;
	// 	if(!ii.getMaterial().Trans())
	// 		return glm::dvec3(0.0, 0.0, 0.0);
	// 	if(glm::dot(shadowRay.getDirection(), ii.getN()) > 0){
	// 		kt = ii.getMaterial().kt(ii);
	// 		for(int k = 0; k < 3; k++){
	// 			intensity[k] *= pow(kt[k], ii.getT());
	// 		}
	// 		pos = ii.getN() * RAY_EPSILON;
	// 	}else
	// 		pos = - ii.getN() * RAY_EPSILON;
	// 	shadowRay.setPosition(shadowRay.at(ii) + pos);// = ray(pos, getDirection(pos), glm::dvec3(1, 1, 1), ray::SHADOW);
	// }
	// return intensity;
}

#define VERBOSE 0

