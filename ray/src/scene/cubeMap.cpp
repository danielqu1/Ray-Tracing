#include "cubeMap.h"
#include "ray.h"
#include "../ui/TraceUI.h"
#include "../scene/material.h"
extern TraceUI* traceUI;

glm::dvec3 CubeMap::getColor(ray r) const
{
	// YOUR CODE HERE
	// FIXME: Implement Cube Map here

	double x = r.getDirection()[0];
	double y = r.getDirection()[1];
	double z = r.getDirection()[2];
	double absValX = glm::abs(x);
	double absValY = glm::abs(y);
	double absValZ = glm::abs(z);
	int index;
	double k, u, v;

	if (absValX > absValY && absValX > absValZ) {
		k = absValX;
		if (x < 0) {
			// Choose Face 1
			index = 1;
			u = z;
			v = y;
		} else {
			// Choose Face 0
			u = -z;
			v = y;
			index = 0;
		}

	} else if (absValY > absValX && absValY > absValZ) {
		k = absValY;
		if (y < 0) {
			// Choose Face 3
			index = 3;
			u = x;
			v = z;
		} else {
			// Choose Face 2
			index = 2;
			u = x;
			v = -z;
		}
	} else  {
		k = absValZ;
		if (z < 0) {
			// Choose Face 5
			index = 5;
			u = -x;
			v = y;
		} else {
			// Choose Face 4
			index = 4;
			u = x;
			v = y;
		}

	} 
	u = 0.5 * (u / k + 1);
	v = 0.5 * (v / k + 1);

	return tMap[index]->getMappedValue(glm::dvec2(u,v));
}

CubeMap::CubeMap()
{
}

CubeMap::~CubeMap()
{
}

void CubeMap::setNthMap(int n, TextureMap* m)
{
	if (m != tMap[n].get())
		tMap[n].reset(m);
}
