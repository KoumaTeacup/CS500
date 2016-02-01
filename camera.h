#pragma once

#include "geom.h"

class Camera {
public:
	Vector3f eye;
	Quaternionf orient;
	float ry;
};