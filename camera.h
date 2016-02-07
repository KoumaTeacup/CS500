#pragma once

#include "geom.h"

struct Camera {
public:
	Vector3f eye;
	Quaternionf orient;
	float ry;
};