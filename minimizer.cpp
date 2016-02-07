#include "minimizer.h"

float Minimizer::minimumOnObject(Shape * obj) {
	Intersection It;
	if (obj->intersect(ray, It))
		if (It.t < minIt.t) {
			minIt = It;
			return It.t;
		}
	return FLT_MAX; // or whatever
}

float Minimizer::minimumOnVolume(const Box3d & box) {
	Vector3f L = box.corner(Box3d::BottomLeftFloor);
	Vector3f U = box.corner(Box3d::TopRightCeil);
	if (ray.x == 293 && ray.y == 72) {
		printf("box (%.2f, %.2f, %.2f - %.2f, %.2f, %.2f) is under checked\n", L[0], L[1], L[2], U[0], U[1], U[2]);
	}
	Slab slabs[3];
	slabs[0] = Slab(Vector3f(1.0f, 0.0f, 0.0f), -L[0], -U[0]);
	slabs[1] = Slab(Vector3f(0.0f, 1.0f, 0.0f), -L[1], -U[1]);
	slabs[2] = Slab(Vector3f(0.0f, 0.0f, 1.0f), -L[2], -U[2]);

	float t0 = 0, t1 = FLT_MAX;
	float _t0, _t1;
	for (int i = 0; i < 3; ++i) {
		if (slabs[i].normal.dot(ray.D)) {
			float NdotQ = slabs[i].normal.dot(ray.Q);
			float NdotD = slabs[i].normal.dot(ray.D);
			_t0 = -(slabs[i].d0 + NdotQ) / NdotD;
			_t1 = -(slabs[i].d1 + NdotQ) / NdotD;
			if (_t0 > _t1) {
				float temp = _t0;
				_t0 = _t1;
				_t1 = temp;
			}
		} else {
			float NdotQ = slabs[i].normal.dot(ray.Q);
			float s0 = NdotQ + slabs[i].d0;
			float s1 = NdotQ + slabs[i].d1;
			if (s0 * s1 < 0.0f) {
				_t0 = 0.0f;
				_t1 = FLT_MAX;
			} else return FLT_MAX;
		}

		if (t0 < _t0) t0 = _t0;
		if (t1 > _t1) t1 = _t1;
	}

	if (t0 > t1) return FLT_MAX;
	else if (t0 > 0.0f) {
		if (ray.x == 293 && ray.y == 72) {
			printf("has intersection t = %.2f\n", t0);
		}
		return t0;
	}
	else if (t1 > 0.0f) {
		if (ray.x == 293 && ray.y == 72) {
			printf("has intersection t = %.2f\n", t1);
		}
		return t1;
	}
	else return FLT_MAX;
}