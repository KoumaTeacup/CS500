#include "shape.h"

bool Sphere::intersect(const Ray & ray, Intersection & it)
{
	Vector3f Qbar = ray.Q - center;
	float QdotD = Qbar.dot(ray.D);
	float fvar = sqrt(QdotD*QdotD - Qbar.dot(Qbar) + radius*radius);
	if (-QdotD - fvar > 0.0f) it.t = -QdotD - fvar;
	else if (-QdotD + fvar > 0.0f) it.t = -QdotD + fvar;
	else return false;

	it.pos = ray.eval(it.t);
	it.normal = (it.pos - center).normalized();
	it.pS = this;

	return true;
}

Box::Box(Vector3f c, Vector3f d, Material * m) : Shape(m), corner(c), diag(d){
	slabs[0] = Slab(Vector3f(1.0f, 0.0f, 0.0f), -c[0], -(c + d)[0]);
	slabs[1] = Slab(Vector3f(0.0f, 1.0f, 0.0f), -c[1], -(c + d)[1]);
	slabs[2] = Slab(Vector3f(0.0f, 0.0f, 1.0f), -c[2], -(c + d)[2]);
	position = corner + diag / 2.0f;
}

bool Box::intersect(const Ray & ray, Intersection & it)
{
	Vector3f normal0, normal1, normalVec;
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
			normalVec = slabs[i].normal;
		} else {
			float NdotQ = slabs[i].normal.dot(ray.Q);
			float s0 = NdotQ + slabs[i].d0;
			float s1 = NdotQ + slabs[i].d1;
			if (s0 * s1 < 0.0f) {
				_t0 = 0.0f;
				_t1 = FLT_MAX;
			} else return false;
		}

		if (t0 < _t0) {
			t0 = _t0;
			normal0 = normalVec;
		}
		if (t1 > _t1) {
			t1 = _t1;
			normal1 = normalVec;
		}
	}

	if (t0 > t1) return false;
	else if (t0 > 0.0f) {
		it.t = t0; 
		it.normal = normal0;
	}
	else if (t1 > 0.0f) {
		it.t = t1;
		it.normal = normal1;
	}
	else return false;

	it.pos = ray.eval(it.t);
	it.pS = this;
	return true;
}

bool Cylinder::intersect(const Ray & ray, Intersection & it)
{
	return false;
}

bool Triangle::intersect(const Ray & ray, Intersection & it)
{
	return false;
}
