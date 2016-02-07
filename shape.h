#pragma once


#include "material.h"

typedef Eigen::AlignedBox<float, 3> Box3d; // The BV type provided by Eigen

class Shape;

struct Slab
{
	Slab() {}
	Slab(Vector3f _normal, float _d0, float _d1):normal(_normal), d0(_d0), d1(_d1){}
	Vector3f normal;
	float d0, d1;
};

class Ray {
public:
	int x, y;
	Ray(Vector3f q, Vector3f d) :Q(q), D(d) {}

	Vector3f eval(float t)const { return Q + t * D; }
	Vector3f Q, D;
};

struct Intersection {
	Shape *pS;
	float t;
	Vector3f pos, normal;
};

class Shape {
public:
	Vector3f position;
	Shape(Material *m):mat(m) {}
	virtual ~Shape() {}

	virtual bool intersect(const Ray& ray, Intersection& it) = 0;

	virtual Box3d bbox() const { return Box3d(); }

	Material *mat;
};

Box3d bounding_box(const Shape *pS);

class Sphere : public Shape {
public:
	int thisId;

	Vector3f center;
	float radius;
	Sphere(Vector3f _center, float _radius, Material *m) : Shape(m), center(_center), radius(_radius),thisId(id++) { position = _center; }
	bool intersect(const Ray& ray, Intersection& it);

	virtual Box3d bbox()const {
		return Box3d(Vector3f(center[0] - radius, center[1]- radius, center[2]- radius), Vector3f(center[0] + radius, center[1] + radius, center[2] + radius));
	}

	static int id;
};

class Box : public Shape {
public:
	Slab slabs[3];
	Vector3f corner, diag;
	Box(Vector3f c, Vector3f d, Material *m);
	bool intersect(const Ray& ray, Intersection& it);

	virtual Box3d bbox()const {
		Vector3f min = corner, max;
		/*for (int i = 0; i < 3; ++i)
			if (diag[i] > 0) max[i] = corner[i] + diag[i];
			else {
				max[i] = corner[i];
				min[i] = corner[i] + diag[i];
			}*/
		return Box3d(corner, corner + diag);
	}
};

class Cylinder : public Shape {
public:
	int thisId;
	Quaternionf q;
	Slab slab;
	Vector3f base, axis;
	float radius;
	Cylinder(Vector3f b, Vector3f a, float r, Material *m) : 
		Shape(m), base(b), axis(a), radius(r) ,thisId(id++),
		slab(Vector3f(0.0f, 0.0f, 1.0f), 0.0f, -a.norm()), 
		q(Quaternionf::FromTwoVectors(a, Vector3f::UnitZ()))
	{ position = base + axis / 2.0f; }
	bool intersect(const Ray& ray, Intersection& it);

	virtual Box3d bbox()const {
		Vector3f min = base, max;
		for (int i = 0; i < 3; ++i)
			if (axis[i] > 0) max[i] = base[i] + axis[i];
			else {
				max[i] = base[i];
				min[i] = base[i] + axis[i];
			}
		return Box3d(min - Vector3f(radius, radius, radius), max + Vector3f(radius, radius, radius));
	}

	static int id;
};

class Triangle : public Shape {
public:
	Vector3f v0, v1, v2, n0, n1, n2;
	Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2, Vector3f _n0, Vector3f _n1, Vector3f _n2, Material *m) : 
		Shape(m), v0(_v0), v1(_v1), v2(_v2),n0(_n0), n1(_n1), n2(_n2) {}
	bool intersect(const Ray& ray, Intersection& it);

	virtual Box3d bbox()const {
		Vector3f min, max;
		for (int i = 0; i < 3; ++i) {
			if (v0[i] > v1[i])
				if (v1[i] > v2[i]) min[i] = v2[i];
				else min[i] = v1[i];
			else if (v0[i] > v2[i])min[i] = v2[i];
			else min[i] = v0[i];

			if (v0[i] < v1[i])
				if (v1[i] < v2[i]) max[i] = v2[i];
				else max[i] = v1[i];
			else if (v0[i] < v2[i])max[i] = v2[i];
			else max[i] = v0[i];
		}
		return Box3d(min, max);
	}
};