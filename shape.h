#pragma once

#include "material.h"

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

	Material *mat;
};

class Sphere : public Shape {
public:
	Vector3f center;
	float radius;
	Sphere(Vector3f _center, float _radius, Material *m) : Shape(m), center(_center), radius(_radius) { position = _center; }
	bool intersect(const Ray& ray, Intersection& it);
};

class Box : public Shape {
public:
	Slab slabs[3];
	Vector3f corner, diag;
	Box(Vector3f c, Vector3f d, Material *m);
	bool intersect(const Ray& ray, Intersection& it);
};

class Cylinder : public Shape {
public:
	Quaternionf q;
	Slab slab;
	Vector3f base, axis;
	float radius;
	Cylinder(Vector3f b, Vector3f a, float r, Material *m) : 
		Shape(m), base(b), axis(a), radius(r) ,
		slab(Vector3f(0.0f, 0.0f, 1.0f), 0.0f, -a.norm()), 
		q(Quaternionf::FromTwoVectors(a, Vector3f::UnitZ()))
	{ position = base + axis / 2.0f; }
	bool intersect(const Ray& ray, Intersection& it);
};

class Triangle : public Shape {
public:
	Triangle(Material *m) : Shape(m) {}
	bool intersect(const Ray& ray, Intersection& it);
};
