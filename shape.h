#pragma once

#include "material.h"

class Shape;

class Ray {
public:
	Ray(Vector3f q, Vector3f d) :Q(q), D(d) {}

	Vector3f eval(float t) { return Q + t * D; }
	Vector3f Q, D;
};

struct Intersection {
	Shape *pS;
	float t;
	Vector3f pos, normal;
};

class Shape {
public:
	Shape(Material *m):mat(m) {}
	virtual ~Shape() {}

	virtual bool intersect(const Ray& ray, Intersection& it) = 0;

	Material *mat;
};

class Sphere : public Shape {
public:
	Vector3f center;
	float radius;
	Sphere(Vector3f _center, float _radius, Material *m) : Shape(m), center(_center), radius(_radius) {}
	bool intersect(const Ray& ray, Intersection& it);
};

class Box : public Shape {
public:
	Vector3f corner, diag;
	Box(Vector3f c, Vector3f d, Material *m) : Shape(m), corner(c), diag(d) {}
	bool intersect(const Ray& ray, Intersection& it);
};

class Cylinder : public Shape {
public:
	Vector3f base, axis;
	float radius;
	Cylinder(Vector3f b, Vector3f a, float r, Material *m) : Shape(m), base(b), axis(a), radius(r) {}
	bool intersect(const Ray& ray, Intersection& it);
};

class Triangle : public Shape {
public:
	Triangle(Material *m) : Shape(m) {}
	bool intersect(const Ray& ray, Intersection& it);
};
