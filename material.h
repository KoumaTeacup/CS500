#pragma once

#include <string>
#include "geom.h"

class Material
{
public:
	Vector3f Kd, Ks;
	float alpha;
	Vector3f color;
	unsigned int texid;

	virtual bool isLight() { return false; }

	Material(): texid(0) {}
	Material(const Vector3f _color): color(_color), texid(0) {}
	Material(const Vector3f d, const Vector3f s, const float a)
		: Kd(d), Ks(s), alpha(a), texid(0) {}
	Material(const Material &rhs):
		Kd(rhs.Kd), Ks(rhs.Ks), alpha(rhs.alpha), color(rhs.color), texid(rhs.texid){}

	void setTexture(const std::string path);
	//virtual void apply(const unsigned int program);
};

class Light : public Material
{
public:
	Light(const Vector3f _color) :Material(_color) {}

	virtual bool isLight() { return true; }
	//virtual void apply(const unsigned int program);
};

class BRDF : public Material
{
public:
	BRDF(const Vector3f d, const Vector3f s, const float a):Material(d,s,a) {}

	virtual bool isLight() { return false; }
	//virtual void apply(const unsigned int program);
};