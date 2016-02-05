//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <vector>
#include <climits>

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
#else
    // Includes for Linux
#endif

#include "geom.h"
#include "raytrace.h"
//#include "realtime.h"
#include "material.h"
#include "camera.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::mt19937_64 RNGen;
std::uniform_real_distribution<> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].

Scene::Scene() 
{ 
    //realtime = new Realtime(); 
	camera = new Camera();
}

void Scene::Finit()
{
}

void Scene::triangleMesh(MeshData* mesh) 
{ 
    //realtime->triangleMesh(mesh); 
}

Intersection * Scene::traceRay(Ray ray)
{
	Intersection result = Intersection();
	result.t = FLT_MAX;
	Intersection it = Intersection();
	for (auto s : shapes) {
		if (s->intersect(ray, it)) {
			if (result.t > it.t) result = it;
		}
	}
	if (result.t == FLT_MAX) return nullptr;
	else return new Intersection(result);
}

const float Radians = PI / 180.0f;    // Convert degrees to radians
Quaternionf Orientation(int i, 
                        const std::vector<std::string>& strings,
                        const std::vector<float>& f)
{
    Quaternionf q(1,0,0,0); // Unit quaternion
    while (i<strings.size()) {
        std::string c = strings[i++];
        if (c == "x")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitX());
        else if (c == "y")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitY());
        else if (c == "z")  
            q *= angleAxis(f[i++]*Radians, Vector3f::UnitZ());
        else if (c == "q")  {
            q *= Quaternionf(f[i+0], f[i+1], f[i+2], f[i+3]);
            i+=4; }
        else if (c == "a")  {
            q *= angleAxis(f[i+0]*Radians, Vector3f(f[i+1], f[i+2], f[i+3]).normalized());
            i+=4; } }
    return q;
}

////////////////////////////////////////////////////////////////////////
// Material: encapsulates surface properties
void Material::setTexture(const std::string path)
{
    int width, height, n;
    stbi_set_flip_vertically_on_load(true);
    unsigned char* image = stbi_load(path.c_str(), &width, &height, &n, 0);
}

void Scene::Command(const std::vector<std::string>& strings,
                    const std::vector<float>& f)
{
    if (strings.size() == 0) return;
    std::string c = strings[0];
    
    if (c == "screen") {
        // syntax: screen width height
        // realtime->setScreen(int(f[1]),int(f[2]));
        width = int(f[1]);
        height = int(f[2]); }

	else if (c == "camera") {
		// syntax: camera x y z   ry   <orientation spec>
		// Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
		// realtime->setCamera(Vector3f(f[1],f[2],f[3]), Orientation(5,strings,f), f[4]);
		camera->eye = Vector3f(f[1], f[2], f[3]);
		camera->orient = Orientation(5, strings, f);
		camera->ry = f[4];
	}

	else if (c == "ambient") {
		// syntax: ambient r g b
		// Sets the ambient color.  Note: This parameter is temporary.
		// It will be ignored once your raytracer becomes capable of
		// accurately *calculating* the true ambient light.
		// realtime->setAmbient(Vector3f(f[1], f[2], f[3]));
		ambient = Vector3f(f[1], f[2], f[3]);
	}

    else if (c == "brdf")  {
        // syntax: brdf  r g b   r g b  alpha
        // later:  brdf  r g b   r g b  alpha  r g b ior
        // First rgb is Diffuse reflection, second is specular reflection.
        // third is beer's law transmission followed by index of refraction.
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new BRDF(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7]); 
	}

    else if (c == "light") {
        // syntax: light  r g b   
        // The rgb is the emission of the light
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Light(Vector3f(f[1], f[2], f[3])); 
	}
   
    else if (c == "sphere") {
        // syntax: sphere x y z   r
        // Creates a Shape instance for a sphere defined by a center and radius
        // realtime->sphere(Vector3f(f[1], f[2], f[3]), f[4], currentMat);
		shapes.push_back(new Sphere(Vector3f(f[1], f[2], f[3]), f[4], currentMat));
		if (currentMat->isLight()) emitters.push_back(shapes.back());
	}

    else if (c == "box") {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector
        // realtime->box(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), currentMat); 
		shapes.push_back(new Box(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), currentMat));
		if (currentMat->isLight()) emitters.push_back(shapes.back());
	}

    else if (c == "cylinder") {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        // realtime->cylinder(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], currentMat); 
		shapes.push_back(new Cylinder(Vector3f(f[1], f[2], f[3]), Vector3f(f[4], f[5], f[6]), f[7], currentMat));
		if (currentMat->isLight()) emitters.push_back(shapes.back());
	}

    else if (c == "mesh") {
        // syntax: mesh   filename   tx ty tz   s   <orientation>
        // Creates many Shape instances (one per triangle) by reading
        // model(s) from filename. All triangles are rotated by a
        // quaternion (qw qx qy qz), uniformly scaled by s, and
        // translated by (tx ty tz) .
        Matrix4f modelTr = translate(Vector3f(f[2],f[3],f[4]))
                          *scale(Vector3f(f[5],f[5],f[5]))
                          *toMat4(Orientation(6,strings,f));
        ReadAssimpFile(strings[1], modelTr);  }

    
    else {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}

void Scene::TraceImage(Color* image, const int pass)
{
    // realtime->run();                          // Remove this (realtime stuff)

	// Build unit vector for camera space.
	float rx = camera->ry * width / height;
	Vector3f camX = rx * camera->orient._transformVector(Vector3f::UnitX());
	Vector3f camY = camera->ry * camera->orient._transformVector(Vector3f::UnitY());
	Vector3f camZ = -1 * camera->orient._transformVector(Vector3f::UnitZ());

#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
    for (int y=0;  y<height - 1;  ++y) {
		for (int x = 0; x < width - 1; ++x) {
        fprintf(stderr, "Rendering y: %4d, x: %4d\r", y, x);
		// define the ray
		// transform x and y to [-1,1] screen space
		float dx = (x + 0.5f) / width * 2 - 1;
		float dy = (y + 0.5f) / height * 2 - 1;
		Vector3f rayDir = dx*camX + dy*camY + camZ;
		rayDir.normalize();
		Ray ray(camera->eye, rayDir);

		// trace the ray
		Intersection *intersection = traceRay(ray);

		// compute the color
		// Initialized default color to black.
		Color color = Vector3f(0.0f, 0.0f, 0.0f);

		// Compute brdf if intersection exists and is not a light source
		if (intersection) {
			if (!intersection->pS->mat->isLight()) {
				Vector3f result(0.0f, 0.0f, 0.0f);
				for (auto e : emitters)
					// Combine the value from all non-attenuative lights.
					result += intersection->pS->mat->Kd / PI * intersection->normal.dot((e->position - intersection->pos).normalized());
				color = result;
				//color = (intersection->normal + Vector3f(1.0f, 1.0f, 1.0f))/2.0f;
				//color = intersection->pS->mat->Kd;
			}
			// Use pure color for light sources
			else color = intersection->pS->mat->color;

			delete intersection;
		}

		image[y*width + x] = color;
		}
	}
    fprintf(stderr, "\n");
}
