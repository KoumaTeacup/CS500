//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#include <Eigen/StdVector>
#include <Eigen_unsupported/Eigen/BVH>
#include "minimizer.h"
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

#define MAX_PASS 4096
#define RUSSIAN_ROULETTE 0.8f
#define epsilon 0.000001

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::mt19937_64 RNGen;
std::uniform_real_distribution<> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].

// Write the image as a HDR(RGBE) image.  
#include "rgbe.h"
void WriteHdrImage(const std::string outName, const int width, const int height, Color* image) {
	// Turn image from a 2D-bottom-up array of Vector3D to an top-down-array of floats
	float* data = new float[width*height * 3];
	float* dp = data;
	for (int y = height - 1; y >= 0; --y) {
		for (int x = 0; x<width; ++x) {
			Color pixel = image[y*width + x];
			*dp++ = pixel[0];
			*dp++ = pixel[1];
			*dp++ = pixel[2];
		}
	}

	// Write image to file in HDR (a.k.a RADIANCE) format
	rgbe_header_info info;
	char errbuf[100] = { 0 };

	FILE* fp = fopen(outName.c_str(), "wb");
	info.valid = false;
	int r = RGBE_WriteHeader(fp, width, height, &info, errbuf);
	if (r != RGBE_RETURN_SUCCESS)
		printf("error: %s\n", errbuf);

	r = RGBE_WritePixels_RLE(fp, data, width, height, errbuf);
	if (r != RGBE_RETURN_SUCCESS)
		printf("error: %s\n", errbuf);
	fclose(fp);

	delete data;
}

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
	for (auto i : mesh->triangles) {
		shapes.push_back(new Triangle(
			mesh->vertices[i[0]].pnt,
			mesh->vertices[i[1]].pnt,
			mesh->vertices[i[2]].pnt,
			mesh->vertices[i[0]].nrm,
			mesh->vertices[i[1]].nrm,
			mesh->vertices[i[2]].nrm,
			mesh->mat));
	}
}

void Scene::buildKDTree() {
}

Intersection * Scene::traceRay(Ray ray, std::vector<Shape*> shapes)
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

Vector3f Scene::sampleLope(Vector3f normal, float cTheta, float Phi) {
	double sTheta = sqrt(1 - cTheta * cTheta);
	Vector3f K = Vector3f(sTheta * cosf(Phi), sTheta * sinf(Phi), cTheta);
	Quaternionf q = Quaternionf::FromTwoVectors(Vector3f::UnitZ(), normal);
	return q._transformVector(K);
}

Intersection Scene::sampleLight() {
	int index = myrandom(RNGen) * emitters.size();
	return sampleSphere((Sphere*)emitters[index]);
}

Intersection Scene::sampleSphere(Sphere *s) {
	if (!s) return Intersection();
	float z = 2.0f * myrandom(RNGen) - 1;
	float r = sqrt(1 - z * z);
	float a = 2.0f * PI * myrandom(RNGen);
	Intersection it;
	it.normal = Vector3f(r * cosf(a), r*sinf(a), z);
	it.pos = s->center + it.normal * s->radius;
	it.pS = s;
	return it;
}

Color Scene::evalBrdf(Intersection &it) {
	return it.pS->mat->Kd / PI;
}

float Scene::pdfBrdf(Intersection & it, Vector3f wi) {
	return it.normal.dot(wi) / PI;
}

float Scene::pdfLight(Intersection & it) {
	return 1.0f / emitters.size() / it.pS->area;
}

float Scene::geomertryFactor(Intersection & A, Intersection & B) {
	Vector3f D = A.pos - B.pos;
	float result = A.normal.dot(D) * B.normal.dot(D) / D.dot(D) / D.dot(D);
	return result > 0 ? result : -result;
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

	std::vector<Shape*> bboxes;
	for (auto i : shapes) {
		bboxes.push_back(new Box(i->bbox().corner(Box3d::BottomLeftFloor), i->bbox().corner(Box3d::TopRightCeil) - i->bbox().corner(Box3d::BottomLeftFloor) ,currentMat));
	}

	KdBVH<float, 3, Shape*> tree(shapes.begin(), shapes.end());

	// Build unit vector for camera space.
	float rx = camera->ry * width / height;
	Vector3f camX = rx * camera->orient._transformVector(Vector3f::UnitX());
	Vector3f camY = camera->ry * camera->orient._transformVector(Vector3f::UnitY());
	Vector3f camZ = -1 * camera->orient._transformVector(Vector3f::UnitZ());

	//fprintf(stderr,	"Rendering Starts.\n¡¾Render Pass¡¿%d\n¡¾Resolution¡¿%d ¡Á %d\n", MAX_PASS, width, height);
	for (int pass = 0; pass < MAX_PASS; pass++) {
		#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
		for (int y = 0; y < height - 1; ++y) {
			for (int x = 0; x < width - 1; ++x) {
				//fprintf(stderr, "Progress: %2d%%, current Pass: %4d\r", pass * 100 / MAX_PASS, pass+1);
				//fprintf(stderr, "Rendering Pass: %d, y: %4d, x: %4d\r",pass, y, x);

				// Variable decleration
				Color color;
				Vector3f rayDir, Weight, expWeight, brdf;
				Ray ray, shadowRay;
				float ProbLightSample, ProbBRDFSample, MIS;
				Minimizer minimizer, shadowMinimizer;
				Intersection *pCurrentIt, *pShadowIt, expLight, lastIt;

				// define the ray
				// transform x and y to [-1,1] screen space
				float dx = (x + myrandom(RNGen)) / width * 2 - 1;
				float dy = (y + myrandom(RNGen)) / height * 2 - 1;
				rayDir = dx*camX + dy*camY + camZ;
				rayDir.normalize();
				ray = Ray(camera->eye, rayDir);

				// trace the initial ray
				minimizer = Minimizer(ray);
				pCurrentIt = BVMinimize(tree, minimizer) == FLT_MAX ? NULL : &minimizer.minIt;

				// compute the color
				// reset color and weights
				color = Vector3f(0.0f, 0.0f, 0.0f);
				Weight = Vector3f(1.0f, 1.0f, 1.0f);

				// Compute brdf if intersection exists and is not a light source
				if (pCurrentIt) {
					if (!pCurrentIt->pS->mat->isLight()) {
						while (myrandom(RNGen) < RUSSIAN_ROULETTE) {
							// Explicit Sampling (Light Sampling) --------------------------------------------------------------
							expLight = sampleLight();
							rayDir = expLight.pos - pCurrentIt->pos;
							if (rayDir.dot(pCurrentIt->normal) > 0) {
								rayDir.normalize();
								shadowRay = Ray(pCurrentIt->pos, rayDir);
								shadowMinimizer = Minimizer(shadowRay);
								pShadowIt = BVMinimize(tree, shadowMinimizer) == FLT_MAX ? NULL : &shadowMinimizer.minIt;
								if (pShadowIt && (pShadowIt->pos - expLight.pos).squaredNorm() < epsilon) {
									ProbLightSample = pdfLight(expLight) / geomertryFactor(*pCurrentIt, expLight);
									ProbBRDFSample = pdfBrdf(*pCurrentIt, shadowRay.D) * RUSSIAN_ROULETTE;
									MIS = ProbLightSample * ProbLightSample / (ProbLightSample * ProbLightSample + ProbBRDFSample * ProbBRDFSample);
									brdf = pCurrentIt->normal.dot(shadowRay.D) * evalBrdf(*pCurrentIt);
									expWeight = Weight.cwiseProduct(brdf / ProbLightSample);
									color += MIS * (Color)(expWeight.cwiseProduct(expLight.pS->mat->color));
								}
							}
							// -------------------------------------------------------------------------------------------------

							// Implicit Sampling (BRDF Sampling) ---------------------------------------------------------------
							// Save current intersection info and extend path
							ray.Q = pCurrentIt->pos;
							ray.D = sampleLope(pCurrentIt->normal, sqrt((float)myrandom(RNGen)), 2.0f * PI * myrandom(RNGen));
							lastIt = *pCurrentIt;
							minimizer = Minimizer(ray);
							if (BVMinimize(tree, minimizer) == FLT_MAX)	break;
							else {
								ProbBRDFSample = pdfBrdf(lastIt, ray.D) * RUSSIAN_ROULETTE;
								brdf = lastIt.normal.dot(ray.D) * evalBrdf(lastIt);
								Weight = Weight.cwiseProduct(brdf / ProbBRDFSample);
								if (pCurrentIt->pS->mat->isLight()) {
									ProbLightSample = pdfLight(*pCurrentIt) / geomertryFactor(lastIt, *pCurrentIt);
									MIS = ProbBRDFSample * ProbBRDFSample / (ProbLightSample * ProbLightSample + ProbBRDFSample * ProbBRDFSample);
									color += MIS * (Color)(Weight.cwiseProduct(pCurrentIt->pS->mat->color));
									break;
								}
							}
							// -------------------------------------------------------------------------------------------------
						}
					}
					// Use pure color for light sources
					else color = pCurrentIt->pS->mat->color;
				}

				image[y*width + x] += color/(float)MAX_PASS;
			}
		}
		//fprintf(stderr, "\n");

		// Out HDR Image whenever Pass is 2 exponential.
		if (pass == 0 || pass == 3 || pass == 15 || pass == 63 || pass == 255 || pass == 1023 || pass == 4095){
			char filename[32];
			sprintf(filename, "Image_Pass_%d.hdr", pass + 1);
			std::string hdrName = filename;
			// Write the image
			WriteHdrImage(hdrName, width, height, image);
		}
	}
}
