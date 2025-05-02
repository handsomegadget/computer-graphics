#include "Renderer.h"

#include "ArgParser.h"
#include "Camera.h"
#include "Image.h"
#include "Ray.h"
#include "VecUtils.h"

#include <limits>


Renderer::Renderer(const ArgParser &args) :
    _args(args),
    _scene(args.input_file)
{
}

void
Renderer::Render()
{
    int w = _args.width;
    int h = _args.height;

    Image image(w, h);
    Image nimage(w, h);
    Image dimage(w, h);

    // loop through all the pixels in the image
    // generate all the samples

    // This look generates camera rays and callse traceRay.
    // It also write to the color, normal, and depth images.
    // You should understand what this code does.
    Camera* cam = _scene.getCamera();
    for (int y = 0; y < h; ++y) {
        float ndcy = 2 * (y / (h - 1.0f)) - 1.0f;
        for (int x = 0; x < w; ++x) {
            float ndcx = 2 * (x / (w - 1.0f)) - 1.0f;
            // Use PerspectiveCamera to generate a ray.
            // You should understand what generateRay() does.
            Ray r = cam->generateRay(Vector2f(ndcx, ndcy));

            Hit h;
            Vector3f color = traceRay(r, cam->getTMin(), _args.bounces, h);

            image.setPixel(x, y, color);
            nimage.setPixel(x, y, (h.getNormal() + 1.0f) / 2.0f);
            float range = (_args.depth_max - _args.depth_min);
            if (range) {
                dimage.setPixel(x, y, Vector3f((h.t - _args.depth_min) / range));
            }
        }
    }
    // END SOLN

    // save the files 
    if (_args.output_file.size()) {
        image.savePNG(_args.output_file);
    }
    if (_args.depth_file.size()) {
        dimage.savePNG(_args.depth_file);
    }
    if (_args.normals_file.size()) {
        nimage.savePNG(_args.normals_file);
    }
}



Vector3f Renderer::traceRay(const Ray &ray,
    float tmin,
    int bounces,
    Hit &hit) const
{
// Step 1: Intersect ray with scene geometry
if (!_scene.getGroup()->intersect(ray, tmin, hit)) {
return _scene.getBackgroundColor(ray.getDirection());
}

Vector3f color(0, 0, 0);                         // Final color
Vector3f hitPoint = ray.pointAtParameter(hit.getT());
Vector3f normal = hit.getNormal();
Material *material = hit.getMaterial();

// Step 2: Direct illumination from all lights
for (int i = 0; i < _scene.getNumLights(); ++i) {
Vector3f dirToLight, lightIntensity;
float distToLight;
_scene.getLight(i)->getIllumination(hitPoint, dirToLight, lightIntensity, distToLight);

// Step 2.1: Shadow ray check
bool inShadow = false;
if (_args.shadows) {
Ray shadowRay(hitPoint + dirToLight * 1e-3f, dirToLight); // Avoid self-intersection
Hit shadowHit;
if (_scene.getGroup()->intersect(shadowRay, 1e-3f, shadowHit) && shadowHit.t < distToLight) {
inShadow = true;
}
}

// Step 2.2: Add contribution if not in shadow
if (!inShadow) {
color += material->shade(ray, hit, dirToLight, lightIntensity);
}
}

// Step 3: Add ambient light
color += _scene.getAmbientLight() * material->getDiffuseColor();

// Step 4: Recursive reflection
if (bounces > 0) {
Vector3f viewDir = -ray.getDirection();
Vector3f reflectDir = ray.getDirection() - 2 * Vector3f::dot(ray.getDirection(), normal) * normal;
Ray reflectRay(hitPoint + reflectDir * 1e-3f, reflectDir); // Avoid self-hit
Hit reflectHit;
Vector3f reflectedColor = traceRay(reflectRay, 1e-3f, bounces - 1, reflectHit);
color += reflectedColor * material->getSpecularColor();
}

return color;
}

