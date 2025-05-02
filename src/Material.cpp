#include "Material.h"
Vector3f Material::shade(const Ray &ray,
    const Hit &hit,
    const Vector3f &dirToLight,
    const Vector3f &lightIntensity)
{
// Step 1: Prepare vectors
Vector3f hitPoint = ray.pointAtParameter(hit.getT());
Vector3f offsetHitPoint = hitPoint + 0.0001f * dirToLight;  // avoid self-intersection
Ray shadowRay(offsetHitPoint, dirToLight);

Vector3f normal = hit.getNormal();               // Surface normal at hit point
Vector3f viewDir = -ray.getDirection();          // Direction to camera (eye)

// Step 2: Diffuse term (Lambertian)
float dotNL = Vector3f::dot(dirToLight, normal); // Cosine of angle between light and normal
float diffuseFactor = std::max(dotNL, 0.0f);
Vector3f diffuse = diffuseFactor * lightIntensity * _diffuseColor;

// Step 3: Specular term (Phong)
Vector3f reflectDir = 2 * Vector3f::dot(normal, viewDir) * normal - viewDir; // Reflection direction
float dotLR = Vector3f::dot(dirToLight, reflectDir);
float specularFactor = std::max(dotLR, 0.0f);
Vector3f specular = pow(specularFactor, _shininess) * lightIntensity * _specularColor;

// Step 4: Combine
return diffuse + specular;
}

