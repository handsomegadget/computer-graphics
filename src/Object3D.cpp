#include "Object3D.h"

bool Sphere::intersect(const Ray &r, float tmin, Hit &h) const
{
    // BEGIN STARTER

    // We provide sphere intersection code for you.
    // You should model other intersection implementations after this one.

    // Locate intersection point ( 2 pts )
    const Vector3f &rayOrigin = r.getOrigin(); //Ray origin in the world coordinate
    const Vector3f &dir = r.getDirection();

    Vector3f origin = rayOrigin - _center;      //Ray origin in the sphere coordinate

    float a = dir.absSquared();
    float b = 2 * Vector3f::dot(dir, origin);
    float c = origin.absSquared() - _radius * _radius;

    // no intersection
    if (b * b - 4 * a * c < 0) {
        return false;
    }

    float d = sqrt(b * b - 4 * a * c);

    float tplus = (-b + d) / (2.0f*a);
    float tminus = (-b - d) / (2.0f*a);

    // the two intersections are at the camera back
    if ((tplus < tmin) && (tminus < tmin)) {
        return false;
    }

    float t = 10000;
    // the two intersections are at the camera front
    if (tminus > tmin) {
        t = tminus;
    }

    // one intersection at the front. one at the back 
    if ((tplus > tmin) && (tminus < tmin)) {
        t = tplus;
    }

    if (t < h.getT()) {
        Vector3f normal = r.pointAtParameter(t) - _center;
        normal = normal.normalized();
        h.set(t, this->material, normal);
        return true;
    }
    // END STARTER
    return false;
}

// Add object to group
void Group::addObject(Object3D *obj) {
    m_members.push_back(obj);
}

// Return number of objects in group
int Group::getGroupSize() const {
    return (int)m_members.size();
}

bool Group::intersect(const Ray &r, float tmin, Hit &h) const
{
    // BEGIN STARTER
    // we implemented this for you
    bool hit = false;
    for (Object3D* o : m_members) {
        if (o->intersect(r, tmin, h)) {
            hit = true;
        }
    }
    return hit;
    // END STARTER
}


Plane::Plane(const Vector3f &normal, float d, Material *m)
    : Object3D(m), _normal(normal.normalized()), _d(d)
{
    // Ensure the normal is a unit vector for consistent shading
}

bool Plane::intersect(const Ray &ray, float tmin, Hit &hit) const
{
    const Vector3f &rayOrigin = ray.getOrigin();
    const Vector3f &rayDir = ray.getDirection();

    float denom = Vector3f::dot(rayDir, _normal);

    // If denom is near zero, the ray is parallel to the plane
    if (fabs(denom) < 1e-6f) {
        return false;
    }

    float t = (_d - Vector3f::dot(rayOrigin, _normal)) / denom;

    // Ignore intersections behind the ray start or too close
    if (t < tmin || t >= hit.getT()) {
        return false;
    }

    // Valid intersection, update the hit record
    hit.set(t, this->material, _normal);
    return true;
}




bool Triangle::intersect(const Ray &ray, float tmin, Hit &hit) const 
{
    // Step 1: Define triangle edges
    Vector3f edge1 = _v[1] - _v[0];
    Vector3f edge2 = _v[2] - _v[0];

    // Step 2: Build matrix for solving [u, v, t]
    Matrix3f M(edge1, edge2, -ray.getDirection());

    // Step 3: Solve the linear system M * [u, v, t]^T = ray.origin - v0
    Vector3f rhs = ray.getOrigin() - _v[0];

    // Check if the matrix is invertible (triangle not degenerate or ray parallel)
    if (fabs(M.determinant()) < 1e-6f) {
        return false;
    }

    Vector3f solution = M.inverse() * rhs;
    float u = solution.x();
    float v = solution.y();
    float t = solution.z();

    // Step 4: Validate barycentric coordinates and ray parameter t
    bool isInsideTriangle = (u >= 0.0f) && (v >= 0.0f) && (u + v <= 1.0f);
    bool isValidT = (t >= tmin) && (t < hit.getT());

    if (!isInsideTriangle || !isValidT) {
        return false;
    }

    // Step 5: Compute interpolated normal (barycentric)
    Vector3f interpolatedNormal = (1.0f - u - v) * _normals[0] + u * _normals[1] + v * _normals[2];
    interpolatedNormal.normalize();

    // Step 6: Update hit record
    hit.set(t, this->material, interpolatedNormal);
    return true;
}



Transform::Transform(const Matrix4f &m, Object3D *obj)
    : _m(m), _object(obj)
{
    // Store transformation matrix and target object
}

bool Transform::intersect(const Ray &ray, float tmin, Hit &hit) const
{
    // Step 1: Transform ray from world space to object (local) space
    Matrix4f worldToLocal = _m.inverse();

    Vector3f transformedOrigin = (worldToLocal * Vector4f(ray.getOrigin(), 1)).xyz();
    Vector3f transformedDirection = (worldToLocal * Vector4f(ray.getDirection(), 0)).xyz();

    Ray rayLocal(transformedOrigin, transformedDirection);

    // Step 2: Intersect in object’s local space
    Hit localHit;
    float scaledTmin = tmin * transformedDirection.abs(); // conservative scaling

    if (_object->intersect(rayLocal, scaledTmin, localHit)) {
        // Step 3: Transform the local-space normal back to world space
        Matrix4f normalTransform = worldToLocal.transposed(); // transpose(inverse(M))
        Vector3f worldNormal = (normalTransform * Vector4f(localHit.getNormal(), 0)).xyz().normalized();

        // Step 4: Commit the hit with world-space normal
        hit.set(localHit.getT(), localHit.getMaterial(), worldNormal);
        return true;
    }

    return false;
}
