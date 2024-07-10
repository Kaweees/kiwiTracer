#pragma once

#include "../include/vector.hpp"

// Represents a ray in three-dimensional space
// class Ray {
//   public:
//   // Constructor to initialize memory
//   Ray() : origin(0, 0, 0), direction(0, 0, 0) {}
//   Ray(const Vector3D& origin, const Vector3D& direction)
//       : origin(origin), direction(direction) {}

//   // Destructor to free the memory allocated
//   ~Ray() = default;

//   // The origin of the ray
//   // A
//   Point3D origin;
//   // The direction of the ray
//   // B
//   Vector3D direction;

//   // Get the point at a distance t along the ray (parameterized form)
//   // P(t) = A + t * B
//   Point3D at(double t) const { return origin + t * direction; }
// };