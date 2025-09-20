#pragma once

#include "vector.hpp"

namespace kiwitracer {
// Represents a two-dimensional (2D) triangle

class Triangle {
  public:
    // Constructor to initialize memory
    Triangle(Vector2D v1, Vector2D v2, Vector2D v3)
      : v1(v1),
        v2(v2),
        v3(v3) {}

    // Destructor to free the memory allocated
    ~Triangle() = default;

    // The vertices of the triangle
    Vector2D v1, v2, v3;
};
} // namespace kiwitracer
