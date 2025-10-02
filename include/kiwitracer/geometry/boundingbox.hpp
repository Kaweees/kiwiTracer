
#pragma once

#include <algorithm>
#include <limits>
#include "vector.hpp"
#include "tiny_obj_loader.h"
#include <vector>

namespace kiwitracer {
// Represents a two-dimensional (2D) bounding box

class BoundingBox {
  public:
    // Constructor to initialize memory
    BoundingBox(float xmin, float xmax, float ymin, float ymax)
      : xmin(xmin),
        xmax(xmax),
        ymin(ymin),
        ymax(ymax) {}

    // Destructor to free the memory allocated
    ~BoundingBox() = default;

    static BoundingBox calculateBoundingBox(const Vector2D& v1, const Vector2D& v2, const Vector2D& v3) {
      float xmin = std::min({(float)v1.x, (float)v2.x, (float)v3.x});
      float xmax = std::max({(float)v1.x, (float)v2.x, (float)v3.x});
      float ymin = std::min({(float)v1.y, (float)v2.y, (float)v3.y});
      float ymax = std::max({(float)v1.y, (float)v2.y, (float)v3.y});

      return BoundingBox(xmin, xmax, ymin, ymax);
    }

    static BoundingBox calculateBoundingBox(const std::vector<tinyobj::shape_t>& shapes) {
      float xmin = std::numeric_limits<float>::max();
      float xmax = std::numeric_limits<float>::lowest();
      float ymin = std::numeric_limits<float>::max();
      float ymax = std::numeric_limits<float>::lowest();

      for (const auto& shape : shapes) {
        // Step by 3 since positions are stored as [x, y, z, x, y, z, ...]
        for (size_t i = 0; i < shape.mesh.positions.size(); i += 3) {
          float x = shape.mesh.positions[i];
          float y = shape.mesh.positions[i + 1];
          xmin = std::min(xmin, x);
          xmax = std::max(xmax, x);
          ymin = std::min(ymin, y);
          ymax = std::max(ymax, y);
        }
      }

      return BoundingBox(xmin, xmax, ymin, ymax);
    }

    // Calculate bounding box for three screen coordinates
    static BoundingBox calculateBoundingBox(float x1, float y1, float x2, float y2, float x3, float y3) {
      float xmin = std::min({x1, x2, x3});
      float xmax = std::max({x1, x2, x3});
      float ymin = std::min({y1, y2, y3});
      float ymax = std::max({y1, y2, y3});

      return BoundingBox(xmin, xmax, ymin, ymax);
    }

    // The bounding box coordinates
    float xmin, xmax, ymin, ymax;
};
} // namespace kiwitracer
