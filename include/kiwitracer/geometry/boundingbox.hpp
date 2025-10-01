
#pragma once

#include <algorithm>
#include "vector.hpp"
#include "tiny_obj_loader.h"
#include <vector>

namespace kiwitracer {
// Represents a two-dimensional (2D) bounding box

class BoundingBox {
  public:
    // Constructor to initialize memory
    BoundingBox(int xmin, int xmax, int ymin, int ymax)
      : xmin(xmin),
        xmax(xmax),
        ymin(ymin),
        ymax(ymax) {}

    // Destructor to free the memory allocated
    ~BoundingBox() = default;

    static BoundingBox calculateBoundingBox(const Vector2D& v1, const Vector2D& v2, const Vector2D& v3) {
      int xmin = std::min({v1.x, v2.x, v3.x});
      int xmax = std::max({v1.x, v2.x, v3.x});
      int ymin = std::min({v1.y, v2.y, v3.y});
      int ymax = std::max({v1.y, v2.y, v3.y});

      return BoundingBox(xmin, xmax, ymin, ymax);
    }

    static BoundingBox calculateBoundingBox(const std::vector<tinyobj::shape_t>& shapes) {
      int xmin = std::numeric_limits<int>::max();
      int xmax = std::numeric_limits<int>::min();
      int ymin = std::numeric_limits<int>::max();
      int ymax = std::numeric_limits<int>::min();

      for (const auto& shape : shapes) {
        for (size_t i = 0; i < shape.mesh.positions.size(); i++) {
          int x = shape.mesh.positions[i];
          int y = shape.mesh.positions[i + 1];
          xmin = std::min(xmin, x);
          xmax = std::max(xmax, x);
          ymin = std::min(ymin, y);
          ymax = std::max(ymax, y);
        }
      }

      return BoundingBox(xmin, xmax, ymin, ymax);
    }

    // The bounding box coordinates
    int xmin, xmax, ymin, ymax;
};
} // namespace kiwitracer
