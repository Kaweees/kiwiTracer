
#pragma once

#include <algorithm>
#include "vector.hpp"

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

    // The bounding box coordinates
    int xmin, xmax, ymin, ymax;
};
} // namespace kiwitracer
