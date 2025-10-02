#pragma once

#include "vertex.hpp"
#include <algorithm>
#include <cmath>

namespace kiwitracer {
// Represents a two-dimensional (2D) triangle

struct Triangle {
    // Constructor to initialize memory
    Triangle(const Vertex& v0, const Vertex& v1, const Vertex& v2, int g_width, int g_height)
      : v0(v0),
        v1(v1),
        v2(v2) {
      computeBoundingBox(g_width, g_height);
    }

    // Destructor to free the memory allocated
    ~Triangle() = default;

    void computeBoundingBox(int g_width, int g_height) {
      xmin = (int)std::floor(std::min({v0.x, v1.x, v2.x}));
      xmax = (int)std::ceil(std::max({v0.x, v1.x, v2.x}));
      ymin = (int)std::floor(std::min({v0.y, v1.y, v2.y}));
      ymax = (int)std::ceil(std::max({v0.y, v1.y, v2.y}));

      // Clamp to image bounds
      xmin = std::max(0, xmin);
      xmax = std::min(g_width - 1, xmax);
      ymin = std::max(0, ymin);
      ymax = std::min(g_height - 1, ymax);
    }

    // Compute barycentric coordinates (u, v, w) for point p with respect to triangle
    bool barycentric(Vertex p, float& u, float& v, float& w) {
      Vertex v0 = v1 - this->v0, v1 = v2 - this->v0, v2 = p - this->v0;
      float d00 = v0.dot(v0);
      float d01 = v0.dot(v1);
      float d11 = v1.dot(v1);
      float d20 = v2.dot(v0);
      float d21 = v2.dot(v1);
      float denom = d00 * d11 - d01 * d01;
      v = (d11 * d20 - d01 * d21) / denom;
      w = (d00 * d21 - d01 * d20) / denom;
      u = 1.0f - v - w;

      return (u >= 0 && v >= 0 && w >= 0);
    }

    // The vertices of the triangle
    Vertex v0, v1, v2;

    // The bounding box coordinates
    int xmin, xmax, ymin, ymax;
};
} // namespace kiwitracer
