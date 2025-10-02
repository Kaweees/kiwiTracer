#pragma once

namespace kiwitracer {
// Represents a three-dimensional (3D) vertex position, color, and depth information
struct Vertex {
    // Constructor to initialize memory
    Vertex(float x = 0, float y = 0, float z = 0, float r = 0, float g = 0, float b = 0, float depth = 0)
      : x(x),
        y(y),
        z(z),
        r(r),
        g(g),
        b(b),
        depth(depth) {}

    // Destructor to free the memory allocated
    ~Vertex() = default;

    // Convert 3D world coordinates to 2D window coordinates
    Vertex worldToWindow(int width, int height) {
      Vertex window;

      // Project to 2D (orthographic projection looking down -z)
      window.x = (this->x + 1.0f) * 0.5f * width;
      window.y = (this->y + 1.0f) * 0.5f * height;
      window.z = this->z; // Keep z for depth testing
      window.depth = this->z;

      // Set default color (will be overridden by color modes)
      window.r = window.g = window.b = 0.5f;

      return window;
    }

    // Get the dot product of two vectors
    inline float dot(const Vertex& v) { return x * v.x + y * v.y + z * v.z; }

    // Overload the subtraction operator
    inline Vertex operator-(const Vertex& v) { return Vertex(x - v.x, y - v.y, z - v.z); }

    // Position in 3D space
    float x, y, z;
    // Color components (RGB)
    float r, g, b;
    // Depth value
    float depth;
};
} // namespace kiwitracer
