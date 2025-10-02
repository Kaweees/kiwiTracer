#pragma once

namespace kiwitracer {
// Represents a three-dimensional (3D) vertex position, color, and depth information
struct Vertex {
    // Constructor to initialize memory
    Vertex()
      : x(0),
        y(0),
        z(0),
        r(0),
        g(0),
        b(0),
        depth(0) {}

    Vertex(float x = 0, float y = 0, float z = 0, float r = 0, float g = 0, float b = 0)
      : x(x),
        y(y),
        z(z),
        r(r),
        g(g),
        b(b),
        depth(z) {}

    // Destructor to free the memory allocated
    ~Vertex() = default;

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
