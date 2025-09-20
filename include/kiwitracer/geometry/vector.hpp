#pragma once

namespace kiwitracer {
// Represents a two-dimensional (2D) vector
class Vector2D {
  public:
    // Constructor to initialize memory
    Vector2D()
      : x(0),
        y(0) {}

    Vector2D(int x = 0, int y = 0)
      : x(x),
        y(y) {}

    // Destructor to free the memory allocated
    ~Vector2D() = default;
  private:
    // The x-coordinate of the vector
    int x;
    // The y-coordinate of the vector
    int y;
};
} // namespace kiwitracer
