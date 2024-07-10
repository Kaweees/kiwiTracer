#include "../include/vector.hpp"

// Type alias for ColorRGB
using ColorRGB = Vector3D;

// Write a pixel's color to the standard output stream

inline void write_color(std::ostream& out, const ColorRGB& pixel_color) {
  auto r = pixel_color.x;
  auto g = pixel_color.y;
  auto b = pixel_color.z;

  // Translate the [0,1] component values to the byte range [0,255].
  int rbyte = int(255.999 * r);
  int gbyte = int(255.999 * g);
  int bbyte = int(255.999 * b);

  // Write out the pixel color components.
  out << rbyte << ' ' << gbyte << ' ' << bbyte << '\n';
}
