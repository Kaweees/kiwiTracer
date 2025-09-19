#include <raylib.h>
#include <stddef.h>
#include <stdio.h>

#include <iostream>

#include "../include/color.hpp"
#include "../include/ray.hpp"
#include "../include/vector.hpp"
#include "raymath.h"
#include "rlgl.h"

// Helper function to return the color for a given ray
ColorRGB ray_color(const Ray3D& r) { return ColorRGB(0, 0, 0); }

//------------------------------------------------------------------------------------
// Program main entry point
//------------------------------------------------------------------------------------
int main(int argc, char* argv[]) {
  int image_width = 256;
  int image_height = 256;

  // Render

  std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";
  for (int j = 0; j < image_height; ++j) {
    std::clog << "\rScanlines remaining: " << (image_height - j) << ' ' << std::flush;
    for (int i = 0; i < image_width; i++) {
      auto pixel_color = ColorRGB(double(i) / (image_width - 1), double(j) / (image_height - 1), 0);
      write_color(std::cout, pixel_color);
    }
  }
  std::clog << "\rDone.                 \n";
}