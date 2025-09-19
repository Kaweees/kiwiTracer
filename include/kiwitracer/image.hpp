#pragma once

#include <string>
#include <vector>
#include <cassert>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace kiwitracer {

class Image {
  public:
    Image(int width, int height)
      : width(width),
        height(height),
        comp(3),
        pixels(width * height * comp, 0) {}

    virtual ~Image() = default;

    void setPixel(int x, int y, unsigned char r, unsigned char g, unsigned char b) {
      // First check for bounds
      if (y < 0 || y >= height) {
        std::cout << "Row " << y << " is out of bounds" << std::endl;
        return;
      }
      if (x < 0 || x >= width) {
        std::cout << "Col " << x << " is out of bounds" << std::endl;
        return;
      }

      // Since the origin (0, 0) of the image is the upper left corner, we need
      // to flip the row to make the origin be the lower left corner.
      y = height - y - 1;
      // index corresponding to row and col, (assuming single component image)
      int index = y * width + x;
      // Multiply by 3 to get the index for the rgb components.
      assert(index >= 0);
      assert(3 * index + 2 < (int)pixels.size());
      pixels[3 * index + 0] = r;
      pixels[3 * index + 1] = g;
      pixels[3 * index + 2] = b;
    }

    void writeToFile(const std::string& filename) {
      // The distance in bytes from the first byte of a row of pixels to the
      // first byte of the next row of pixels
      int stride_in_bytes = width * comp * sizeof(unsigned char);
      int rc = stbi_write_png(filename.c_str(), width, height, comp, pixels.data(), stride_in_bytes);
      if (rc) {
        std::cout << "Wrote to " << filename << std::endl;
      } else {
        std::cout << "Couldn't write to " << filename << std::endl;
      }
    }

    int getWidth() const { return width; }

    int getHeight() const { return height; }
  private:
    int width;
    int height;
    int comp;
    std::vector<unsigned char> pixels;
};

} // namespace kiwitracer
