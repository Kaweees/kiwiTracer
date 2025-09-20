#include <iostream>
#include <kiwitracer/kiwitracer.hpp>
#include <memory>
#include <lyra/lyra.hpp>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <algorithm>

namespace fs = std::filesystem;

// Data structure to represent a 2D point
struct Point2D {
    int x, y;

    Point2D(int x = 0, int y = 0)
      : x(x),
        y(y) {}
};

// Data structure to represent a triangle
struct Triangle {
    Point2D v1, v2, v3;

    Triangle(Point2D v1, Point2D v2, Point2D v3)
      : v1(v1),
        v2(v2),
        v3(v3) {}
};

// Data structure to represent a bounding box
struct BoundingBox {
    int xmin, xmax, ymin, ymax;

    BoundingBox(int xmin, int xmax, int ymin, int ymax)
      : xmin(xmin),
        xmax(xmax),
        ymin(ymin),
        ymax(ymax) {}
};

// Function to calculate bounding box from triangle vertices
BoundingBox calculateBoundingBox(const Triangle& triangle) {
  int xmin = std::min({triangle.v1.x, triangle.v2.x, triangle.v3.x});
  int xmax = std::max({triangle.v1.x, triangle.v2.x, triangle.v3.x});
  int ymin = std::min({triangle.v1.y, triangle.v2.y, triangle.v3.y});
  int ymax = std::max({triangle.v1.y, triangle.v2.y, triangle.v3.y});

  return BoundingBox(xmin, xmax, ymin, ymax);
}

int main(int argc, char** argv) {
  fs::path filename;
  int width = 0, height = 0;
  int vax = 0, vay = 0, vbx = 0, vby = 0, vcx = 0, vcy = 0;

  auto cli = lyra::cli() | lyra::arg(filename, "filename").required().help("Output filename (e.g., foo.png)") |
             lyra::arg(width, "width").required().help("Image width (e.g., 512)") |
             lyra::arg(height, "height").required().help("Image height (e.g., 512)") |
             lyra::arg(vax, "vax").required().help("Vertex 1 x-coordinate (e.g., 100)") |
             lyra::arg(vay, "vay").required().help("Vertex 1 y-coordinate (e.g., 100)") |
             lyra::arg(vbx, "vbx").required().help("Vertex 2 x-coordinate") |
             lyra::arg(vby, "vby").required().help("Vertex 2 y-coordinate") |
             lyra::arg(vcx, "vcx").required().help("Vertex 3 x-coordinate") |
             lyra::arg(vcy, "vcy").required().help("Vertex 3 y-coordinate");

  auto result = cli.parse({argc, argv});
  if (!result) {
    std::cerr << result.message() << "\n";
    return 1;
  }

  // Create triangle from input vertices
  Triangle triangle(Point2D(vax, vay), Point2D(vbx, vby), Point2D(vcx, vcy));

  // Calculate bounding box
  BoundingBox bbox = calculateBoundingBox(triangle);

  std::cout << "Filename: " << filename << "\n";
  std::cout << "Width: " << width << ", Height: " << height << "\n";
  std::cout << "Vertex A: (" << vax << ", " << vay << ")\n";
  std::cout << "Vertex B: (" << vbx << ", " << vby << ")\n";
  std::cout << "Vertex C: (" << vcx << ", " << vcy << ")\n";
  std::cout << "Bounding Box: (" << bbox.xmin << ", " << bbox.ymin << ") to (" << bbox.xmax << ", " << bbox.ymax
            << ")\n";

  // Create the image. We're using a `shared_ptr`, a C++11 feature.
  auto image = std::make_shared<kiwitracer::Image>(width, height);

  // Draw the bounding box
  for (int y = bbox.ymin; y <= bbox.ymax; ++y) {
    for (int x = bbox.xmin; x <= bbox.xmax; ++x) {
      // Use a different color for the bounding box (yellow)
      unsigned char r = 255;
      unsigned char g = 255;
      unsigned char b = 0;
      image->setPixel(x, y, r, g, b);
    }
  }

  // Draw the three vertices as different colored pixels
  image->setPixel(vax, vay, 255, 0, 0); // Red for vertex A
  image->setPixel(vbx, vby, 0, 255, 0); // Green for vertex B
  image->setPixel(vcx, vcy, 0, 0, 255); // Blue for vertex C

  // Write image to file
  image->writeToFile(filename);
}
