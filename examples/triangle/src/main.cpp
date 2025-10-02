#include <iostream>
#include <kiwitracer/kiwitracer.hpp>
#include <memory>
#include <lyra/lyra.hpp>
#include <iostream>
#include <filesystem>
#include <sys/types.h>
#include <iostream>
#include <string>
#include <memory>
#include <assert.h>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  fs::path filename;
  int g_width = 0, g_height = 0;
  int vax = 0, vay = 0, vbx = 0, vby = 0, vcx = 0, vcy = 0;

  auto cli = lyra::cli() | lyra::arg(filename, "filename").required().help("Output filename (e.g., foo.png)") |
             lyra::arg(g_width, "width").required().help("Image width (e.g., 512)") |
             lyra::arg(g_height, "height").required().help("Image height (e.g., 512)") |
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
  kiwitracer::Triangle triangle(kiwitracer::Vertex(vax, vay, 0, 255, 0, 0), kiwitracer::Vertex(vbx, vby, 0, 0, 255, 0),
                                kiwitracer::Vertex(vcx, vcy, 0, 0, 0, 255), g_width, g_height);

  std::cout << "Filename: " << filename << "\n";
  std::cout << "Width: " << g_width << ", Height: " << g_height << "\n";
  std::cout << "Vertex A: (" << vax << ", " << vay << ")\n";
  std::cout << "Vertex B: (" << vbx << ", " << vby << ")\n";
  std::cout << "Vertex C: (" << vcx << ", " << vcy << ")\n";
  std::cout << "Bounding Box: (" << triangle.xmin << ", " << triangle.ymin << ") to (" << triangle.xmax << ", "
            << triangle.ymax << ")\n";

  // Create the image. We're using a `shared_ptr`, a C++11 feature.
  auto image = std::make_shared<kiwitracer::Image>(g_width, g_height);

  // Draw the bounding box
  for (float y = triangle.ymin; y <= triangle.ymax; ++y) {
    for (float x = triangle.xmin; x <= triangle.xmax; ++x) {
      unsigned char r = 0, g = 0, b = 0;

      // compute the barycentric coordinates
      float u = 0, v = 0, w = 0;
      triangle.barycentric(kiwitracer::Vertex(x, y, 0), u, v, w);

      // if the point is inside the triangle, use the color of the triangle
      if (u >= 0 && u <= 1 && v >= 0 && v <= 1 && w >= 0 && w <= 1) {
        r = u * 255;
        g = v * 255;
        b = w * 255;
      }
      if (x == triangle.xmin || x == triangle.xmax || y == triangle.ymin || y == triangle.ymax) {
        // Use a different color for the bounding box (yellow)
        r = 255;
        g = 255;
        b = 0;
      }

      // Set the pixel color
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
