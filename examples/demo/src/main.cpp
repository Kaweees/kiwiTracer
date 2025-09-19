#include <iostream>
#include <kiwitracer/kiwitracer.hpp>
#include <memory>
#include <lyra/lyra.hpp>
#include <iostream>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  fs::path filename;
  int width = 0, height = 0;
  int vax = 0, vay = 0, vbx = 0, vby = 0, vcx = 0, vcy = 0;

  // auto cli = lyra::cli() | lyra::arg(input_file, "file").required().help("Path to an input file") |
  //            lyra::opt(verbose)["-v"]["--verbose"]("Enable verbose mode");

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
  std::cout << "Filename: " << filename << "\n";
  std::cout << "Width: " << width << ", Height: " << height << "\n";
  std::cout << "Vertex A: (" << vax << ", " << vay << ")\n";
  std::cout << "Vertex B: (" << vbx << ", " << vby << ")\n";
  std::cout << "Vertex C: (" << vcx << ", " << vcy << ")\n";

  // Create the image. We're using a `shared_ptr`, a C++11 feature.
  auto image = std::make_shared<kiwitracer::Image>(width, height);

  image->setPixel(vax, vay, 255, 0, 0);
  image->setPixel(vbx, vby, 0, 255, 0);
  image->setPixel(vcx, vcy, 0, 0, 255);

  // Draw a rectangle
  for (int y = 10; y < 20; ++y) {
    for (int x = 20; x < 40; ++x) {
      unsigned char r = 255;
      unsigned char g = 0;
      unsigned char b = 0;
      image->setPixel(x, y, r, g, b);
    }
  }
  // Write image to file
  image->writeToFile(filename);
}
