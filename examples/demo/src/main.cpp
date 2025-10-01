#include <iostream>
#include <kiwitracer/kiwitracer.hpp>
#include <memory>
#include <lyra/lyra.hpp>
#include <iostream>
#include <filesystem>
#include <sys/types.h>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <assert.h>
#include "tiny_obj_loader.h"

namespace fs = std::filesystem;

/*
   Helper function you will want all quarter
   Given a vector of shapes which has already been read from an obj file
   resize all vertices to the range [-1, 1]
 */
void resize_obj(std::vector<tinyobj::shape_t>& shapes) {
  float minX, minY, minZ;
  float maxX, maxY, maxZ;
  float scaleX, scaleY, scaleZ;
  float shiftX, shiftY, shiftZ;
  float epsilon = 0.001;

  minX = minY = minZ = 1.1754E+38F;
  maxX = maxY = maxZ = -1.1754E+38F;

  // Go through all vertices to determine min and max of each dimension
  for (size_t i = 0; i < shapes.size(); i++) {
    for (size_t v = 0; v < shapes[i].mesh.positions.size() / 3; v++) {
      if (shapes[i].mesh.positions[3 * v + 0] < minX) minX = shapes[i].mesh.positions[3 * v + 0];
      if (shapes[i].mesh.positions[3 * v + 0] > maxX) maxX = shapes[i].mesh.positions[3 * v + 0];

      if (shapes[i].mesh.positions[3 * v + 1] < minY) minY = shapes[i].mesh.positions[3 * v + 1];
      if (shapes[i].mesh.positions[3 * v + 1] > maxY) maxY = shapes[i].mesh.positions[3 * v + 1];

      if (shapes[i].mesh.positions[3 * v + 2] < minZ) minZ = shapes[i].mesh.positions[3 * v + 2];
      if (shapes[i].mesh.positions[3 * v + 2] > maxZ) maxZ = shapes[i].mesh.positions[3 * v + 2];
    }
  }

  // From min and max compute necessary scale and shift for each dimension
  float maxExtent, xExtent, yExtent, zExtent;
  xExtent = maxX - minX;
  yExtent = maxY - minY;
  zExtent = maxZ - minZ;
  if (xExtent >= yExtent && xExtent >= zExtent) { maxExtent = xExtent; }
  if (yExtent >= xExtent && yExtent >= zExtent) { maxExtent = yExtent; }
  if (zExtent >= xExtent && zExtent >= yExtent) { maxExtent = zExtent; }
  scaleX = 2.0 / maxExtent;
  shiftX = minX + (xExtent / 2.0);
  scaleY = 2.0 / maxExtent;
  shiftY = minY + (yExtent / 2.0);
  scaleZ = 2.0 / maxExtent;
  shiftZ = minZ + (zExtent) / 2.0;

  // Go through all verticies shift and scale them
  for (size_t i = 0; i < shapes.size(); i++) {
    for (size_t v = 0; v < shapes[i].mesh.positions.size() / 3; v++) {
      shapes[i].mesh.positions[3 * v + 0] = (shapes[i].mesh.positions[3 * v + 0] - shiftX) * scaleX;
      assert(shapes[i].mesh.positions[3 * v + 0] >= -1.0 - epsilon);
      assert(shapes[i].mesh.positions[3 * v + 0] <= 1.0 + epsilon);
      shapes[i].mesh.positions[3 * v + 1] = (shapes[i].mesh.positions[3 * v + 1] - shiftY) * scaleY;
      assert(shapes[i].mesh.positions[3 * v + 1] >= -1.0 - epsilon);
      assert(shapes[i].mesh.positions[3 * v + 1] <= 1.0 + epsilon);
      shapes[i].mesh.positions[3 * v + 2] = (shapes[i].mesh.positions[3 * v + 2] - shiftZ) * scaleZ;
      assert(shapes[i].mesh.positions[3 * v + 2] >= -1.0 - epsilon);
      assert(shapes[i].mesh.positions[3 * v + 2] <= 1.0 + epsilon);
    }
  }
}

int main(int argc, char** argv) {
  fs::path meshfile, imagefile;
  int g_width = 100, g_height = 100;

  auto cli = lyra::cli() | lyra::arg(meshfile, "meshfile").required().help("Mesh file (e.g., foo.obj)") |
             lyra::arg(imagefile, "imagefile").required().help("Image file (e.g., foo.png)") |
             lyra::arg(g_width, "width").help("Image width (e.g., 512)") |
             lyra::arg(g_height, "height").help("Image height (e.g., 512)");

  auto result = cli.parse({argc, argv});
  if (!result) {
    std::cerr << result.message() << "\n";
    return 1;
  }

  std::cout << "Filename: " << imagefile << "\n";

  // Create the image. We're using a `shared_ptr`, a C++11 feature.
  auto image = std::make_shared<kiwitracer::Image>(g_width, g_height);

  // triangle buffer
  std::vector<unsigned int> triBuf;
  // position buffer
  std::vector<float> posBuf;
  // Some obj files contain material information.
  // We'll ignore them for this assignment.
  std::vector<tinyobj::shape_t> shapes; // geometry
  std::vector<tinyobj::material_t> objMaterials; // material
  std::string errStr;

  bool rc = tinyobj::LoadObj(shapes, objMaterials, errStr, meshfile.c_str());
  /* error checking on read */
  if (!rc) {
    std::cerr << errStr << std::endl;
    return 1;
  } else {
    // keep this code to resize your object to be within -1 -> 1
    resize_obj(shapes);
    posBuf = shapes[0].mesh.positions;
    triBuf = shapes[0].mesh.indices;
  }
  std::cout << "Number of vertices: " << posBuf.size() / 3 << std::endl;
  std::cout << "Number of triangles: " << triBuf.size() / 3 << std::endl;

  // Process each triangle
  for (size_t i = 0; i < triBuf.size(); i += 3) {
    // Get the three vertex indices for this triangle
    unsigned int idx0 = triBuf[i];
    unsigned int idx1 = triBuf[i + 1];
    unsigned int idx2 = triBuf[i + 2];

    std::cout << "Triangle " << i / 3 << ": indices " << idx0 << ", " << idx1 << ", " << idx2 << std::endl;

    // Get vertex positions and convert to screen coordinates
    // Map from [-1, 1] to [0, image_size-1]
    int vax = (int)((posBuf[3 * idx0] + 1.0f) * 0.5f * (g_width - 1));
    int vay = (int)((posBuf[3 * idx0 + 1] + 1.0f) * 0.5f * (g_height - 1));
    int vbx = (int)((posBuf[3 * idx1] + 1.0f) * 0.5f * (g_width - 1));
    int vby = (int)((posBuf[3 * idx1 + 1] + 1.0f) * 0.5f * (g_height - 1));
    int vcx = (int)((posBuf[3 * idx2] + 1.0f) * 0.5f * (g_width - 1));
    int vcy = (int)((posBuf[3 * idx2 + 1] + 1.0f) * 0.5f * (g_height - 1));

    std::cout << "Vertices: (" << vax << "," << vay << "), (" << vbx << "," << vby << "), (" << vcx << "," << vcy << ")"
              << std::endl;

    // Calculate bounding box for this triangle
    int xmin = std::max(0, std::min({vax, vbx, vcx}));
    int xmax = std::min(g_width - 1, std::max({vax, vbx, vcx}));
    int ymin = std::max(0, std::min({vay, vby, vcy}));
    int ymax = std::min(g_height - 1, std::max({vay, vby, vcy}));

    // Draw the triangle using barycentric coordinates
    for (int y = ymin; y <= ymax; ++y) {
      for (int x = xmin; x <= xmax; ++x) {
        unsigned char r = 0;
        unsigned char g = 0;
        unsigned char b = 0;

        // Compute the barycentric coordinates
        float denom = (float)((vby - vcy) * (vax - vcx) + (vcx - vbx) * (vay - vcy));
        if (std::abs(denom) < 1e-6f) continue; // Skip degenerate triangles

        float u = ((vby - vcy) * (x - vcx) + (vcx - vbx) * (y - vcy)) / denom;
        float v = ((vcx - vax) * (y - vcy) + (vax - vcx) * (x - vcx)) / denom;
        float w = 1.0f - u - v;

        // If the point is inside the triangle, use the color of the triangle
        if (u >= 0 && v >= 0 && w >= 0) {
          r = (unsigned char)(u * 255);
          g = (unsigned char)(v * 255);
          b = (unsigned char)(w * 255);
          image->setPixel(x, y, r, g, b);
        }
      }
    }

    // Draw the three vertices as different colored pixels
    image->setPixel(vax, vay, 255, 0, 0); // Red for vertex A
    image->setPixel(vbx, vby, 0, 255, 0); // Green for vertex B
    image->setPixel(vcx, vcy, 0, 0, 255); // Blue for vertex C
  }

  // Write image to file
  image->writeToFile(imagefile);

  return 0;
}
