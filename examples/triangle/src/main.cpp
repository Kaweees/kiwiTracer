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
#include "Image.h"

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
  kiwitracer::Triangle triangle(kiwitracer::Vector2D(vax, vay), kiwitracer::Vector2D(vbx, vby),
                                kiwitracer::Vector2D(vcx, vcy));

  // Calculate bounding box
  kiwitracer::BoundingBox bbox = kiwitracer::BoundingBox::calculateBoundingBox(triangle.v1, triangle.v2, triangle.v3);

  std::cout << "Filename: " << filename << "\n";
  std::cout << "Width: " << g_width << ", Height: " << g_height << "\n";
  std::cout << "Vertex A: (" << vax << ", " << vay << ")\n";
  std::cout << "Vertex B: (" << vbx << ", " << vby << ")\n";
  std::cout << "Vertex C: (" << vcx << ", " << vcy << ")\n";
  std::cout << "Bounding Box: (" << bbox.xmin << ", " << bbox.ymin << ") to (" << bbox.xmax << ", " << bbox.ymax
            << ")\n";

  // Create the image. We're using a `shared_ptr`, a C++11 feature.
  auto image = std::make_shared<kiwitracer::Image>(g_width, g_height);

  // Draw the bounding box
  for (float y = bbox.ymin; y <= bbox.ymax; ++y) {
    for (float x = bbox.xmin; x <= bbox.xmax; ++x) {
      unsigned char r = 0;
      unsigned char g = 0;
      unsigned char b = 0;
      // compute the barycentric coordinates
      float u =
          ((vby - vcy) * (x - vcx) + (vcx - vbx) * (y - vcy)) / ((vby - vcy) * (vax - vcx) + (vcx - vbx) * (vay - vcy));
      float v =
          ((vbx - vax) * (y - vby) + (vay - vby) * (x - vbx)) / ((vbx - vax) * (vcy - vby) + (vay - vby) * (vcx - vbx));
      float w = 1.0f - u - v;
      // if the point is inside the triangle, use the color of the triangle
      if (u >= 0 && u <= 1 && v >= 0 && v <= 1 && w >= 0 && w <= 1) {
        r = u * 255;
        g = v * 255;
        b = w * 255;
      }

      if (x == bbox.xmin || x == bbox.xmax || y == bbox.ymin || y == bbox.ymax) {
        // Use a different color for the bounding box (yellow)
        r = 255;
        g = 255;
        b = 0;
      }

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

// /*
//    Helper function you will want all quarter
//    Given a vector of shapes which has already been read from an obj file
//    resize all vertices to the range [-1, 1]
//  */
// void resize_obj(std::vector<tinyobj::shape_t>& shapes) {
//   float minX, minY, minZ;
//   float maxX, maxY, maxZ;
//   float scaleX, scaleY, scaleZ;
//   float shiftX, shiftY, shiftZ;
//   float epsilon = 0.001;

//   minX = minY = minZ = 1.1754E+38F;
//   maxX = maxY = maxZ = -1.1754E+38F;

//   // Go through all vertices to determine min and max of each dimension
//   for (size_t i = 0; i < shapes.size(); i++) {
//     for (size_t v = 0; v < shapes[i].mesh.positions.size() / 3; v++) {
//       if (shapes[i].mesh.positions[3 * v + 0] < minX) minX = shapes[i].mesh.positions[3 * v + 0];
//       if (shapes[i].mesh.positions[3 * v + 0] > maxX) maxX = shapes[i].mesh.positions[3 * v + 0];

//       if (shapes[i].mesh.positions[3 * v + 1] < minY) minY = shapes[i].mesh.positions[3 * v + 1];
//       if (shapes[i].mesh.positions[3 * v + 1] > maxY) maxY = shapes[i].mesh.positions[3 * v + 1];

//       if (shapes[i].mesh.positions[3 * v + 2] < minZ) minZ = shapes[i].mesh.positions[3 * v + 2];
//       if (shapes[i].mesh.positions[3 * v + 2] > maxZ) maxZ = shapes[i].mesh.positions[3 * v + 2];
//     }
//   }

//   // From min and max compute necessary scale and shift for each dimension
//   float maxExtent, xExtent, yExtent, zExtent;
//   xExtent = maxX - minX;
//   yExtent = maxY - minY;
//   zExtent = maxZ - minZ;
//   if (xExtent >= yExtent && xExtent >= zExtent) { maxExtent = xExtent; }
//   if (yExtent >= xExtent && yExtent >= zExtent) { maxExtent = yExtent; }
//   if (zExtent >= xExtent && zExtent >= yExtent) { maxExtent = zExtent; }
//   scaleX = 2.0 / maxExtent;
//   shiftX = minX + (xExtent / 2.0);
//   scaleY = 2.0 / maxExtent;
//   shiftY = minY + (yExtent / 2.0);
//   scaleZ = 2.0 / maxExtent;
//   shiftZ = minZ + (zExtent) / 2.0;

//   // Go through all verticies shift and scale them
//   for (size_t i = 0; i < shapes.size(); i++) {
//     for (size_t v = 0; v < shapes[i].mesh.positions.size() / 3; v++) {
//       shapes[i].mesh.positions[3 * v + 0] = (shapes[i].mesh.positions[3 * v + 0] - shiftX) * scaleX;
//       assert(shapes[i].mesh.positions[3 * v + 0] >= -1.0 - epsilon);
//       assert(shapes[i].mesh.positions[3 * v + 0] <= 1.0 + epsilon);
//       shapes[i].mesh.positions[3 * v + 1] = (shapes[i].mesh.positions[3 * v + 1] - shiftY) * scaleY;
//       assert(shapes[i].mesh.positions[3 * v + 1] >= -1.0 - epsilon);
//       assert(shapes[i].mesh.positions[3 * v + 1] <= 1.0 + epsilon);
//       shapes[i].mesh.positions[3 * v + 2] = (shapes[i].mesh.positions[3 * v + 2] - shiftZ) * scaleZ;
//       assert(shapes[i].mesh.positions[3 * v + 2] >= -1.0 - epsilon);
//       assert(shapes[i].mesh.positions[3 * v + 2] <= 1.0 + epsilon);
//     }
//   }
// }

// int main(int argc, char** argv) {
//   if (argc < 3) {
//     cout << "Usage: raster meshfile imagefile" << endl;
//     return 0;
//   }
//   // OBJ filename
//   string meshName(argv[1]);
//   string imgName(argv[2]);

//   // set g_width and g_height appropriately!
//   g_width = g_height = 100;

//   // create an image
//   auto image = make_shared<Image>(g_width, g_height);

//   // triangle buffer
//   vector<unsigned int> triBuf;
//   // position buffer
//   vector<float> posBuf;
//   // Some obj files contain material information.
//   // We'll ignore them for this assignment.
//   vector<tinyobj::shape_t> shapes; // geometry
//   vector<tinyobj::material_t> objMaterials; // material
//   string errStr;

//   bool rc = tinyobj::LoadObj(shapes, objMaterials, errStr, meshName.c_str());
//   /* error checking on read */
//   if (!rc) {
//     cerr << errStr << endl;
//   } else {
//     // keep this code to resize your object to be within -1 -> 1
//     resize_obj(shapes);
//     posBuf = shapes[0].mesh.positions;
//     triBuf = shapes[0].mesh.indices;
//   }
//   cout << "Number of vertices: " << posBuf.size() / 3 << endl;
//   cout << "Number of triangles: " << triBuf.size() / 3 << endl;

//   // TODO add code to iterate through each triangle and rasterize it

//   // write out the image
//   image->writeToFile(imgName);

//   return 0;
// }
