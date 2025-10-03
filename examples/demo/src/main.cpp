#include <iostream>
#include <kiwitracer/kiwitracer.hpp>
#include <memory>
#include <lyra/lyra.hpp>
#include <filesystem>
#include <sys/types.h>
#include <string>
#include <vector>
#include <assert.h>
#include <limits>
#include <algorithm>
#include <cmath>
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

// Color mode 1: Depth-based coloring (closer = brighter)
void setDepthColors(std::vector<kiwitracer::Vertex>& vertices, float minZ, float maxZ) {
  float denom = (maxZ - minZ <= 0.0f) ? 1.0f : (maxZ - minZ);

  // Normalize z values and set colors (closer = brighter red)
  for (auto& v : vertices) {
    float zNormalized = (v.z - minZ) / denom;
    float brightness = 1.0f - zNormalized; // invert so closer (smaller z) is brighter
    v.r = brightness;
    v.g = 0.0f;
    v.b = 0.0f;
  }
}

// Rasterize a single triangle
void rasterizeTriangle(kiwitracer::Triangle& triangle, std::shared_ptr<kiwitracer::Image> image,
                       std::vector<std::vector<float>>& zBuffer, int colorMode) {
  // Calculate bounding box for this triangle
  triangle.computeBoundingBox(image->getWidth(), image->getHeight());

  // Clamp bounding box to image bounds
  int xmin = std::max(0, (int)std::floor(triangle.xmin));
  int xmax = std::min(image->getWidth() - 1, (int)std::ceil(triangle.xmax));
  int ymin = std::max(0, (int)std::floor(triangle.ymin));
  int ymax = std::min(image->getHeight() - 1, (int)std::ceil(triangle.ymax));

  // Rasterize the triangle
  for (int y = ymin; y <= ymax; y++) {
    for (int x = xmin; x <= xmax; x++) {
      // compute the barycentric coordinates
      float u = 0, v = 0, w = 0;

      // Check if point is inside triangle
      if (triangle.barycentric(kiwitracer::Vertex(x + 0.5f, y + 0.5f, 0), u, v, w)) {
        printf("Barycentric coordinates: (%f, %f, %f)\n", u, v, w);
        // Interpolate vertex attributes
        kiwitracer::Vertex interpolated = triangle.interpolateVertex(triangle.v0, triangle.v1, triangle.v2, u, v, w);
        printf("Interpolated vertex: (%f, %f, %f, %f, %f, %f, %f)\n", interpolated.x, interpolated.y, interpolated.z,
               interpolated.r, interpolated.g, interpolated.b, interpolated.depth);

        // Z-buffer test: only draw if this pixel is closer than what's in the buffer
        if (interpolated.depth < zBuffer[y][x]) {
          // Update the z-buffer
          zBuffer[y][x] = interpolated.depth;

          unsigned char r, g, b;
          if (colorMode == 2) {
            float minw = std::min(u, std::min(v, w));
            if (u >= 0.2f && v >= 0.2f && w >= 0.2f) {
              r = 255;
              g = 0;
              b = 0; // red core
            } else if (minw >= 0.05f && minw <= 0.2f) {
              r = 0;
              g = 0;
              b = 255; // blue soft border
            } else {
              r = 0;
              g = 255;
              b = 0; // green elsewhere
            }
          } else {
            // Clamp colors to [0, 1] and convert to [0, 255]
            r = (unsigned char)(std::max(0.0f, std::min(1.0f, interpolated.r)) * 255);
            g = (unsigned char)(std::max(0.0f, std::min(1.0f, interpolated.g)) * 255);
            b = (unsigned char)(std::max(0.0f, std::min(1.0f, interpolated.b)) * 255);
          }

          printf("Pixel: (%d, %d, %d)\n", r, g, b);
          image->setPixel(x, y, r, g, b);
        }
      }
    }
  }
}

int main(int argc, char** argv) {
  fs::path meshfile, imagefile;
  int g_width = 100, g_height = 100, colorMode = 1;

  auto cli =
      lyra::cli() | lyra::arg(meshfile, "meshfile").required().help("Mesh file (e.g., foo.obj)") |
      lyra::arg(imagefile, "imagefile").required().help("Image file (e.g., foo.png)") |
      lyra::arg(g_width, "width").help("Image width (e.g., 512)") |
      lyra::arg(g_height, "height").help("Image height (e.g., 512)") |
      lyra::arg(colorMode, "mode").help("Coloring mode: 1 (depth) or 2 (binned distance) or 3 (barycentric regions)");

  auto result = cli.parse({argc, argv});
  if (!result) {
    std::cerr << result.message() << "\n";
    return 1;
  }

  // Validate color mode
  if (colorMode < 1 || colorMode > 3) {
    std::cerr << "Color mode must be 1, 2 or 3\n";
    return 1;
  }

  // Create the image
  auto image = std::make_shared<kiwitracer::Image>(g_width, g_height);

  // Create z-buffer (initialized to very large values - far away from camera)
  std::vector<std::vector<float>> zbuffer(g_height, std::vector<float>(g_width, std::numeric_limits<float>::max()));

  // triangle buffer
  std::vector<unsigned int> triBuf;
  // position buffer
  std::vector<float> posBuf;
  // Some obj files contain material information.
  std::vector<tinyobj::shape_t> shapes; // geometry
  std::vector<tinyobj::material_t> objMaterials; // material
  std::string errStr;

  bool rc = tinyobj::LoadObj(shapes, objMaterials, errStr, meshfile.c_str());
  /* error checking on read */
  if (!rc) {
    std::cerr << "Error loading mesh: " << errStr << std::endl;
    return 1;
  }

  if (shapes.empty() || shapes[0].mesh.positions.empty()) {
    std::cerr << "Error: Mesh file contains no geometry\n";
    return 1;
  }

  // Resize object to be within -1 -> 1
  resize_obj(shapes);
  posBuf = shapes[0].mesh.positions;
  triBuf = shapes[0].mesh.indices;

  // Find min and max z values for normalization
  float minZ = std::numeric_limits<float>::max();
  float maxZ = std::numeric_limits<float>::min();

  std::cout << "Number of vertices: " << posBuf.size() / 3 << std::endl;
  std::cout << "Number of triangles: " << triBuf.size() / 3 << std::endl;

  // Convert vertices to window coordinates
  std::vector<kiwitracer::Vertex> vertices;
  for (size_t i = 0; i < posBuf.size(); i += 3) {
    kiwitracer::Vertex world(posBuf[i], posBuf[i + 1], posBuf[i + 2], 0, 0, 0);
    kiwitracer::Vertex window = world.worldToWindow(g_width, g_height);
    vertices.push_back(window);
    minZ = std::min(minZ, window.z);
    maxZ = std::max(maxZ, window.z);
  }

  // Apply color mode
  if (colorMode == 1) {
    setDepthColors(vertices, minZ, maxZ);
  } else if (colorMode == 3) {
    // Per-pixel barycentric region coloring; no per-vertex setup needed
  }

  // Rasterize each triangle
  for (size_t i = 0; i < triBuf.size(); i += 3) {
    // Get the three vertex indices for this triangle
    unsigned int idx0 = triBuf[i];
    unsigned int idx1 = triBuf[i + 1];
    unsigned int idx2 = triBuf[i + 2];

    kiwitracer::Triangle tri(vertices[idx0], vertices[idx1], vertices[idx2]);
    rasterizeTriangle(tri, image, zbuffer, colorMode);
  }

  // write out the image
  image->writeToFile(imagefile);

  return 0;
}
