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

// Rasterize a single triangle
void rasterizeTriangle(kiwitracer::Triangle& triangle, std::shared_ptr<kiwitracer::Image> image,
                       std::vector<std::vector<float>>& zBuffer) {
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

          // Clamp colors to [0, 1] and convert to [0, 255]
          unsigned char r = (unsigned char)(std::max(0.0f, std::min(1.0f, interpolated.r)) * 255);
          unsigned char g = (unsigned char)(std::max(0.0f, std::min(1.0f, interpolated.g)) * 255);
          unsigned char b = (unsigned char)(std::max(0.0f, std::min(1.0f, interpolated.b)) * 255);

          printf("Pixel: (%d, %d, %d)\n", r, g, b);

          image->setPixel(x, y, r, g, b);
        }
      }
    }
  }
}

// Color mode 1: Depth-based coloring (closer = brighter)
void setDepthColors(std::vector<kiwitracer::Vertex>& vertices) {
  // Find min and max z values for normalization
  float minZ = vertices[0].z;
  float maxZ = vertices[0].z;

  for (const auto& v : vertices) {
    minZ = std::min(minZ, v.z);
    maxZ = std::max(maxZ, v.z);
  }

  // Normalize z values and set colors
  for (auto& v : vertices) {
    float normalizedZ = (v.z - minZ) / (maxZ - minZ);
    v.r = normalizedZ; // Red component based on depth
    v.g = 0.0f;
    v.b = 0.0f;
  }
}

// Color mode 2: Distance-based banded coloring
void setDistanceColors(std::vector<kiwitracer::Vertex>& vertices, int width, int height) {
  // Reference point in window space (center of image)
  float refX = width / 2.0f;
  float refY = height / 2.0f;

  float minDist = 1e10f;
  float maxDist = 0.0f;

  // Find min and max distances
  for (const auto& v : vertices) {
    float dist = std::sqrt((v.x - refX) * (v.x - refX) + (v.y - refY) * (v.y - refY));
    minDist = std::min(minDist, dist);
    maxDist = std::max(maxDist, dist);
  }

  // Set banded colors based on distance
  for (auto& v : vertices) {
    float dist = std::sqrt((v.x - refX) * (v.x - refX) + (v.y - refY) * (v.y - refY));
    float normalizedDist = (dist - minDist) / (maxDist - minDist);

    // Create banded effect
    float banded = std::floor(normalizedDist * 8.0f) / 8.0f;

    // Blend between blue and yellow
    v.r = static_cast<unsigned char>(banded);
    v.g = static_cast<unsigned char>(banded);
    v.b = static_cast<unsigned char>(1.0f - banded);
  }
}

int main(int argc, char** argv) {
  fs::path meshfile, imagefile;
  int g_width = 100, g_height = 100, colorMode = 1;

  auto cli = lyra::cli() | lyra::arg(meshfile, "meshfile").required().help("Mesh file (e.g., foo.obj)") |
             lyra::arg(imagefile, "imagefile").required().help("Image file (e.g., foo.png)") |
             lyra::arg(g_width, "width").help("Image width (e.g., 512)") |
             lyra::arg(g_height, "height").help("Image height (e.g., 512)") |
             lyra::arg(colorMode, "mode").help("Coloring mode: 1 (depth) or 2 (binned distance)");

  auto result = cli.parse({argc, argv});
  if (!result) {
    std::cerr << result.message() << "\n";
    return 1;
  }

  // Validate color mode
  if (colorMode < 1 || colorMode > 2) {
    std::cerr << "Color mode must be 1 or 2\n";
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

  std::cout << "Number of vertices: " << posBuf.size() / 3 << std::endl;
  std::cout << "Number of triangles: " << triBuf.size() / 3 << std::endl;

  // Convert vertices to window coordinates
  std::vector<kiwitracer::Vertex> vertices;
  for (size_t i = 0; i < posBuf.size(); i += 3) {
    kiwitracer::Vertex world(posBuf[i], posBuf[i + 1], posBuf[i + 2], 0, 0, 0);
    vertices.push_back(world.worldToWindow(g_width, g_height));
  }

  // Apply color mode
  if (colorMode == 1) {
    setDepthColors(vertices);
  } else if (colorMode == 2) {
    setDistanceColors(vertices, g_width, g_height);
  }

  // Rasterize each triangle
  for (size_t i = 0; i < triBuf.size(); i += 3) {
    // Get the three vertex indices for this triangle
    unsigned int idx0 = triBuf[i];
    unsigned int idx1 = triBuf[i + 1];
    unsigned int idx2 = triBuf[i + 2];

    kiwitracer::Triangle tri(vertices[idx0], vertices[idx1], vertices[idx2]);
    rasterizeTriangle(tri, image, zbuffer);
  }

  // write out the image
  image->writeToFile(imagefile);

  return 0;
}
