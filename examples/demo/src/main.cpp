#include <iostream>
#include <kiwitracer/kiwitracer.hpp>
#include <memory>
#include <lyra/lyra.hpp>
#include <iostream>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  fs::path input_file;
  bool verbose = false;

  auto cli = lyra::cli() | lyra::arg(input_file, "file").required().help("Path to an input file") |
             lyra::opt(verbose)["-v"]["--verbose"]("Enable verbose mode");

  auto result = cli.parse({argc, argv});
  if (!result) {
    std::cerr << result.message() << "\n";
    return 1;
  }

  // Custom file validation (after parsing)
  if (!fs::exists(input_file)) {
    std::cerr << "Error: file does not exist: " << input_file << "\n";
    return 1;
  }
  if (!fs::is_regular_file(input_file)) {
    std::cerr << "Error: not a regular file: " << input_file << "\n";
    return 1;
  }

  std::cout << "Input file: " << input_file << "\n";
  if (verbose) std::cout << "Verbose mode enabled\n";

  // Safe file open
  std::ifstream fin(input_file);
  if (!fin) {
    std::cerr << "Error: failed to open file\n";
    return 1;
  }

  std::cout << "File size: " << fs::file_size(input_file) << " bytes\n";
}

