#include <bits/stdc++.h>
#include <gflags/gflags.h>

#include <iostream>
#include <fstream>
#include <cstring>
#include <cerrno>

#include "path_planning/static_generation.hpp"

DEFINE_string(dir, "", "Output directory for the resulting .yaml file");
DEFINE_bool(swap_xy, false, "Swap x and y axis.");
DEFINE_double(x, 1.0, "Offset in x direction ");
DEFINE_double(y, 2.0, "Offset in y direction");
DEFINE_double(z, -0.7, "Offset in z direction.");
DEFINE_string(bounds, "2.5x1.0",
              "Bounding box before other transformations are applied.");
DEFINE_double(scale, 1.0, "Scaling factor for the path size.");

int main(int argc, char **argv) {
  gflags::AllowCommandLineReparsing();
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::string output_dir = FLAGS_dir;
  if (output_dir.empty()) {
    std::cout << "No output path given.\n";
    return 1;
  }

  std::stringstream stream(FLAGS_bounds);
  std::string token;
  double xlim, ylim;
  if (stream.eof()) {
    std::cout << "Could not parse [bounds] parameter." << std::endl;
    return 1;
  }
  getline(stream, token, 'x');
  xlim = std::stod(token);
  if (stream.eof()) {
    std::cout << "Could not parse [bounds] paramter." << std::endl;
    return 1;
  }
  getline(stream, token, 'x');
  ylim = std::stod(token);

  path_planning::Path path =
      path_planning::static_generation::LemniscateOfBernoulli(100, xlim, ylim);
  if (FLAGS_swap_xy) {
    path.SwapXY();
  }
  path.Move(Eigen::Vector3d{FLAGS_x, FLAGS_y, FLAGS_z});
  YAML::Node node;
  node["generation"]["algorithm"] = "LemniscateOfBernoulli";
  node["generation"]["bounds"]["x"] = xlim;
  node["generation"]["bounds"]["y"] = ylim;
  node["generation"]["scale"] = FLAGS_scale;
  node["generation"]["swap_xy"] = FLAGS_swap_xy;
  node["generation"]["offset"]["x"] = FLAGS_x;
  node["generation"]["offset"]["y"] = FLAGS_y;
  node["generation"]["offset"]["z"] = FLAGS_z;
  path.AddToYAML(node);
  std::string file_path{output_dir + "/waypoints.yaml"};
  std::ofstream f;
  f.open(file_path);
  if (f.fail()) {
    std::cout << "Failed to write to " << file_path << ": " << std::strerror(errno) << std::endl;
    return 1;
  }
  f << node;
  f.close();
  return 0;
}
