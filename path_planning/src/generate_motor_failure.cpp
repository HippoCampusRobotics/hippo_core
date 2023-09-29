#include <bits/stdc++.h>
#include <gflags/gflags.h>

#include <iostream>
#include <fstream>
#include <cstring>
#include <cerrno>

#include "path_planning/static_generation.hpp"

DEFINE_string(dir, "", "Output directory for the resulting .yaml file");
DEFINE_double(x, 1.0, "Offset in x direction ");
DEFINE_double(y, 0.0, "Offset in y direction");
DEFINE_double(z, 0.0, "Offset in z direction.");

int main(int argc, char **argv) {
  gflags::AllowCommandLineReparsing();
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::string output_dir = FLAGS_dir;
  if (output_dir.empty()) {
    std::cout << "No output path given.\n";
    return 1;
  }

  path_planning::Path path =
      path_planning::static_generation::MotorFailureSurface(425);
  path.Move(Eigen::Vector3d{FLAGS_x, FLAGS_y, FLAGS_z});
  YAML::Node node;
  node["generation"]["algorithm"] = "MotorFailureSurface";
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
