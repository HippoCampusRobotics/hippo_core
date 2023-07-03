#include <gflags/gflags.h>

#include <iostream>

#include "path_planning/static_generation.hpp"

DEFINE_string(dir, "", "Output directory for the resulting .yaml file");
DEFINE_bool(swap_xy, false, "Swap x and y axis.");
DEFINE_double(x, 0.0, "Offset in x direction ");
DEFINE_double(y, 0.0, "Offset in y direction");
DEFINE_double(z, -1.3, "Offset in z direction.");
DEFINE_double(scale, 1.0, "Scaling factor for the path size.");



int main(int argc, char **argv) {
  gflags::AllowCommandLineReparsing();
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::string output_dir = FLAGS_dir;
  if (output_dir.empty()) {
    std::cout << "No output path given.\n";
    return 1;
  }
  path_planning::Path path =
      path_planning::static_generation::LemniscateOfBernoulli(100, 2.0, 1.0);
  path.WriteToYAML(output_dir + "/waypoints.yaml");
  return 0;
}
