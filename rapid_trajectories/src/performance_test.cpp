#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

#include "rapid_trajectories/trajectory/collision_checker.hpp"
#include "rapid_trajectories/trajectory/generator.hpp"
#include "rapid_trajectories/trajectory_math/convex_obj.hpp"
#include "rapid_trajectories/trajectory_math/rect_prims.hpp"
#include "rapid_trajectories/trajectory_math/sphere.hpp"
using namespace rapid_trajectories;
int main() {
  static constexpr int runs = 1000;
  double fmin = 0.0;
  double fmax = 24.0;
  double omegamax = 6.0;
  double dt_min = 0.02;
  double pos_range = 2.0;
  double vel_range = 1.0;
  double acc_range = 1.0;
  double t_min = 0.5;
  double t_max = 5.0;
  double r_min = 0.1;
  double r_max = 0.5;

  std::mt19937 gen(0);
  std::uniform_real_distribution<> p_rand(-pos_range, pos_range);
  std::uniform_real_distribution<> v_rand(-vel_range, vel_range);
  std::uniform_real_distribution<> a_rand(-acc_range, acc_range);
  std::uniform_real_distribution<> t_rand(t_min, t_max);
  std::uniform_real_distribution<> r_rand(r_min, r_max);

  Eigen::Vector3d p0{0.0, 0.0, 0.0};

  int n_input_feasible = 0;
  int n_state_feasible = 0;
  std::array<long, runs> t_gen;
  std::array<long, runs> t_input;
  std::vector<long> t_input_feasible;
  std::vector<long> t_input_infeasible;
  std::array<long, runs> t_state;
  std::vector<long> t_state_infeasible;
  std::vector<long> t_state_feasible;

  for (int i = 0; i < runs; ++i) {
    if (i % 10000 == 0) {
      printf("Progress: %.2lf%%\n", 100.0 * i / runs);
    }
    Eigen::Vector3d v0{v_rand(gen), v_rand(gen), v_rand(gen)};
    Eigen::Vector3d a0{a_rand(gen), a_rand(gen), a_rand(gen)};
    Eigen::Vector3d pf{p_rand(gen), p_rand(gen), p_rand(gen)};
    Eigen::Vector3d vf{v_rand(gen), v_rand(gen), v_rand(gen)};
    Eigen::Vector3d af{a_rand(gen), a_rand(gen), a_rand(gen)};
    double tf = t_rand(gen);

    minimum_jerk::Trajectory traj(p0, v0, a0, Eigen::Vector3d::Zero(), 2.31,
                                  2.0, 5.4,
                                  Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0});
    traj.SetGoalPosition(pf);
    traj.SetGoalVelocity(vf);
    traj.SetGoalAcceleration(af);

    Eigen::Vector3d p_obst(p_rand(gen), p_rand(gen), p_rand(gen));
    std::shared_ptr<trajectory_math::ConvexObj> obstacle =
        std::make_shared<trajectory_math::Sphere>(p_obst, r_rand(gen));

    std::chrono::high_resolution_clock::time_point start_gen =
        std::chrono::high_resolution_clock::now();
    traj.Generate(tf, 0);
    std::chrono::high_resolution_clock::time_point end_gen =
        std::chrono::high_resolution_clock::now();
    t_gen.at(i) = std::chrono::duration_cast<std::chrono::nanoseconds>(
                      end_gen - start_gen)
                      .count();

    std::chrono::high_resolution_clock::time_point start_input =
        std::chrono::high_resolution_clock::now();
    minimum_jerk::Trajectory::InputFeasibilityResult input_result =
        traj.CheckInputFeasibility(fmin, fmax, omegamax, dt_min);
    std::chrono::high_resolution_clock::time_point end_input =
        std::chrono::high_resolution_clock::now();
    t_input.at(i) = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        end_input - start_input)
                        .count();
    if (input_result == traj.InputFeasible) {
      t_input_feasible.push_back(t_input[i]);
    } 
    else {
      t_input_infeasible.push_back(t_input[i]);
    }

      std::chrono::high_resolution_clock::time_point start_state =
      std::chrono::high_resolution_clock::now();
      minimum_jerk::CollisionChecker checker(traj.GetTrajectory());
      minimum_jerk::CollisionChecker::CollisionResult state_result =
      checker.CollisionCheck(obstacle, dt_min);
      std::chrono::high_resolution_clock::time_point end_state =
      std::chrono::high_resolution_clock::now(); t_state[i] =
      std::chrono::duration_cast<std::chrono::nanoseconds>(end_state -
      start_state).count(); if (state_result == checker.NoCollision) {
        t_state_feasible.push_back(t_state[i]);
      } else {
        t_state_infeasible.push_back(t_state[i]);
      }
  }

  std::ofstream total;
  total.open("t_all.txt");
  total << "generation, input, state, input_feasible, input_infeasible, state_feasible, state_infeasible\n";
  for (int i = 0; i<runs; ++i) {
    total << t_gen.at(i) << ",";
    total << t_input.at(i) << ",";
    total << t_state.at(i) << ",";
    if (i < t_input_feasible.size()) {
      total << t_input_feasible.at(i);
    }
    total << ",";
    if (i < t_input_infeasible.size()) {
      total << t_input_infeasible.at(i);
    }
    total << ",";
    if (i < t_state_feasible.size()) {
      total << t_state_feasible.at(i);
    }
    total << ",";
    if (i < t_state_infeasible.size()) {
      total << t_state_infeasible.at(i);
    }
    total << "\n";
  }

  std::ofstream input_feasible;
  input_feasible.open("input_feasible.txt");
  for (auto meas : t_input_feasible) {
    input_feasible << std::to_string(meas) << "\n";
  }
  input_feasible.close();

  std::ofstream input_infeasible;
  input_infeasible.open("input_infeasible.txt");
  for (auto meas : t_input_infeasible) {
    input_infeasible << std::to_string(meas) << "\n";
  }
  input_infeasible.close();

  std::ofstream state_infeasible;
  state_infeasible.open("state_infeasible.txt");
  for (auto meas : t_state_infeasible) {
    state_infeasible << std::to_string(meas) << "\n";
  }
  state_infeasible.close();

  std::ofstream state_feasible;
  state_feasible.open("state_feasible.txt");
  for (auto meas : t_state_feasible) {
    state_feasible << std::to_string(meas) << "\n";
  }
  state_feasible.close();

  std::ofstream state;
  state.open("state.txt");
  for (auto meas : t_state) {
    state << std::to_string(meas) << "\n";
  }
  state.close();

  std::ofstream input;
  input.open("input.txt");
  for (auto meas : t_input) {
    input << std::to_string(meas) << "\n";
  }
  input.close();

  std::ofstream fgen;
  fgen.open("gen.txt");
  for (auto meas : t_gen) {
    fgen << std::to_string(meas) << "\n";
  }
  fgen.close();
  return 0;
}
