#include <gflags/gflags.h>
#include <ignition/msgs/entity_factory.pb.h>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

DEFINE_string(world, "", "World name.");
DEFINE_string(param, "", "Load XML from a ROS param.");
DEFINE_string(name, "", "Name for spawned entity.");
DEFINE_bool(allow_renaming, false, "Rename entity if name already used.");
DEFINE_bool(remove_on_exit, false,
            "Keep the spawner running. Remove model on exit.");
DEFINE_double(x, 0, "X component of initial position, in meters.");
DEFINE_double(y, 0, "Y component of initial position, in meters.");
DEFINE_double(z, 0, "Z component of initial position, in meters.");
DEFINE_double(R, 0, "Roll component of initial orientation, in radians.");
DEFINE_double(P, 0, "Pitch component of initial orientation, in radians.");
DEFINE_double(Y, 0, "Yaw component of initial orientation, in radians.");

std::string get_world_name(rclcpp::Node::SharedPtr ros_node) {
  ignition::transport::Node node;
  unsigned int timeout{5000};
  bool done{false};
  bool result{false};
  std::string service{"/gazebo/worlds"};
  ignition::msgs::StringMsg_V worlds_msgs;

  while (rclcpp::ok() && !done) {
    RCLCPP_INFO(ros_node->get_logger(), "Requesting list of world names.");
    done = node.Request(service, timeout, worlds_msgs, result);
  }

  if (!done) {
    RCLCPP_ERROR(ros_node->get_logger(),
                 "Timed out when requesting world names.");
    return "";
  }

  if (!result || worlds_msgs.data().empty()) {
    RCLCPP_ERROR(ros_node->get_logger(), "Failed to get world names.");
    return "";
  }
  return worlds_msgs.data(0);
}

std::string get_sdf(rclcpp::Node::SharedPtr ros_node) {
  if (FLAGS_param.empty()) {
    RCLCPP_ERROR(ros_node->get_logger(),
                 "No robot description paramter provided!");
    return "";
  }
  ros_node->declare_parameter<std::string>(FLAGS_param);
  std::string xml_string;
  if (ros_node->get_parameter(FLAGS_param, xml_string)) {
    return xml_string;
  }
  RCLCPP_ERROR(ros_node->get_logger(), "Failed to get SDF from parameter [%s].",
               FLAGS_param.c_str());
  return "";
}

bool DeleteModel(const std::string &_model_name,
                 const std::string &_world_name) {
  std::string service{"/world/" + _world_name + "/state"};
  ignition::msgs::SerializedStepMap response;
  ignition::transport::Node node;
  bool result{false};
  if (!node.Request(service, 5000, response, result)) {
    std::cerr << std::endl
              << "Request of state of world [" << _world_name << "] timed out."
              << std::endl;
    return false;
  }
  if (!result) {
    std::cerr << std::endl
              << "Service call of [" << service << "] failed." << std::endl;
    return false;
  }
  ignition::gazebo::EntityComponentManager ecm;
  ecm.SetState(response.state());

  service = "/world/" + _world_name + "/remove";
  ignition::msgs::Entity request;
  ignition::gazebo::Entity entity =
      ecm.EntityByComponents(ignition::gazebo::components::Name(_model_name),
                             ignition::gazebo::components::Model());
  if (entity == ignition::gazebo::kNullEntity) {
    std::cerr << "Model with name [" << _model_name << "] does not exist.";
    return false;
  }
  // request.set_name(_model_name);
  request.set_id(entity);
  ignition::msgs::Boolean delete_response;
  if (!node.Request(service, request, 5000, delete_response, result)) {
    std::cerr << std::endl
              << "Delete model [" << _model_name << "] timed out." << std::endl;
    return false;
  }
  if (!result || !delete_response.data()) {
    std::cerr << std::endl
              << "Service call of [" << service << "] failed." << std::endl;
    return false;
  }
  return true;
}

int main(int _argc, char **_argv) {
  rclcpp::init(_argc, _argv);
  auto ros_node = rclcpp::Node::make_shared("vehicle_spawner");
  gflags::AllowCommandLineReparsing();
  gflags::ParseCommandLineFlags(&_argc, &_argv, true);
  std::string world_name = FLAGS_world;
  if (world_name.empty()) {
    world_name = get_world_name(ros_node);
    if (world_name == "") {
      return -1;
    }
  }

  std::string service{"/world/" + world_name + "/create"};
  ignition::msgs::EntityFactory req;
  std::string sdf_string = get_sdf(ros_node);
  if (sdf_string == "") {
    return -1;
  }
  req.set_sdf(sdf_string);

  ignition::math::Pose3d pose{FLAGS_x, FLAGS_y, FLAGS_z,
                              FLAGS_R, FLAGS_P, FLAGS_Y};
  ignition::msgs::Set(req.mutable_pose(), pose);

  if (!FLAGS_name.empty()) {
    req.set_name(FLAGS_name);
  } else {
    std::string n;
    n = ros_node->get_namespace();
    if ('/' == n.back()) {
      n.pop_back();
    }
    if ('/' == n.front()) {
      n.erase(0, 1);
    }
    if (n != "") {
      req.set_name(n);
    }
  }

  if (FLAGS_allow_renaming) {
    req.set_allow_renaming(FLAGS_allow_renaming);
  }

  ignition::transport::Node node;
  ignition::msgs::Boolean response;

  bool result;
  unsigned int timeout = 5000;
  bool done = node.Request(service, req, timeout, response, result);
  if (done) {
    if (result && response.data()) {
      RCLCPP_INFO(ros_node->get_logger(), "Created Entity.");
    } else {
      RCLCPP_ERROR(ros_node->get_logger(), "Failed entity creation request! %s",
                   req.DebugString().c_str());
    }
  } else {
    RCLCPP_ERROR(ros_node->get_logger(), "Entity creation timed out.");
  }
  if (FLAGS_remove_on_exit) {
    rclcpp::spin(ros_node);
    DeleteModel(req.name(), world_name);
  }
  return 0;
}
