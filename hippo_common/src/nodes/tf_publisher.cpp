#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <hippo_common/param_utils.hpp>
#include <hippo_common/tf2_utils.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hippo_common {

class TfPublisher : public rclcpp::Node {
 public:
  TfPublisher() : Node("tf_publisher") {
    static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    dynamic_broadcaster_ =
        std::make_shared<tf2_ros::TransformBroadcaster>(this);
    DeclareParameters();
    BroadCastStatic();
  }

 private:
  void DeclareParameters() { DeclareVerticalCameraParameters(); }

  void DeclareVerticalCameraParameters() {
    std::string name;
    rcl_interfaces::msg::ParameterDescriptor descr;
    std::string descr_text;

    name = "vertical_camera.x";
    descr_text = "Camera offset in x direction relative to base_link";
    descr = param_utils::Description(descr_text, true);
    params_.vertical_camera.x =
        declare_parameter(name, params_.vertical_camera.x, descr);

    name = "vertical_camera.y";
    descr_text = "Camera offset in y direction relative to base_link";
    descr = param_utils::Description(descr_text, true);
    params_.vertical_camera.y =
        declare_parameter(name, params_.vertical_camera.y, descr);

    name = "vertical_camera.z";
    descr_text = "Camera offset in z direction relative to base_link";
    descr = param_utils::Description(descr_text, true);
    params_.vertical_camera.z =
        declare_parameter(name, params_.vertical_camera.z, descr);

    name = "vertical_camera.qw";
    descr_text =
        "Quaternion component of the camera orientation relative to base_link";
    descr = param_utils::Description(descr_text, true);
    params_.vertical_camera.qw =
        declare_parameter(name, params_.vertical_camera.qw, descr);

    name = "vertical_camera.qx";
    descr_text =
        "Quaternion component of the camera orientation relative to base_link";
    descr = param_utils::Description(descr_text, true);
    params_.vertical_camera.qx =
        declare_parameter(name, params_.vertical_camera.qx, descr);

    name = "vertical_camera.qy";
    descr_text =
        "Quaternion component of the camera orientation relative to base_link";
    descr = param_utils::Description(descr_text, true);
    params_.vertical_camera.qy =
        declare_parameter(name, params_.vertical_camera.qy, descr);

    name = "vertical_camera.qz";
    descr_text =
        "Quaternion component of the camera orientation relative to base_link";
    descr = param_utils::Description(descr_text, true);
    params_.vertical_camera.qz =
        declare_parameter(name, params_.vertical_camera.qz, descr);
  }
  void BroadCastStatic() {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    {
      geometry_msgs::msg::TransformStamped t;
      t.transform = hippo_common::tf2_utils::ENUtoNED();
      std::string child_frame =
          hippo_common::tf2_utils::frame_id::InertialFrame() + "_ned";
      t.header.frame_id = hippo_common::tf2_utils::frame_id::InertialFrame();
      t.child_frame_id = child_frame;
      transforms.push_back(t);
    }
    {
      geometry_msgs::msg::TransformStamped t;
      t.transform = hippo_common::tf2_utils::FLUtoFRD();
      std::string child_frame =
          hippo_common::tf2_utils::frame_id::BaseLink(this) + "_frd";
      t.header.frame_id = hippo_common::tf2_utils::frame_id::BaseLink(this);
      t.child_frame_id = child_frame;
      transforms.push_back(t);
    }
    {
      geometry_msgs::msg::TransformStamped t;
      t.transform.translation.x = params_.vertical_camera.x;
      t.transform.translation.y = params_.vertical_camera.y;
      t.transform.translation.z = params_.vertical_camera.z;
      t.transform.rotation.w = params_.vertical_camera.qw;
      t.transform.rotation.x = params_.vertical_camera.qx;
      t.transform.rotation.y = params_.vertical_camera.qy;
      t.transform.rotation.z = params_.vertical_camera.qz;
      t.header.frame_id = hippo_common::tf2_utils::frame_id::BaseLink(this);
      t.child_frame_id =
          hippo_common::tf2_utils::frame_id::VerticalCameraLink(this);
      transforms.push_back(t);

      geometry_msgs::msg::TransformStamped t2;
      t2.transform = hippo_common::tf2_utils::CameraLinkToCameraFrame();
      t2.header.frame_id =
          hippo_common::tf2_utils::frame_id::VerticalCameraLink(this);
      t2.child_frame_id =
          hippo_common::tf2_utils::frame_id::VerticalCameraFrame(this);
      transforms.push_back(t2);
    }
    static_broadcaster_->sendTransform(transforms);
  }
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
  struct CameraPose {
    double x{-0.1};
    double y{0.0};
    double z{0.0};
    double qw{0.7071068};
    double qx{0.0};
    double qy{0.7071068};
    double qz{0.0};
  };
  struct Params {
    struct CameraPose vertical_camera;
  } params_;
};
}  // namespace hippo_common

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hippo_common::TfPublisher>());
  return 0;
}
