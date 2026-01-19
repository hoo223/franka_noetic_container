#pragma once

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_example_controllers {

class CartesianPoseController : public controller_interface::MultiInterfaceController<
                                            franka_hw::FrankaModelInterface,
                                            hardware_interface::VelocityJointInterface,
                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // Target Pose (Thread-safe)
  std::mutex target_pose_mutex_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  // Gains (P-gain for velocity control)
  double kp_pos_{5.0}; // 낮게 시작해서 점진적으로 높이세요 (최대 20~30)
  double kp_ori_{5.0};

  // Subscriber
  ros::Subscriber sub_target_pose_;
  void targetPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  std::array<double, 7> dq_last_; // 이전 루프의 속도를 저장할 배열
  int start_count_{0};
};

}  // namespace franka_example_controllers