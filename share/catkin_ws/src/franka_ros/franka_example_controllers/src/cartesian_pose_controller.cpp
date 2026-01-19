#include <franka_example_controllers/cartesian_pose_controller.h>
#include <franka_example_controllers/pseudo_inversion.h>

#include <cmath>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool CartesianPoseController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPoseController: Could not read parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR("CartesianPoseController: Invalid or no joint_names parameters provided");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));

  auto* velocity_joint_interface = robot_hw->get<hardware_interface::VelocityJointInterface>();
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_.push_back(velocity_joint_interface->getHandle(joint_names[i]));
  }

  sub_target_pose_ = node_handle.subscribe(
      "tcp_target_pose", 1, &CartesianPoseController::targetPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  return true;
}

void CartesianPoseController::starting(const ros::Time& /*time*/) {
  // 시작 시점의 포즈를 목표 포즈로 초기화하여 급격한 움직임 방지
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());

  // dq_last_.fill(0.0);
  for (size_t i = 0; i < 7; ++i) {
    dq_last_[i] = initial_state.dq[i]; 
  }

  start_count_ = 0; // 카운터 초기화
}

void CartesianPoseController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  // 1. 현재 로봇 상태 및 자코비안 획득
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  // 2. Cartesian Error 계산
  Eigen::Matrix<double, 6, 1> error;
  {
    std::lock_guard<std::mutex> lock(target_pose_mutex_);
    // 위치 오차
    error.head(3) << position_d_target_ - position;

    // 자세 오차 (Quaternion Difference)
    if (orientation_d_target_.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_target_);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // 로봇 베이스 좌표계로 변환
    error.tail(3) << transform.rotation() * error.tail(3);
  }

  // 3. Cartesian Velocity 명령 생성 (P-Control)
  Eigen::Matrix<double, 6, 1> cartesian_velocity;
  cartesian_velocity.head(3) << kp_pos_ * error.head(3);
  cartesian_velocity.tail(3) << kp_ori_ * error.tail(3);

  // 4. IK 연산: dq = J# * dx
  Eigen::MatrixXd jacobian_pinv;
  pseudoInverse(jacobian, jacobian_pinv);
  Eigen::Matrix<double, 7, 1> dq_d = jacobian_pinv * cartesian_velocity;

  // 5. Joint Velocity 명령 인가
  // for (size_t i = 0; i < 7; ++i) {
  //   joint_handles_[i].setCommand(dq_d[i]);
  // }

  // [보완] 전환 초기 100ms(100루프) 동안은 필터를 아주 강하게 걸어 부드럽게 이행
  double alpha = 0.95; 
  if (start_count_ < 100) {
      alpha = 0.999; // 거의 이전 속도를 유지
      start_count_++;
  }

  // for (size_t i = 0; i < 7; ++i) {
  //     // 이전 명령값(dq_last)과 새 명령값(dq_d)을 9:1 정도로 섞음
  //     double filtered_dq = 0.9 * dq_last_[i] + 0.1 * dq_d[i];
  //     joint_handles_[i].setCommand(filtered_dq);
  //     dq_last_[i] = filtered_dq; // 다음 루프를 위해 저장
  // }
  // [수정 후]
  for (size_t i = 0; i < 7; ++i) {
      // 계산된 alpha 값을 사용하여 이전 속도와 새 속도를 블렌딩
      double filtered_dq = alpha * dq_last_[i] + (1.0 - alpha) * dq_d[i];
      joint_handles_[i].setCommand(filtered_dq);
      dq_last_[i] = filtered_dq; 
  }
}

void CartesianPoseController::targetPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> lock(target_pose_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  
  Eigen::Quaterniond new_ori(msg->pose.orientation.w, msg->pose.orientation.x,
                             msg->pose.orientation.y, msg->pose.orientation.z);
  orientation_d_target_ = new_ori.normalized();
}

} // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseController, 
                       controller_interface::ControllerBase)