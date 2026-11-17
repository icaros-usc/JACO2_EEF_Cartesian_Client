// Timed Cartesian velocity control through the Gen2 SDK, as in its GUI example.
#include <Kinova.API.UsbCommandLayerUbuntu.h>
#include <fcntl.h>
#include <sys/file.h>
#include <tf2/LinearMath/Transform.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <jaco_vla_interfaces/action/cartesian_move.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace jaco_vla
{
using CartesianMove = jaco_vla_interfaces::action::CartesianMove;
using GoalHandle = rclcpp_action::ServerGoalHandle<CartesianMove>;
// All Cartesian messages use the SDK base frame; no world transform is applied.
constexpr char BASE_FRAME[] = "j2n6s200_link_base";
// Gen2 finger calibration: 6800 SDK ticks correspond to 80 degrees.
constexpr double FINGER_TICKS_PER_RADIAN = 6800.0 / (80.0 * M_PI / 180.0);
// Kinova's velocity-with-fingers callback uses this gain/deadband. The SDK GUI
// uses 4500 for finger velocity. Keep these SDK details out of policy settings.
// https://github.com/Kinovarobotics/kinova-ros/blob/noetic-devel/kinova_driver/src/kinova_arm.cpp#L455-L485
constexpr double FINGER_POSITION_GAIN = 2.0;
constexpr double FINGER_TOLERANCE_TICKS = 20.0;
constexpr double MAX_FINGER_VELOCITY_TICKS_S = 4500.0;

// Monotonic seconds for watchdogs and deadlines; independent of ROS clock changes.
static double steady_seconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
static void check_sdk_result(int code, const char * operation)
{
  if (code != NO_ERROR_KINOVA) {
    throw std::runtime_error(std::string(operation) + ": SDK error " + std::to_string(code));
  }
}
#define CHECK_SDK(call) check_sdk_result(call, #call)

// Required, validated motion settings supplied by control.yaml.
struct MotionLimits
{
  double translation_speed_m_s;
  double rotation_speed_rad_s;
  double action_duration_s;
  double command_watchdog_s;
};

// Only this object owns a physical SDK connection. Mock mode never opens one.
class KinovaSession
{
public:
  KinovaSession() = default;
  KinovaSession(const KinovaSession &) = delete;
  KinovaSession & operator=(const KinovaSession &) = delete;

  void open(int expected_frame_type)
  {
    const auto path = "/tmp/jaco_gen2_usb_" + std::to_string(getuid()) + ".lock";
    lock_fd_ = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    if (lock_fd_ < 0 || flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
      throw std::runtime_error(
        "Another native Jaco driver owns the SDK, or its lock is unavailable");
    }
    CHECK_SDK(InitAPI());
    initialized_ = true;
    KinovaDevice devices[MAX_KINOVA_DEVICE]{};
    int status = 0;
    const int count = GetDevices(devices, status);
    check_sdk_result(status, "GetDevices");
    if (count != 1) {
      throw std::runtime_error(
        "Expected exactly one connected Kinova arm; found " + std::to_string(count));
    }
    if (devices[0].DeviceType != 3 && devices[0].DeviceType != 6) {
      throw std::runtime_error("Expected a non-spherical Gen2 6DOF Jaco");
    }
    CHECK_SDK(SetActiveDevice(devices[0]));
    QuickStatus quick_status{};
    CHECK_SDK(GetQuickStatus(quick_status));
    if (quick_status.ControlFrameType != expected_frame_type) {
      throw std::runtime_error(
        "Kinova frame type mismatch: expected " + std::to_string(expected_frame_type) +
        ", actual " + std::to_string(quick_status.ControlFrameType) +
        " (0=fixed, 1=rotating). Select the matching mode in the Kinova SDK GUI before launching");
    }
    if (quick_status.Finger1Status != 1 || quick_status.Finger2Status != 1) {
      throw std::runtime_error(
        "Jaco fingers are not initialized (finger1=" + std::to_string(quick_status.Finger1Status) +
        ", finger2=" + std::to_string(quick_status.Finger2Status) +
        "). Use the native Kinova joystick to open and close the gripper, then restart the driver");
    }
    // No MoveHome, InitFingers or StartControlAPI during startup.
  }

  // Stream Cartesian velocities every 5 ms, without clearing the FIFO per sample.
  void send_velocity(const TrajectoryPoint & point)
  {
    const bool starting = !controlling_;
    controlling_ = true;  // Cleanup must also run if startup fails partway through.
    if (starting) {
      CHECK_SDK(EraseAllTrajectories());
      CHECK_SDK(StartControlAPI());
      CHECK_SDK(SwitchTrajectoryTorque(POSITION));
      CHECK_SDK(SetCartesianControl());
    }
    CHECK_SDK(SendBasicTrajectory(point));
  }

  void stop()
  {
    if (!controlling_) {
      return;
    }
    TrajectoryPoint neutral{};
    neutral.InitStruct();
    neutral.Position.Type = CARTESIAN_VELOCITY;
    neutral.Position.HandMode = VELOCITY_MODE;
    // Attempt every cleanup operation even if sending zero velocity fails.
    const int neutral_result = SendBasicTrajectory(neutral);
    const int clear_result = EraseAllTrajectories();
    const int stop_result = StopControlAPI();
    check_sdk_result(neutral_result, "SendBasicTrajectory(zero velocity)");
    check_sdk_result(clear_result, "EraseAllTrajectories");
    check_sdk_result(stop_result, "StopControlAPI");
    controlling_ = false;
  }

  ~KinovaSession()
  {
    try {
      stop();
    } catch (const std::exception & error) {
      fprintf(stderr, "%s\n", error.what());
    }
    if (initialized_) {
      const int result = CloseAPI();
      if (result != NO_ERROR_KINOVA) {
        fprintf(stderr, "CloseAPI: SDK error %d\n", result);
      }
    }
    if (lock_fd_ >= 0) {
      ::close(lock_fd_);
    }
  }

private:
  bool initialized_ = false;
  bool controlling_ = false;
  int lock_fd_ = -1;
};

class CartesianDriver : public rclcpp::Node
{
public:
  CartesianDriver() : Node("jaco_cartesian")
  {
    const auto sim = parameter<std::string>("sim");
    if (sim != "mock" && sim != "real") {
      throw std::runtime_error("sim must be mock or real");
    }
    use_real_hardware_ = sim == "real";
    const auto frame_type = parameter<std::string>("kinova_frame_type");
    if (frame_type != "fixed" && frame_type != "rotating") {
      throw std::runtime_error("kinova_frame_type must be fixed or rotating");
    }
    tool_transform_ = transform_parameter("tool");
    read_limit("translation_speed_m_s", limits_.translation_speed_m_s);
    read_limit("rotation_speed_rad_s", limits_.rotation_speed_rad_s);
    read_limit("action_duration_s", limits_.action_duration_s);
    read_limit("command_watchdog_s", limits_.command_watchdog_s);
    if (use_real_hardware_) {
      sdk_.open(frame_type == "fixed" ? 0 : 1);
    }
    pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", rclcpp::SensorDataQoS());
    joint_publisher_ =
      create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());
    heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
      "~/heartbeat", 10,
      [this](std_msgs::msg::Empty::ConstSharedPtr) { last_heartbeat_at_ = steady_seconds(); });
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/stop", [this](
                  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        abort("Operator stop; restart the driver to run again");
        response->success = fault_.find("stop failed:") == std::string::npos;
        response->message = fault_;
      });
    action_ = rclcpp_action::create_server<CartesianMove>(
      this, "~/move",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const CartesianMove::Goal> goal) {
        try {
          if (goal_reserved_) {
            throw std::runtime_error("A Cartesian goal is already active");
          }
          validate_goal(*goal);
          goal_reserved_ = true;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        } catch (const std::exception & error) {
          RCLCPP_WARN(get_logger(), "Rejected Cartesian goal: %s", error.what());
          return rclcpp_action::GoalResponse::REJECT;
        }
      },
      [this](std::shared_ptr<GoalHandle>) {
        try {
          stop();
          return rclcpp_action::CancelResponse::ACCEPT;
        } catch (const std::exception & error) {
          abort(error.what());
          return rclcpp_action::CancelResponse::REJECT;
        }
      },
      [this](std::shared_ptr<GoalHandle> handle) { dispatch(handle); });
    timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() { tick(); });
    RCLCPP_INFO(
      get_logger(), "Native Cartesian endpoint ready; sim=%s; kinova_frame_type=%s; firmware IK",
      sim.c_str(), frame_type.c_str());
  }

  void shutdown()
  {
    stop();  // SDK cleanup works even after SIGINT shuts down the ROS context.
    RCLCPP_INFO(get_logger(), "Driver shutdown; native FIFO cleared");
  }

private:
  template <typename T>
  T parameter(const std::string & name)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    try {
      return declare_parameter<T>(name, descriptor);
    } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException &) {
      throw std::runtime_error(
        "Missing required parameter '" + name + "'; load control.yaml and supply launch arguments");
    }
  }

  // Read a required, read-only limit and reject nonfinite or nonpositive values.
  void read_limit(const std::string & name, double & value)
  {
    value = parameter<double>(name);
    if (!std::isfinite(value) || value <= 0) {
      throw std::runtime_error(name + " must be finite and positive");
    }
  }

  std::vector<double> vector_parameter(const std::string & name, std::size_t size)
  {
    const auto result = parameter<std::vector<double>>(name);
    if (result.size() != size || !std::all_of(result.begin(), result.end(), [](double v) {
          return std::isfinite(v);
        })) {
      throw std::runtime_error(name + " has invalid dimensions or values");
    }
    return result;
  }

  tf2::Transform transform_parameter(const std::string & prefix)
  {
    const auto p = vector_parameter(prefix + "_translation", 3);
    const auto q = vector_parameter(prefix + "_rotation", 4);
    return make_transform(p[0], p[1], p[2], q[0], q[1], q[2], q[3]);
  }

  static tf2::Transform make_transform(
    double x, double y, double z, double qx, double qy, double qz, double qw)
  {
    tf2::Quaternion q(qx, qy, qz, qw);
    if (
      !std::isfinite(x + y + z + qx + qy + qz + qw) || !std::isfinite(q.length2()) ||
      q.length2() < 1e-12) {
      throw std::runtime_error("Invalid Cartesian pose");
    }
    return tf2::Transform(q.normalized(), tf2::Vector3(x, y, z));
  }

  static tf2::Transform make_transform(const geometry_msgs::msg::Pose & p)
  {
    return make_transform(
      p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z,
      p.orientation.w);
  }

  void validate_goal(const CartesianMove::Goal & goal)
  {
    if (!fault_.empty()) {
      throw std::runtime_error("Driver requires restart: " + fault_);
    }
    if (steady_seconds() - last_feedback_at_ > 0.5) {
      throw std::runtime_error("Native feedback unavailable/stale");
    }
    if (steady_seconds() - last_heartbeat_at_ > limits_.command_watchdog_s) {
      throw std::runtime_error("Client heartbeat unavailable/stale");
    }
    if (goal.target.header.frame_id != BASE_FRAME) {
      throw std::runtime_error(std::string("Goal must use frame ") + BASE_FRAME);
    }
    const double age = (now() - rclcpp::Time(goal.target.header.stamp)).seconds();
    if (age < -0.05 || age > 0.5) {
      throw std::runtime_error("Goal timestamp stale or clock mismatch");
    }
    // Validate finite coordinates and a nonzero quaternion before acceptance.
    make_transform(goal.target.pose);
    for (double finger : goal.finger_positions) {
      if (!std::isfinite(finger) || finger < 0 || finger > 1.51) {
        throw std::runtime_error("Finger target outside [0, 1.51] radians");
      }
    }
    if (!std::isfinite(goal.timeout_s) || goal.timeout_s <= limits_.action_duration_s) {
      throw std::runtime_error("Timeout must be finite and greater than action_duration_s");
    }
  }

  // Position targets are radians; the SDK finger fields are velocities whenever
  // the arm uses velocity mode, regardless of the supplied HandMode.
  static double finger_velocity(double target, double measured)
  {
    const double error = (target - measured) * FINGER_TICKS_PER_RADIAN;
    if (std::abs(error) <= FINGER_TOLERANCE_TICKS) {
      return 0;
    }
    return std::clamp(
      FINGER_POSITION_GAIN * error, -MAX_FINGER_VELOCITY_TICKS_S, MAX_FINGER_VELOCITY_TICKS_S);
  }

  // Convert an absolute SDK TCP target into constant velocities for one interval.
  // Translation is in base axes; angular velocity is in current SDK tool axes.
  // https://github.com/Kinovarobotics/kinova-ros/issues/154
  TrajectoryPoint make_velocity_point(
    const tf2::Transform & native_target, const std::array<double, 2> & fingers) const
  {
    TrajectoryPoint point{};
    point.InitStruct();
    point.Position.Type = CARTESIAN_VELOCITY;
    point.Position.HandMode = VELOCITY_MODE;
    auto & p = point.Position.CartesianPosition;
    auto linear =
      (native_target.getOrigin() - native_pose_.getOrigin()) / limits_.action_duration_s;
    auto delta = (native_pose_.getRotation().inverse() * native_target.getRotation()).normalized();
    if (delta.w() < 0) {
      delta *= -1.0;  // Equivalent quaternions must select the same shortest rotation.
    }
    auto angular = delta.getAxis() * (delta.getAngle() / limits_.action_duration_s);
    if (linear.length() > limits_.translation_speed_m_s) {
      linear *= limits_.translation_speed_m_s / linear.length();
    }
    if (angular.length() > limits_.rotation_speed_rad_s) {
      angular *= limits_.rotation_speed_rad_s / angular.length();
    }
    p.X = linear.x();
    p.Y = linear.y();
    p.Z = linear.z();
    p.ThetaX = angular.x();
    p.ThetaY = angular.y();
    p.ThetaZ = angular.z();
    if (!std::isfinite(double(p.X) + p.Y + p.Z + p.ThetaX + p.ThetaY + p.ThetaZ)) {
      throw std::runtime_error("Invalid Cartesian velocity from target pose");
    }
    point.Position.Fingers.Finger1 = finger_velocity(fingers[0], joint_positions_[6]);
    point.Position.Fingers.Finger2 = finger_velocity(fingers[1], joint_positions_[7]);
    // BasicTrajectory ignores per-point limits; cap the velocity vectors above.
    return point;
  }

  // Refresh finger velocity from feedback until the latest aperture is reached.
  void update_finger_velocity()
  {
    trajectory_point_.Position.Fingers.Finger1 =
      finger_velocity(target_fingers_[0], joint_positions_[6]);
    trajectory_point_.Position.Fingers.Finger2 =
      finger_velocity(target_fingers_[1], joint_positions_[7]);
  }

  void dispatch(std::shared_ptr<GoalHandle> handle)
  {
    active_goal_ = handle;
    try {
      const auto & goal = *handle->get_goal();
      validate_goal(goal);
      // Targets use the SDK base frame; account only for the tool reference point.
      native_target_pose_ = make_transform(goal.target.pose) * tool_transform_.inverse();
      target_fingers_ = goal.finger_positions;
      trajectory_point_ = make_velocity_point(native_target_pose_, target_fingers_);
      control_active_ = true;
      if (use_real_hardware_) {
        sdk_.send_velocity(trajectory_point_);
      } else {
        mock_start_pose_ = native_pose_;
        mock_moving_ = true;
      }
      goal_started_at_ = steady_seconds();
      deadline_ = goal_started_at_ + goal.timeout_s;
    } catch (const std::exception & error) {
      abort(error.what());
    }
  }

  // Read SDK feedback (or the protocol mock); angles returned to ROS are radians.
  void read_feedback()
  {
    joint_velocities_.fill(0);
    if (use_real_hardware_) {
      CartesianPosition cart{};
      AngularPosition joints{}, speed{};
      CHECK_SDK(GetCartesianPosition(cart));
      CHECK_SDK(GetAngularPosition(joints));
      CHECK_SDK(GetAngularVelocity(speed));
      const auto & p = cart.Coordinates;
      const auto q = tf2::Quaternion(tf2::Vector3(1, 0, 0), p.ThetaX) *
                     tf2::Quaternion(tf2::Vector3(0, 1, 0), p.ThetaY) *
                     tf2::Quaternion(tf2::Vector3(0, 0, 1), p.ThetaZ);
      native_pose_ = make_transform(p.X, p.Y, p.Z, q.x(), q.y(), q.z(), q.w());
      const std::array<double, 6> arm{joints.Actuators.Actuator1, joints.Actuators.Actuator2,
                                      joints.Actuators.Actuator3, joints.Actuators.Actuator4,
                                      joints.Actuators.Actuator5, joints.Actuators.Actuator6};
      const std::array<double, 6> vel{speed.Actuators.Actuator1, speed.Actuators.Actuator2,
                                      speed.Actuators.Actuator3, speed.Actuators.Actuator4,
                                      speed.Actuators.Actuator5, speed.Actuators.Actuator6};
      // Select the initial equivalent joint angles near the URDF convention.
      const std::array<double, 6> middle{0, M_PI, M_PI, 0, 0, 0};
      for (size_t i = 0; i < arm.size(); ++i) {
        const double radians = arm[i] * M_PI / 180.0;
        if (last_feedback_at_ == 0) {
          joint_wrap_offsets_[i] = 2 * M_PI * std::round((middle[i] - radians) / (2 * M_PI));
        }
        joint_positions_[i] = radians + joint_wrap_offsets_[i];
        joint_velocities_[i] = (vel[i] > 180 ? vel[i] - 360 : vel[i]) * M_PI / 180.0;
      }
      joint_positions_[6] = joints.Fingers.Finger1 / FINGER_TICKS_PER_RADIAN;
      joint_positions_[7] = joints.Fingers.Finger2 / FINGER_TICKS_PER_RADIAN;
      joint_velocities_[6] = speed.Fingers.Finger1 / FINGER_TICKS_PER_RADIAN;
      joint_velocities_[7] = speed.Fingers.Finger2 / FINGER_TICKS_PER_RADIAN;
    } else if (control_active_) {
      if (mock_moving_) {
        // Integrate the same base-linear/tool-angular command, without joint IK.
        const double elapsed =
          std::min(steady_seconds() - goal_started_at_, limits_.action_duration_s);
        const auto & p = trajectory_point_.Position.CartesianPosition;
        const tf2::Vector3 linear(p.X, p.Y, p.Z), angular(p.ThetaX, p.ThetaY, p.ThetaZ);
        native_pose_.setOrigin(mock_start_pose_.getOrigin() + linear * elapsed);
        if (angular.length() > 1e-12) {
          native_pose_.setRotation(
            mock_start_pose_.getRotation() *
            tf2::Quaternion(angular.normalized(), angular.length() * elapsed));
        }
        for (size_t i = 0; i < 6; ++i) {
          joint_velocities_[i] = linear.length() > 1e-12 || angular.length() > 1e-12 ? 0.05 : 0;
        }
      }
      const double dt = steady_seconds() - last_feedback_at_;
      for (size_t i = 0; i < 2; ++i) {
        joint_velocities_[6 + i] =
          finger_velocity(target_fingers_[i], joint_positions_[6 + i]) / FINGER_TICKS_PER_RADIAN;
        joint_positions_[6 + i] += joint_velocities_[6 + i] * dt;
      }
    }
    for (size_t i = 0; i < joint_positions_.size(); ++i) {
      if (!std::isfinite(joint_positions_[i]) || !std::isfinite(joint_velocities_[i])) {
        throw std::runtime_error("Invalid native joint feedback");
      }
    }
    current_pose_ = native_pose_ * tool_transform_;
    last_feedback_at_ = steady_seconds();
  }

  void stop()
  {
    if (use_real_hardware_) {
      sdk_.stop();
    }
    mock_moving_ = false;
    joint_velocities_.fill(0);
    control_active_ = false;
  }

  void finish(bool success, const std::string & message, bool canceled = false)
  {
    auto result = std::make_shared<CartesianMove::Result>();
    result->success = success;
    result->message = message;
    if (canceled) {
      active_goal_->canceled(result);
    } else if (success) {
      active_goal_->succeed(result);
    } else {
      active_goal_->abort(result);
    }
    active_goal_.reset();
    goal_reserved_ = false;
  }

  void abort(const std::string & reason)
  {
    fault_ = reason;
    try {
      stop();
    } catch (const std::exception & error) {
      fault_ += std::string("; stop failed: ") + error.what();
    }
    if (active_goal_) {
      finish(false, fault_);
    }
    RCLCPP_ERROR(get_logger(), "%s", fault_.c_str());
  }

  // Publish SDK-derived arm/finger state and the policy-frame tool pose.
  geometry_msgs::msg::PoseStamped publish_state()
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = now();
    pose_msg.header.frame_id = BASE_FRAME;
    const auto p = current_pose_.getOrigin();
    const auto q = current_pose_.getRotation();
    pose_msg.pose.position.x = p.x();
    pose_msg.pose.position.y = p.y();
    pose_msg.pose.position.z = p.z();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    pose_publisher_->publish(pose_msg);
    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header = pose_msg.header;
    for (int i = 1; i <= 6; ++i) {
      joint_msg.name.push_back("j2n6s200_joint_" + std::to_string(i));
    }
    for (int i = 1; i <= 2; ++i) {
      joint_msg.name.push_back("j2n6s200_joint_finger_" + std::to_string(i));
    }
    joint_msg.position.assign(joint_positions_.begin(), joint_positions_.end());
    joint_msg.velocity.assign(joint_velocities_.begin(), joint_velocities_.end());
    joint_publisher_->publish(joint_msg);
    return pose_msg;
  }

  // Stream at the existing 5 ms timer rate, including zero velocity between goals.
  void update_active_goal(const geometry_msgs::msg::PoseStamped & pose_msg)
  {
    if (active_goal_ && active_goal_->is_canceling()) {
      stop();
      finish(false, "Native FIFO cleared; cancellation completed", true);
      return;
    }
    if (!control_active_) {
      return;
    }
    const double time = steady_seconds();
    if (time - last_heartbeat_at_ > limits_.command_watchdog_s) {
      throw std::runtime_error("Client heartbeat timeout");
    }
    if (active_goal_ && time > deadline_) {
      throw std::runtime_error("Cartesian action execution timeout");
    }
    update_finger_velocity();
    if (!active_goal_) {
      if (use_real_hardware_) {
        sdk_.send_velocity(trajectory_point_);
      }
      return;
    }
    auto feedback = std::make_shared<CartesianMove::Feedback>();
    feedback->current_pose = pose_msg;
    feedback->finger_positions = {joint_positions_[6], joint_positions_[7]};
    active_goal_->publish_feedback(feedback);
    if (time - goal_started_at_ >= limits_.action_duration_s) {
      // Only the arm delta expires; keep pursuing the latest gripper aperture.
      trajectory_point_.Position.CartesianPosition.InitStruct();  // Exact zero, including roundoff.
      if (use_real_hardware_) {
        sdk_.send_velocity(trajectory_point_);
      }
      mock_moving_ = false;
      std::fill_n(joint_velocities_.begin(), 6, 0);
      finish(true, "Arm action duration elapsed; gripper target remains active");
    } else if (use_real_hardware_) {
      sdk_.send_velocity(trajectory_point_);
    }
  }

  void tick()
  {
    try {
      read_feedback();
      const auto pose_msg = publish_state();
      update_active_goal(pose_msg);
    } catch (const std::exception & error) {
      if (fault_ != error.what()) {
        abort(error.what());
      }
    }
  }

  KinovaSession sdk_;
  bool use_real_hardware_ = false, goal_reserved_ = false, mock_moving_ = false;
  bool control_active_ = false;
  std::string fault_;
  MotionLimits limits_{};
  // T_native_tool_policy_end_effector; all poses use the SDK base frame.
  tf2::Transform tool_transform_;
  tf2::Transform current_pose_, native_target_pose_, mock_start_pose_;
  tf2::Transform native_pose_{tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0.3)};
  std::array<double, 8> joint_positions_{0, 2, 2, 0, 0, 0, 0.7, 0.7}, joint_velocities_{};
  std::array<double, 6> joint_wrap_offsets_{};
  std::array<double, 2> target_fingers_{};
  double last_heartbeat_at_ = 0, last_feedback_at_ = 0, goal_started_at_ = 0;
  double deadline_ = 0;
  TrajectoryPoint trajectory_point_{};
  std::shared_ptr<GoalHandle> active_goal_;
  rclcpp_action::Server<CartesianMove>::SharedPtr action_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace jaco_vla

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<jaco_vla::CartesianDriver>();
    rclcpp::spin(node);
    node->shutdown();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("jaco_cartesian"), "%s", error.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
