#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "xiaoxdummy_interface/srv/init_firmware.hpp"
#include "xiaoxdummy_interface/srv/read_firmware.hpp"
#include "xiaoxdummy_interface/srv/write_firmware.hpp"

namespace xiaoxdummy_hw
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace
{

constexpr size_t kNumJoints = 6;
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

constexpr double kJointDirections[kNumJoints] = {
  -1.0, 1.0, -1.0, 1.0, -1.0, 1.0,
};

struct JointLimit
{
  double min_deg;
  double max_deg;
};

constexpr JointLimit kJointLimits[kNumJoints] = {
  {-170.0, 170.0},
  {-73.0, 90.0},
  {35.0, 180.0},
  {-180.0, 180.0},
  {-120.0, 120.0},
  {-720.0, 720.0},
};

}  // namespace

class XiaoxdummyHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    if (info_.joints.size() != kNumJoints) {
      RCLCPP_ERROR(
        rclcpp::get_logger("XiaoxdummyHardware"),
        "Expected %zu joints, got %zu",
        kNumJoints,
        info_.joints.size());
      return CallbackReturn::ERROR;
    }

    initialize_vectors();

    node_ = std::make_shared<rclcpp::Node>("xiaoxdummy_hardware_node");
    init_firmware_client_ =
      node_->create_client<xiaoxdummy_interface::srv::InitFirmware>("init_firmware");
    write_firmware_client_ =
      node_->create_client<xiaoxdummy_interface::srv::WriteFirmware>("write_firmware");
    read_firmware_client_ =
      node_->create_client<xiaoxdummy_interface::srv::ReadFirmware>("read_firmware");

    if (!wait_for_services()) {
      return CallbackReturn::ERROR;
    }

    if (!init_firmware("start")) {
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("XiaoxdummyHardware"), "Initialized");
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      state_interfaces.emplace_back(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &joint_positions_[i]);
      state_interfaces.emplace_back(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &joint_velocities_[i]);
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      command_interfaces.emplace_back(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &joint_position_commands_[i]);
    }
    return command_interfaces;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (!init_firmware("active")) {
      return CallbackReturn::ERROR;
    }

    if (!refresh_feedback(true)) {
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("XiaoxdummyHardware"), "Activated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    init_firmware("deactive");
    RCLCPP_INFO(rclcpp::get_logger("XiaoxdummyHardware"), "Deactivated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    init_firmware("stop");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    init_firmware("stop");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State &) override
  {
    init_firmware("stop");
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
    const rclcpp::Time &,
    const rclcpp::Duration &) override
  {
    refresh_feedback(false);
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time &,
    const rclcpp::Duration &) override
  {
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      actuator_pos_commands_[i] = clamp_deg(
        joint_position_commands_[i] * kRadToDeg * kJointDirections[i], i);
      actuator_vel_commands_[i] = 0.0;
    }

    if (!write_firmware(actuator_pos_commands_, actuator_vel_commands_)) {
      return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  }

private:
  bool wait_for_services()
  {
    if (!init_firmware_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(rclcpp::get_logger("XiaoxdummyHardware"), "init_firmware is unavailable");
      return false;
    }

    if (!write_firmware_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(rclcpp::get_logger("XiaoxdummyHardware"), "write_firmware is unavailable");
      return false;
    }

    if (!read_firmware_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(rclcpp::get_logger("XiaoxdummyHardware"), "read_firmware is unavailable");
      return false;
    }

    return true;
  }

  void initialize_vectors()
  {
    actuator_pos_commands_.assign(kNumJoints, 0.0);
    actuator_vel_commands_.assign(kNumJoints, 0.0);
    actuator_positions_.assign(kNumJoints, 0.0);
    actuator_velocities_.assign(kNumJoints, 0.0);
    joint_positions_.assign(kNumJoints, 0.0);
    joint_velocities_.assign(kNumJoints, 0.0);
    joint_position_commands_.assign(kNumJoints, 0.0);
  }

  bool refresh_feedback(bool reset_commands)
  {
    if (!read_firmware(actuator_positions_, actuator_velocities_)) {
      return false;
    }

    for (size_t i = 0; i < info_.joints.size(); ++i) {
      joint_positions_[i] = actuator_positions_[i] * kJointDirections[i] * kDegToRad;
      joint_velocities_[i] = actuator_velocities_[i] * kJointDirections[i] * kDegToRad;
      if (reset_commands) {
        joint_position_commands_[i] = joint_positions_[i];
      }
    }

    return true;
  }

  bool init_firmware(const std::string & action)
  {
    auto request = std::make_shared<xiaoxdummy_interface::srv::InitFirmware::Request>();
    request->action = action;

    auto future = init_firmware_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("XiaoxdummyHardware"),
        "Failed to call init_firmware with action %s",
        action.c_str());
      return false;
    }

    const auto result = future.get();
    if (!result->success) {
      RCLCPP_ERROR(
        rclcpp::get_logger("XiaoxdummyHardware"),
        "init_firmware(%s) failed: %s",
        action.c_str(),
        result->message.c_str());
      return false;
    }

    return true;
  }

  bool read_firmware(std::vector<double> & positions, std::vector<double> & velocities)
  {
    auto request = std::make_shared<xiaoxdummy_interface::srv::ReadFirmware::Request>();
    request->action = "read";

    auto future = read_firmware_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger("XiaoxdummyHardware"), "Failed to call read_firmware");
      return false;
    }

    const auto result = future.get();
    if (!result->success) {
      RCLCPP_WARN(
        rclcpp::get_logger("XiaoxdummyHardware"),
        "read_firmware failed: %s",
        result->message.c_str());
      return false;
    }

    if (result->pos_commands.size() != kNumJoints || result->vel_commands.size() != kNumJoints) {
      RCLCPP_ERROR(
        rclcpp::get_logger("XiaoxdummyHardware"),
        "Unexpected firmware feedback size: pos=%zu vel=%zu",
        result->pos_commands.size(),
        result->vel_commands.size());
      return false;
    }

    positions = result->pos_commands;
    velocities = result->vel_commands;
    return true;
  }

  bool write_firmware(
    const std::vector<double> & positions,
    const std::vector<double> & velocities)
  {
    auto request = std::make_shared<xiaoxdummy_interface::srv::WriteFirmware::Request>();
    request->pos_commands = positions;
    request->vel_commands = velocities;

    auto future = write_firmware_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger("XiaoxdummyHardware"), "Failed to call write_firmware");
      return false;
    }

    const auto result = future.get();
    if (!result->success) {
      RCLCPP_ERROR(
        rclcpp::get_logger("XiaoxdummyHardware"),
        "write_firmware failed: %s",
        result->message.c_str());
      return false;
    }

    return true;
  }

  double clamp_deg(double value, size_t joint_idx) const
  {
    return std::clamp(value, kJointLimits[joint_idx].min_deg, kJointLimits[joint_idx].max_deg);
  }

  std::vector<double> actuator_pos_commands_;
  std::vector<double> actuator_vel_commands_;
  std::vector<double> actuator_positions_;
  std::vector<double> actuator_velocities_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_position_commands_;

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<xiaoxdummy_interface::srv::InitFirmware>::SharedPtr init_firmware_client_;
  rclcpp::Client<xiaoxdummy_interface::srv::WriteFirmware>::SharedPtr write_firmware_client_;
  rclcpp::Client<xiaoxdummy_interface::srv::ReadFirmware>::SharedPtr read_firmware_client_;
};

}  // namespace xiaoxdummy_hw

PLUGINLIB_EXPORT_CLASS(xiaoxdummy_hw::XiaoxdummyHardware, hardware_interface::SystemInterface)
