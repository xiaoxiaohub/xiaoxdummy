#ifndef XIAOXDUMMY_HARDWARE_INTERFACE_HPP
#define XIAOXDUMMY_HARDWARE_INTERFACE_HPP

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace xiaoxdummy_control
{

class XiaoxdummyHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(XiaoxdummyHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr size_t NUM_JOINTS = 6;
  static constexpr double DEG_TO_RAD = M_PI / 180.0;
  static constexpr double RAD_TO_DEG = 180.0 / M_PI;

  struct JointLimit {
    double min_deg;
    double max_deg;
  };

  static constexpr JointLimit JOINT_LIMITS[NUM_JOINTS] = {
    {-170.0,  170.0},  // J1
    { -73.0,   90.0},  // J2
    {  35.0,  180.0},  // J3
    {-180.0,  180.0},  // J4
    {-120.0,  120.0},  // J5
    {-720.0,  720.0},  // J6
  };

  std::string serial_port_;
  int baud_rate_;
  double command_speed_deg_s_;
  int serial_fd_ = -1;

  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_commands_positions_;
  std::vector<double> prev_positions_;

  bool serial_open();
  void serial_close();
  bool serial_send(const std::string & msg);
  std::string serial_read_line(int timeout_ms = 200);
  void serial_flush_input();
  bool serial_send_and_read_reply(
    const std::string & msg,
    std::string & reply,
    int retries = 1,
    int timeout_ms = 200,
    int retry_sleep_ms = 100);
  bool request_joint_positions(
    std::array<double, NUM_JOINTS> & joints_deg,
    int retries = 1,
    int timeout_ms = 200,
    int retry_sleep_ms = 100);

  double clamp_deg(double value, size_t joint_idx);
};

}  // namespace xiaoxdummy_control

#endif  // XIAOXDUMMY_HARDWARE_INTERFACE_HPP
