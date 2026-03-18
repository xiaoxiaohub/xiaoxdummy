#include "xiaoxdummy_control/xiaoxdummy_hardware_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cerrno>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <chrono>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace xiaoxdummy_control
{

static rclcpp::Logger get_logger()
{
  return rclcpp::get_logger("XiaoxdummyHardwareInterface");
}

namespace
{

constexpr int kSerialInitRetries = 25;
constexpr int kSerialReplyTimeoutMs = 200;
constexpr int kSerialRetrySleepMs = 200;
constexpr int kReadFailureThreshold = 3;
constexpr int kWriteFailureThreshold = 3;
constexpr double kRecoveryPauseThresholdSec = 5.0;

}  // namespace


// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn XiaoxdummyHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  serial_port_ = info_.hardware_parameters.count("serial_port")
    ? info_.hardware_parameters.at("serial_port") : "/dev/ttyACM0";
  baud_rate_ = info_.hardware_parameters.count("baud_rate")
    ? std::stoi(info_.hardware_parameters.at("baud_rate")) : 115200;
  command_speed_deg_s_ = info_.hardware_parameters.count("command_speed")
    ? std::stod(info_.hardware_parameters.at("command_speed")) : 180.0;

  if (info_.joints.size() != NUM_JOINTS) {
    RCLCPP_ERROR(get_logger(), "Expected %zu joints, got %zu", NUM_JOINTS, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_positions_.resize(NUM_JOINTS, 0.0);
  hw_states_velocities_.resize(NUM_JOINTS, 0.0);
  hw_commands_positions_.resize(NUM_JOINTS, 0.0);
  prev_positions_.resize(NUM_JOINTS, 0.0);

  RCLCPP_INFO(get_logger(), "Initialized: port=%s baud=%d speed=%.1f deg/s",
    serial_port_.c_str(), baud_rate_, command_speed_deg_s_);

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn XiaoxdummyHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!serial_open()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!initialize_serial_session(true)) {
    serial_close();
    return hardware_interface::CallbackReturn::ERROR;
  }

  motors_enabled_ = false;
  consecutive_read_failures_ = 0;
  consecutive_write_failures_ = 0;

  RCLCPP_INFO(get_logger(), "Serial port %s opened", serial_port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn XiaoxdummyHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!send_start_command()) {
    if (!recover_serial_connection("activate")) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!send_start_command()) {
      RCLCPP_ERROR(get_logger(), "No response to START command on %s", serial_port_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  motors_enabled_ = true;

  std::array<double, NUM_JOINTS> joints_deg{};
  if (!request_joint_positions(
      joints_deg, kSerialInitRetries, kSerialReplyTimeoutMs, kSerialRetrySleepMs))
  {
    if (!recover_serial_connection("readback after enable")) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to read joint positions from %s after enabling",
        serial_port_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    apply_joint_state_feedback(joints_deg, 0.0, true);
    RCLCPP_INFO(get_logger(),
      "Enabled joints (deg): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
      joints_deg[0], joints_deg[1], joints_deg[2],
      joints_deg[3], joints_deg[4], joints_deg[5]);
  }

  consecutive_read_failures_ = 0;
  consecutive_write_failures_ = 0;

  RCLCPP_INFO(get_logger(), "Activated - motors enabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn XiaoxdummyHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (!send_stop_command()) {
    RCLCPP_WARN(get_logger(), "STOP command did not get a response on %s", serial_port_.c_str());
  }

  motors_enabled_ = false;
  consecutive_write_failures_ = 0;

  RCLCPP_INFO(get_logger(), "Deactivated - motors disabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn XiaoxdummyHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  send_stop_command();
  motors_enabled_ = false;
  serial_close();
  RCLCPP_INFO(get_logger(), "Serial port closed");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn XiaoxdummyHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  send_stop_command();
  motors_enabled_ = false;
  serial_close();
  RCLCPP_INFO(get_logger(), "Shutdown complete");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn XiaoxdummyHardwareInterface::on_error(
  const rclcpp_lifecycle::State &)
{
  send_stop_command();
  motors_enabled_ = false;
  serial_close();
  RCLCPP_ERROR(get_logger(), "Hardware interface entered error state");
  return hardware_interface::CallbackReturn::SUCCESS;
}


// ---------------------------------------------------------------------------
// Interfaces
// ---------------------------------------------------------------------------

std::vector<hardware_interface::StateInterface>
XiaoxdummyHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]);
  }
  return interfaces;
}


std::vector<hardware_interface::CommandInterface>
XiaoxdummyHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]);
  }
  return interfaces;
}


// ---------------------------------------------------------------------------
// Read / Write
// ---------------------------------------------------------------------------

hardware_interface::return_type XiaoxdummyHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (period.seconds() > kRecoveryPauseThresholdSec) {
    RCLCPP_WARN(
      get_logger(),
      "Detected a control-loop pause of %.2f s, reinitializing serial session",
      period.seconds());
    recover_serial_connection("long control loop pause");
  }

  std::array<double, NUM_JOINTS> joints_deg{};
  if (!request_joint_positions(joints_deg, 1, 100, 20)) {
    ++consecutive_read_failures_;
    if (consecutive_read_failures_ >= kReadFailureThreshold) {
      recover_serial_connection("joint feedback timeout");
    }
    return hardware_interface::return_type::OK;
  }

  consecutive_read_failures_ = 0;
  apply_joint_state_feedback(joints_deg, period.seconds(), false);

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type XiaoxdummyHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!motors_enabled_) {
    consecutive_write_failures_ = 0;
    return hardware_interface::return_type::OK;
  }

  std::ostringstream cmd;
  cmd << std::fixed << std::setprecision(2) << ">";

  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    double deg = hw_commands_positions_[i] * RAD_TO_DEG;
    deg = clamp_deg(deg, i);
    if (i > 0) cmd << ",";
    cmd << deg;
  }

  cmd << "," << command_speed_deg_s_ << "\n";

  if (!serial_send(cmd.str())) {
    ++consecutive_write_failures_;
    if (consecutive_write_failures_ >= kWriteFailureThreshold) {
      recover_serial_connection("joint command write failure");
    }
    return hardware_interface::return_type::ERROR;
  }

  consecutive_write_failures_ = 0;
  return hardware_interface::return_type::OK;
}


// ---------------------------------------------------------------------------
// Session helpers
// ---------------------------------------------------------------------------

bool XiaoxdummyHardwareInterface::initialize_serial_session(bool reset_commands)
{
  // Give USB CDC devices a short settling window after the port is opened.
  serial_flush_input();
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  serial_flush_input();

  std::string resp;
  if (!serial_send_and_read_reply(
      "#CMDMODE 2\n", resp, kSerialInitRetries, kSerialReplyTimeoutMs, kSerialRetrySleepMs))
  {
    RCLCPP_ERROR(get_logger(), "No response to CMDMODE command on %s", serial_port_.c_str());
    return false;
  }
  RCLCPP_INFO(get_logger(), "CMDMODE response: %s", resp.c_str());

  std::array<double, NUM_JOINTS> joints_deg{};
  if (!request_joint_positions(
      joints_deg, kSerialInitRetries, kSerialReplyTimeoutMs, kSerialRetrySleepMs))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to read initial joint positions from %s",
      serial_port_.c_str());
    return false;
  }

  apply_joint_state_feedback(joints_deg, 0.0, reset_commands);
  RCLCPP_INFO(get_logger(),
    "Initial joints (deg): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
    joints_deg[0], joints_deg[1], joints_deg[2],
    joints_deg[3], joints_deg[4], joints_deg[5]);

  return true;
}


bool XiaoxdummyHardwareInterface::send_start_command()
{
  std::string resp;
  if (!serial_send_and_read_reply(
      "!START\n", resp, kSerialInitRetries, kSerialReplyTimeoutMs, kSerialRetrySleepMs))
  {
    return false;
  }

  RCLCPP_INFO(get_logger(), "START response: %s", resp.c_str());
  return true;
}


bool XiaoxdummyHardwareInterface::send_stop_command()
{
  if (serial_fd_ < 0) {
    return false;
  }

  std::string resp;
  if (!serial_send_and_read_reply("!STOP\n", resp, 3, 100, 50)) {
    return false;
  }

  RCLCPP_INFO(get_logger(), "STOP response: %s", resp.c_str());
  return true;
}


bool XiaoxdummyHardwareInterface::recover_serial_connection(const std::string & reason)
{
  const bool restore_motor_state = motors_enabled_;

  RCLCPP_WARN(
    get_logger(),
    "Serial communication issue detected (%s); attempting recovery on %s",
    reason.c_str(),
    serial_port_.c_str());

  serial_close();
  if (!serial_open()) {
    return false;
  }

  if (!initialize_serial_session(true)) {
    serial_close();
    return false;
  }

  if (restore_motor_state && !send_start_command()) {
    RCLCPP_ERROR(
      get_logger(),
      "Recovered serial port %s but failed to re-enable motors",
      serial_port_.c_str());
    serial_close();
    return false;
  }

  motors_enabled_ = restore_motor_state;
  consecutive_read_failures_ = 0;
  consecutive_write_failures_ = 0;

  RCLCPP_INFO(get_logger(), "Serial recovery succeeded on %s", serial_port_.c_str());
  return true;
}


void XiaoxdummyHardwareInterface::apply_joint_state_feedback(
  const std::array<double, NUM_JOINTS> & joints_deg,
  double dt,
  bool reset_commands)
{
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    const double new_pos = joints_deg[i] * DEG_TO_RAD;
    if (dt > 0.0) {
      hw_states_velocities_[i] = (new_pos - hw_states_positions_[i]) / dt;
    } else {
      hw_states_velocities_[i] = 0.0;
    }
    hw_states_positions_[i] = new_pos;
    prev_positions_[i] = new_pos;
    if (reset_commands) {
      hw_commands_positions_[i] = new_pos;
    }
  }
}


// ---------------------------------------------------------------------------
// Serial helpers
// ---------------------------------------------------------------------------

bool XiaoxdummyHardwareInterface::serial_open()
{
  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(get_logger(), "Cannot open %s: %s", serial_port_.c_str(), strerror(errno));
    return false;
  }

  // Restore blocking mode after open
  int flags = fcntl(serial_fd_, F_GETFL, 0);
  fcntl(serial_fd_, F_SETFL, flags & ~O_NONBLOCK);

  struct termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", strerror(errno));
    serial_close();
    return false;
  }

  speed_t speed;
  switch (baud_rate_) {
    case 9600:   speed = B9600; break;
    case 19200:  speed = B19200; break;
    case 38400:  speed = B38400; break;
    case 57600:  speed = B57600; break;
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    case 921600: speed = B921600; break;
    default:     speed = B115200; break;
  }

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag &= ~PARENB;   // no parity
  tty.c_cflag &= ~CSTOPB;   // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;        // 8 data bits
  tty.c_cflag &= ~CRTSCTS;  // no HW flow control
  tty.c_cflag &= ~HUPCL;    // avoid toggling hangup on close/open for USB CDC
  tty.c_cflag |= CREAD | CLOCAL;

  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // raw input
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);           // no SW flow control
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;  // raw output

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 2;  // 200ms read timeout (in deciseconds)

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", strerror(errno));
    serial_close();
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}


void XiaoxdummyHardwareInterface::serial_close()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}


bool XiaoxdummyHardwareInterface::serial_send(const std::string & msg)
{
  if (serial_fd_ < 0) return false;
  ssize_t written = ::write(serial_fd_, msg.c_str(), msg.size());
  if (written != static_cast<ssize_t>(msg.size())) {
    RCLCPP_WARN(
      get_logger(), "Serial write short write (%zd/%zu bytes)", written, msg.size());
    return false;
  }
  return true;
}


std::string XiaoxdummyHardwareInterface::serial_read_line(int timeout_ms)
{
  std::string line;
  if (serial_fd_ < 0) return line;

  auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(serial_fd_, &rfds);

    auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
      deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) break;

    struct timeval tv;
    tv.tv_sec = remaining.count() / 1000000;
    tv.tv_usec = remaining.count() % 1000000;

    int ret = select(serial_fd_ + 1, &rfds, nullptr, nullptr, &tv);
    if (ret <= 0) break;

    char c;
    ssize_t n = ::read(serial_fd_, &c, 1);
    if (n <= 0) break;

    if (c == '\n') break;
    if (c != '\r') line += c;
  }

  return line;
}


void XiaoxdummyHardwareInterface::serial_flush_input()
{
  if (serial_fd_ >= 0) {
    tcflush(serial_fd_, TCIFLUSH);
  }
}


bool XiaoxdummyHardwareInterface::serial_send_and_read_reply(
  const std::string & msg,
  std::string & reply,
  int retries,
  int timeout_ms,
  int retry_sleep_ms)
{
  reply.clear();

  for (int attempt = 0; attempt < retries; ++attempt) {
    serial_flush_input();
    if (!serial_send(msg)) {
      return false;
    }

    reply = serial_read_line(timeout_ms);
    if (!reply.empty()) {
      return true;
    }

    if (attempt + 1 < retries) {
      std::this_thread::sleep_for(std::chrono::milliseconds(retry_sleep_ms));
    }
  }

  return false;
}


bool XiaoxdummyHardwareInterface::request_joint_positions(
  std::array<double, NUM_JOINTS> & joints_deg,
  int retries,
  int timeout_ms,
  int retry_sleep_ms)
{
  for (int attempt = 0; attempt < retries; ++attempt) {
    serial_flush_input();
    if (!serial_send("#GETJPOS\n")) {
      return false;
    }

    for (int read_attempt = 0; read_attempt < 3; ++read_attempt) {
      std::string resp = serial_read_line(timeout_ms);
      auto ok_pos = resp.find("ok");
      if (ok_pos != std::string::npos && ok_pos > 0) {
        resp = resp.substr(ok_pos);
      }

      if (resp.rfind("ok", 0) != 0) {
        if (resp.empty()) {
          break;
        }
        continue;
      }

      int n = sscanf(resp.c_str(), "ok %lf %lf %lf %lf %lf %lf",
        &joints_deg[0], &joints_deg[1], &joints_deg[2],
        &joints_deg[3], &joints_deg[4], &joints_deg[5]);
      if (n == static_cast<int>(NUM_JOINTS)) {
        return true;
      }
    }

    if (attempt + 1 < retries) {
      std::this_thread::sleep_for(std::chrono::milliseconds(retry_sleep_ms));
    }
  }

  return false;
}


double XiaoxdummyHardwareInterface::clamp_deg(double value, size_t joint_idx)
{
  if (joint_idx >= NUM_JOINTS) return value;
  return std::clamp(value, JOINT_LIMITS[joint_idx].min_deg, JOINT_LIMITS[joint_idx].max_deg);
}

}  // namespace xiaoxdummy_control

PLUGINLIB_EXPORT_CLASS(
  xiaoxdummy_control::XiaoxdummyHardwareInterface,
  hardware_interface::SystemInterface)
