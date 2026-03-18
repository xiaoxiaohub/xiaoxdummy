#include "xiaoxdummy_control/xiaoxdummy_hardware_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
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

  // Give USB CDC devices a short settling window after the port is opened.
  serial_flush_input();
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
  serial_flush_input();

  RCLCPP_INFO(get_logger(), "Serial port %s opened", serial_port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn XiaoxdummyHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  std::string resp;

  // Retry startup commands so USB devices that reset on open have time to boot.
  if (!serial_send_and_read_reply("#CMDMODE 2\n", resp, 25, 200, 200)) {
    RCLCPP_ERROR(get_logger(), "No response to CMDMODE command on %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "CMDMODE response: %s", resp.c_str());

  if (!serial_send_and_read_reply("!START\n", resp, 25, 200, 200)) {
    RCLCPP_ERROR(get_logger(), "No response to START command on %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "START response: %s", resp.c_str());

  std::array<double, NUM_JOINTS> joints_deg{};
  if (!request_joint_positions(joints_deg, 25, 200, 200)) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to read initial joint positions from %s after startup",
      serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    hw_states_positions_[i] = joints_deg[i] * DEG_TO_RAD;
    hw_commands_positions_[i] = hw_states_positions_[i];
    prev_positions_[i] = hw_states_positions_[i];
  }
  RCLCPP_INFO(get_logger(),
    "Initial joints (deg): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
    joints_deg[0], joints_deg[1], joints_deg[2],
    joints_deg[3], joints_deg[4], joints_deg[5]);

  RCLCPP_INFO(get_logger(), "Activated – motors enabled");
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn XiaoxdummyHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  serial_send("!STOP\n");
  auto resp = serial_read_line();
  RCLCPP_INFO(get_logger(), "STOP response: %s", resp.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn XiaoxdummyHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  serial_close();
  RCLCPP_INFO(get_logger(), "Serial port closed");
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
  serial_flush_input();
  serial_send("#GETJPOS\n");

  // Read lines until we find one starting with "ok" (skip stale write responses)
  std::string resp;
  for (int attempt = 0; attempt < 5; ++attempt) {
    resp = serial_read_line();
    auto ok_pos = resp.find("ok");
    if (ok_pos != std::string::npos && ok_pos > 0) {
      resp = resp.substr(ok_pos);
    }
    if (resp.rfind("ok", 0) == 0) break;
  }

  if (resp.rfind("ok", 0) != 0) {
    return hardware_interface::return_type::OK;
  }

  double j[NUM_JOINTS];
  int n = sscanf(resp.c_str(), "ok %lf %lf %lf %lf %lf %lf",
    &j[0], &j[1], &j[2], &j[3], &j[4], &j[5]);
  if (n != static_cast<int>(NUM_JOINTS)) {
    return hardware_interface::return_type::OK;
  }

  double dt = period.seconds();
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    double new_pos = j[i] * DEG_TO_RAD;
    if (dt > 0.0) {
      hw_states_velocities_[i] = (new_pos - hw_states_positions_[i]) / dt;
    }
    hw_states_positions_[i] = new_pos;
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type XiaoxdummyHardwareInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::ostringstream cmd;
  cmd << std::fixed << std::setprecision(2) << ">";

  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    double deg = hw_commands_positions_[i] * RAD_TO_DEG;
    deg = clamp_deg(deg, i);
    if (i > 0) cmd << ",";
    cmd << deg;
  }

  cmd << "," << command_speed_deg_s_ << "\n";

  serial_send(cmd.str());

  return hardware_interface::return_type::OK;
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
