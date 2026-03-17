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

  serial_flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  serial_flush();

  RCLCPP_INFO(get_logger(), "Serial port %s opened", serial_port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn XiaoxdummyHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  // Set interruptable command mode (mode 2, the firmware default)
  serial_send("#CMDMODE 2\n");
  auto resp = serial_read_line();
  RCLCPP_INFO(get_logger(), "CMDMODE response: %s", resp.c_str());

  // Enable motors
  serial_send("!START\n");
  resp = serial_read_line();
  RCLCPP_INFO(get_logger(), "START response: %s", resp.c_str());

  // Read current joint positions to initialize state
  serial_send("#GETJPOS\n");
  resp = serial_read_line();

  if (resp.rfind("ok", 0) == 0) {
    double j[NUM_JOINTS];
    int n = sscanf(resp.c_str(), "ok %lf %lf %lf %lf %lf %lf",
      &j[0], &j[1], &j[2], &j[3], &j[4], &j[5]);
    if (n == static_cast<int>(NUM_JOINTS)) {
      for (size_t i = 0; i < NUM_JOINTS; ++i) {
        hw_states_positions_[i] = j[i] * DEG_TO_RAD;
        hw_commands_positions_[i] = hw_states_positions_[i];
        prev_positions_[i] = hw_states_positions_[i];
      }
      RCLCPP_INFO(get_logger(),
        "Initial joints (deg): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
        j[0], j[1], j[2], j[3], j[4], j[5]);
    }
  } else {
    RCLCPP_WARN(get_logger(), "Failed to read initial joint positions: %s", resp.c_str());
  }

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
  serial_send("#GETJPOS\n");
  auto resp = serial_read_line();

  if (resp.rfind("ok", 0) != 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *rclcpp::Clock::make_shared(), 2000,
      "Bad GETJPOS response: %s", resp.c_str());
    return hardware_interface::return_type::OK;
  }

  double j[NUM_JOINTS];
  int n = sscanf(resp.c_str(), "ok %lf %lf %lf %lf %lf %lf",
    &j[0], &j[1], &j[2], &j[3], &j[4], &j[5]);
  if (n != static_cast<int>(NUM_JOINTS)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *rclcpp::Clock::make_shared(), 2000,
      "GETJPOS parse error (got %d values)", n);
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
  serial_read_line(50);  // read and discard queue-size response

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
  return written == static_cast<ssize_t>(msg.size());
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


void XiaoxdummyHardwareInterface::serial_flush()
{
  if (serial_fd_ >= 0) {
    tcflush(serial_fd_, TCIOFLUSH);
  }
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
