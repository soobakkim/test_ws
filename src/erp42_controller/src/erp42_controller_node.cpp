#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>

class Erp42ControllerNode : public rclcpp::Node {
public:
  Erp42ControllerNode()
  : Node("erp42_controller_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<double>("max_linear_speed", 1.0); // m/s
    this->declare_parameter<double>("max_angular_speed", 0.5); // rad/s
    this->declare_parameter<double>("speed_scale_factor", 10.0); // Scale for ERP42 speed command
    this->declare_parameter<double>("steering_scale_factor", 100.0); // Scale for ERP42 steering command

    // Get parameters
    this->get_parameter("serial_port", serial_port_);
    this->get_parameter("baud_rate", baud_rate_);
    this->get_parameter("max_linear_speed", max_linear_speed_);
    this->get_parameter("max_angular_speed", max_angular_speed_);
    this->get_parameter("speed_scale_factor", speed_scale_factor_);
    this->get_parameter("steering_scale_factor", steering_scale_factor_);

    // ROS2 cmd_vel subscriber
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&Erp42ControllerNode::cmd_vel_callback, this, std::placeholders::_1));

    // Timer for sending serial commands periodically
    serial_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), // Send commands at 20 Hz
      std::bind(&Erp42ControllerNode::send_serial_command, this));

    // Open serial port
    fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
      rclcpp::shutdown();
      return;
    }
    configure_port(fd_, baud_rate_);
    RCLCPP_INFO(get_logger(), "Serial port %s opened successfully with baud rate %d", serial_port_.c_str(), baud_rate_);
  }

  ~Erp42ControllerNode() {
    if (fd_ >= 0) {
      close(fd_);
      RCLCPP_INFO(get_logger(), "Serial port %s closed.", serial_port_.c_str());
    }
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Store received linear and angular velocities
    current_linear_speed_ = msg->linear.x;
    current_angular_speed_ = msg->angular.z;
    RCLCPP_INFO(get_logger(), "Received cmd_vel: linear.x=%.2f, angular.z=%.2f", current_linear_speed_, current_angular_speed_);
  }

  void send_serial_command() {
    if (fd_ < 0) {
      return; // Serial port not open
    }

    // Convert ROS2 Twist values to ERP42 command values
    // Linear speed: 0.0 to max_linear_speed_ (m/s) -> 0 to 20 (ERP42 speed command)
    // Angular speed: -max_angular_speed_ to max_angular_speed_ (rad/s) -> -2000 to 2000 (ERP42 steering command)

    int erp42_speed = static_cast<int>(std::round(current_linear_speed_ * speed_scale_factor_));
    // Clamp speed to ERP42's valid range (e.g., 0-20 for forward, -20-0 for backward)
    erp42_speed = std::max(-20, std::min(20, erp42_speed));

    // ERP42 steering command: 0 is straight, positive is left, negative is right
    // Assuming angular.z positive is counter-clockwise (left turn)
    int erp42_steering = static_cast<int>(std::round(current_angular_speed_ * steering_scale_factor_));
    // Clamp steering to ERP42's valid range (e.g., -2000 to 2000)
    erp42_steering = std::max(-2000, std::min(2000, erp42_steering));

    // Determine gear (forward: 1, backward: 2, stop: 0)
    uint8_t gear = 0; // Stop
    if (erp42_speed > 0) {
      gear = 1; // Forward
    } else if (erp42_speed < 0) {
      gear = 2; // Backward
      erp42_speed = std::abs(erp42_speed); // ERP42 speed command is always positive
    }

    // Construct ERP42 serial packet
    std::vector<uint8_t> buf = {
      0x53, 0x54, 0x58,  // 'S', 'T', 'X'
      0x00,              // A (Auto/Manual): 0 for Manual
      0x01,              // E (Estop): 1 for Normal
      gear,              // Gear: 0=Stop, 1=Forward, 2=Backward
      static_cast<uint8_t>(erp42_speed), // Speed (0-20)
      static_cast<uint8_t>(erp42_steering & 0xFF), // Steering LSB
      static_cast<uint8_t>((erp42_steering >> 8) & 0xFF), // Steering MSB
      0x01,              // Br (Brake): 1 for Normal (adjust as needed)
      0x00,              // Al (Auxiliary): 0 for Normal (adjust as needed)
      0x0D, 0x0A         // ETX0, ETX1
    };

    // Send packet
    ssize_t bytes_written = write(fd_, buf.data(), buf.size());
    if (bytes_written < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to write to serial port: %s", strerror(errno));
    } else if (bytes_written != (ssize_t)buf.size()) {
      RCLCPP_WARN(get_logger(), "Incomplete write to serial port: wrote %zd of %zu bytes", bytes_written, buf.size());
    }
  }

  void configure_port(int fd, int baud) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "Error from tcgetattr: %s", strerror(errno));
      return;
    }

    speed_t baud_rate_const;
    switch (baud) {
      case 9600: baud_rate_const = B9600; break;
      case 19200: baud_rate_const = B19200; break;
      case 38400: baud_rate_const = B38400; break;
      case 57600: baud_rate_const = B57600; break;
      case 115200: baud_rate_const = B115200; break;
      default: 
        RCLCPP_WARN(get_logger(), "Unsupported baud rate: %d. Defaulting to 115200.", baud);
        baud_rate_const = B115200; break;
    }

    cfsetispeed(&tty, baud_rate_const);
    cfsetospeed(&tty, baud_rate_const);

    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;  // Clear all the size bits
    tty.c_cflag |= CS8;     // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable XON/XOFF flow control
    tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input

    tty.c_oflag &= ~OPOST; // Raw output

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "Error from tcsetattr: %s", strerror(errno));
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr serial_timer_;
  int fd_{-1};

  std::string serial_port_;
  int baud_rate_;
  double max_linear_speed_;
  double max_angular_speed_;
  double speed_scale_factor_;
  double steering_scale_factor_;

  double current_linear_speed_{0.0};
  double current_angular_speed_{0.0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Erp42ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
