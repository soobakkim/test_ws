#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

class ForwardDriveNode : public rclcpp::Node {
public:
  ForwardDriveNode()
  : Node("forward_drive_node")
  {
    // ROS 토픽 퍼블리셔 설정
    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ForwardDriveNode::on_timer, this));

    // POSIX 시리얼 포트 열기
    fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Serial port open failed");
      rclcpp::shutdown();
      return;
    }
    configure_port(fd_, B115200);
  }

  ~ForwardDriveNode() {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  void on_timer() {
    // 1) ROS 2 Twist 메시지 발행
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.5;
    msg.angular.z = 0.0;
    pub_->publish(msg);

    // 2) 시리얼 포트로 바이너리 데이터 전송
    std::vector<uint8_t> buf = {
      0x53,0x54,0x58,  // 'S','T','X'
      0x00,0x01,0x01,  // A=0, E=1, Gear=1
      10,              // Speed
      0,               // Steer
      1,               // Br
      0,               // Al
      0x0D,0x0A        // ETX0, ETX1
    };
    write(fd_, buf.data(), buf.size());
  }

  void configure_port(int fd, speed_t baud) {
    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &tty);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int fd_{-1};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForwardDriveNode>());
  rclcpp::shutdown();
  return 0;
}
