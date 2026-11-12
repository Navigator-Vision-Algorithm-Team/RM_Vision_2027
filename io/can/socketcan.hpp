#ifndef IO__SOCKETCAN_HPP
#define IO__SOCKETCAN_HPP

#include <linux/can.h>
#include <net/if.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "tools/logger.hpp"

using namespace std::chrono_literals;

constexpr int MAX_EVENTS = 10;

namespace io
{
class SocketCAN
{
public:
  SocketCAN(
    const std::string & interface, std::function<void(const can_frame & frame)> rx_handler,
    std::function<void()> before_open = {})
  : interface_(interface),
    socket_fd_(-1),
    epoll_fd_(-1),
    quit_(false),
    ok_(false),
    rx_handler_(rx_handler),
    before_open_(std::move(before_open))
  {
    try_open();

    // 守护线程
    try {
      daemon_thread_ = std::thread{[this] {
        while (!quit_) {
          std::this_thread::sleep_for(100ms);

          if (ok_) continue;

          if (read_thread_.joinable()) read_thread_.join();

          close();
          try_open();
        }
      }};
    } catch (...) {
      quit_ = true;
      if (read_thread_.joinable()) read_thread_.join();
      close();
      throw;
    }
  }

  ~SocketCAN()
  {
    quit_ = true;
    if (daemon_thread_.joinable()) daemon_thread_.join();
    if (read_thread_.joinable()) read_thread_.join();
    close();
    tools::logger()->info("SocketCAN destructed.");
  }

  void write(can_frame * frame, bool nonblocking = false) const
  {
    std::lock_guard lock(socket_mutex_);
    if (socket_fd_ < 0 ||
        ::send(socket_fd_, frame, sizeof(can_frame), nonblocking ? MSG_DONTWAIT : 0) !=
          static_cast<ssize_t>(sizeof(can_frame)))
      throw std::runtime_error("Unable to write!");
  }

private:
  std::string interface_;
  int socket_fd_;
  int epoll_fd_;
  std::atomic<bool> quit_;
  std::atomic<bool> ok_;
  mutable std::mutex socket_mutex_;
  std::thread read_thread_;
  std::thread daemon_thread_;
  can_frame frame_;
  epoll_event events_[MAX_EVENTS];
  std::function<void(const can_frame & frame)> rx_handler_;
  std::function<void()> before_open_;

  void open()
  {
    std::lock_guard lock(socket_mutex_);
    if (interface_.size() >= IFNAMSIZ) throw std::runtime_error("CAN interface name too long!");
    socket_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
    if (socket_fd_ < 0) throw std::runtime_error("Error opening socket!");

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
      throw std::runtime_error("Error getting interface index!");

    sockaddr_can addr;
    std::memset(&addr, 0, sizeof(sockaddr_can));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd_, (sockaddr *)&addr, sizeof(sockaddr_can)) < 0) {
      throw std::runtime_error("Error binding socket to interface!");
    }

    epoll_event ev;
    epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
    if (epoll_fd_ == -1) throw std::runtime_error("Error creating epoll file descriptor!");

    ev.events = EPOLLIN;
    ev.data.fd = socket_fd_;
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, ev.data.fd, &ev))
      throw std::runtime_error("Error adding socket to epoll file descriptor!");

    tools::logger()->info("SocketCAN opened.");

    // 接收线程
    ok_ = true;
    read_thread_ = std::thread([this]() {
      while (!quit_) {
        std::this_thread::sleep_for(10us);

        try {
          read();
        } catch (const std::exception & e) {
          tools::logger()->warn("SocketCAN::read() failed: {}", e.what());
          ok_ = false;
          break;
        }
      }
    });
  }

  void try_open()
  {
    try {
      if (before_open_) before_open_();
      open();
    } catch (const std::exception & e) {
      close();
      tools::logger()->warn("SocketCAN::open() failed: {}", e.what());
    }
  }

  void read()
  {
    int num_events = epoll_wait(epoll_fd_, events_, MAX_EVENTS, 2);
    if (num_events == -1 && errno == EINTR) return;
    if (num_events == -1) throw std::runtime_error("Error wating for events!");

    for (int i = 0; i < num_events; i++) {
      ssize_t num_bytes = recv(socket_fd_, &frame_, sizeof(can_frame), MSG_DONTWAIT);
      if (num_bytes == -1 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) continue;
      if (num_bytes == -1) throw std::runtime_error("Error reading from SocketCAN!");
      if (num_bytes != static_cast<ssize_t>(sizeof(can_frame)))
        throw std::runtime_error("Incomplete CAN frame!");

      rx_handler_(frame_);
    }
  }

  void close()
  {
    std::lock_guard lock(socket_mutex_);
    ok_ = false;
    if (epoll_fd_ != -1) ::close(epoll_fd_);
    if (socket_fd_ != -1) ::close(socket_fd_);
    epoll_fd_ = -1;
    socket_fd_ = -1;
  }
};

}  // namespace io

#endif  // IO__SOCKETCAN_HPP
