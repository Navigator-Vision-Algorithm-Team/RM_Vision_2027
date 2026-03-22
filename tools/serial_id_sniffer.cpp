#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace
{
volatile sig_atomic_t g_running = 1;

void handleSignal(int)
{
  g_running = 0;
}

speed_t baudToTermios(int baud)
{
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return 0;
  }
}

uint16_t readLe16(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

uint64_t nowMs()
{
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

std::set<uint16_t> parseFilter(const std::string & s)
{
  std::set<uint16_t> ids;
  if (s.empty()) {
    return ids;
  }

  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (item.empty()) {
      continue;
    }
    char * end = nullptr;
    unsigned long v = std::strtoul(item.c_str(), &end, 0);
    if (end == item.c_str() || *end != '\0' || v > 0xFFFFUL) {
      std::cerr << "Ignore invalid id in filter: " << item << std::endl;
      continue;
    }
    ids.insert(static_cast<uint16_t>(v));
  }
  return ids;
}

void printUsage(const char * prog)
{
  std::cout << "Usage: " << prog << " --device /dev/ttyACM0 [options]\n"
            << "Options:\n"
            << "  --device <path>          Serial device path, e.g. /dev/ttyACM0\n"
            << "  --baud <num>             Baud rate, default 921600\n"
            << "  --filter <id1,id2,...>   ID filter, e.g. 0x0001,0x0201\n"
            << "  --show-all               Print all IDs (default behavior if no filter)\n"
            << "  --max-payload <num>      Max payload length accepted, default 1024\n"
            << "  --help                   Show this help\n";
}

bool configurePort(int fd, int baud)
{
  termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    std::cerr << "tcgetattr failed: " << strerror(errno) << std::endl;
    return false;
  }

  cfmakeraw(&tty);

  speed_t speed = baudToTermios(baud);
  if (speed == 0) {
    std::cerr << "Unsupported baud: " << baud << std::endl;
    return false;
  }

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    std::cerr << "tcsetattr failed: " << strerror(errno) << std::endl;
    return false;
  }

  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string device;
  int baud = 921600;
  std::string filterArg;
  bool showAll = true;
  uint16_t maxPayload = 1024;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--device" && i + 1 < argc) {
      device = argv[++i];
    } else if (arg == "--baud" && i + 1 < argc) {
      baud = std::atoi(argv[++i]);
    } else if (arg == "--filter" && i + 1 < argc) {
      filterArg = argv[++i];
      showAll = false;
    } else if (arg == "--show-all") {
      showAll = true;
    } else if (arg == "--max-payload" && i + 1 < argc) {
      int v = std::atoi(argv[++i]);
      if (v > 0 && v <= 65535) {
        maxPayload = static_cast<uint16_t>(v);
      }
    } else if (arg == "--help") {
      printUsage(argv[0]);
      return 0;
    } else {
      std::cerr << "Unknown argument: " << arg << std::endl;
      printUsage(argv[0]);
      return 1;
    }
  }

  if (device.empty()) {
    std::cerr << "Missing --device" << std::endl;
    printUsage(argv[0]);
    return 1;
  }

  std::set<uint16_t> filterIds = parseFilter(filterArg);

  signal(SIGINT, handleSignal);
  signal(SIGTERM, handleSignal);

  int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    std::cerr << "Open serial failed: " << strerror(errno) << std::endl;
    return 1;
  }

  if (!configurePort(fd, baud)) {
    close(fd);
    return 1;
  }

  std::cout << "Listening on " << device << " @ " << baud << " baud" << std::endl;
  if (!showAll && !filterIds.empty()) {
    std::cout << "Filter IDs:";
    for (uint16_t id : filterIds) {
      std::cout << " 0x" << std::hex << std::setw(4) << std::setfill('0') << id;
    }
    std::cout << std::dec << std::setfill(' ') << std::endl;
  } else {
    std::cout << "Showing all IDs" << std::endl;
  }

  std::vector<uint8_t> buf;
  buf.reserve(4096);

  while (g_running) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;

    int ret = select(fd + 1, &rfds, nullptr, nullptr, &tv);
    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      }
      std::cerr << "select failed: " << strerror(errno) << std::endl;
      break;
    }

    if (ret == 0) {
      continue;
    }

    uint8_t temp[512];
    ssize_t n = read(fd, temp, sizeof(temp));
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }
      std::cerr << "read failed: " << strerror(errno) << std::endl;
      break;
    }

    if (n == 0) {
      continue;
    }

    buf.insert(buf.end(), temp, temp + n);

    while (true) {
      auto it = std::find(buf.begin(), buf.end(), 0xA5);
      if (it == buf.end()) {
        buf.clear();
        break;
      }

      if (it != buf.begin()) {
        buf.erase(buf.begin(), it);
      }

      if (buf.size() < 7) {
        break;
      }

      uint16_t dataLen = readLe16(&buf[1]);
      if (dataLen > maxPayload) {
        std::cerr << "drop invalid frame, payload too large: " << dataLen << std::endl;
        buf.erase(buf.begin());
        continue;
      }

      uint16_t cmdId = readLe16(&buf[5]);
      size_t frameSize = static_cast<size_t>(7) + static_cast<size_t>(dataLen) + static_cast<size_t>(2);
      if (buf.size() < frameSize) {
        break;
      }

      bool pass = showAll || filterIds.empty() || (filterIds.find(cmdId) != filterIds.end());
      if (pass) {
        std::cout << "[" << nowMs() << " ms]"
                  << " cmd_id=0x" << std::hex << std::setw(4) << std::setfill('0') << cmdId
                  << std::dec << std::setfill(' ')
                  << " payload_len=" << dataLen
                  << " frame_len=" << frameSize
                  << std::endl;
      }

      buf.erase(buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(frameSize));
    }
  }

  close(fd);
  std::cout << "Exit." << std::endl;
  return 0;
}
