#include "io/can/soldier_cboard.hpp"

// Keep checks active when the standalone project is built in Release mode.
#ifdef NDEBUG
#undef NDEBUG
#endif

#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

using Board = io::SoldierCBoard;
using namespace std::chrono_literals;

io::minipc::Payload pose(std::uint8_t sequence, std::uint16_t stamp, std::uint8_t flags = 0x17)
{
  return {sequence, flags, 0, 0, 0, 0, static_cast<std::uint8_t>(stamp & 255),
          static_cast<std::uint8_t>(stamp >> 8)};
}

struct Fixture
{
  Board::TimePoint now{1s};
  std::vector<io::minipc::Payload> written;
  bool fail = false;
  Board board{
    [this](std::uint32_t identifier, const io::minipc::Payload & payload) {
      assert(identifier == io::minipc::kCommandId);
      if (fail) throw std::runtime_error("simulated CAN failure");
      written.push_back(payload);
    },
    [this] { return now; }};
};

int main()
{
  const io::minipc::Command target{99, 9, -1.234, -0.25, true, false};
  {
    Fixture f;
    assert(!f.board.snapshot());
    assert(!f.board.pose_at(f.now));
    assert(f.board.send(target) == Board::SendResult::stale_reference);
    assert(f.written.back()[1] == 0x10);
    assert(f.board.receive(0x511, pose(254, 65530)));
    const auto reference = f.board.pose_at(f.now);
    assert(reference && reference->value.reference_ms == 65530);
    assert(f.board.send(target, reference) == Board::SendResult::sent);
    assert(f.written.back()[0] == 1 && f.written.back()[1] == 0x11);
    assert(f.written.back()[6] == 0xFA && f.written.back()[7] == 0xFF);
    assert(!f.board.pose_at(f.now - 1ms));
    assert(!f.board.pose_at(f.now + 1ms));
    f.now += 10ms;
    assert(f.board.receive(0x511, pose(255, 4)));
    assert(f.board.session() == reference->session);
    assert(f.board.pose_at(f.now - 1ms)->value.reference_ms == 65530);
    assert(f.board.send(target, reference) == Board::SendResult::sent);
    assert(f.written.back()[6] == 0xFA); // Original reference, not the new stamp.
    f.now += 10ms;
    assert(f.board.receive(0x511, pose(0, 14)));
    assert(f.board.session() == reference->session);
    f.now += 70ms;
    assert(!f.board.receive(0x511, pose(0, 14)));
    f.now += 30ms;
    assert(!f.board.snapshot());
    assert(f.board.send(target, reference) == Board::SendResult::stale_reference);
    assert(f.written.back()[1] == 0x10 && f.written.back()[2] == 0);
  }
  {
    Fixture f;
    f.board.receive(0x511, pose(1, 1000));
    const auto reference = f.board.pose_at(f.now);
    f.now += 1ms;
    f.board.receive(0x511, pose(2, 1099));
    f.now += 1ms;
    assert(f.board.send(target, reference) == Board::SendResult::stale_reference);
  }
  {
    Fixture f;
    f.board.receive(0x511, pose(1, 100));
    const auto reference = f.board.pose_at(f.now);
    f.now += 100ms;
    f.board.receive(0x511, pose(1, 100));
    assert(f.board.session() != reference->session);
    assert(f.board.send(target, reference) == Board::SendResult::stale_reference);
    f.board.reset();
    assert(!f.board.snapshot());
  }
  {
    Fixture f;
    f.board.receive(0x511, pose(1, 100));
    const auto old = f.board.pose_at(f.now);
    f.now += 10ms;
    f.board.receive(0x511, pose(2, 110, 0x10));
    assert(f.board.snapshot() && !f.board.snapshot()->pose.value.pose_valid);
    assert(f.board.send(target, old) == Board::SendResult::stale_reference);
    f.now += 10ms;
    f.board.receive(0x511, pose(3, 120));
    assert(f.board.snapshot());
    f.now += 10ms;
    f.board.receive(0x511, pose(4, 10)); // Backward tick jump.
    assert(f.board.snapshot()->pose.session != old->session);
  }
  {
    Fixture f;
    f.board.receive(0x511, pose(1, 100));
    const io::minipc::Payload chassis{1, 0, 0, 0, 0, 0, 0, 0};
    const io::minipc::Payload heat{2, 1, 0, 0, 0, 0, 0, 0};
    f.board.receive(0x512, chassis);
    f.board.receive(0x514, heat);
    assert(f.board.snapshot()->chassis);
    assert(!f.board.snapshot()->heat); // Different snapshot sequence.
    const io::minipc::Payload robot{1, 0xC1, 3, 7, 0xF0, 0x0A, 0, 0};
    f.board.receive(0x513, robot);
    assert(f.board.snapshot()->robot->bullet_speed_m_s == 28.0);
    const auto old = f.board.pose_at(f.now);
    auto request = robot;
    request[0] = 2;
    request[1] |= 2;
    f.board.receive(0x513, request);
    assert(!f.board.snapshot());
    assert(f.board.send(target, old) == Board::SendResult::stale_reference);
  }
  {
    Fixture f;
    f.board.receive(0x511, pose(1, 100));
    const auto reference = f.board.pose_at(f.now);
    auto invalid = target;
    invalid.pitch_raw_rad = 0.401;
    assert(f.board.send(invalid, reference) == Board::SendResult::invalid_command);
    assert(f.written.back()[1] == 0x10);
    f.fail = true;
    assert(f.board.send(target, reference) == Board::SendResult::transport_error);
    f.fail = false;
    assert(f.board.send(target, reference) == Board::SendResult::sent);
    assert(f.written.back()[0] == 1);
    for (int i = 0; i < 255; ++i) assert(f.board.send({}) == Board::SendResult::sent);
    assert(f.written.back()[0] == 0); // Transmit sequence wraps through 255.
    assert(!f.board.receive(0x80000511, pose(2, 110)));
  }
  {
    std::atomic<int> sent{0};
    std::atomic<bool> finished{false};
    Board board([&](std::uint32_t id, const io::minipc::Payload & payload) {
      assert(id == 0x510 && (payload[1] & 0xF0) == 0x10);
      ++sent;
    });
    std::thread receiver([&] {
      for (unsigned i = 0; i < 5000; ++i)
        board.receive(0x511, pose(static_cast<std::uint8_t>(i), static_cast<std::uint16_t>(i)));
      finished = true;
    });
    do {
      const auto reference = board.pose_at(Board::Clock::now());
      board.send(target, reference);
      board.snapshot();
    } while (!finished);
    receiver.join();
    assert(sent > 0);
  }
  std::cout << "PASS: freshness, original references, receive/transmit wrap, session resets, "
               "snapshot grouping, stale fallback, transport errors and concurrent access\n";
}
