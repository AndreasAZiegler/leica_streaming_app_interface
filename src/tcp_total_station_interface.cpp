#include <iostream>
#include <string>
#include <thread>

#include "tcp_total_station_interface.h"

TCPTSInterface::TCPTSInterface(void (*f)(const double, const double, const double))
  : io_context_(new boost::asio::io_context()),
    socket_(new boost::asio::ip::tcp::socket(*io_context_)),
    readData_(61),
    locationCallback(f),
    timer_(*io_context_, boost::asio::chrono::milliseconds(500)),
    TSInterface() {}

TCPTSInterface::~TCPTSInterface() {
  contextThread_.join();
}

bool TCPTSInterface::connect(boost::asio::ip::tcp::endpoint endpoint) {
  try {
    socket_->async_connect(endpoint,
          [this](const boost::system::error_code& ec) {
            if (!ec) {
              startReader();
              startTimer();
            }
          });

    contextThread_ = std::thread([this](){ io_context_->run(); });
  } catch (std::exception& e) {
    std::cerr << "Exception: " << e.what() << "\n";
  }
}

void TCPTSInterface::start() {
  std::vector<char> command{'%', 'R', '8', 'Q', ',', '1', ':', 0x0d/*CR*/, 0x0a/*LF*/};

  boost::asio::async_write(*socket_,
                           boost::asio::buffer(command),
                           std::bind(&TCPTSInterface::writeHandler,
                                     this,
                                     std::placeholders::_1,
                                     std::placeholders::_2)
                          );
}

void TCPTSInterface::end() {
  std::vector<char> command {'%', 'R', '8', 'Q', ',', '2', ':', 0x0d/*CR*/, 0x0a/*LF*/};

  boost::asio::async_write(*socket_,
                           boost::asio::buffer(command),
                           std::bind(&TCPTSInterface::writeHandler,
                                     this,
                                     std::placeholders::_1,
                                     std::placeholders::_2)
                          );
}

bool TCPTSInterface::write(std::string str) {}

void TCPTSInterface::startReader() {
  boost::asio::async_read(*socket_,
                          boost::asio::buffer(&readData_[0], readData_.size()),
                          std::bind(&TCPTSInterface::readHandler,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2)
                          );
}

void TCPTSInterface::startTimer() {
  timer_.async_wait(std::bind(&TCPTSInterface::timerHandler,
                              this));
}

void TCPTSInterface::readHandler(const boost::system::error_code& ec,
                                 std::size_t bytes_transferred) {
  std::string str = "";
  for (char ch : readData_) {
    std::cout << ch;
    str += ch;
  }

  if (readData_[0] == 'T') {
    std::vector<std::string> results;

    boost::split(results, str, [](char c){return c == ',';});

    double x = std::stod(results[1]);
    double y = std::stod(results[2]);
    double z = std::stod(results[3]);

    locationCallback(x, y, z);
  }

  boost::asio::async_read(*socket_,
                          boost::asio::buffer(readData_),
                          std::bind(&TCPTSInterface::readHandler,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2)
                          );
}

void TCPTSInterface::writeHandler(const boost::system::error_code& ec,
                                  std::size_t bytes_transferred) {
  if (!ec) {
    std::cout << "Command sent." << std::endl;
  }
}

void TCPTSInterface::timerHandler() {
  std::cout << "Timer" << std::endl;
  startTimer();
}
