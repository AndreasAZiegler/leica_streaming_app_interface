/**
 * \file tcp_total_station_interface.cpp
 * \author Andreas Ziegler
 * \date 31.05.2018
 * \brief Implementation of the tcp total station interface
 */

#include <iostream>
#include <string>
#include <thread>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "tcp_total_station_interface.h"

TCPTSInterface::TCPTSInterface(void (*f)(const double, const double, const double))
  : io_context_(new boost::asio::io_context()),
    socket_(new boost::asio::ip::tcp::socket(*io_context_)),
    //readData_(80),
    //readData_(""),
    locationCallback(f),
    timer_(*io_context_, boost::posix_time::milliseconds(200)),
    TSInterface() {}

TCPTSInterface::~TCPTSInterface() {
  contextThread_.join();
}

void TCPTSInterface::connect(boost::asio::ip::tcp::endpoint endpoint) {
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
  write(command);
}

void TCPTSInterface::end() {
  std::vector<char> command {'%', 'R', '8', 'Q', ',', '2', ':', 0x0d/*CR*/, 0x0a/*LF*/};
  write(command);
}

void TCPTSInterface::startReader() {
  //std::string data;
  boost::asio::async_read_until(*socket_,
                                readData_,
                                "\r\n",
                                std::bind(&TCPTSInterface::readHandler,
                                          this,
                                          std::placeholders::_1,
                                          std::placeholders::_2)
                                );
}

void TCPTSInterface::write(std::vector<char> command) {
  boost::asio::async_write(*socket_,
                           boost::asio::buffer(command),
                           std::bind(&TCPTSInterface::writeHandler,
                                     this,
                                     std::placeholders::_1,
                                     std::placeholders::_2)
                          );
}

void TCPTSInterface::startTimer() {
  timer_.async_wait(std::bind(&TCPTSInterface::timerHandler,
                              this));
}

/**
 * @brief Callback method when a message was received.
 *        Calls the registered callback function with the
 *        x, y and z coordinates of the prism. Also sets flag
 *        to indicate that a message was received.
 *
 * @param ec error code
 * @param bytes_transferred Amount of bytes received
 */
void TCPTSInterface::readHandler(const boost::system::error_code& ec,
                                 std::size_t size) {
  if (!ec) {
    boost::asio::streambuf::const_buffers_type bufs = readData_.data();
    std::string data(boost::asio::buffers_begin(bufs),
                     boost::asio::buffers_begin(bufs) + readData_.size());

    /*
    std::string str = "";
    for (char ch : readData_) {
      std::cout << ch;
      str += ch;
    }
    */
    std::cout << data << std::endl;

    if (data[0] == 'T') {
      std::vector<std::string> results;

      boost::split(results, data, [](char c){return c == ',';});

      double x = std::stod(results[1]);
      double y = std::stod(results[2]);
      double z = std::stod(results[3]);

      locationCallback(x, y, z);

      std::lock_guard<std::mutex> guard(messageReceivedMutex_);
      messagesReceivedFlag_ = true;
    }

    boost::asio::async_read_until(*socket_,
                                  readData_,
                                  "\r\n",
                                  std::bind(&TCPTSInterface::readHandler,
                                            this,
                                            std::placeholders::_1,
                                            std::placeholders::_2)
                                 );
  }
}

void TCPTSInterface::writeHandler(const boost::system::error_code& ec,
                                  std::size_t bytes_transferred) {
  if (!ec) {
    std::cout << "Command sent." << std::endl;
  }
}

void TCPTSInterface::timerHandler() {
  {
    std::lock_guard<std::mutex> guard(messageReceivedMutex_);
    if (!messagesReceivedFlag_) {
      searchPrism();
    }

    messagesReceivedFlag_ = false;
  }

  // Restart timer
  timer_.expires_at(timer_.expires_at() + boost::posix_time::milliseconds(200));
  timer_.async_wait(std::bind(&TCPTSInterface::timerHandler, this));
}

void TCPTSInterface::searchPrism(void) {
  std::cout << "Search prism" << std::endl;
}
