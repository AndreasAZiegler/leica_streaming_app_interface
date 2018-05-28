#include <iostream>
#include <string>
#include <thread>
#include <boost/asio.hpp>

#include "tcp_total_station_interface.h"

TCPTSInterface::TCPTSInterface()
  : io_context_(new boost::asio::io_context()),
    socket_(new boost::asio::ip::tcp::socket(*io_context_)),
    readData_(100),
    TSInterface() {}

TCPTSInterface::~TCPTSInterface() {
  contextThread_.join(); 
}

bool TCPTSInterface::connect(boost::asio::ip::tcp::endpoint endpoint) {
  try
  {
    socket_->async_connect(endpoint,
          [this](const boost::system::error_code& ec) {
            if (!ec) {
              startReader();
            }
          });

    //io_context_->run();
    contextThread_ = std::thread([this](){ io_context_->run(); });
  } catch (std::exception& e) {
    std::cerr << "Exception: " << e.what() << "\n";
  }
}

void TCPTSInterface::start() {
  std::vector<char> command{'%', 'R', '8', 'Q', ',', '1', ':'};

  boost::asio::async_write(*socket_,
                           boost::asio::buffer(command),
                           std::bind(&TCPTSInterface::writeHandler,
                                     this,
                                     std::placeholders::_1,
                                     std::placeholders::_2)
                          );
}

void TCPTSInterface::end() {
  std::vector<char> command {'%', 'R', '8', 'Q', ',', '2', ':'};

  boost::asio::async_write(*socket_,
                           boost::asio::buffer(command),
                           std::bind(&TCPTSInterface::writeHandler,
                                     this,
                                     std::placeholders::_1,
                                     std::placeholders::_2)
                          );
}

bool TCPTSInterface::write(std::string str) {
}

void TCPTSInterface::startReader() {
  boost::asio::async_read(*socket_,
                          boost::asio::buffer(readData_),
                          std::bind(&TCPTSInterface::readHandler,
                                    this,
                                    std::placeholders::_1,
                                    std::placeholders::_2)
                          );
}

void TCPTSInterface::readHandler(const boost::system::error_code& ec,
                                 std::size_t bytes_transferred) {
  for (char ch : readData_) {
    std::cout << ch;
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
