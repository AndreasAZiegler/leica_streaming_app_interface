#pragma once

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <functional>

#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>

#include "total_station_interface.h"

class TCPTSInterface: public TSInterface {
 public:
  TCPTSInterface(void (*f)(const double, const double, const double));
  ~TCPTSInterface();

  bool connect(boost::asio::ip::tcp::endpoint endpoint);

  void start() override;
  void end() override;


 private:
  void startReader();

  void startTimer();

  bool write(std::string str);

  void readHandler(const boost::system::error_code& ec,
                   std::size_t bytes_transferred);

  void writeHandler(const boost::system::error_code& ec,
                    std::size_t bytes_transferred);

  void timerHandler();

  std::unique_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<boost::asio::ip::tcp::socket> socket_;

  std::vector<char> readData_;
  std::thread contextThread_;

  std::function<void(const double, const double, const double)> locationCallback;

  boost::asio::steady_timer timer_;
};
