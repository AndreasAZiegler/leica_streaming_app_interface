#pragma once

#include <vector>
#include <string>
#include <memory>
#include <thread>

#include "total_station_interface.h"

class TCPTSInterface: public TSInterface {
 public:
  TCPTSInterface();
  ~TCPTSInterface();

  bool connect(boost::asio::ip::tcp::endpoint endpoint);

  void start() override;
  void end() override;


 private:
  void startReader();
  bool write(std::string str);
  void readHandler(const boost::system::error_code& ec,
                   std::size_t bytes_transferred);
  void writeHandler(const boost::system::error_code& ec,
                    std::size_t bytes_transferred);

  std::unique_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<boost::asio::ip::tcp::socket> socket_;

  std::vector<char> readData_;
  std::thread contextThread_;
};
