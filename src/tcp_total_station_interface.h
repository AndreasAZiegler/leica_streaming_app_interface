/**
 * \file tcp_total_station_interface.h
 * \author Andreas Ziegler
 * \date 31.05.2018
 * \brief Header file containing the required defintion for the tcp total station interface
 */

#pragma once

#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <functional>
#include <mutex>

#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>

#include "total_station_interface.h"

/**
 * @brief TCP interface for the Leica total station
 */
class TCPTSInterface: public TSInterface {
 public:
  /**
   * @brief Constructor
   *
   * @param f Callback function to receive the x, y and z location of the tracked prism.
   */
  TCPTSInterface(void (*f)(const double, const double, const double));
  ~TCPTSInterface();

  /**
   * @brief Tries to open a tcp connection to the total station
   *        Initializes the tcp connection to the total station
   *        and the timer to detect if no message are received anymore.
   * @param endpoint boost::asio::ip::tcp::endpoint
   */
  void connect(boost::asio::ip::tcp::endpoint endpoint);

  /**
   * @brief Sends the start command to the total station.
   */
  void start() override;

  /**
   * @brief Sends the end command to the total station.
   */
  void end() override;

 private:
  /**
   * @brief Starts the tcp reader.
   */
  void startReader();

  /**
   * @brief Starts the timer to detect if no messages are received anymore.
   */
  void startTimer();

  /**
   * @brief Sends the total station a command.
   *
   * @param command std::vector<char> with the command.
   */
  void write(std::vector<char> str);

  void readHandler(const boost::system::error_code& ec,
                   std::size_t size);

  void writeHandler(const boost::system::error_code& ec,
                    std::size_t bytes_transferred);

  void timerHandler();

  void searchPrism(void);

  std::unique_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<boost::asio::ip::tcp::socket> socket_;

  boost::asio::streambuf readData_;
  std::thread contextThread_;

  std::function<void(const double, const double, const double)> locationCallback;

  boost::asio::deadline_timer timer_;
  bool messagesReceivedFlag_;
  std::mutex messageReceivedMutex_;
};
