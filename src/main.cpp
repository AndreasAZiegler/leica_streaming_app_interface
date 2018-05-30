/*
g++ main.cpp -lboost_system -lboost_thread -lpthread -o leica_streaming_receiver
*/

#include <iostream>
#include <string>
#include <boost/asio.hpp>

#include "tcp_total_station_interface.h"

void locationCallback(const double x,
                      const double y,
                      const double z) {
  std::cout << "Prism is at x: " << x 
            << " y: " << y
            << " z: " << z << std::endl;
  std::cout << std::endl;
}


int main(void) {

  // Getting the IP address
  std::string ip;
  std::cout << "Please enter the simulator's IP address: ";
  std::cin >> ip;

  int port = 5001;

  boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(ip),
                                          port);

  TCPTSInterface ts(&locationCallback);
  ts.connect(endpoint);

  std::string ch;
  while (true) {
    getline(std::cin, ch);

    if (!ch.empty()) {
      std::cout << ch << std::endl;
      switch (ch.c_str()[0]) {
        case 'e':
          ts.end();
          std::cout << "Measurements stopped." << std::endl;
          break;
        case 's':
          ts.start();
          break;
      }
    }
  }

  return 0;
}
