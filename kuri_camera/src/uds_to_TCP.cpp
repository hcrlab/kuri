#include <ros/ros.h>
#include <boost/asio.hpp>
#include <iostream>
#include <chrono>

int main(int argc, char **arcv) {
  ros::init(argc, arcv, "uds_to_tcp");

  // Parameters
  const char* udsSocketPath = "/var/run/madmux/ch2.sock";
  const int bufferSize = 16384; // bytes
  int tcpSocketPort;
  ros::param::param<int>("tcpSocketPort", tcpSocketPort, 1234);

  // Create the TCP Socket
  boost::asio::io_service tcp_io_service;
  boost::asio::ip::tcp::acceptor tcp_acceptor(tcp_io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), tcpSocketPort));
  boost::asio::ip::tcp::socket tcp_socket(tcp_io_service);

  // Main loop -- only connect to the Unix Domain Socket once a TCP connection has been established
  while (ros::ok()) {

    // Wait for a TCP client
    ROS_INFO("Waiting for TCP Client");
    tcp_acceptor.accept(tcp_socket);
    ROS_INFO("Connected to TCP Client");

    // Create a connection to the Madmux Unix Domain Socket
    boost::asio::io_service uds_io_service;
    boost::asio::local::stream_protocol::socket uds_socket(uds_io_service);
    boost::system::error_code err;

    uds_socket.connect(boost::asio::local::stream_protocol::endpoint(udsSocketPath));

    uint8_t buf[bufferSize] = {0};

    size_t readSize = 0, len=0;

    // Inner loop -- read bufferSize bytes at a time and send them to the TCP socket
    while(ros::ok()) {
      readSize = uds_socket.read_some(boost::asio::buffer(buf, 256));
      boost::asio::write(tcp_socket, boost::asio::buffer(buf, readSize), err);
      if (err) {
        ROS_INFO("Error writing to TCP socket: %s", err.message().c_str());
        break;
      }
    /*  // Uncomment these to determine (through manual calculation) network latency for one H264 packet
      len += readSize;
      for (uint32_t i = 0; i < readSize; i++)
      {
          if (i < readSize-5 && buf[i] == 0 && buf[i+1] == 0 && buf[i+2] == 0 && buf[i+3] == 1 && buf[i+4] == 9) {
              len -= readSize-i;

              uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

              std::cout << "read " << len << " " << now << std::endl;
              len = readSize-i;
              //for (uint32_t j = 0; j < readSize; j++)
              //{
              //    if (j > 0) printf(":");
              //    printf("%02X", buf[j]);
              //}
              //printf("\n");
              break;
          }
      }*/
    }

    uds_socket.close();
    tcp_socket.close();

  }

  return 0;
}
