#include <ros/ros.h>
#include <boost/asio.hpp>
#include <iostream>
#include <chrono>

int main(int argc, char **arcv) {
  ros::init(argc, arcv, "uds_to_tcp");
  ros::NodeHandle nh;

  // Parameters
  const char* uds_socket_path = "/var/run/madmux/ch2.sock"; // ch1 is also fine -- that just contains a higher-quality H264 stream
  const int buffer_size = 16384; // bytes
  int tcp_socket_port;
  if (!nh.getParam("/tcp_socket_port", tcp_socket_port)) {
    ROS_ERROR("No port set at param /tcp_socket_port . Exiting.");
    return -1;
  }

  // The number of seconds to sleep between successive attempts to reconnect to madmux's UDS stream
  int sleep_secs;
  if (!ros::param::get(ros::this_node::getName()+"/sleep_secs", sleep_secs))
  {
    sleep_secs = 1;
  }

  // Create the TCP Socket
  boost::asio::io_service tcp_io_service;
  boost::asio::ip::tcp::acceptor tcp_acceptor(tcp_io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), tcp_socket_port));
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
    boost::system::error_code err = boost::asio::error::host_not_found;

    while (err) {
      uds_socket.connect(boost::asio::local::stream_protocol::endpoint(uds_socket_path), err);
      if (err) {
        ROS_INFO("Error connecting to the UDS socket, will retry after %d seconds: %s", sleep_secs, err.message().c_str());
        ros::Duration(sleep_secs).sleep();
      }
    }

    uint8_t buf[buffer_size] = {0};

    size_t read_size = 0, len=0;

    // Inner loop -- read buffer_size bytes at a time and send them to the TCP socket
    while(ros::ok()) {
      read_size = uds_socket.read_some(boost::asio::buffer(buf, 256));
      boost::asio::write(tcp_socket, boost::asio::buffer(buf, read_size), err);
      if (err) {
        ROS_INFO("Error writing to TCP socket: %s", err.message().c_str());
        break;
      }
    /*  // Uncomment these to determine (through manual calculation) network latency for one H264 packet
      len += read_size;
      for (uint32_t i = 0; i < read_size; i++)
      {
          if (i < read_size-5 && buf[i] == 0 && buf[i+1] == 0 && buf[i+2] == 0 && buf[i+3] == 1 && buf[i+4] == 9) {
              len -= read_size-i;

              uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

              std::cout << "read " << len << " " << now << std::endl;
              len = read_size-i;
              //for (uint32_t j = 0; j < read_size; j++)
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
