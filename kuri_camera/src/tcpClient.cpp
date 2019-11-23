#include <ros/ros.h>
#include <boost/asio.hpp>
#include <iostream>

int main(int argc, char **arcv) {
  ros::init(argc, arcv, "tcp_client");

  boost::asio::io_service io_service;
  boost::asio::ip::tcp::socket socket(io_service);
  boost::asio::ip::tcp::resolver resolver(io_service);
  boost::asio::ip::tcp::resolver::query query("cococutkuri.personalrobotics.cs.washington.edu", "1234");
  boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);
  ROS_INFO("Before connect");
  socket.connect(boost::asio::ip::tcp::endpoint(iter->endpoint()));  
  
  uint8_t buf[256] = {0};

  size_t len = 0, readSize = 0;

  while(ros::ok()) {
    //printf("%d:", len);
    readSize = socket.read_some(boost::asio::buffer(buf, 256));
    len += readSize;
      for (uint32_t i = 0; i < readSize; i++)
      {
          if (i < readSize-5 && buf[i] == 0 && buf[i+1] == 0 && buf[i+2] == 0 && buf[i+3] == 1 && buf[i+4] == 9) {
              len -= readSize-i;
              std::cout << "read " << len << std::endl;
              len = readSize-i;
              for (uint32_t j = 0; j < readSize; j++)
              {
                  if (j > 0) printf(":");
                  printf("%02X", buf[j]);
              }
              printf("\n");
              break;
          }
      }
    }

    

  return 0;
}
