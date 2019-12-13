#include <ros/ros.h>
#include <boost/asio.hpp>
#include <iostream>
#include <mutex>
#include <thread>
#include <condition_variable>

void establishConnection(boost::asio::ip::tcp::socket& socket, const boost::asio::ip::tcp::endpoint& endpoint, boost::asio::io_service& io_service) {
  std::mutex connectMutex;
  std::condition_variable connectCvar;

  ros::Rate retryRate(1.0);
  boost::system::error_code error = boost::asio::error::host_not_found;

  while (ros::ok() && error) {
    std::unique_lock<std::mutex> connectLock(connectMutex);
    std::cout << "before async connect\n";
    socket.async_connect(endpoint, [&error, &connectCvar](const boost::system::error_code& err){std::cout << "in connect handler\n"; error = err; connectCvar.notify_all();});
    std::cout << "after async connect\n";
    if (connectCvar.wait_for(connectLock, std::chrono::seconds(5)) == std::cv_status::timeout) {
      std::cout << "time out\n";
      error = boost::asio::error::timed_out;
    } else {
      std::cout << "not timeout Error: " << error.message() << "\n";
      }
    if (error) {
      ROS_ERROR("Error connecting, will keep trying. Error: %s", error.message().c_str());
      socket.close();
      retryRate.sleep();
    }
  }
}

int main(int argc, char **arcv) {
  ros::init(argc, arcv, "tcp_client");
  ros::Time::init();

  boost::asio::io_service io_service;
  boost::asio::io_service::work work(io_service);
  boost::asio::ip::tcp::socket socket(io_service);
  boost::asio::ip::tcp::resolver resolver(io_service);

  std::string tcpSocketHostname;
  int tcpSocketPort;
  ros::param::param<std::string>("tcpSocketHostname", tcpSocketHostname, "cococutkuri.personalrobotics.cs.washington.edu");
  ros::param::param<int>("tcpSocketPort", tcpSocketPort, 1234);

  boost::asio::ip::tcp::resolver::query query(tcpSocketHostname, std::to_string(tcpSocketPort));
  boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);

  ROS_INFO("Before connect");
  establishConnection(socket, boost::asio::ip::tcp::endpoint(iter->endpoint()), io_service);


  uint8_t buf[256] = {0};

  size_t len = 0, readSize = 0;

  boost::system::error_code error;

  while(ros::ok()) {
    //printf("%d:", len);
    readSize = socket.read_some(boost::asio::buffer(buf, 256), error);
    if (error) {
      ROS_ERROR("Error reading, will disconnect and reconnect. Error: %s", error.message().c_str());
      socket.close();
      establishConnection(socket, boost::asio::ip::tcp::endpoint(iter->endpoint()), io_service);
    }
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
