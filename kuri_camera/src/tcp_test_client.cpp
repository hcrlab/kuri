#include <ros/ros.h>
#include <boost/asio.hpp>
#include <iostream>
#include <mutex>
#include <thread>
#include <boost/thread.hpp>
#include <condition_variable>

void async_io_service_run(boost::asio::io_service& io_service) {
  ROS_INFO("async_io_service_run");
  boost::thread t(boost::bind(&boost::asio::io_service::run, &io_service));
  t.detach();
}

void deestablishConnection(boost::asio::ip::tcp::socket& socket, boost::asio::io_service& io_service) {
  socket.close();
  io_service.reset();
  async_io_service_run(io_service);
}

void establishConnection(boost::asio::ip::tcp::socket& socket, const boost::asio::ip::tcp::endpoint& endpoint, boost::asio::io_service& io_service) {

  std::mutex connectMutex;
  std::condition_variable connectCvar;

  ros::Rate retryRate(1.0);
  boost::system::error_code error = boost::asio::error::host_not_found;

  while (ros::ok() && error) {
    std::unique_lock<std::mutex> connectLock(connectMutex);
    socket.async_connect(endpoint, [&error, &connectCvar, &connectMutex](const boost::system::error_code& err) {
      // ROS_ERROR("In connect handler");
      std::unique_lock<std::mutex> connectLock2(connectMutex);
      error = err;
      connectLock2.unlock();
      connectCvar.notify_all();
    });
    // async_io_service_run(io_service);
    std::cv_status timerResult = connectCvar.wait_for(connectLock, std::chrono::seconds(5));
    if (timerResult == std::cv_status::timeout) {
      ROS_ERROR("Connect Timeout");
      error = boost::asio::error::timed_out;
    } else {
      // ROS_ERROR("Connect No timeout");
    }
    if (error) {
      ROS_ERROR("Error connecting, will keep trying. Error: %s", error.message().c_str());
      deestablishConnection(socket, io_service);
      retryRate.sleep();
    }
    connectLock.unlock();
  }
}

int main(int argc, char **arcv) {
  ros::init(argc, arcv, "tcp_client");
  ros::Time::init();

  boost::asio::io_service io_service;
  // boost::asio::io_service::work work(io_service);
  boost::asio::ip::tcp::socket socket(io_service);
  boost::asio::ip::tcp::resolver resolver(io_service);

  std::string tcpSocketHostname;
  int tcpSocketPort;
  ros::param::param<std::string>("tcpSocketHostname", tcpSocketHostname, "cococutkuri.personalrobotics.cs.washington.edu");
  ros::param::param<int>("tcpSocketPort", tcpSocketPort, 1234);

  boost::asio::ip::tcp::resolver::query query(tcpSocketHostname, std::to_string(tcpSocketPort));
  boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);

  ROS_INFO("Before connect");
  boost::asio::io_service::work work(io_service);
  async_io_service_run(io_service);
  establishConnection(socket, boost::asio::ip::tcp::endpoint(iter->endpoint()), io_service);

  uint8_t buf[256] = {0};

  size_t len = 0, readSize = 0;

  boost::system::error_code error;
  std::mutex readSomeMutex;
  std::condition_variable readSomeCvar;

  while(ros::ok()) {
    std::unique_lock<std::mutex> readSomeLock(readSomeMutex);
    socket.async_read_some(boost::asio::buffer(buf, 256), [&error, &readSize, &readSomeCvar, &readSomeMutex](const boost::system::error_code& err, std::size_t bytes_transferred) {
      // ROS_ERROR("In readSome handler");
      std::unique_lock<std::mutex> readSomeLock2(readSomeMutex);
      error = err;
      readSize = bytes_transferred;
      readSomeLock2.unlock();
      readSomeCvar.notify_all();
    });
    // async_io_service_run(io_service);
    std::cv_status timerResult = readSomeCvar.wait_for(readSomeLock, std::chrono::seconds(5));
    if (timerResult == std::cv_status::timeout) {
      ROS_ERROR("Read Timeout");
      error = boost::asio::error::timed_out;
    } else {
      // ROS_ERROR("Read No timeout");
    }
    if (error) {
      ROS_ERROR("Error reading, will disconnect and reconnect. Error: %s", error.message().c_str());
      deestablishConnection(socket, io_service);
      establishConnection(socket, boost::asio::ip::tcp::endpoint(iter->endpoint()), io_service);
    }
    len += readSize;
      for (uint32_t i = 0; i < readSize; i++)
      {
          // std::ostringstream oldBufStream;
          // oldBufStream << "oldBufStream: ";
          // for (uint32_t j = 0; j < readSize; j++)
          // {
          //     if (j > 0) oldBufStream << ":";
          //     oldBufStream <<std::hex << buf[j];
          // }
          if (i < readSize-5 && buf[i] == 0 && buf[i+1] == 0 && buf[i+2] == 0 && buf[i+3] == 1 && buf[i+4] == 9) {
              len -= readSize-i;
              std::cout << "read " << len << std::endl;
              len = readSize-i;
              for (uint32_t j = i; j < readSize; j++)
              {
                  if (j > i) printf(":");
                  printf("%02X", buf[j]);
              }
              printf("\n");
              break;
          }
      }
      readSomeLock.unlock();
    }



  return 0;
}
