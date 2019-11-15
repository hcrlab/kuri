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

  std::mutex connect_mutex;
  std::condition_variable connect_cvar;

  ros::Rate retryRate(1.0);
  boost::system::error_code error = boost::asio::error::host_not_found;

  while (ros::ok() && error) {
    std::unique_lock<std::mutex> connect_lock(connect_mutex);
    socket.async_connect(endpoint, [&error, &connect_cvar, &connect_mutex](const boost::system::error_code& err) {
      // ROS_ERROR("In connect handler");
      std::unique_lock<std::mutex> connect_lock2(connect_mutex);
      error = err;
      connect_lock2.unlock();
      connect_cvar.notify_all();
    });
    // async_io_service_run(io_service);
    std::cv_status timerResult = connect_cvar.wait_for(connect_lock, std::chrono::seconds(5));
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
    connect_lock.unlock();
  }
}

int main(int argc, char **arcv) {
  ros::init(argc, arcv, "tcp_client");
  ros::Time::init();

  ros::NodeHandle nh;

  boost::asio::io_service io_service;
  // boost::asio::io_service::work work(io_service);
  boost::asio::ip::tcp::socket socket(io_service);
  boost::asio::ip::tcp::resolver resolver(io_service);

  std::string tcp_socket_hostname;
  int tcp_socket_port;
  if (!nh.getParam("/tcp_socket_hostname", tcp_socket_hostname)) {
    ROS_ERROR("No hostname set at param /tcp_socket_hostname . Exiting.");
    return -1;
  }
  if (!nh.getParam("/tcp_socket_port", tcp_socket_port)) {
    ROS_ERROR("No port set at param /tcp_socket_port . Exiting.");
    return -1;
  }

  boost::asio::ip::tcp::resolver::query query(tcp_socket_hostname, std::to_string(tcp_socket_port));
  boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);

  ROS_INFO("Before connect");
  boost::asio::io_service::work work(io_service);
  async_io_service_run(io_service);
  establishConnection(socket, boost::asio::ip::tcp::endpoint(iter->endpoint()), io_service);

  uint8_t buf[256] = {0};

  size_t len = 0, readSize = 0;

  boost::system::error_code error;
  std::mutex read_some_mutex;
  std::condition_variable read_some_cvar;

  while(ros::ok()) {
    std::unique_lock<std::mutex> read_some_lock(read_some_mutex);
    socket.async_read_some(boost::asio::buffer(buf, 256), [&error, &readSize, &read_some_cvar, &read_some_mutex](const boost::system::error_code& err, std::size_t bytes_transferred) {
      // ROS_ERROR("In readSome handler");
      std::unique_lock<std::mutex> read_some_lock2(read_some_mutex);
      error = err;
      readSize = bytes_transferred;
      read_some_lock2.unlock();
      read_some_cvar.notify_all();
    });
    // async_io_service_run(io_service);
    std::cv_status timerResult = read_some_cvar.wait_for(read_some_lock, std::chrono::seconds(5));
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

          // This is the NAL unit header of H264 frames sent by the camera.
          // See https://yumichan.net/video-processing/video-compression/introduction-to-h264-nal-unit/
          // for more information. Note, however, that segmenting frames by this
          // header does not always work. That is why in h264_decoder.cpp we use
          // av_parser_parse2 instead.
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
      read_some_lock.unlock();
    }



  return 0;
}
