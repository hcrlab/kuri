#include <cassert>
#include <madmux/madmux.h>
#include <ros/ros.h>
#include <vector>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <condition_variable>
#include <sensor_msgs/CompressedImage.h>
#include <boost/asio.hpp>

struct KuriCameraROSPublisher {
  mdx_stream *stream;
  boost::asio::io_service tcp_io_service;
  boost::asio::ip::tcp::socket tcp_socket;

  // You can read about this pattern in the boost docs:
  // http://www.crystalclearsoftware.com/cgi-bin/boost_wiki/wiki.pl?Generalizing_C-Style_Callbacks
  static void stream_callback_thunk(uint8_t *buffer, uint32_t size, void *self) {
    // Use the user data pointer as a pointer to an instance of the Publisher, then call
    // the nice callback on it.
    static_cast<KuriCameraROSPublisher *>(self)->data_callback(buffer, size);
  }

  KuriCameraROSPublisher(ros::NodeHandle &nh) : tcp_socket(tcp_io_service) {
    int tcp_socket_port;
    if (!nh.getParam("/tcp_socket_port", tcp_socket_port)) {
      ROS_ERROR("No port set at param /tcp_socket_port . Exiting.");
    }

    // Create the TCP Socket
    boost::asio::ip::tcp::acceptor tcp_acceptor(tcp_io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), tcp_socket_port));
    // Wait for a TCP client -- TODO put this in a loop
    ROS_INFO("Waiting for TCP Client");
    tcp_acceptor.accept(tcp_socket);
    ROS_INFO("Connected to TCP Client");

    stream = mdx_open("/var/run/madmux/ch2.sock");
    mdx_register_cb(stream, stream_callback_thunk, static_cast<void *>(this));
  }

  ~KuriCameraROSPublisher() {
    mdx_close(stream);
  }

  void data_callback(uint8_t *buffer, uint32_t size) {
    boost::system::error_code err = boost::asio::error::host_not_found;
    boost::asio::write(tcp_socket, boost::asio::buffer(buffer, size), err);
    if (err) {
      ROS_INFO("Error writing to TCP socket: %s", err.message().c_str());
    }
  }
};

int main(int argc, char **arcv) {
  ros::init(argc, arcv, "uds_to_tcp_using_madmux_library");
  ros::NodeHandle nh;
  KuriCameraROSPublisher wrapper(nh);
  ros::spin();
}
