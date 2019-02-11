#include <cassert>
#include "/opt/gizmo/include/madmux/madmux.h"
#include <vector>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <fstream>

using std::string;
using std::vector;

mdx_stream *stream;
std::mutex m;
std::condition_variable got_frame;
bool frame_processed;


static void stream_callback(uint8_t *buffer, uint32_t size, void *self) {
  // Use the user data pointer as a pointer to an instance of the Publisher, then call
  // the nice callback on it.
  std::unique_lock<std::mutex> lock(m);
  std::cout << size << std::endl;
  std::cout << (void*)buffer << std::endl;
  std::cout << buffer[0] << std::endl;
  std::ofstream image_file;
  image_file.open("camera_image.jpg", std::ios::binary);
  image_file.write((char*)buffer, size);
  image_file.close();
  frame_processed = true;
  lock.unlock();
  got_frame.notify_all();

}




int main(int argc, char **arcv) {
  stream = mdx_open("/var/run/madmux/ch3.sock");
  mdx_register_cb(stream, stream_callback, nullptr);
  // Causes a linker error "undefined reference"
  //mdx_set_resolution(stream, 1920,1080);
  {
    std::unique_lock <std::mutex> lock(m);
    frame_processed = false;
    got_frame.wait(lock, []() { return frame_processed; });
  }
  {
    std::unique_lock <std::mutex> lock(m);
    frame_processed = false;
    got_frame.wait(lock, []() { return frame_processed; });
  }

  mdx_close(stream);
}