/*
 * Code adapted from https://www.roxlu.com/2014/039/decoding-h264-and-yuv420p-playback
 */

#include "h264_decoder.h"

H264Decoder::H264Decoder()
  :codec(NULL)
  ,codec_context(NULL)
  ,parser(NULL)
  ,frame(0)
  ,is_alive(false)
  ,tcp_socket(io_service)
  ,work(NULL)
  ,tcp_timeout(5)
{
  got_picture = 0;
  len_callback_value = new int;
  avcodec_register_all();
}

H264Decoder::~H264Decoder() {

  if(parser) {
    av_parser_close(parser);
    parser = NULL;
  }

  if(codec_context) {
    avcodec_close(codec_context);
    av_free(codec_context);
    codec_context = NULL;
  }

  std::unique_lock<std::mutex> picture_lock(picture_mutex);
  if(picture) {
    av_free(picture);
    picture = NULL;
  }
  picture_lock.lock();

  if(packet) {
    av_free_packet(packet);
    packet = NULL;
  }

  if (is_alive) {
    is_alive = false;
    read_buffer_thread.join();
    read_frame_thread.join();
    convert_to_mat_thread.join();
  }

  got_picture = 0;
  frame = 0;
}

// https://timvanoosterhout.wordpress.com/2015/07/02/converting-an-ffmpeg-avframe-to-and-opencv-mat/
void H264Decoder::avframeToMat(const AVFrame * frame, cv::Mat& image)
{
    int width = frame->width;
    int height = frame->height;

    // Allocate the opencv mat and store its stride in a 1-element array
    if (image.rows != height || image.cols != width || image.type() != CV_8UC3) {
      image = cv::Mat(height, width, CV_8UC3);
    }
    int cv_linesizes[1];
    cv_linesizes[0] = image.step1();

    // Convert the color format and write directly to the opencv matrix
    SwsContext* conversion = sws_getContext(width, height, /*(AVPixelFormat) frame->format*/AV_PIX_FMT_YUV420P, width, height, AV_PIX_FMT_BGR24 /* PIX_FMT_BGR24 */, SWS_FAST_BILINEAR, NULL, NULL, NULL);
    sws_scale(conversion, frame->data, frame->linesize, 0, height, &image.data, cv_linesizes); // copies data
    sws_freeContext(conversion);
}

void H264Decoder::convertToMat() {
  while (is_alive) {
    std::unique_lock<std::mutex> picture_lock(picture_mutex);
    got_picture_cvar.wait(picture_lock);
    std::unique_lock<std::mutex> cv_image_lock(cv_image_mutex);
    if (got_picture) {
      avframeToMat(picture, cv_image);
      cv_image_recv_timestamp = picture_recv_timestamp;
      cv_image_lock.unlock();
      decoded_new_frame.notify_all();
    }
    picture_lock.unlock();
  }
}

int H264Decoder::getMostRecentFrame(cv::Mat &image, uint64_t &recv_timestamp) {
  std::unique_lock<std::mutex> cv_image_lock(cv_image_mutex);
  image = cv_image.clone();
  recv_timestamp = cv_image_recv_timestamp;
  cv_image_lock.unlock();
}

bool H264Decoder::load(std::string hostname, std::string port) {

  codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if(!codec) {
    printf("Error: cannot find the h264 codec: %s:%s\n", hostname.c_str(), port.c_str());
    return false;
  }

  codec_context = avcodec_alloc_context3(codec);

  if(codec->capabilities & CODEC_CAP_TRUNCATED) {
    codec_context->flags |= CODEC_FLAG_TRUNCATED;
  }

  if(avcodec_open2(codec_context, codec, NULL) < 0) {
    printf("Error: could not open codec.\n");
    return false;
  }

  // Get the TCP endpoint
  boost::asio::ip::tcp::resolver resolver(io_service);
  boost::asio::ip::tcp::resolver::query query(hostname, port);
  iter = resolver.resolve(query);

  std::unique_lock<std::mutex> picture_lock(picture_mutex);
  // NOTE: depending on your version of libav, you may need the first or second
  // function below to allocate a frame.
  picture = av_frame_alloc(); // avcodec_alloc_frame(); // 
  picture_lock.unlock();
  parser = av_parser_init(AV_CODEC_ID_H264);
  packet = new AVPacket;
  av_init_packet(packet);

  if(!parser) {
    printf("Erorr: cannot create H264 parser.\n");
    return false;
  }

  return true;
}

void H264Decoder::readFrame() {
  while (is_alive) {
    std::unique_lock<std::mutex> got_frame_lock(got_frame_mutex);
    got_frame.wait(got_frame_lock);

    std::unique_lock<std::mutex> buffer_lock(buffer_mutex);
    if (buffer_frame_begin_indices.size() > 1) {

      // Only process the most recent frame (to prevent buffer overflow)
      size_t index0 = buffer_frame_begin_indices[buffer_frame_begin_indices.size()-2];
      size_t index1 = buffer_frame_begin_indices[buffer_frame_begin_indices.size()-1];

      int decodedAmount = decodeFrame(&buffer[index0], index1-index0);
      picture_recv_timestamp = buffer_frame_recv_timestamps[buffer_frame_recv_timestamps.size()-1];
      int amountToDelete = index0 + decodedAmount;
      buffer.erase(buffer.begin(), buffer.begin() + amountToDelete);
      buffer_frame_begin_indices.clear();
      buffer_frame_begin_indices.push_back(index1-amountToDelete);
      buffer_frame_recv_timestamps.clear();
    }
    buffer_lock.unlock();
  }
  got_picture_cvar.notify_all(); // to allow convertToMat to terminate
}

int H264Decoder::decodeFrame(uint8_t* data, int size) {
  int len = 0;

  if (size <= 0) {
    return 0;
  }

  packet->data = data;
  packet->size = size;

  std::unique_lock<std::mutex> picture_lock(picture_mutex);
  len = avcodec_decode_video2(codec_context, picture, &got_picture, packet);
  picture_lock.unlock();
  if(len < 0) {
    printf("Error while decoding a frame.\n");
  }

  if(got_picture == 0) {
    return 0;
  } else {
    got_picture_cvar.notify_all();
  }

  ++frame;

  // NOTE: len_callback_value can be a good debugging tool to ensure the remote
  // computer is reading messages of the same number of bytes as the Kuri is
  // sending (especially if you pass len_callback_value to the callback instead
  // of cb_user)
  (*len_callback_value) = len;

  return len;
}

// // NOTE: this function assumes there is maximally one beginning of a message
// // per read. This is not strictly true -- however, decode still seems to work
// // even if the "frame" is actually 2 frames, so I have left it at this for
// // efficiency
// int H264Decoder::findBeginningOfH264Message(int bytes_read) {
//   int i = -1;
//   for (int j = 0; j < bytes_read-initial_sequence_len; j++) {
//     int k;
//     for (k = 0; k < initial_sequence_len; k++) {
//       if (inbuf[j+k] != initial_sequence[k]) {
//         break;
//       }
//     }
//     if (k == initial_sequence_len) { // found initial sequence!
//       i = j;
//       break;
//     }
//   }
//   return i;
// }

void H264Decoder::deestablishTCPConnection() {
  tcp_socket.close();
  if (work) {
    work->~work();
    free(work);
    work = NULL;
  }
  io_service.stop();
  async_io_service_thread.join(); // should be joinable since we destroyed the work and stopped the io_service
}

void H264Decoder::establishTCPConnection() {
  // Create work and start running the io_service, to enbale asynchronous operation
  work = new boost::asio::io_service::work(io_service);
  io_service.reset();
  async_io_service_thread = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));

  boost::system::error_code error = boost::asio::error::host_not_found;

  while (is_alive && error) {
    // Run an synchronous connect with a timeout of tcp_timeout
    std::unique_lock<std::mutex> tcp_connect_lock(tcp_connect_mutex);
    tcp_socket.async_connect(boost::asio::ip::tcp::endpoint(iter->endpoint()),
      [&error, this](const boost::system::error_code& err) {
        std::unique_lock<std::mutex> tcp_connect_lock_in_handler(tcp_connect_mutex); // don't start the handler until after the cvar timer has starter
        error = err;
        tcp_connect_lock_in_handler.unlock();
        tcp_connect_cvar.notify_all();
      });
    std::cv_status timer_result = tcp_connect_cvar.wait_for(tcp_connect_lock, std::chrono::seconds(tcp_timeout));
    if (timer_result == std::cv_status::timeout || !is_alive) { // when we stop we terminate the timeout early. But it should still count as an error
      error = boost::asio::error::timed_out;
    }

    if (error) {
      std::cout << "Error connecting, will keep trying. Error: " << error.message() << std::endl;
      deestablishTCPConnection();
    }
    tcp_connect_lock.unlock();
  }
}

void H264Decoder::readBuffer() {
  // Connect to the TCP socket
  establishTCPConnection();
  bool got_frame_bool;

  while (is_alive) {
    boost::system::error_code error = boost::asio::error::host_not_found;;
    int bytes_read;
    // Read Bytes
    while (is_alive && error) {
      // Run an asynchronous connect with a timeout of tcp_timeout
      std::unique_lock<std::mutex> tcp_read_some_lock(tcp_read_some_mutex);
      tcp_socket.async_read_some(boost::asio::buffer(inbuf, H264_INBUF_SIZE),
        [&error, &bytes_read, this](const boost::system::error_code& err, std::size_t bytes_transferred) {
          std::unique_lock<std::mutex> tcp_read_some_lock_in_handler(tcp_read_some_mutex); // ensure that the timeout timer has started before we start the handler
          error = err;
          bytes_read = bytes_transferred;
          tcp_read_some_lock_in_handler.unlock();
          tcp_read_some_cvar.notify_all();
        });
      std::cv_status timer_result = tcp_read_some_cvar.wait_for(tcp_read_some_lock, std::chrono::seconds(tcp_timeout));
      if (timer_result == std::cv_status::timeout || !is_alive) { // when we stop we terminate the timeout early. But it should still count as an error
        error = boost::asio::error::timed_out;
      }

      if (error) {
        std::cout << "Error reading, will disconnect and reconnect. Error: " << error.message() << std::endl;
        deestablishTCPConnection();
        establishTCPConnection();
      }
      tcp_read_some_lock.unlock();
    }
    // Check if we have received a full frame yet
    got_frame_bool = false;
    if(bytes_read) {

      std::unique_lock<std::mutex> buffer_lock(buffer_mutex);
      std::copy(inbuf, inbuf + bytes_read, std::back_inserter(buffer));
      int lastFrameBeginning = buffer_frame_begin_indices.size() > 0 ? buffer_frame_begin_indices[buffer_frame_begin_indices.size()-1] : 0;

      uint8_t* data = NULL;
      int size = 0;
      int len = av_parser_parse2(parser, codec_context, &data, &size,
                             &buffer[lastFrameBeginning], buffer.size()-lastFrameBeginning, 0, 0, AV_NOPTS_VALUE);

      if (size > 0 && len) {
        uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        /* // Uncomment this to determine (through manual calculation) network latency for one H264 packet
        std::cout << "got bytes " << len << " at time " << now << "\n";
        */

        buffer_frame_recv_timestamps.push_back(now);

        buffer_frame_begin_indices.push_back(lastFrameBeginning+len);
        got_frame_bool = true;
      }
      buffer_lock.unlock();


      if (got_frame_bool) {
        got_frame.notify_all();
      }
    }
  }
  got_frame.notify_all(); // to allow readFrame to terminate
}

void H264Decoder::startRead() {
  is_alive = true;

  read_buffer_thread = std::thread(&H264Decoder::readBuffer, this);
  read_frame_thread = std::thread(&H264Decoder::readFrame, this);
  convert_to_mat_thread = std::thread(&H264Decoder::convertToMat, this);
}

void H264Decoder::stopRead() {
  if (is_alive) {
    is_alive = false;
    deestablishTCPConnection();
    tcp_connect_cvar.notify_all(); // terminate any timeout for asynchronous connect
    tcp_read_some_cvar.notify_all(); // terminate any timeout for asynchronous read_some
    read_buffer_thread.join();
    read_frame_thread.join();
    convert_to_mat_thread.join();
  }
}
