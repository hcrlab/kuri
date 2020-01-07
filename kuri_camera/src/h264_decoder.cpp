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
  ,tcpTimeout(5)
{
  len_callback_value = new int;
  avcodec_register_all();
  // haveGottenFirstFrame = false;
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

  std::unique_lock<std::mutex> pictureLock(pictureMutex);
  if(picture) {
    av_free(picture);
    picture = NULL;
  }
  pictureLock.lock();

  if(packet) {
    av_free_packet(packet);
    packet = NULL;
  }

  if (is_alive) {
    is_alive = false;
    read_buffer_thread.join();
    read_frame_thread.join();
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
    if (image.rows != height || image.cols != width || image.type() != CV_8UC3) image = cv::Mat(height, width, CV_8UC3);
    int cvLinesizes[1];
    cvLinesizes[0] = image.step1();

    // Convert the color format and write directly to the opencv matrix
    SwsContext* conversion = sws_getContext(width, height, (AVPixelFormat) frame->format, width, height, AV_PIX_FMT_BGR24 /* PIX_FMT_BGR24 */, SWS_FAST_BILINEAR, NULL, NULL, NULL);
    sws_scale(conversion, frame->data, frame->linesize, 0, height, &image.data, cvLinesizes); // copies data
    sws_freeContext(conversion);
}

int H264Decoder::getMostRecentFrame(cv::Mat &image) {
  // std::cout << "getMostRecentFrame\n";
  int retVal = 0;
  std::unique_lock<std::mutex> pictureLock(pictureMutex);
  if (got_picture) {
    // std::cout << "pre avFrrameToMat\n";
    avframeToMat(picture, image);
    // std::cout << "post avFrrameToMat\n";
    pictureLock.unlock();
  } else {
    pictureLock.unlock();
    retVal = 1;
  }
  return retVal;
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

  std::unique_lock<std::mutex> pictureLock(pictureMutex);
  picture = avcodec_alloc_frame(); // av_frame_alloc(); //
  pictureLock.unlock();
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
    // std::cout << "readFrame \n";
    std::unique_lock<std::mutex> got_frame_lock(got_frame_mutex);
    got_frame.wait(got_frame_lock);

    std::unique_lock<std::mutex> buffer_lock(buffer_mutex);
    if (buffer_frame_begin_indices.size() > 1) {

      size_t index0 = buffer_frame_begin_indices[0];
      size_t index1 = buffer_frame_begin_indices[1];

      // uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      // std::cout << "got bytes " << (index1-index0) << " " << now;

      int decodedAmount = decodeFrame(&buffer[index0], index1-index0);
      buffer.erase(buffer.begin(), buffer.begin() + decodedAmount);
      buffer_frame_begin_indices.erase(buffer_frame_begin_indices.begin());
      for (int i = 0; i < buffer_frame_begin_indices.size(); i++) {
        buffer_frame_begin_indices[i] = buffer_frame_begin_indices[i] - decodedAmount;
      }
    }
    buffer_lock.unlock();
  }
}

int H264Decoder::decodeFrame(uint8_t* data, int size) {
  // std::cout << "decodeFrame\n";
  int len = 0;

  if (size <= 0) {
    return 0;
  }

  packet->data = data;
  packet->size = size;

  std::unique_lock<std::mutex> pictureLock(pictureMutex);
  len = avcodec_decode_video2(codec_context, picture, &got_picture, packet);
  pictureLock.unlock();
  if(len < 0) {
    printf("Error while decoding a frame.\n");
  }

  if(got_picture == 0) {
    return 0;
  }

  ++frame;

  // NOTE: len_callback_value can be a good debugging tool to ensure the remote
  // computer is reading messages of the same number of bytes as the Kuri is
  // sending (especially if you pass len_callback_value to the callback instead
  // of cb_user)
  (*len_callback_value) = len;

  // haveGottenFirstFrame = true;
  // std::cout << "pre decodedNewFrame.notify_all()\n";
  decodedNewFrame.notify_all();

  return len;
}

// NOTE: this function assumes there is maximally one beginning of a message
// per read. This is not strictly true -- however, decode still seems to work
// even if the "frame" is actually 2 frames, so I have left it at this for
// efficiency
int H264Decoder::findBeginningOfH264Message(int bytes_read) {
  int i = -1;
  for (int j = 0; j < bytes_read-initial_sequence_len; j++) {
    int k;
    for (k = 0; k < initial_sequence_len; k++) {
      if (inbuf[j+k] != initial_sequence[k]) {
        break;
      }
    }
    if (k == initial_sequence_len) { // found initial sequence!
      i = j;
      break;
    }
  }
  return i;
}

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
    // Run an synchronous connect with a timeout of tcpTimeout
    std::unique_lock<std::mutex> tcpConnectLock(tcpConnectMutex);
    tcp_socket.async_connect(boost::asio::ip::tcp::endpoint(iter->endpoint()),
      [&error, this](const boost::system::error_code& err) {
        std::unique_lock<std::mutex> tcpConnectLockInHandler(tcpConnectMutex); // don't start the handler until after the cvar timer has starter
        error = err;
        tcpConnectLockInHandler.unlock();
        tcpConnectCvar.notify_all();
      });
    std::cv_status timerResult = tcpConnectCvar.wait_for(tcpConnectLock, std::chrono::seconds(tcpTimeout));
    if (timerResult == std::cv_status::timeout || !is_alive) { // when we stop we terminate the timeout early. But it should still count as an error
      error = boost::asio::error::timed_out;
    }

    if (error) {
      std::cout << "Error connecting, will keep trying. Error: " << error.message() << std::endl;
      deestablishTCPConnection();
    }
    tcpConnectLock.unlock();
  }
}

void H264Decoder::readBuffer() {
  // Connect to the TCP socket
  establishTCPConnection();
  bool got_frame_bool;

  while (is_alive) {
    // std::cout << "readBuffer\n";
    boost::system::error_code error = boost::asio::error::host_not_found;;
    int bytes_read;
    while (is_alive && error) {
      // Run an asynchronous connect with a timeout of tcpTimeout
      std::unique_lock<std::mutex> tcpReadSomeLock(tcpReadSomeMutex);
      tcp_socket.async_read_some(boost::asio::buffer(inbuf, H264_INBUF_SIZE),
        [&error, &bytes_read, this](const boost::system::error_code& err, std::size_t bytes_transferred) {
          // std::cout << "before tcpReadSomeLockInHandler\n";
          std::unique_lock<std::mutex> tcpReadSomeLockInHandler(tcpReadSomeMutex); // ensure that the timeout timer has started before we start the handler
          // std::cout << "after tcpReadSomeLockInHandler\n";
          error = err;
          bytes_read = bytes_transferred;
          tcpReadSomeLockInHandler.unlock();
          tcpReadSomeCvar.notify_all();
        });
      std::cv_status timerResult = tcpReadSomeCvar.wait_for(tcpReadSomeLock, std::chrono::seconds(tcpTimeout));
      if (timerResult == std::cv_status::timeout || !is_alive) { // when we stop we terminate the timeout early. But it should still count as an error
        error = boost::asio::error::timed_out;
      }

      if (error) {
        std::cout << "Error reading, will disconnect and reconnect. Error: " << error.message() << std::endl;
        deestablishTCPConnection();
        establishTCPConnection();
      }
      tcpReadSomeLock.unlock();
    }
    got_frame_bool = false;
    if(bytes_read) {

      // int i = findBeginningOfH264Message(bytes_read);
      // std::unique_lock<std::mutex> buffer_lock(buffer_mutex);
      //
      // if (i >= 0) {
      //   buffer_frame_begin_indices.push_back(i+buffer.size());
      //   got_frame_bool = true;
      // }
      //
      // std::copy(inbuf, inbuf + bytes_read, std::back_inserter(buffer));
      //
      // buffer_lock.unlock();

      std::unique_lock<std::mutex> buffer_lock(buffer_mutex);
      std::copy(inbuf, inbuf + bytes_read, std::back_inserter(buffer));
      int lastFrameBeginning = buffer_frame_begin_indices.size() > 0 ? buffer_frame_begin_indices[buffer_frame_begin_indices.size()-1] : 0;

      uint8_t* data = NULL;
      int size = 0;
      int len = av_parser_parse2(parser, codec_context, &data, &size,
                             &buffer[lastFrameBeginning], buffer.size()-lastFrameBeginning, 0, 0, AV_NOPTS_VALUE);

      if (size > 0 && len) {
        buffer_frame_begin_indices.push_back(lastFrameBeginning+len);
        got_frame_bool = true;
      }
      buffer_lock.unlock();


      if (got_frame_bool) {
        // std::cout << "pre gotFrame notify_all\n";
        got_frame.notify_all();
      }
    }
  }
  got_frame.notify_all(); // to allow readFrame to terminate
}

void H264Decoder::startRead() {
  is_alive = true;
  // haveGottenFirstFrame = false;

  read_buffer_thread = std::thread(&H264Decoder::readBuffer, this);
  read_frame_thread = std::thread(&H264Decoder::readFrame, this);
}

void H264Decoder::stopRead() {
  if (is_alive) {
    is_alive = false;
    deestablishTCPConnection();
    tcpConnectCvar.notify_all(); // terminate any timeout for asynchronous connect
    tcpReadSomeCvar.notify_all(); // terminate any timeout for asynchronous read_some
    read_buffer_thread.join();
    read_frame_thread.join();
  }
}
