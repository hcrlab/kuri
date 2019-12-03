/*
 * Code adapted from https://www.roxlu.com/2014/039/decoding-h264-and-yuv420p-playback
 */

#include "h264_decoder.h"

H264Decoder::H264Decoder(h264_decoder_callback frame_callback, void* user)
  :codec(NULL)
  ,codec_context(NULL)
  ,parser(NULL)
  ,frame(0)
  ,cb_frame(frame_callback)
  ,cb_user(user)
  ,is_alive(false)
  ,tcp_socket(io_service)
{
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

  if(picture) {
    av_free(picture);
    picture = NULL;
  }

  if (is_alive) {
    is_alive = false;
    read_buffer_thread.join();
    read_frame_thread.join();
  }

  cb_frame = NULL;
  cb_user = NULL;
  frame = 0;
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

  picture = avcodec_alloc_frame();
  parser = av_parser_init(AV_CODEC_ID_H264);
  packet = new AVPacket;

  if(!parser) {
    printf("Erorr: cannot create H264 parser.\n");
    return false;
  }

  return true;
}

void H264Decoder::readFrame() {
  while (is_alive) {
    std::unique_lock<std::mutex> got_frameLock(got_frame_mutex);
    got_frame.wait(got_frameLock);

    std::unique_lock<std::mutex> bufferLock(buffer_mutex);
    if (buffer_frame_begin_indices.size() > 1) {

      size_t index0 = buffer_frame_begin_indices[0];
      size_t index1 = buffer_frame_begin_indices[1];

      // uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      // std::cout << "got bytes " << (index1-index0) << " " << now;

      decodeFrame(&buffer[index0], index1-index0);
      buffer.erase(buffer.begin(), buffer.begin() + index1);
      buffer_frame_begin_indices.erase(buffer_frame_begin_indices.begin());
      for (int i = 0; i < buffer_frame_begin_indices.size(); i++) {
        buffer_frame_begin_indices[i] = buffer_frame_begin_indices[i] - index1;
      }
    }
    bufferLock.unlock();
  }
}

void H264Decoder::decodeFrame(uint8_t* data, int size) {
  int got_picture = 0;
  int len = 0;
  av_init_packet(packet);

  packet->data = data;
  packet->size = size;

  len = avcodec_decode_video2(codec_context, picture, &got_picture, packet);
  if(len < 0) {
    printf("Error while decoding a frame.\n");
  }

  if(got_picture == 0) {
    return;
  }

  ++frame;

  // NOTE: len_callback_value can be a good debugging tool to ensure the remote
  // computer is reading messages of the same number of bytes as the Kuri is
  // sending (especially if you pass len_callback_value to the callback instead
  // of cb_user)
  (*len_callback_value) = len;

  if(cb_frame) {
    cb_frame(picture, packet, cb_user/*len_callback_value*/);
  }
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

void H264Decoder::readBuffer() {
  // Connect to the TCP socket
  tcp_socket.connect(boost::asio::ip::tcp::endpoint(iter->endpoint()));
  bool got_frameBool;

  while (is_alive) {
    int bytes_read = tcp_socket.read_some(boost::asio::buffer(inbuf, H264_INBUF_SIZE));
    got_frameBool = false;
    if(bytes_read) {
      int i = findBeginningOfH264Message(bytes_read);
      std::unique_lock<std::mutex> bufferLock(buffer_mutex);

      if (i >= 0) {
        buffer_frame_begin_indices.push_back(i+buffer.size());
        got_frameBool = true;
      }

      std::copy(inbuf, inbuf + bytes_read, std::back_inserter(buffer));

      bufferLock.unlock();

      if (got_frameBool) {
        got_frame.notify_all();
      }
    }
  }
}

void H264Decoder::startRead() {
  read_buffer_thread = std::thread(&H264Decoder::readBuffer, this);
  read_frame_thread = std::thread(&H264Decoder::readFrame, this);
  is_alive = true;
}

void H264Decoder::stopRead() {
  if (is_alive) {
    is_alive = false;
    read_buffer_thread.join();
    read_frame_thread.join();
  }
}
