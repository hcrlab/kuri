/*
 * Code adapted from https://www.roxlu.com/2014/039/decoding-h264-and-yuv420p-playback
 */

#include "h264_decoder.h"

H264_Decoder::H264_Decoder(h264_decoder_callback frameCallback, void* user)
  :codec(NULL)
  ,codec_context(NULL)
  ,parser(NULL)
  ,frame(0)
  ,cb_frame(frameCallback)
  ,cb_user(user)
  ,isAlive(false)
  ,tcpSocket(io_service)
{
  lenCallbackValue = new int;
  avcodec_register_all();
}

H264_Decoder::~H264_Decoder() {

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

  if (isAlive) {
    isAlive = false;
    readBufferThread.join();
    readFrameThread.join();
  }

  cb_frame = NULL;
  cb_user = NULL;
  frame = 0;
}

bool H264_Decoder::load(std::string hostname, std::string port) {

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

  if(!parser) {
    printf("Erorr: cannot create H264 parser.\n");
    return false;
  }

  return true;
}

void H264_Decoder::readFrame() {
  // std::cout << "readFrame\n";
  while (isAlive) {
    // std::cout << "readFrame b4 got frame\n";
    std::unique_lock<std::mutex> gotFrameLock(gotFrameMutex);
    gotFrame.wait(gotFrameLock);
    // gotFrameLock.unlock();
    // std::cout << "readFrame after got frame\n";

    // bool needs_more = false;
    // update(needs_more);
    // std::cout << "readFrame before buffer\n";
    std::unique_lock<std::mutex> bufferLock(bufferMutex);
    if (bufferFrameBeginIndices.size() > 1) {
      size_t index0 = bufferFrameBeginIndices[0];
      size_t index1 = bufferFrameBeginIndices[1];
      uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      std::cout << "got bytes " << (index1-index0) << " " << now;
      decodeFrame(&buffer[index0], index1-index0);
      // // std::cout << "readFrame pre erase0\n";
      buffer.erase(buffer.begin(), buffer.begin() + index1);
      // // std::cout << "readFrame pre erase1\n";
      bufferFrameBeginIndices.erase(bufferFrameBeginIndices.begin());
      // // std::cout << "readFrame pre fix frames\n";
      for (int i = 0; i < bufferFrameBeginIndices.size(); i++) {
        bufferFrameBeginIndices[i] = bufferFrameBeginIndices[i] - index1;
      }
      // // std::cout << "readFrame post all\n";
    }
    bufferLock.unlock();
    // std::cout << "readFrame after buffer\n";
  }
}

void H264_Decoder::decodeFrame(uint8_t* data, int size) {

  AVPacket pkt;
  int got_picture = 0;
  int len = 0;

  // // std::cout << "decodeFrame pre-init-packet\n";
  av_init_packet(&pkt);

  pkt.data = data;
  pkt.size = size;

  // // std::cout << "decodeFrame pre-decode-packet\n";
  len = avcodec_decode_video2(codec_context, picture, &got_picture, &pkt);
  // // std::cout << "decodeFrame post-decode-packet " << len << " \n";
  if(len < 0) {
    printf("Error while decoding a frame.\n");
  }

  (*lenCallbackValue) = len;

  if(got_picture == 0) {
    // // std::cout << "decodeFrame got no pic\n";
    return;
  }

  ++frame;

  if(cb_frame) {
    // // std::cout << "decodeFrame pre cb\n";
    cb_frame(picture, &pkt, lenCallbackValue/*cb_user*/);
    // // std::cout << "decodeFrame post cb\n";
  }
}

// NOTE: assumes there is maximally one beginning of a message per read
int H264_Decoder::findBeginningOfH264Message(int bytes_read) {
  int i = -1;
  for (int j = 0; j < bytes_read-initialSequenceLen; j++) {
    int k;
    for (k = 0; k < initialSequenceLen; k++) {
      if (inbuf[j+k] != initialSequence[k]) {
        break;
      }
    }
    if (k == initialSequenceLen) { // found initial sequence!
      i = j;
      break;
    }
  }
  return i;
}

void H264_Decoder::readBuffer() {
  // Connect to the TCP socket
  tcpSocket.connect(boost::asio::ip::tcp::endpoint(iter->endpoint()));
  // std::cout << "readBuffer\n";
  bool gotFrameBool;
  while (isAlive) {
    // std::cout << "readBuffer 0\n";
    int bytes_read = tcpSocket.read_some(boost::asio::buffer(inbuf, H264_INBUF_SIZE));
    gotFrameBool = false;
    if(bytes_read) {
      int i = findBeginningOfH264Message(bytes_read);
      // std::cout << "readBuffer before buffer\n";
      std::unique_lock<std::mutex> bufferLock(bufferMutex);

      if (i >= 0) {
        bufferFrameBeginIndices.push_back(i+buffer.size());
        gotFrameBool = true;
      }

      std::copy(inbuf, inbuf + bytes_read, std::back_inserter(buffer));

      bufferLock.unlock();
      // std::cout << "readBuffer after buffer\n";

      if (gotFrameBool) {
        // std::cout << "readBuffer before got frame\n";
        // std::unique_lock<std::mutex> gotFrameLock(gotFrameMutex);
        gotFrame.notify_all();
        // gotFrameLock.unlock();
        // std::cout << "readBuffer after got frame\n";
      }

      // update(needs_more);
    }
  }
}

bool H264_Decoder::update(bool& needsMoreBytes) {
  // // std::cout << "update\n";
  needsMoreBytes = false;

  std::unique_lock<std::mutex> bufferLock(bufferMutex);

  if(buffer.size() == 0) {
    needsMoreBytes = true;
    bufferLock.unlock();
    return false;
  }

  uint8_t* data = NULL;
  int size = 0;
  // // std::cout << "pre-parser\n";
  int len = av_parser_parse2(parser, codec_context, &data, &size,
                             &buffer[0], buffer.size(), 0, 0, AV_NOPTS_VALUE);
  // // std::cout << "post-parser\n";

  if(size == 0 && len >= 0) {
    // // std::cout << "need more bytes\n";
    needsMoreBytes = true;
    bufferLock.unlock();
    return false;
  }

  if(size /*len*/) {
    // // std::cout << "pre-decode\n";
    decodeFrame(&buffer[0], len);
    // // std::cout << "pre-erase\n";
    buffer.erase(buffer.begin(), buffer.begin() + len);
    // // std::cout << "post-erase\n";
    bufferLock.unlock();
    return true;
  }

  // // std::cout << "not sure what happened...\n";

  bufferLock.unlock();
  return false;
}

void H264_Decoder::startRead() {
  readBufferThread = std::thread(&H264_Decoder::readBuffer, this);
  readFrameThread = std::thread(&H264_Decoder::readFrame, this);
  isAlive = true;
}

void H264_Decoder::stopRead() {
  if (isAlive) {
    isAlive = false;
    readBufferThread.join();
    readFrameThread.join();
  }
}
