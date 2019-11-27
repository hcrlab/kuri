/*
  https://www.roxlu.com/2014/039/decoding-h264-and-yuv420p-playback
  H264_Decoder
  ---------------------------------------

  Example that shows how to use the libav parser system. This class forces a
  H264 parser and codec. You use it by opening a file that is encoded with x264
  using the `load()` function. You can pass the framerate you want to use for playback.
  If you don't pass the framerate, we will detect it as soon as the parser found
  the correct information.

  After calling load(), you can call readFrame() which will read a new frame when
  necessary. It will also make sure that it will read enough data from the buffer/file
  when there is not enough data in the buffer.

  `readFrame()` will trigger calls to the given `h264_decoder_callback` that you pass
  to the constructor.

 */
#ifndef H264_DECODER_H
#define H264_DECODER_H

#define H264_INBUF_SIZE 16384                                                           /* number of bytes we read per chunk */

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
// #include <tinylib.h>
#include <boost/asio.hpp>
#include <iostream>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/mem.h>
#include <libswscale/swscale.h>
}

typedef void(*h264_decoder_callback)(AVFrame* frame, AVPacket* pkt, void* user);         /* the decoder callback, which will be called when we have decoded a frame */

class H264_Decoder {

 public:
  H264_Decoder(h264_decoder_callback frameCallback, void* user);                            /* pass in a callback function that is called whenever we decoded a video frame, make sure to call `readFrame()` repeatedly */
  ~H264_Decoder();                                                                          /* d'tor, cleans up the allocated objects and closes the codec context */
  bool load(std::string hostname, std::string port);                                        /* load a video file which is encoded with x264 */
  void startRead();
  void stopRead();

 private:
  bool update(bool& needsMoreBytes);                                                     /* internally used to update/parse the data we read from the buffer or file */
  void readFrame();                                                                      /* read a frame */
  void readBuffer();                                                                      /* reads data from the buffer */
  void decodeFrame(uint8_t* data, int size);                                             /* decode a frame we read from the buffer */
  int findBeginningOfH264Message(int bytes_read);

 public:
  AVCodec* codec;                                                                        /* the AVCodec* which represents the H264 decoder */
  AVCodecContext* codec_context;                                                         /* the context; keeps generic state */
  AVCodecParserContext* parser;                                                          /* parser that is used to decode the h264 bitstream */
  AVFrame* picture;                                                                      /* will contain a decoded picture */
  uint8_t inbuf[H264_INBUF_SIZE + FF_INPUT_BUFFER_PADDING_SIZE];                         /* used to read chunks from the file */
  // FILE* fp;                                                                              /* file pointer to the file from which we read the h264 data */
  boost::asio::io_service io_service;
  boost::asio::ip::tcp::socket tcpSocket;                                                /* the tcp socket connection */
  boost::asio::ip::tcp::resolver::iterator iter;
  int frame;                                                                             /* the number of decoded frames */
  h264_decoder_callback cb_frame;                                                        /* the callback function which will receive the frame/packet data */
  void* cb_user;                                                                         /* the void* with user data that is passed into the set callback */
  int* lenCallbackValue;
  std::vector<uint8_t> buffer;                                                           /* buffer we use to keep track of read/unused bitstream data */
  std::vector<size_t> bufferFrameBeginIndices;
  std::thread readBufferThread;
  std::thread readFrameThread;
  std::mutex bufferMutex;
  std::atomic<bool> isAlive;
  std::condition_variable gotFrame;
  std::mutex gotFrameMutex;
  const size_t initialSequenceLen = 5;
  const uint8_t initialSequence[5] = {0x00, 0x00, 0x00, 0x01, 0x09};
};

#endif
