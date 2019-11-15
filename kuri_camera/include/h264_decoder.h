/*
  Adapted from https://www.roxlu.com/2014/039/decoding-h264-and-yuv420p-playback
 */
#ifndef H264_DECODER_H
#define H264_DECODER_H

#define H264_INBUF_SIZE 16384                                                    /* number of bytes we read per chunk */

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

typedef void(*h264_decoder_callback)(AVFrame* frame, AVPacket* pkt, void* user); /* the decoder callback, which will be called when we have decoded a frame */

class H264Decoder {

 public:
  H264Decoder(h264_decoder_callback frame_callback, void* user);                 /* pass in a callback function that is called whenever we decoded a video frame, make sure to call `readFrame()` repeatedly */
  ~H264Decoder();                                                                /* cleans up the allocated objects and closes the codec context */
  bool load(std::string hostname, std::string port);                             /* load a video file which is encoded with x264 */
  void startRead();                                                              /* start the readBuffer and readFrame threads */
  void stopRead();                                                               /* stop reading */

 private:
  void readFrame();                                                              /* loops and reads a frame as it become available */
  void readBuffer();                                                             /* loops and continuously reads from the TCP socket */
  void decodeFrame(uint8_t* data, int size);                                     /* decode a frame we read from the buffer */
  int findBeginningOfH264Message(int bytes_read);                                /* as we readBuffer, this function finds the beginning of H264 messages */
  void establishTCPConnection();                                                 /* keep retrying until it establishes a TCP connection */

 public:
  AVCodec* codec;                                                                /* the AVCodec* which represents the H264 decoder */
  AVCodecContext* codec_context;                                                 /* the context; keeps generic state */
  AVCodecParserContext* parser;                                                  /* parser that is used to decode the h264 bitstream */
  AVFrame* picture;                                                              /* will contain a decoded picture */
  AVPacket* packet;                                                              /* will contain the bytes for one frame */
  uint8_t inbuf[H264_INBUF_SIZE + FF_INPUT_BUFFER_PADDING_SIZE];                 /* used to read chunks from the file */
  boost::asio::io_service io_service;                                            /* the io_service */
  boost::asio::ip::tcp::socket tcp_socket;                                       /* the tcp socket connection */
  boost::asio::ip::tcp::resolver::iterator iter;                                 /* used to resolve the query to the Kuri */
  int frame;                                                                     /* the number of decoded frames */
  h264_decoder_callback cb_frame;                                                /* the callback function which will receive the frame/packet data */
  void* cb_user;                                                                 /* the void* with user data that is passed into the set callback */
  int* len_callback_value;                                                       /* stores the size (bytes) of the most recent frame -- can be used for debugging */
  std::vector<uint8_t> buffer;                                                   /* buffer we use to keep track of read/unused bitstream data */
  std::vector<size_t> buffer_frame_begin_indices;                                /* the buffer indices that refer to the beginning of frames (as determined by findBeginningOfH264Message) */
  std::thread read_buffer_thread;                                                /* the thread that runs readBuffer */
  std::thread read_frame_thread;                                                 /* the thread that runs readFrame */
  std::mutex buffer_mutex;                                                       /* the mutex that controls access to the buffer */
  std::atomic<bool> is_alive;                                                    /* the threads loop as long as is_alive is true */
  std::condition_variable got_frame;                                             /* the cvar that tells readFrame that we have a full frame to debug */
  std::mutex got_frame_mutex;                                                    /* the mutex associated with got_frame */
  const uint8_t initial_sequence[5] = {0x00, 0x00, 0x00, 0x01, 0x09};            /* the 5-byte sequence that indicates the beginning of a new H264 frame */
  const size_t initial_sequence_len = 5;                                         /* the len of the initial_sequence */
};

#endif
