/*
  Adapted from https://www.roxlu.com/2014/039/decoding-h264-and-yuv420p-playback
 */
#ifndef H264_DECODER_H
#define H264_DECODER_H

#define H264_INBUF_SIZE 16384                                                   /* number of bytes we read per chunk */

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>
#include <opencv2/highgui/highgui.hpp>
#include <madmux/madmux.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/mem.h>
#include <libswscale/swscale.h>
}


class H264DecoderOnKuri {

 public:
  H264DecoderOnKuri();                                                                 /* pass in a callback function that is called whenever we decoded a video frame, make sure to call `readFrame()` repeatedly */
  ~H264DecoderOnKuri();                                                                /* cleans up the allocated objects and closes the codec context */
  bool load(std::string hostname, std::string port);                             /* load a video file which is encoded with x264 */
  void startRead();                                                              /* start the readBuffer and readFrame threads */
  void stopRead();                                                               /* stop reading */
  int getMostRecentFrame(cv::Mat &image, uint64_t &recv_timestamp);              /* returns the most recent fully-processed frame */
  std::condition_variable decoded_new_frame;                                     /* notifies clients that a new frame is available */
  void h264_frame_callback(uint8_t *buffer, uint32_t size);

 private:
  void readFrame();                                                              /* loops and reads a frame as it become available */
  void readFrameOne();
  // void readBuffer();                                                             /* loops and continuously reads from the TCP socket */
  void convertToMat();
  int decodeFrame(uint8_t* data, int size, bool& wasIFrame);                                      /* decode a frame we read from the buffer */
  // int findBeginningOfH264Message(int bytes_read);                                /* as we readBuffer, this function finds the beginning of H264 messages */
  // void establishTCPConnection();                                                 /* keep retrying until it establishes a TCP connection */
  // void deestablishTCPConnection();                                               /* close the socket and reset the io_service */
  static void avframeToMat(const AVFrame * frame, cv::Mat& image);               /* Convert the AV Frames to CV Matrices */

  AVCodec* codec;                                                                /* the AVCodec* which represents the H264 decoder */
  AVCodecContext* codec_context;                                                 /* the context; keeps generic state */
  AVCodecParserContext* parser;                                                  /* parser that is used to decode the h264 bitstream */
  AVFrame* picture;                                                              /* will contain a decoded picture */
  std::mutex picture_mutex;                                                      /* used to sync access to picture */
  int got_picture;                                                               /* whether or not the picture was succesfully received */
  uint64_t picture_recv_timestamp;                                               /* when the H264 frame corresponding to the picture that is currently stored was received */
  std::condition_variable got_picture_cvar;                                      /* used to notify the convertToMat thread that we have a decoded AVFrame */
  cv::Mat cv_image;                                                              /* will contain a decoded cv::Mat */
  std::mutex cv_image_mutex;                                                     /* used to synchronize access to the cv_image */
  uint64_t cv_image_recv_timestamp;                                              /* when the H264 frame corresponding to the cv_image that is currently stored was received */
  AVPacket* packet;                                                              /* will contain the bytes for one frame */
  // uint8_t inbuf[H264_INBUF_SIZE + FF_INPUT_BUFFER_PADDING_SIZE];                 /* used to read chunks from the file */
  // boost::asio::io_service io_service;                                            /* the io_service */
  // boost::asio::io_service::work* work;                                           /* the filler work for the io_service's queue */
  // boost::asio::local::stream_protocol::socket uds_socket;                        /* the uds socket connection */
  mdx_stream *madmux_stream;
  int frame;                                                                     /* the number of decoded frames */
  int* len_callback_value;                                                       /* stores the size (bytes) of the most recent frame -- can be used for debugging */
  std::vector<uint8_t> buffer;                                                   /* buffer we use to keep track of read/unused bitstream data */
  std::vector<size_t> buffer_frame_end_indices;                                /* the buffer indices that refer to the beginning of frames (as determined by findBeginningOfH264Message) */
  std::vector<uint64_t> buffer_frame_recv_timestamps;                            /* contains the timestamps when the complete H264 frames currently in the buffer were received */
  std::thread read_buffer_thread;                                                /* the thread that runs readBuffer */
  std::thread read_frame_thread;                                                 /* the thread that runs readFrame */
  std::thread convert_to_mat_thread;                                             /* the thread that runs converToMat */
  std::mutex buffer_mutex;                                                       /* the mutex that controls access to the buffer */
  std::atomic<bool> is_alive;                                                    /* the threads loop as long as is_alive is true */
  std::condition_variable got_frame;                                             /* the cvar that tells readFrame that we have a full frame to debug */
  std::mutex got_frame_mutex;                                                    /* the mutex associated with got_frame */
  const uint8_t initial_sequence[5] = {0x00, 0x00, 0x00, 0x01, 0x09};            /* the 5-byte sequence that indicates the beginning of a new H264 frame */
  const size_t initial_sequence_len = 5;                                         /* the len of the initial_sequence */
  boost::thread async_io_service_thread;                                         /* used to asynchronously run the io_service's run method */
  // int tcp_timeout;                                                               /* how many seconds to wait for a connection and/or read before timing out */
  // std::mutex tcp_connect_mutex;                                                  /* mutex for tcp_connect_cvar */
  // std::condition_variable tcp_connect_cvar;                                      /* used to synchronize the asynchronous tcp connection function with the timeout */
  // std::mutex tcp_read_some_mutex;                                                /* mutex for tcp_read_some_cvar */
  // std::condition_variable tcp_read_some_cvar;                                    /* used to synchronize the asynchronous tcp read_some function with the timeout */
  // int connection_sleep_time;                                                     /* if the connection fails, how long to sleep before trying again */
};

#endif
