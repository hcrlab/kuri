#include "ros/ros.h"
#include "h264_decoder.h"
#include <opencv2/highgui/highgui.hpp>
#include <chrono>

// https://timvanoosterhout.wordpress.com/2015/07/02/converting-an-ffmpeg-avframe-to-and-opencv-mat/
void avframeToMat(const AVFrame * frame, cv::Mat& image)
{
    int width = frame->width;
    int height = frame->height;

    // Allocate the opencv mat and store its stride in a 1-element array
    if (image.rows != height || image.cols != width || image.type() != CV_8UC3) image = cv::Mat(height, width, CV_8UC3);
    int cvLinesizes[1];
    cvLinesizes[0] = image.step1();

    // Convert the color format and write directly to the opencv matrix
    SwsContext* conversion = sws_getContext(width, height, (AVPixelFormat) frame->format, width, height, PIX_FMT_BGR24, SWS_FAST_BILINEAR, NULL, NULL, NULL);
    sws_scale(conversion, frame->data, frame->linesize, 0, height, &image.data, cvLinesizes);
    sws_freeContext(conversion);
}

void frameCallback(AVFrame* frame, AVPacket* pkt, void* user) {
  cv::Mat cvImage;
  avframeToMat(frame, cvImage);
  // uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  // std::cout << "got cv::Mat " << *((int*)user) << " " << now << std::endl;
  cv::imshow("view", cvImage);
  cv::waitKey(1);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "h264_decoder_node");
  ros::Time::init();
  cv::namedWindow("view");
	H264Decoder h264Decoder(frameCallback, NULL);

  std::string tcpSocketHostname;
  int tcpSocketPort;
  ros::param::param<std::string>("tcpSocketHostname", tcpSocketHostname, "cococutkuri.personalrobotics.cs.washington.edu");
  ros::param::param<int>("tcpSocketPort", tcpSocketPort, 1234);
  h264Decoder.load(tcpSocketHostname, std::to_string(tcpSocketPort));

  h264Decoder.startRead();
	ros::spin();
  h264Decoder.stopRead();
	return 0;
}
